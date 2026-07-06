#include "basements/io/urdf_physics_bridge.h"

#include "basements/core/math/matrix3.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

namespace basements {
namespace io {

using basements::math::Matrix3;
using basements::math::Quaternion;
using basements::math::Vec3;
using basements::engine::BodyHandle;
using basements::engine::ConstraintHandle;
using basements::engine::JointDescriptor;
using basements::engine::JointType;
using basements::engine::PhysicsWorldGPU;
using basements::engine::RigidBodyDesc;
using basements::engine::BodyType;

namespace {

inline Vec3 to_vec3(const URDFVector3& v)       { return Vec3(v.x, v.y, v.z); }
// basements::Quaternion ctor is (w, x, y, z); URDFQuaternion stores (x, y, z, w).
inline Quaternion to_quat(const URDFQuaternion& q) { return Quaternion(q.w, q.x, q.y, q.z); }

// Pose composition: T_world = T_parent ∘ T_local, where T = (R, t).
struct Pose {
    Vec3       p = Vec3::zero();
    Quaternion q = Quaternion::identity();

    Pose then(const Pose& local) const {
        Pose out;
        out.q = q * local.q;
        out.p = p + q.rotate(local.p);
        return out;
    }
};

// Symmetric 3x3 inertia tensor from URDFInertial.
Matrix3 inertia_from_urdf(const URDFInertial& i) {
    return Matrix3(
        i.ixx, i.ixy, i.ixz,
        i.ixy, i.iyy, i.iyz,
        i.ixz, i.iyz, i.izz
    );
}

// Rough half-extents from the first collision element. Defaults if absent.
Vec3 link_half_extents(const URDFLink& link, const Vec3& fallback) {
    if (link.collisions.empty()) return fallback;
    const URDFGeometry& g = link.collisions.front().geometry;
    switch (g.type) {
    case URDFGeometry::BOX:      return Vec3(g.size.x * 0.5f, g.size.y * 0.5f, g.size.z * 0.5f);
    case URDFGeometry::SPHERE:   return Vec3(g.radius, g.radius, g.radius);
    case URDFGeometry::CYLINDER: return Vec3(g.radius, g.length * 0.5f, g.radius);
    case URDFGeometry::MESH:     return fallback;
    }
    return fallback;
}

} // anonymous namespace

URDFBridgeResult URDFPhysicsBridge::spawn(const URDFRobot& robot,
                                         PhysicsWorldGPU& world,
                                         const URDFBridgeOptions& opts) {
    URDFBridgeResult result;
    const size_t n_links  = robot.links.size();
    const size_t n_joints = robot.joints.size();
    if (n_links == 0) {
        result.warnings.push_back("URDF has no links — nothing to spawn.");
        return result;
    }

    // ---- 1. Build child→joint and parent→children maps for FK traversal -----
    std::unordered_map<std::string, const URDFJoint*> parent_joint_of_child;
    std::unordered_map<std::string, std::vector<const URDFJoint*>> joints_of_parent;
    for (const auto& j : robot.joints) {
        parent_joint_of_child[j.child_link] = &j;
        joints_of_parent[j.parent_link].push_back(&j);
    }

    // Identify root links (no parent joint). A well-formed URDF has exactly one,
    // but we tolerate forests and spawn each tree from base_position.
    std::vector<size_t> root_indices;
    for (size_t i = 0; i < n_links; ++i) {
        if (parent_joint_of_child.find(robot.links[i].name) == parent_joint_of_child.end())
            root_indices.push_back(i);
    }
    if (root_indices.empty()) {
        result.warnings.push_back("URDF has a cyclic parent chain — no root link found.");
        return result;
    }

    // ---- 2. Forward kinematics: world pose per link ------------------------
    std::vector<Pose> link_world(n_links);
    std::vector<bool> visited(n_links, false);
    std::unordered_map<std::string, size_t> idx_of_link;
    for (size_t i = 0; i < n_links; ++i) idx_of_link[robot.links[i].name] = i;

    for (size_t root : root_indices) {
        Pose root_pose{ opts.base_position, opts.base_orientation };
        link_world[root] = root_pose;
        visited[root] = true;

        std::queue<size_t> q;
        q.push(root);
        while (!q.empty()) {
            size_t parent_idx = q.front(); q.pop();
            const std::string& pname = robot.links[parent_idx].name;
            auto it = joints_of_parent.find(pname);
            if (it == joints_of_parent.end()) continue;
            for (const URDFJoint* j : it->second) {
                auto cit = idx_of_link.find(j->child_link);
                if (cit == idx_of_link.end()) {
                    result.warnings.push_back("Joint '" + j->name + "' references missing link '" + j->child_link + "'.");
                    continue;
                }
                size_t child_idx = cit->second;
                if (visited[child_idx]) {
                    result.warnings.push_back("Cyclic / duplicate parent for link '" + j->child_link + "' — ignoring extra edge.");
                    continue;
                }
                // Compose: parent_world ∘ joint_origin ∘ joint_motion.
                Pose joint_origin{ to_vec3(j->origin.position), to_quat(j->origin.orientation) };
                Pose joint_motion;  // identity

                auto qit = opts.initial_joint_positions.find(j->name);
                if (qit != opts.initial_joint_positions.end()) {
                    float coord = qit->second;
                    Vec3 axis = to_vec3(j->axis);
                    float al = axis.length();
                    if (al > 1e-9f) axis = axis * (1.0f / al);
                    if (j->type == URDFJointType::REVOLUTE ||
                        j->type == URDFJointType::CONTINUOUS) {
                        joint_motion.q = Quaternion::from_axis_angle(axis, coord);
                    } else if (j->type == URDFJointType::PRISMATIC) {
                        joint_motion.p = axis * coord;
                    }
                    // FIXED/FLOATING/PLANAR — coord is meaningless; skip silently.
                }

                Pose local = joint_origin.then(joint_motion);
                link_world[child_idx] = link_world[parent_idx].then(local);
                visited[child_idx] = true;
                q.push(child_idx);
            }
        }
    }

    // ---- 3. Create bodies in URDF link order -------------------------------
    result.link_bodies.resize(n_links);
    for (size_t i = 0; i < n_links; ++i) {
        const URDFLink& link = robot.links[i];

        RigidBodyDesc desc;
        desc.position    = link_world[i].p;
        desc.orientation = link_world[i].q;

        bool is_root_fixed = opts.fix_base && std::find(root_indices.begin(), root_indices.end(), i) != root_indices.end();
        desc.type = is_root_fixed ? BodyType::STATIC : BodyType::DYNAMIC;

        if (desc.type == BodyType::DYNAMIC) {
            desc.mass = link.inertial.mass > 0.0f ? link.inertial.mass : opts.default_link_mass;
            desc.inertia_tensor = inertia_from_urdf(link.inertial);
        } else {
            desc.mass = 0.0f;
            desc.inertia_tensor = Matrix3::identity();
        }
        desc.half_extents = link_half_extents(link, opts.default_half_extents);

        BodyHandle h = world.create_body(desc);
        result.link_bodies[i]                = h;
        result.link_by_name[link.name]       = h;
    }

    // ---- 4. Create joints in URDF order -----------------------------------
    result.joint_constraints.resize(n_joints);
    for (size_t k = 0; k < n_joints; ++k) {
        const URDFJoint& j = robot.joints[k];

        auto pit = idx_of_link.find(j.parent_link);
        auto cit = idx_of_link.find(j.child_link);
        if (pit == idx_of_link.end() || cit == idx_of_link.end()) {
            result.warnings.push_back("Joint '" + j.name + "' references missing link.");
            continue;
        }
        BodyHandle ha = result.link_bodies[pit->second];
        BodyHandle hb = result.link_bodies[cit->second];

        JointDescriptor d;
        d.body_a         = ha;
        d.body_b         = hb;
        d.local_anchor_a = to_vec3(j.origin.position);   // parent-local
        d.local_anchor_b = Vec3::zero();                 // child origin = joint origin
        d.local_axis_a   = to_vec3(j.axis);
        d.local_axis_b   = to_vec3(j.axis);

        bool create = true;
        switch (j.type) {
        case URDFJointType::REVOLUTE:
            d.type            = JointType::HINGE;
            d.enable_limits   = true;
            d.lower_limit     = j.lower_limit;
            d.upper_limit     = j.upper_limit;
            break;
        case URDFJointType::CONTINUOUS:
            d.type            = JointType::HINGE;
            d.enable_limits   = false;
            break;
        case URDFJointType::PRISMATIC:
            d.type            = JointType::SLIDER;
            d.enable_limits   = true;
            d.lower_limit     = j.lower_limit;
            d.upper_limit     = j.upper_limit;
            break;
        case URDFJointType::FIXED:
            d.type            = JointType::FIXED;
            break;
        case URDFJointType::FLOATING:
            create = false;
            result.warnings.push_back("Joint '" + j.name + "' is FLOATING — no constraint created (free 6-DOF).");
            break;
        case URDFJointType::PLANAR:
            create = false;
            result.warnings.push_back("Joint '" + j.name + "' is PLANAR — no native equivalent; child link left free.");
            break;
        }

        if (create) {
            ConstraintHandle ch = world.create_joint(d);
            result.joint_constraints[k]   = ch;
            result.joint_by_name[j.name]  = ch;
        }
    }

    return result;
}

} // namespace io
} // namespace basements

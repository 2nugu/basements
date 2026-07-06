#include "basements/physics/coupling/mpm_rigid_coupler.h"

#include <algorithm>
#include <cmath>

namespace basements {
namespace physics {
namespace coupling {

using basements::math::Matrix3;
using basements::math::Quaternion;
using basements::math::Vec3;
using basements::mpm::BLOCK_MASK;
using basements::mpm::BLOCK_SIZE;
using basements::mpm::BLOCK_SIZE_LOG2;
using basements::mpm::GridBlock;
using basements::mpm::GridNode;
using basements::mpm::SPGridCPU;

namespace {

// Returns (inside, normal, depth) where normal is the body-local outward
// face normal of the closest OBB face and depth is the local-space penetration.
// If the node is outside the OBB, inside == false.
struct OBBProbe {
    bool inside = false;
    Vec3 normal_local{0, 0, 0};
    float depth = 0.0f;
};

// Shape-aware probe. Delegates inside-check and outward-normal to the helpers
// in mpm_rigid_coupler.h so BOX and CYLINDER share the same Dirichlet pipeline.
OBBProbe probe_obb(const Vec3& world_point,
                   const RigidColliderState& body,
                   const Matrix3& R_world_to_local) {
    OBBProbe r;
    Vec3 local = R_world_to_local * (world_point - body.center);
    if (!shape_inside(body, local)) return r;
    r.inside       = true;
    r.normal_local = shape_outward_normal_local(body, local);
    // Penetration depth ≈ distance from local to the surface along the normal.
    if (body.shape == ShapeKind::CYLINDER) {
        const float R = body.half_extents.x;
        const float L = body.half_extents.y;
        const float radial = std::sqrt(local.x * local.x + local.z * local.z);
        r.depth = std::min(L - std::abs(local.y), R - radial);
    } else {
        const float dx = body.half_extents.x - std::abs(local.x);
        const float dy = body.half_extents.y - std::abs(local.y);
        const float dz = body.half_extents.z - std::abs(local.z);
        r.depth = std::min({dx, dy, dz});
    }
    return r;
}

} // anonymous namespace

CouplingReaction MPMRigidCoupler::apply(SPGridCPU& grid,
                                        const RigidColliderState& body,
                                        float dt) {
    CouplingReaction out;
    if (dt <= 0.0f) return out;

    const float dx = grid.get_dx();

    Matrix3 R_l2w = Matrix3::from_quaternion(body.orientation);
    Matrix3 R_w2l = R_l2w.transposed();

    // Conservative AABB of the OBB: |R|·half — used to skip blocks early.
    Vec3 aabb_h(
        std::abs(R_l2w.m[0][0]) * body.half_extents.x + std::abs(R_l2w.m[0][1]) * body.half_extents.y + std::abs(R_l2w.m[0][2]) * body.half_extents.z,
        std::abs(R_l2w.m[1][0]) * body.half_extents.x + std::abs(R_l2w.m[1][1]) * body.half_extents.y + std::abs(R_l2w.m[1][2]) * body.half_extents.z,
        std::abs(R_l2w.m[2][0]) * body.half_extents.x + std::abs(R_l2w.m[2][1]) * body.half_extents.y + std::abs(R_l2w.m[2][2]) * body.half_extents.z
    );
    Vec3 aabb_min = body.center - aabb_h;
    Vec3 aabb_max = body.center + aabb_h;

    auto& block_map = grid.get_blocks_mutable();
    for (auto& kv : block_map) {
        GridBlock& block = kv.second;

        // Block AABB (cell-centered nodes from origin_* through origin_* + BLOCK_SIZE-1).
        Vec3 b_lo((block.origin_x + 0.5f) * dx,
                  (block.origin_y + 0.5f) * dx,
                  (block.origin_z + 0.5f) * dx);
        Vec3 b_hi((block.origin_x + BLOCK_SIZE - 1 + 0.5f) * dx,
                  (block.origin_y + BLOCK_SIZE - 1 + 0.5f) * dx,
                  (block.origin_z + BLOCK_SIZE - 1 + 0.5f) * dx);
        if (b_hi.x < aabb_min.x || b_lo.x > aabb_max.x) continue;
        if (b_hi.y < aabb_min.y || b_lo.y > aabb_max.y) continue;
        if (b_hi.z < aabb_min.z || b_lo.z > aabb_max.z) continue;

        for (int li = 0; li < BLOCK_SIZE; ++li)
        for (int lj = 0; lj < BLOCK_SIZE; ++lj)
        for (int lk = 0; lk < BLOCK_SIZE; ++lk) {
            int local_idx = (li * BLOCK_SIZE * BLOCK_SIZE) + (lj * BLOCK_SIZE) + lk;
            GridNode& node = block.nodes[local_idx];
            if (!node.active || node.mass <= 1e-8f) continue;

            int gi = block.origin_x + li;
            int gj = block.origin_y + lj;
            int gk = block.origin_z + lk;
            Vec3 x_node = node_world(gi, gj, gk, dx);

            OBBProbe probe = probe_obb(x_node, body, R_w2l);
            if (!probe.inside) continue;

            // Rigid body velocity at the node position.
            Vec3 r           = x_node - body.center;
            Vec3 v_body      = body.linear_velocity + body.angular_velocity.cross(r);
            Vec3 v_node      = node.velocity;
            Vec3 n_world     = (R_l2w * probe.normal_local).normalized();

            // Decompose along outward normal.
            float vn_node = v_node.dot(n_world);
            float vn_body = v_body.dot(n_world);

            // No-penetration: clamp node velocity so it cannot push deeper
            // into the body than the body itself is moving.
            float vn_clamped = std::max(vn_node, vn_body);

            // Tangential component: blend toward body's tangential with friction.
            Vec3 vt_node = v_node - n_world * vn_node;
            Vec3 vt_body = v_body - n_world * vn_body;
            Vec3 vt_new  = vt_node + (vt_body - vt_node) * std::min(1.0f, body.friction);

            Vec3 v_after = n_world * vn_clamped + vt_new;
            Vec3 dv      = v_after - v_node;
            Vec3 dp      = dv * node.mass;   // momentum change of grid node

            // Apply BC.
            node.velocity = v_after;

            // Reaction = opposite of the impulse delivered to the grid.
            // Force = -Δp / dt, applied at the node position.
            out.force  -= dp / dt;
            out.torque -= r.cross(dp) / dt;
            ++out.nodes_inside;
        }
    }

    return out;
}

} // namespace coupling
} // namespace physics
} // namespace basements

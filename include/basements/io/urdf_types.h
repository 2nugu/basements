#pragma once
#include <string>
#include <vector>

namespace basements {
namespace io {

/**
 * @brief 3D Vector for URDF
 */
struct URDFVector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    URDFVector3() = default;
    URDFVector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Quaternion for URDF orientation
 */
struct URDFQuaternion {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;  // Identity quaternion

    URDFQuaternion() = default;
    URDFQuaternion(float x_, float y_, float z_, float w_) 
        : x(x_), y(y_), z(z_), w(w_) {}
};

/**
 * @brief Pose (position + orientation)
 */
struct URDFPose {
    URDFVector3 position;
    URDFQuaternion orientation;

    URDFPose() = default;
};

/**
 * @brief Color (RGBA)
 */
struct URDFColor {
    float r = 0.8f;
    float g = 0.8f;
    float b = 0.8f;
    float a = 1.0f;

    URDFColor() = default;
    URDFColor(float r_, float g_, float b_, float a_ = 1.0f)
        : r(r_), g(g_), b(b_), a(a_) {}
};

/**
 * @brief Geometry types
 */
struct URDFGeometry {
    enum Type {
        BOX,
        SPHERE,
        CYLINDER,
        MESH
    } type = BOX;

    // Box dimensions
    URDFVector3 size{1.0f, 1.0f, 1.0f};

    // Sphere radius
    float radius = 0.5f;

    // Cylinder
    float length = 1.0f;
    // radius is shared with sphere

    // Mesh
    std::string mesh_filename;
    URDFVector3 mesh_scale{1.0f, 1.0f, 1.0f};

    URDFGeometry() = default;
};

/**
 * @brief Material definition
 */
struct URDFMaterial {
    std::string name;
    URDFColor color;
    std::string texture_filename;

    URDFMaterial() = default;
    URDFMaterial(const std::string& name_) : name(name_) {}
};

/**
 * @brief Inertial properties
 */
struct URDFInertial {
    float mass = 1.0f;
    URDFPose origin;

    // Inertia tensor (symmetric 3x3)
    float ixx = 1.0f;
    float ixy = 0.0f;
    float ixz = 0.0f;
    float iyy = 1.0f;
    float iyz = 0.0f;
    float izz = 1.0f;

    URDFInertial() = default;
};

/**
 * @brief Visual element
 */
struct URDFVisual {
    std::string name;
    URDFPose origin;
    URDFGeometry geometry;
    std::string material_name;
    URDFMaterial material;  // Inline material (if not referencing)

    URDFVisual() = default;
};

/**
 * @brief Collision element
 */
struct URDFCollision {
    std::string name;
    URDFPose origin;
    URDFGeometry geometry;

    URDFCollision() = default;
};

/**
 * @brief Link (rigid body)
 */
struct URDFLink {
    std::string name;
    URDFInertial inertial;
    std::vector<URDFVisual> visuals;
    std::vector<URDFCollision> collisions;

    URDFLink() = default;
    URDFLink(const std::string& name_) : name(name_) {}
};

/**
 * @brief Joint types
 */
enum class URDFJointType {
    REVOLUTE,    // Hinge with limits
    CONTINUOUS,  // Hinge without limits
    PRISMATIC,   // Slider
    FIXED,       // Rigid connection
    FLOATING,    // 6-DOF
    PLANAR       // 2D motion
};

/**
 * @brief Joint definition
 */
struct URDFJoint {
    URDFJointType type = URDFJointType::FIXED;
    std::string name;
    std::string parent_link;
    std::string child_link;
    URDFPose origin;
    URDFVector3 axis{1.0f, 0.0f, 0.0f};  // Default: X-axis

    // Limits (for revolute/prismatic)
    float lower_limit = 0.0f;
    float upper_limit = 0.0f;
    float effort_limit = 0.0f;
    float velocity_limit = 0.0f;

    // Dynamics
    float damping = 0.0f;
    float friction = 0.0f;

    URDFJoint() = default;
    URDFJoint(const std::string& name_) : name(name_) {}
};

/**
 * @brief Complete robot description
 */
struct URDFRobot {
    std::string name;
    std::vector<URDFLink> links;
    std::vector<URDFJoint> joints;
    std::vector<URDFMaterial> materials;  // Global materials

    URDFRobot() = default;
    URDFRobot(const std::string& name_) : name(name_) {}

    // Helper: Find link by name
    const URDFLink* find_link(const std::string& name) const {
        for (const auto& link : links) {
            if (link.name == name) return &link;
        }
        return nullptr;
    }

    // Helper: Find joint by name
    const URDFJoint* find_joint(const std::string& name) const {
        for (const auto& joint : joints) {
            if (joint.name == name) return &joint;
        }
        return nullptr;
    }

    // Helper: Find material by name
    const URDFMaterial* find_material(const std::string& name) const {
        for (const auto& material : materials) {
            if (material.name == name) return &material;
        }
        return nullptr;
    }
};

/**
 * @brief Convert joint type to string
 */
inline const char* joint_type_to_string(URDFJointType type) {
    switch (type) {
        case URDFJointType::REVOLUTE:   return "revolute";
        case URDFJointType::CONTINUOUS: return "continuous";
        case URDFJointType::PRISMATIC:  return "prismatic";
        case URDFJointType::FIXED:      return "fixed";
        case URDFJointType::FLOATING:   return "floating";
        case URDFJointType::PLANAR:     return "planar";
        default: return "unknown";
    }
}

/**
 * @brief Convert string to joint type
 */
inline URDFJointType string_to_joint_type(const std::string& str) {
    if (str == "revolute") return URDFJointType::REVOLUTE;
    if (str == "continuous") return URDFJointType::CONTINUOUS;
    if (str == "prismatic") return URDFJointType::PRISMATIC;
    if (str == "fixed") return URDFJointType::FIXED;
    if (str == "floating") return URDFJointType::FLOATING;
    if (str == "planar") return URDFJointType::PLANAR;
    return URDFJointType::FIXED;  // Default
}

} // namespace io
} // namespace basements

#ifndef BASEMENTS_EDITOR_SCENE_GRAPH_H
#define BASEMENTS_EDITOR_SCENE_GRAPH_H

/**
 * @file scene_graph.h
 * @brief Hierarchical scene management for the editor
 * 
 * Manages the tree structure of scene objects (bodies, joints, groups).
 */

#include "basements/editor/node_id.h"
#include "basements/core/types.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/joints.h"
#include "basements/graphics/pbr_material.h"
#include "basements/graphics/mesh.h"
#include "basements/editor/core/component.h"

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace basements {
namespace editor {

using namespace basements::math;
using namespace basements::physics;
using namespace basements::dynamics;

// ============================================================================
// Scene Node Types
// ============================================================================

enum class NodeType {
    ROOT,
    GROUP,          // Organizational group (no physics)
    RIGID_BODY,     // Physics body
    JOINT,          // Constraint between bodies
    LIGHT,          // Light source (for visualization)
    CAMERA          // Editor camera bookmark
};

// ============================================================================
// Scene Node
// ============================================================================

/**
 * @brief Base class for all scene graph nodes
 */
struct SceneNode {
    NodeID id                   = INVALID_NODE_ID;
    NodeType type               = NodeType::ROOT;
    std::string name            = "Node";
    bool visible                = true;
    bool locked                 = false;  // Can't be modified
    uint32_t layer_mask         = 1;      // bitmask for layering (visibility, physics)
    std::vector<std::string> tags;        // metadata tags
    
    // Hierarchy
    NodeID parent_id            = INVALID_NODE_ID;
    std::vector<NodeID> children;
    
    // Local transform (relative to parent)
    Vec3 local_position         = Vec3::zero();
    Quaternion local_rotation   = Quaternion::identity();
    Vec3 local_scale            = Vec3(1.0f, 1.0f, 1.0f);
    
    // Physics data (only for RIGID_BODY type)
    struct PhysicsData {
        BodyType body_type      = BodyType::DYNAMIC;
        float mass              = 1.0f;
        Vec3 inertia            = Vec3(1.0f, 1.0f, 1.0f);  // Inertia tensor diagonal (Ixx, Iyy, Izz)
        float friction          = 0.5f;
        float restitution       = 0.5f;
        ShapeType shape_type    = ShapeType::BOX;
        Vec3 shape_size         = Vec3(1.0f, 1.0f, 1.0f);  // Half-extents or radius
        bool use_gravity        = true; // Per-object Gravity override
        bool override_settings  = false; // [NEW] If true, use local values instead of parent/global
    } physics;
    
    // Rendering
    std::shared_ptr<Mesh> mesh; 
    std::string mesh_path; // [NEW] For serialization
    graphics::PBRMaterial material; // [NEW] PBR Visual Material
    
    // Multi-geometry support (for URDF links with multiple collision/visual elements)
    struct CollisionElement {
        ShapeType shape = ShapeType::BOX;
        Vec3 size = Vec3(1.0f, 1.0f, 1.0f);  // Half-extents or radius
        Vec3 offset = Vec3::zero();          // Local offset from node origin
        Quaternion rotation = Quaternion::identity();  // Local rotation
    };
    std::vector<CollisionElement> collision_elements;
    
    struct VisualElement {
        Vec3 offset = Vec3::zero();
        Quaternion rotation = Quaternion::identity();
        std::string mesh_path;
        graphics::PBRMaterial material;
    };
    std::vector<VisualElement> visual_elements;
    
    // Joint data (only for JOINT type)
    struct JointData {
        JointType joint_type    = JointType::BALL_SOCKET;
        NodeID connected_body_a = INVALID_NODE_ID;
        NodeID connected_body_b = INVALID_NODE_ID;
        Vec3 anchor_a           = Vec3::zero();
        Vec3 anchor_b           = Vec3::zero();
        Vec3 axis_a             = Vec3(1, 0, 0);
        Vec3 axis_b             = Vec3(1, 0, 0);
        bool enable_limits      = false;
        float lower_limit       = 0.0f;
        float upper_limit       = 0.0f;
        bool enable_motor       = false;
        float motor_velocity    = 0.0f;
        float motor_max_force   = 0.0f;
        // Live joint coordinate (rad for HINGE, m for SLIDER) driven by the
        // Inspector slider. EditorApp::step() reads this every frame and calls
        // PhysicsWorld::set_joint_position. EDIT-mode edits are stored but
        // have no effect until Play.
        float live_position     = 0.0f;
    } joint;
    
    SceneNode() = default;
    explicit SceneNode(NodeType t, const std::string& n = "Node")
        : type(t), name(n) {}

    // Copy constructor: copies all data fields; components are NOT copied (they must be re-added)
    SceneNode(const SceneNode& other)
        : id(other.id), type(other.type), name(other.name), visible(other.visible),
          locked(other.locked), layer_mask(other.layer_mask), tags(other.tags),
          parent_id(other.parent_id), children(other.children),
          local_position(other.local_position), local_rotation(other.local_rotation),
          local_scale(other.local_scale), physics(other.physics),
          mesh(other.mesh), mesh_path(other.mesh_path), material(other.material),
          collision_elements(other.collision_elements), visual_elements(other.visual_elements),
          joint(other.joint) {}

    SceneNode& operator=(const SceneNode& other) {
        if (this != &other) {
            id = other.id; type = other.type; name = other.name;
            visible = other.visible; locked = other.locked; layer_mask = other.layer_mask;
            tags = other.tags; parent_id = other.parent_id; children = other.children;
            local_position = other.local_position; local_rotation = other.local_rotation;
            local_scale = other.local_scale; physics = other.physics;
            mesh = other.mesh; mesh_path = other.mesh_path; material = other.material;
            collision_elements = other.collision_elements; visual_elements = other.visual_elements;
            joint = other.joint;
            components.clear(); // components are not copyable; left empty after assignment
        }
        return *this;
    }

    // Component Management
    template<typename T, typename... Args>
    T& add_component(Args&&... args) {
        auto comp = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *comp;
        components.push_back(std::move(comp));
        return ref;
    }

    template<typename T>
    T* get_component() {
        for (auto& comp : components) {
            if (T* t = dynamic_cast<T*>(comp.get())) return t;
        }
        return nullptr;
    }

    std::vector<std::unique_ptr<Component>> components;
};

// ============================================================================
// Scene Graph
// ============================================================================

/**
 * @brief Manages hierarchical scene structure
 */
class SceneGraph {
public:
    SceneGraph();
    ~SceneGraph();
    
    // Node creation
    NodeID create_node(NodeType type, const std::string& name = "");
    NodeID create_rigid_body(const std::string& name, ShapeType shape = ShapeType::BOX);
    NodeID create_joint(const std::string& name, JointType type, NodeID body_a, NodeID body_b);
    NodeID create_group(const std::string& name);
    
    // Restoration (for Undo)
    void restore_node(std::unique_ptr<SceneNode> node);
    
    // Node removal
    void remove_node(NodeID id);
    void remove_node_recursive(NodeID id);
    
    // Hierarchy
    void set_parent(NodeID child_id, NodeID parent_id);
    void move_to_root(NodeID id);
    std::vector<NodeID> get_root_children() const;
    
    // Queries
    SceneNode* get_node(NodeID id);
    const SceneNode* get_node(NodeID id) const;
    NodeID find_node_by_name(const std::string& name) const;
    std::vector<NodeID> get_all_nodes() const;
    std::vector<NodeID> get_all_bodies() const;
    std::vector<NodeID> get_all_joints() const;
    
    // Transform helpers
    Vec3 get_world_position(NodeID id) const;
    Quaternion get_world_rotation(NodeID id) const;
    
    // Iteration
    template<typename Func>
    void for_each_node(Func&& func);
    
    // Clear
    void clear();
    
    // Statistics
    size_t get_node_count() const;
    size_t get_body_count() const;
    size_t get_joint_count() const;
    
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    NodeID generate_id();
};

// ============================================================================
// Scene Node Factory (Convenience Functions)
// ============================================================================

namespace factory {

/**
 * @brief Create a box rigid body node
 */
inline SceneNode make_box(const std::string& name, const Vec3& size, float mass = 1.0f) {
    SceneNode node(NodeType::RIGID_BODY, name);
    node.physics.shape_type = ShapeType::BOX;
    node.physics.shape_size = size * 0.5f;  // Half-extents
    node.physics.mass = mass;
    return node;
}

/**
 * @brief Create a sphere rigid body node
 */
inline SceneNode make_sphere(const std::string& name, float radius, float mass = 1.0f) {
    SceneNode node(NodeType::RIGID_BODY, name);
    node.physics.shape_type = ShapeType::SPHERE;
    node.physics.shape_size = Vec3(radius, radius, radius);
    node.physics.mass = mass;
    return node;
}

/**
 * @brief Create a capsule rigid body node
 */
inline SceneNode make_capsule(const std::string& name, float radius, float height, float mass = 1.0f) {
    SceneNode node(NodeType::RIGID_BODY, name);
    node.physics.shape_type = ShapeType::CAPSULE;
    node.physics.shape_size = Vec3(radius, height, radius);
    node.physics.mass = mass;
    return node;
}

/**
 * @brief Create a static ground plane
 */
inline SceneNode make_ground(const std::string& name = "Ground") {
    SceneNode node(NodeType::RIGID_BODY, name);
    node.physics.shape_type = ShapeType::BOX;
    node.physics.shape_size = Vec3(50.0f, 0.1f, 50.0f);
    node.physics.body_type = BodyType::STATIC;
    node.physics.mass = 0.0f;
    return node;
}

} // namespace factory

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_SCENE_GRAPH_H

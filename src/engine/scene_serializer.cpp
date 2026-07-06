#include "basements/engine/scene_serializer.h"
#include "basements/io/mesh_importer.h"
#include <fstream>
#include <iostream>
#include <map>

// Assuming nlohmann/json is available. If not, this file will fail to compile.
// We use the path found in _deps or standard include.
// Try standard include first.
#include <nlohmann/json.hpp>

namespace basements {
namespace editor {

using json = nlohmann::json;
using namespace basements::math;

// Helper: Vec3 / Quat to JSON
static json vec3_to_json(const Vec3& v) {
    return { v.x, v.y, v.z };
}

static Vec3 json_to_vec3(const json& j) {
    if (j.is_array() && j.size() >= 3) return Vec3(j[0], j[1], j[2]);
    return Vec3::zero();
}

static json quat_to_json(const Quaternion& q) {
    return { q.w, q.x, q.y, q.z };
}

static Quaternion json_to_quat(const json& j) {
    if (j.is_array() && j.size() >= 4) return Quaternion(j[0], j[1], j[2], j[3]);
    return Quaternion::identity();
}

bool SceneSerializer::save_scene(const SceneGraph& scene, const std::string& filepath) {
    json root;
    root["version"] = "1.0";
    root["nodes"] = json::array();

    auto root_nodes = scene.get_root_children();
    // We should serialize ALL nodes. SceneGraph::get_node works for any valid ID.
    // We can iterate via scene.get_all_bodies() + joints + others, OR expose an iterator.
    // Current SceneGraph doesn't expose full iterator cleanly, but we can access all nodes via ID range approx or better:
    // SceneGraph could/should expose "get_all_node_ids".
    // For now, let's assume we can get all bodies and joints.
    
    // Better: Helper in SceneGraph or just iterate known categories.
    // scene_graph.h has `get_all_bodies` and `get_all_joints`.
    // But what about groups or lights?
    // Let's rely on `impl_->nodes` if we could (we can't).
    
    // Workaround: We only have `get_all_bodies` and `get_all_joints` exposed efficiently?
    // Let's use recursive traversal from root_nodes if hierarchy is enabled.
    // But hierarchy is WIP. `root_nodes` contains top level.
    // If we assume flat structure (current impl), `root_nodes` has EVERYTHING.
    // Let's traverse `root_nodes`.
    
    std::vector<NodeID> queue = root_nodes;
    size_t head = 0;
    while(head < queue.size()) {
        NodeID id = queue[head++];
        const SceneNode* node = scene.get_node(id);
        if (!node) continue;
        
        json j_node;
        j_node["id"] = node->id;
        j_node["type"] = (int)node->type;
        j_node["name"] = node->name;
        j_node["parent_id"] = node->parent_id;
        
        j_node["transform"] = {
            {"position", vec3_to_json(node->local_position)},
            {"rotation", quat_to_json(node->local_rotation)},
            {"scale", vec3_to_json(node->local_scale)}
        };
        
        if (node->type == NodeType::RIGID_BODY) {
            j_node["physics"] = {
                {"body_type", (int)node->physics.body_type},
                {"mass", node->physics.mass},
                {"friction", node->physics.friction},
                {"restitution", node->physics.restitution},
                {"shape_type", (int)node->physics.shape_type},
                {"shape_size", vec3_to_json(node->physics.shape_size)},
                {"use_gravity", node->physics.use_gravity}
            };
            if (!node->mesh_path.empty()) {
                j_node["mesh_path"] = node->mesh_path;
            }
        }
        else if (node->type == NodeType::JOINT) {
            j_node["joint"] = {
                {"type", (int)node->joint.joint_type},
                {"body_a", node->joint.connected_body_a},
                {"body_b", node->joint.connected_body_b},
                {"anchor_a", vec3_to_json(node->joint.anchor_a)},
                {"anchor_b", vec3_to_json(node->joint.anchor_b)},
                {"axis_a", vec3_to_json(node->joint.axis_a)},
                {"axis_b", vec3_to_json(node->joint.axis_b)},
                {"enable_limits", node->joint.enable_limits},
                {"lower_limit", node->joint.lower_limit},
                {"upper_limit", node->joint.upper_limit}
            };
        }
        
        root["nodes"].push_back(j_node);
        
        // Add children
        for (NodeID child : node->children) {
            queue.push_back(child);
        }
    }

    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "[Serializer] Failed to open file for writing: " << filepath << std::endl;
        return false;
    }
    out << root.dump(4);
    return true;
}

bool SceneSerializer::load_scene(SceneGraph& scene, const std::string& filepath) {
    std::ifstream in(filepath);
    if (!in.is_open()) {
        std::cerr << "[Serializer] Failed to open file for reading: " << filepath << std::endl;
        return false;
    }

    json root;
    try {
        in >> root;
    } catch (const std::exception& e) {
        std::cerr << "[Serializer] JSON parse error: " << e.what() << std::endl;
        return false;
    }
    
    scene.clear();
    
    std::map<NodeID, NodeID> id_map; // FileID -> NewID
    
    // First Pass: Create Nodes
    if (root.contains("nodes") && root["nodes"].is_array()) {
        for (const auto& j_node : root["nodes"]) {
            NodeType type = (NodeType)j_node.value("type", 0);
            std::string name = j_node.value("name", "Node");
            NodeID old_id = j_node.value("id", 0);
            
            // Create Node (generates new ID)
            NodeID new_id = INVALID_NODE_ID;
            
            if (type == NodeType::RIGID_BODY) {
                // Physics params needed for creation? No, create blank then populate.
                // create_rigid_body takes shape but we can update it.
                new_id = scene.create_rigid_body(name, ShapeType::BOX);
            } else if (type == NodeType::JOINT) {
                // Joints need bodies, defer creation or create placeholder?
                // create_joint requires bodies.
                // We should handle joints in Second Pass!
                // For now, create generic node? No, SceneGraph enforces type creation via specific methods for efficiency?
                // create_node(JOINT) exists? Yes.
                new_id = scene.create_node(NodeType::JOINT, name);
            } else {
                new_id = scene.create_node(type, name);
            }
            
            if (new_id != INVALID_NODE_ID) {
                id_map[old_id] = new_id;
                SceneNode* node = scene.get_node(new_id);
                
                // Transform
                if (j_node.contains("transform")) {
                    auto jt = j_node["transform"];
                    node->local_position = json_to_vec3(jt["position"]);
                    node->local_rotation = json_to_quat(jt["rotation"]);
                    node->local_scale = json_to_vec3(jt["scale"]);
                }
                
                // Physics
                if (type == NodeType::RIGID_BODY && j_node.contains("physics")) {
                    auto jp = j_node["physics"];
                    node->physics.body_type = (BodyType)jp.value("body_type", 0);
                    node->physics.mass = jp.value("mass", 1.0f);
                    node->physics.friction = jp.value("friction", 0.5f);
                    node->physics.restitution = jp.value("restitution", 0.5f);
                    node->physics.shape_type = (ShapeType)jp.value("shape_type", 0);
                    node->physics.shape_size = json_to_vec3(jp["shape_size"]);
                    node->physics.use_gravity = jp.value("use_gravity", true);
                    
                    if (j_node.contains("mesh_path")) {
                        std::string path = j_node["mesh_path"];
                        node->mesh_path = path;
                        // Trigger Load Mesh
                        auto mesh = std::make_shared<Mesh>();
                        if (MeshImporter::load_mesh(path, *mesh)) {
                            node->mesh = mesh;
                        } else {
                            std::cerr << "[Serializer] Warning: Could not reload mesh: " << path << std::endl;
                        }
                    }
                }
                
                // Store joint data for later (bodies might not exist yet)
                // SceneNode supports storing mismatched IDs temporarily?
                // Yes, IDs are just uint64, validation happens at runtime usually.
                if (type == NodeType::JOINT && j_node.contains("joint")) {
                    auto jj = j_node["joint"];
                    // Store OLD IDs temporarily, we fix them in pass 2
                    node->joint.connected_body_a = jj.value("body_a", 0);
                    node->joint.connected_body_b = jj.value("body_b", 0);
                    node->joint.joint_type = (JointType)jj.value("type", 0);
                    node->joint.anchor_a = json_to_vec3(jj["anchor_a"]);
                    node->joint.anchor_b = json_to_vec3(jj["anchor_b"]);
                    node->joint.axis_a = json_to_vec3(jj["axis_a"]);
                    node->joint.axis_b = json_to_vec3(jj["axis_b"]);
                    node->joint.enable_limits = jj.value("enable_limits", false);
                    node->joint.lower_limit = jj.value("lower_limit", 0.0f);
                    node->joint.upper_limit = jj.value("upper_limit", 0.0f);
                }
            }
        }
    }
    
    // Second Pass: Restore Relationships
    // 1. Parent-Child
    // 2. Joint Connections
    
    // We iterate the JSON again? Or iterate created nodes?
    // Iterate created nodes is better but we need the Old->OldParent relationship.
    // Easier to iterate JSON again.
    
    if (root.contains("nodes")) {
        for (const auto& j_node : root["nodes"]) {
            NodeID old_id = j_node.value("id", 0);
            NodeID new_id = id_map[old_id];
            if (new_id == INVALID_NODE_ID) continue;
            
            // Parent
            NodeID old_parent = j_node.value("parent_id", 0);
            if (old_parent != INVALID_NODE_ID && id_map.count(old_parent)) {
                scene.set_parent(new_id, id_map[old_parent]);
            }
            
            // Joint Fixup
            NodeType type = (NodeType)j_node.value("type", 0);
            if (type == NodeType::JOINT) {
                SceneNode* node = scene.get_node(new_id);
                if (node) {
                    node->joint.connected_body_a = id_map[node->joint.connected_body_a];
                    node->joint.connected_body_b = id_map[node->joint.connected_body_b];
                }
            }
        }
    }
    
    return true;
}

} // namespace editor
} // namespace basements

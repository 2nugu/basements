#include "basements/engine/scene_graph.h"
#include <vector>
#include <unordered_map>
#include <atomic>
#include <algorithm>

namespace basements {
namespace editor {

struct SceneGraph::Impl {
    std::unordered_map<NodeID, std::unique_ptr<SceneNode>> nodes;
    std::vector<NodeID> root_nodes;
    std::atomic<NodeID> next_id{1};

    NodeID generate_id() {
        return next_id++;
    }
};

SceneGraph::SceneGraph() : impl_(std::make_unique<Impl>()) {}
SceneGraph::~SceneGraph() = default;

NodeID SceneGraph::create_node(NodeType type, const std::string& name) {
    auto node = std::make_unique<SceneNode>(type, name);
    NodeID id = impl_->generate_id();
    node->id = id;
    impl_->nodes[id] = std::move(node);
    impl_->root_nodes.push_back(id);
    return id;
}

NodeID SceneGraph::create_rigid_body(const std::string& name, ShapeType shape) {
    auto node = std::make_unique<SceneNode>(NodeType::RIGID_BODY, name);
    NodeID id = impl_->generate_id();
    node->id = id;
    node->physics.shape_type = shape;
    
    impl_->nodes[id] = std::move(node);
    impl_->root_nodes.push_back(id);
    return id;
}

NodeID SceneGraph::create_joint(const std::string& name, JointType type, NodeID body_a, NodeID body_b) {
    auto node = std::make_unique<SceneNode>(NodeType::JOINT, name);
    NodeID id = impl_->generate_id();
    node->id = id;
    node->joint.joint_type = type;
    node->joint.connected_body_a = body_a;
    node->joint.connected_body_b = body_b;
    
    impl_->nodes[id] = std::move(node);
    impl_->root_nodes.push_back(id);
    return id;
}

void SceneGraph::restore_node(std::unique_ptr<SceneNode> node) {
    if (!node) return;
    NodeID id = node->id;
    if (id == INVALID_NODE_ID) return;
    
    // Put back into map and root list
    impl_->nodes[id] = std::move(node);
    impl_->root_nodes.push_back(id);
}

void SceneGraph::remove_node(NodeID id) {
    // Basic removal (doesn't handle children parenting logic yet)
    auto it = impl_->nodes.find(id);
    if (it != impl_->nodes.end()) {
        // Remove from roots if present
        auto& roots = impl_->root_nodes;
        roots.erase(std::remove(roots.begin(), roots.end(), id), roots.end());
        
        impl_->nodes.erase(it);
    }
}

SceneNode* SceneGraph::get_node(NodeID id) {
    auto it = impl_->nodes.find(id);
    if (it != impl_->nodes.end()) {
        return it->second.get();
    }
    return nullptr;
}

const SceneNode* SceneGraph::get_node(NodeID id) const {
    auto it = impl_->nodes.find(id);
    if (it != impl_->nodes.end()) {
        return it->second.get();
    }
    return nullptr;
}

std::vector<NodeID> SceneGraph::get_root_children() const {
    return impl_->root_nodes;
}

size_t SceneGraph::get_node_count() const {
    return impl_->nodes.size();
}

void SceneGraph::set_parent(NodeID child_id, NodeID parent_id) {
    SceneNode* child = get_node(child_id);
    SceneNode* parent = get_node(parent_id);
    if (!child || !parent) return;

    // Remove from old parent's children list
    if (child->parent_id != INVALID_NODE_ID) {
        SceneNode* old_parent = get_node(child->parent_id);
        if (old_parent) {
            auto& oc = old_parent->children;
            oc.erase(std::remove(oc.begin(), oc.end(), child_id), oc.end());
        }
    } else {
        // Remove from root list
        auto& roots = impl_->root_nodes;
        roots.erase(std::remove(roots.begin(), roots.end(), child_id), roots.end());
    }

    child->parent_id = parent_id;
    parent->children.push_back(child_id);
}

void SceneGraph::move_to_root(NodeID id) {
    SceneNode* node = get_node(id);
    if (!node) return;
    if (node->parent_id == INVALID_NODE_ID) return; // Already at root

    SceneNode* old_parent = get_node(node->parent_id);
    if (old_parent) {
        auto& oc = old_parent->children;
        oc.erase(std::remove(oc.begin(), oc.end(), id), oc.end());
    }
    node->parent_id = INVALID_NODE_ID;
    impl_->root_nodes.push_back(id);
}

std::vector<NodeID> SceneGraph::get_all_nodes() const {
    std::vector<NodeID> all;
    all.reserve(impl_->nodes.size());
    for (const auto& pair : impl_->nodes) {
        all.push_back(pair.first);
    }
    return all;
}

// Stubs for complex queries
std::vector<NodeID> SceneGraph::get_all_bodies() const {
    std::vector<NodeID> bodies;
    for (const auto& pair : impl_->nodes) {
        if (pair.second->type == NodeType::RIGID_BODY) {
            bodies.push_back(pair.first);
        }
    }
    return bodies;
}

void SceneGraph::clear() {
    impl_->nodes.clear();
    impl_->root_nodes.clear();
    impl_->next_id = 1;
}

} // namespace editor
} // namespace basements

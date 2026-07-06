#include "basements/editor/core/selection_manager.h"
#include "basements/engine/scene_graph.h"
#include <limits>

namespace basements {
namespace editor {

void SelectionManager::select(NodeID id) {
    clear();
    selected_nodes_.push_back(id);
}

void SelectionManager::deselect(NodeID id) {
    auto it = std::find(selected_nodes_.begin(), selected_nodes_.end(), id);
    if (it != selected_nodes_.end()) {
        selected_nodes_.erase(it);
    }
}

void SelectionManager::toggle(NodeID id) {
    if (is_selected(id)) {
        deselect(id);
    } else {
        add_to_selection(id);
    }
}

void SelectionManager::clear() {
    selected_nodes_.clear();
}

void SelectionManager::set_selection(const std::vector<NodeID>& ids) {
    selected_nodes_ = ids;
}

bool SelectionManager::is_selected(NodeID id) const {
    return std::find(selected_nodes_.begin(), selected_nodes_.end(), id) != selected_nodes_.end();
}

void SelectionManager::add_to_selection(NodeID id) {
    if (!is_selected(id)) {
        selected_nodes_.push_back(id);
    }
}

void SelectionManager::remove_from_selection(NodeID id) {
    deselect(id);
}

SelectionManager::Bounds SelectionManager::get_selection_bounds(SceneGraph* scene_graph) const {
    Bounds bounds;
    
    if (selected_nodes_.empty() || !scene_graph) {
        return bounds;
    }
    
    bounds.min = glm::vec3(std::numeric_limits<float>::max());
    bounds.max = glm::vec3(std::numeric_limits<float>::lowest());
    
    for (NodeID id : selected_nodes_) {
        SceneNode* node = scene_graph->get_node(id);
        if (!node) continue;
        
        glm::vec3 pos(node->local_position.x, node->local_position.y, node->local_position.z);
        glm::vec3 half_size(node->physics.shape_size.x, node->physics.shape_size.y, node->physics.shape_size.z);
        
        glm::vec3 node_min = pos - half_size;
        glm::vec3 node_max = pos + half_size;
        
        bounds.min = glm::min(bounds.min, node_min);
        bounds.max = glm::max(bounds.max, node_max);
    }
    
    bounds.valid = true;
    return bounds;
}

glm::vec3 SelectionManager::get_selection_center(SceneGraph* scene_graph) const {
    if (selected_nodes_.empty() || !scene_graph) {
        return glm::vec3(0.0f);
    }
    
    glm::vec3 center(0.0f);
    int count = 0;
    
    for (NodeID id : selected_nodes_) {
        SceneNode* node = scene_graph->get_node(id);
        if (node) {
            center += glm::vec3(node->local_position.x, node->local_position.y, node->local_position.z);
            count++;
        }
    }
    
    if (count > 0) {
        center /= (float)count;
    }
    
    return center;
}

} // namespace editor
} // namespace basements

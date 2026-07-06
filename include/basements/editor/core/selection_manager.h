#pragma once
#include "basements/editor/node_id.h"
#include <vector>
#include <glm/glm.hpp>

namespace basements {
namespace editor {

class SceneGraph;

class SelectionManager {
public:
    struct Bounds {
        glm::vec3 min{0.0f};
        glm::vec3 max{0.0f};
        bool valid = false;
    };

    void select(NodeID id);
    void deselect(NodeID id);
    void toggle(NodeID id);
    void clear();
    void set_selection(const std::vector<NodeID>& ids);
    void add_to_selection(NodeID id);
    void remove_from_selection(NodeID id);

    bool is_selected(NodeID id) const;
    const std::vector<NodeID>& get_selected() const { return selected_nodes_; }
    bool has_selection() const { return !selected_nodes_.empty(); }
    size_t count() const { return selected_nodes_.size(); }
    NodeID get_primary() const { return selected_nodes_.empty() ? INVALID_NODE_ID : selected_nodes_.front(); }

    Bounds get_selection_bounds(SceneGraph* scene_graph) const;
    glm::vec3 get_selection_center(SceneGraph* scene_graph) const;

private:
    std::vector<NodeID> selected_nodes_;
};

} // namespace editor
} // namespace basements

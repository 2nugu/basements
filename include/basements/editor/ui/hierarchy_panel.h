#ifndef BASEMENTS_EDITOR_HIERARCHY_PANEL_H
#define BASEMENTS_EDITOR_HIERARCHY_PANEL_H

#include "basements/engine/scene_graph.h"
#include "basements/engine/command_history.h" // [NEW]
#include "basements/editor/core/selection_manager.h" // [NEW]

namespace basements {
namespace editor {

class CommandHistory; // Forward declaration

class HierarchyPanel {
public:
    HierarchyPanel() = default;
    ~HierarchyPanel() = default;

    /**
     * @brief Renders the Hierarchy panel
     * @param open Pointer to boolean controlling window visibility
     * @param scene_graph Pointer to the scene graph to traverse
     * @param selection Pointer to the selection manager (in/out)
     */
    void on_render(bool* open, SceneGraph* scene_graph, SelectionManager* selection, CommandHistory* history);

private:
    void draw_node(SceneNode* node, SelectionManager* selection, SceneGraph* scene_graph, CommandHistory* history);
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_HIERARCHY_PANEL_H

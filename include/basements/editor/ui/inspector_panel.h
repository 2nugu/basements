#ifndef BASEMENTS_EDITOR_INSPECTOR_PANEL_H
#define BASEMENTS_EDITOR_INSPECTOR_PANEL_H

#include "basements/engine/scene_graph.h"
#include "basements/engine/command_history.h"
#include "basements/core/math/vec3.h"

namespace basements {
namespace editor {

class SceneGraph;
class CommandHistory;

struct SimulationBodyState {
    math::Vec3 linear_velocity;
    math::Vec3 angular_velocity;
    bool is_sleeping = false;
    bool valid = false;
};

class InspectorPanel {
public:
    InspectorPanel() = default;
    ~InspectorPanel() = default;

    void on_render(bool* open, SceneNode* selected_node, SceneGraph* scene_graph,
                   CommandHistory* history, const SimulationBodyState* sim_state = nullptr);

private:
    void render_transform(SceneNode* node, SceneGraph* scene_graph, CommandHistory* history);
    void render_physics(SceneNode* node, SceneGraph* scene_graph, const SimulationBodyState* sim_state);
    void render_material(SceneNode* node);
};

} // namespace editor
} // namespace basements
#endif

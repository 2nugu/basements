#include "basements/editor/panels/script_panel.h"
#include "basements/editor/scripting/python_bridge.h"
#include "basements/editor/localization.h"
#include "imgui.h"
#include "imgui_stdlib.h" // For std::string support with InputText

namespace basements {
namespace editor {

ScriptPanel::ScriptPanel() {
    // Default example script
    script_buffer_ = R"(import basements as bs

# Get the scene
scene = bs.get_scene()

# Create a new cube
bs.log("Creating a procedural cube...")
cube_id = scene.create_rigid_body("ProceduralCube", bs.ShapeType.BOX)

# Get the node and move it
cube = scene.get_node(cube_id)
if cube:
    cube.position = bs.Vec3(0, 5, 0)
    cube.rotation = bs.Quaternion.from_euler(0, 45 * 3.14159 / 180.0, 0)
    bs.log(f"Created cube at {cube.position}")
)";
}

void ScriptPanel::on_render(bool* open) {
    if (!*open) return;

    if (ImGui::Begin(LOC("Script Editor"), open)) {
        
        // Toolbar
        if (ImGui::Button(LOC("Run Script"))) {
            PythonBridge::execute_string(script_buffer_);
        }
        ImGui::SameLine();
        if (ImGui::Button(LOC("Clear"))) {
            script_buffer_.clear();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(Use 'basements' module)");

        // Editor Area
        // We use -FLT_MIN for height to fill remaining space
        ImGui::InputTextMultiline("##source", &script_buffer_, ImVec2(-FLT_MIN, -FLT_MIN), ImGuiInputTextFlags_AllowTabInput);

    }
    ImGui::End();
}

} // namespace editor
} // namespace basements

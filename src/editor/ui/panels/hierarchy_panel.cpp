#include "basements/editor/ui/hierarchy_panel.h"
#include "basements/editor/ui/hierarchy_panel.h"
#include "basements/editor/localization.h"
#include "basements/engine/command_history.h"
#include "imgui.h"
#include <string>
#include "basements/engine/command_history.h" // [NEW]
#include "basements/graphics/obj_loader.h" // [NEW]

namespace basements {
namespace editor {

void HierarchyPanel::on_render(bool* open, SceneGraph* scene_graph, SelectionManager* selection, CommandHistory* history) {
    if (!*open) return;

    ImGui::Begin(LOC("Hierarchy"), open);

    if (scene_graph) {
        // traversal
        auto roots = scene_graph->get_root_children();
        for (NodeID id : roots) {
            SceneNode* node = scene_graph->get_node(id);
            if (node) {
                draw_node(node, selection, scene_graph, history);
            }
        }
    }

    // Context Menu for empty space (Node Creation)
    // Context Menu for empty space (Node Creation)
    if (ImGui::BeginPopupContextWindow("HierarchyContext", ImGuiPopupFlags_MouseButtonRight | ImGuiPopupFlags_NoOpenOverItems)) {
        if (ImGui::BeginMenu(LOC("Primitives"))) {
            if (ImGui::MenuItem(LOC("Cube"))) {
                if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, "Cube", ShapeType::BOX));
                else scene_graph->create_rigid_body("Cube", ShapeType::BOX);
            }
            if (ImGui::MenuItem(LOC("Sphere"))) {
                if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, "Sphere", ShapeType::SPHERE));
                else scene_graph->create_rigid_body("Sphere", ShapeType::SPHERE);
            }
            if (ImGui::MenuItem(LOC("Capsule"))) {
                if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, "Capsule", ShapeType::CAPSULE));
                else scene_graph->create_rigid_body("Capsule", ShapeType::CAPSULE);
            }
            ImGui::Separator();
            if (ImGui::MenuItem(LOC("Plane"))) {
                if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, "Plane", ShapeType::PLANE));
                else scene_graph->create_rigid_body("Plane", ShapeType::PLANE);
            }
            if (ImGui::MenuItem(LOC("Cylinder"))) {
                if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, "Cylinder", ShapeType::CYLINDER));
                else scene_graph->create_rigid_body("Cylinder", ShapeType::CYLINDER);
            }
            if (ImGui::MenuItem(LOC("Cone"))) {
                if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, "Cone", ShapeType::CONE));
                else scene_graph->create_rigid_body("Cone", ShapeType::CONE);
            }
            if (ImGui::MenuItem(LOC("Torus"))) {
                if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, "Torus", ShapeType::TORUS));
                else scene_graph->create_rigid_body("Torus", ShapeType::TORUS);
            }
            ImGui::EndMenu();
        }

        if (ImGui::MenuItem(LOC("Create Group"))) {
            if (history) history->execute_command(std::make_unique<CreateNodeCommand>(scene_graph, NodeType::GROUP, "Group"));
            else scene_graph->create_node(NodeType::GROUP, "Group");
        }
        
        ImGui::Separator();
        if (ImGui::MenuItem(LOC("Import Mesh..."))) {
            // MVP: Hardcoded path or input text popup?
            // Since we can't open file dialog easily without creating extensive UI or platform specific code,
            // we'll assume a fixed location 'assets/models/import.obj' for MVP or show logging instruction.
            // Better: Simple ImGui popup asking for path.
            ImGui::OpenPopup("ImportMeshPopup");
        }
        
        if (ImGui::BeginPopup("ImportMeshPopup")) {
             static char buf[128] = "assets/models/model.obj";
             ImGui::InputText("Path", buf, 128);
             if (ImGui::Button("Import")) {
                 // Trigger Import Logic
                 // 1. Create Body of type MESH
                 NodeID id = scene_graph->create_rigid_body("ImportedMesh", ShapeType::MESH);
                 SceneNode* node = scene_graph->get_node(id);
                 if (node) {
                     node->mesh = std::make_unique<Mesh>();
                     if (basements::editor::ObjLoader::load(buf, *node->mesh)) {
                         node->name = std::string(buf); // Set name to file path?
                     } else {
                         // Load failed, maybe create fallback or delete
                         // std::cerr << "Failed to load " << buf << std::endl;
                     }
                 }
                 ImGui::CloseCurrentPopup();
             }
             ImGui::EndPopup();
        }

        ImGui::EndPopup();
    }

    // Drop Target for Root (Empty Space)
    // We need to use a dummy item or the window itself?
    // Window drop target is tricky if items cover it.
    // Use a dummy at the end tailored to fill space?
    
    // ImGui::GetContentRegionAvail();
    ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x, 20.0f)); // Small area at bottom
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("SCENE_NODE")) {
             NodeID source_id = *(const NodeID*)payload->Data;
             if (history) {
                 // Parent to Root (-1 or INVALID usually, check `node_to_body_map` or SceneGraph constant)
                 // Assuming -1 is root based on JSON schema `parent_id` example (-1).
                 history->execute_command(std::make_unique<ReparentNodeCommand>(scene_graph, source_id, INVALID_NODE_ID));
             }
        }
        ImGui::EndDragDropTarget();
    }

    ImGui::End();
}

void HierarchyPanel::draw_node(SceneNode* node, SelectionManager* selection, SceneGraph* scene_graph, CommandHistory* history) {
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
    
    if (selection && selection->is_selected(node->id)) {
        flags |= ImGuiTreeNodeFlags_Selected;
    }

    // Leafs? For now simplified.
    // If it has children, generic logic (assuming flat for now based on scene_graph.cpp)
    // But let's pretend we might have children later.
    bool opened = ImGui::TreeNodeEx((void*)(intptr_t)node->id, flags, "%s", node->name.c_str());
    
    if (ImGui::IsItemClicked()) {
        if (selection) {
            bool shift_pressed = ImGui::GetIO().KeyShift;
            bool ctrl_pressed = ImGui::GetIO().KeyCtrl;
            
            if (shift_pressed) {
                // Add to selection
                selection->add_to_selection(node->id);
            } else if (ctrl_pressed) {
                // Toggle selection
                selection->toggle(node->id);
            } else {
                // Replace selection
                selection->select(node->id);
            }
        }
    }

    // Drag Source
    if (ImGui::BeginDragDropSource()) {
        ImGui::SetDragDropPayload("SCENE_NODE", &node->id, sizeof(NodeID));
        ImGui::Text("%s", node->name.c_str());
        ImGui::EndDragDropSource();
    }

    // Drop Target (Reparenting)
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("SCENE_NODE")) {
            NodeID source_id = *(const NodeID*)payload->Data;
            if (source_id != node->id && history) {
                 // Check for cycles? (Parenting to self or child)
                 // MVP: Just execute.
                 history->execute_command(std::make_unique<ReparentNodeCommand>(scene_graph, source_id, node->id));
            }
        }
        ImGui::EndDragDropTarget();
    }

    // Context Menu per node
    if (ImGui::BeginPopupContextItem()) {
        if (ImGui::MenuItem(LOC("Delete"))) {
            if (history) {
                 history->execute_command(std::make_unique<DeleteNodeCommand>(scene_graph, node->id));
                 if (selection) selection->deselect(node->id);
            } else {
                if (scene_graph) scene_graph->remove_node(node->id);
                if (selection) selection->deselect(node->id);
            }
        }
         ImGui::EndPopup();
    }

    if (opened) {
        // Recursive children drawing would go here
        // for (NodeID child_id : node->children) ...
        ImGui::TreePop();
    }
}

} // namespace editor
} // namespace basements

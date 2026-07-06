#include "basements/editor/panels/world_gen_panel.h"
#include "imgui.h"
#include <iostream>

namespace basements {

    WorldGenPanel::WorldGenPanel() {
        // Set some nice default values
        params.width = 100;
        params.depth = 100;
        params.scale = 1.0f;
        params.height_multiplier = 15.0f;
        params.noise_frequency = 0.02f;
        params.octaves = 5;
        params.lacunarity = 2.0f;
        params.gain = 0.5f;
    }

    void WorldGenPanel::on_render(editor::SceneGraph* scene) {
        ImGui::Begin("World Generator");

        ImGui::Text("Procedural Terrain Implementation (FastNoiseLite)");
        ImGui::Separator();

        ImGui::InputText("Node Name", name_buffer, sizeof(name_buffer));
        
        ImGui::Separator();
        ImGui::Text("Grid Settings");
        ImGui::DragInt("Width (Vertices)", &params.width, 1.0f, 2, 512);
        ImGui::DragInt("Depth (Vertices)", &params.depth, 1.0f, 2, 512);
        ImGui::DragFloat("Scale (Unit/Vertex)", &params.scale, 0.1f, 0.1f, 10.0f);

        ImGui::Separator();
        ImGui::Text("Noise Settings");
        ImGui::DragInt("Seed", &params.seed);
        ImGui::DragFloat("Height Multiplier", &params.height_multiplier, 0.5f, 0.0f, 100.0f);
        ImGui::DragFloat("Frequency", &params.noise_frequency, 0.001f, 0.0001f, 0.1f, "%.4f");
        ImGui::DragInt("Octaves", &params.octaves, 1.0f, 1, 10);
        ImGui::DragFloat("Lacunarity", &params.lacunarity, 0.1f, 1.0f, 5.0f);
        ImGui::DragFloat("Gain", &params.gain, 0.05f, 0.0f, 1.0f);

        ImGui::Separator();
        
        if (ImGui::Button("Generate Terrain", ImVec2(-1, 0))) {
            if (scene) {
                std::cout << "[WorldGenPanel] Generating terrain '" << name_buffer << "'..." << std::endl;
                
                auto mesh = WorldGenerator::generate_terrain(params);
                
                // Create a static rigid body for the terrain
                // Passing BOX as placeholder shape, will override to MESH
                auto node_id = scene->create_rigid_body(name_buffer, editor::ShapeType::BOX);
                auto* node = scene->get_node(node_id);
                
                if (node) {
                    node->mesh = mesh;
                    node->physics.shape_type = editor::ShapeType::MESH;
                    node->physics.body_type = editor::BodyType::STATIC;
                    node->physics.use_gravity = false;
                    node->physics.shape_size = glm::vec3(1.0f); // Scale handled by mesh generation, keep node scale 1
                    
                    // Center the view on it? Or just let it spawn at 0,0,0
                    node->local_position = glm::vec3(0, 0, 0);
                    
                    std::cout << "[WorldGenPanel] Terrain added to scene (Node ID: " << node_id << ")." << std::endl;
                }
            }
        }

        ImGui::End();
    }
}

#include "basements/editor/ui/physics_panel.h"
#include "basements/engine/physics_environment.h"
#include "basements/editor/utils/localization.h"
#include "basements/physics/mpm/mpm_solver.h" // MPM Solver
#include "basements/graphics/renderer.h"
#include "basements/graphics/primitive_factory.h"

#include "imgui.h"
#include <vector>

namespace basements {
namespace editor {

using namespace engine;

struct ParamEntry {
    std::string name_key;
    float* value_ptr = nullptr;
    math::Vec3* vec3_ptr = nullptr;
    float step = 0.1f;
    float min = 0.0f;
    float max = 0.0f;
    std::string unit;
    std::string category;
};

struct PhysicsPanel::Impl {
public:
    EnvironmentManager env_manager;
    ImGuiTextFilter filter;
    
    // MPM State
    std::unique_ptr<basements::mpm::MPMSolver> mpm_solver;
    bool mpm_running = false;
    bool mpm_render_particles = true;
    Mesh particle_mesh;
    bool mesh_initialized = false;
    
    Impl() {
        // Lazy init of mesh? Or init here if OpenGL context is ready.
        // Usually safe to init here if EditorApp initialized Renderer.
    }

    // For reset/presets
    void apply_preset(const PhysicsEnvironment& env) {
        env_manager.set_global(env);
    }
    
    void init_mpm() {
        mpm_solver = std::make_unique<basements::mpm::MPMSolver>();
        mpm_solver->initialize(0.1f); // Grid spacing
        
        // Sand Column
        for (float x = 0.5f; x < 1.0f; x += 0.05f) {
            for (float y = 0.5f; y < 1.0f; y += 0.05f) {
                for (float z = 0.5f; z < 1.0f; z += 0.05f) {
                    mpm_solver->add_particle(basements::math::Vec3(x, y, z), basements::math::Vec3(0, 0, 0));
                }
            }
        }
        
        if (!mesh_initialized) {
            particle_mesh = PrimitiveFactory::create_sphere(0.02f, 8, 8); // Low poly for performance
            mesh_initialized = true;
        }
    }
    
    void update_and_render_mpm() {
        if (!mpm_solver) return;
        
        if (mpm_running) {
            float dt = ImGui::GetIO().DeltaTime;
            // Clamp dt for stability
            if (dt > 0.02f) dt = 0.02f;
            mpm_solver->step(dt);
        }
        
        // Rendering
        // Note: This renders in the current ImGui window standard of rendering?
        // No, Renderer::draw_mesh likely submits to a global queue or executes GL commands.
        // If immediate GL, it might be clipped by ImGui window or draw on top.
        // Ideally we should hook into scene render, but this is a quick visualization.
        
        if (mpm_render_particles) {
            const auto& particles = mpm_solver->get_particles();
            for (const auto& p : particles) {
                glm::vec3 pos(p.position.x, p.position.y, p.position.z);
                glm::vec3 color(0.8f, 0.7f, 0.4f); // Sand color
                Renderer::draw_mesh(particle_mesh, pos, color);
            }
        }
    }

    void render_param_float(const char* label, float* v, float step = 0.0f, float min = 0.0f, float max = 0.0f, const char* format = "%.3f") {
        if (!filter.PassFilter(LOC(label))) return;
        
        ImGui::Text("%s", LOC(label));
        ImGui::SameLine(200);
        ImGui::PushItemWidth(150);
        ImGui::DragFloat(std::string("##").append(label).c_str(), v, step, min, max, format);
        ImGui::PopItemWidth();
    }
    
    void render_param_vec3(const char* label, math::Vec3* v) {
        if (!filter.PassFilter(LOC(label))) return;

        ImGui::Text("%s", LOC(label));
        ImGui::SameLine(200);
        ImGui::PushItemWidth(250);
        float p[3] = {v->x, v->y, v->z};
        if (ImGui::DragFloat3(std::string("##").append(label).c_str(), p, 0.1f)) {
            *v = math::Vec3(p[0], p[1], p[2]);
        }
        ImGui::PopItemWidth();
    }
    
    void render_bool(const char* label, bool* v) {
        if (!filter.PassFilter(LOC(label))) return;
        ImGui::Checkbox(LOC(label), v);
    }
};

PhysicsPanel::PhysicsPanel() {
    impl_ = new Impl();
}

PhysicsPanel::~PhysicsPanel() {
    delete impl_;
}

void PhysicsPanel::step(float dt) {
    if (!impl_->mpm_solver || !impl_->mpm_running) return;
    if (dt > 0.02f) dt = 0.02f;
    impl_->mpm_solver->step(dt);
}

basements::mpm::MPMSolver* PhysicsPanel::get_solver() {
    return impl_ ? impl_->mpm_solver.get() : nullptr;
}

void PhysicsPanel::render_particles() {
    if (!impl_->mpm_solver || !impl_->mpm_render_particles) return;
    const auto& particles = impl_->mpm_solver->get_particles();
    for (const auto& p : particles) {
        glm::vec3 pos(p.position.x, p.position.y, p.position.z);
        glm::vec3 color(0.8f, 0.7f, 0.4f);
        Renderer::draw_mesh(impl_->particle_mesh, pos, color);
    }
}

void PhysicsPanel::on_render(bool* open) {
    if (!*open) return;

    ImGui::SetNextWindowSize(ImVec2(400, 600), ImGuiCond_FirstUseEver); // Standardize size
    std::string window_title = std::string(LOC("Settings")) + "###Physics Settings";
    if (ImGui::Begin(window_title.c_str(), open)) {
        
        // Search
        impl_->filter.Draw(LOC("Search"), 200.0f);
        ImGui::Separator();

        // Presets
        ImGui::Text("%s", LOC("Presets"));
        if (ImGui::Button(LOC("Earth"))) impl_->apply_preset(PhysicsEnvironment::Earth()); ImGui::SameLine();
        if (ImGui::Button(LOC("Moon"))) impl_->apply_preset(PhysicsEnvironment::Moon()); ImGui::SameLine();
        if (ImGui::Button(LOC("Mars"))) impl_->apply_preset(PhysicsEnvironment::Mars()); ImGui::SameLine();
        if (ImGui::Button(LOC("Space"))) impl_->apply_preset(PhysicsEnvironment::Space());
        ImGui::Separator();

        auto& env = impl_->env_manager.global_env;

        bool searching = impl_->filter.IsActive();

        if (searching) {
            // Flattened view
            // ... (existing search logic) ...
             ImGui::TextDisabled("%s", LOC("Search Results:"));
             ImGui::Separator();
             impl_->render_param_vec3("Gravity", &env.gravity);
             // ...
        } else {
            // Standard Tab View
            if (ImGui::BeginTabBar("PhysicsSettingsTabs")) {
                
                // Tab 1: MPM (New)
                if (ImGui::BeginTabItem("MPM Sand")) {
                    ImGui::Spacing();
                    
                    if (ImGui::Button("Initialize Sand Column")) {
                        impl_->init_mpm();
                    }
                    
                    if (impl_->mpm_solver) {
                        ImGui::SameLine();
                        if (impl_->mpm_running) {
                            if (ImGui::Button("Pause")) impl_->mpm_running = false;
                        } else {
                            if (ImGui::Button("Run")) impl_->mpm_running = true;
                        }
                        
                        ImGui::Checkbox("Render Particles", &impl_->mpm_render_particles);
                        
                        auto count = impl_->mpm_solver->get_particles().size();
                        ImGui::Text("Particles: %zu", count);
                    } else {
                        ImGui::TextDisabled("Not Initialized");
                    }
                    
                    ImGui::EndTabItem();
                }

                // Tab 2: Mechanics
                if (ImGui::BeginTabItem(LOC("Mechanics"))) {
                    ImGui::Spacing();
                    impl_->render_param_vec3("Gravity", &env.gravity);
                    impl_->render_param_float("Air Density", &env.air_density, 0.01f, 0.0f, 100.0f);
                    impl_->render_param_float("Air Viscosity", &env.air_viscosity, 0.000001f, 0.0f, 1.0f, "%.6f");
                    ImGui::EndTabItem();
                }

                // ... other tabs ...
                if (ImGui::BeginTabItem(LOC("Thermodynamics"))) {
                    impl_->render_param_float("Ambient Temp", &env.ambient_temperature);
                    ImGui::EndTabItem();
                }
                 if (ImGui::BeginTabItem(LOC("Electromagnetism"))) {
                    impl_->render_param_float("Speed of Light", &env.speed_of_light);
                    ImGui::EndTabItem();
                }
                 if (ImGui::BeginTabItem(LOC("Features"))) {
                    impl_->render_bool("Enable Gravity", &env.enable_gravity);
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }
    }
    ImGui::End();
}
} // namespace editor
} // namespace basements

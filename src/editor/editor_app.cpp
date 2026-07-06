#include "basements/editor/editor_app.h"
#include "basements/editor/ui/physics_panel.h"
#include "basements/graphics/renderer.h"     // [NEW]
#include "basements/graphics/framebuffer.h"  // [NEW]
#include "basements/graphics/camera.h"       // [NEW]

#include <iostream>
#include <stdexcept>
#include <filesystem> // [NEW] For font path checking
#include <array>      // for snapshot slot storage
#include <fstream>    // snapshot JSON I/O
#include <ctime>      // auto_label timestamp
#include <sstream>    // editor_trace formatting
#include <nlohmann/json.hpp>

// Dependencies
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_internal.h" // [NEW] Required for DockBuilder
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "basements/graphics/gl_loader.h"

#include "basements/engine/scene_graph.h" // [NEW]
#include "basements/editor/ui/inspector_panel.h"
using basements::editor::SimulationBodyState;
#include "basements/editor/ui/hierarchy_panel.h" // [NEW]
#include "basements/editor/ui/console_panel.h" // [NEW]
// #include "basements/editor/panels/world_gen_panel.h" // [NEW]
#include "basements/engine/command_history.h" // [NEW]
#include "basements/editor/core/selection_manager.h" // [NEW] // [NEW]
// #include "basements/editor/scripting/python_bridge.h" // [NEW]
// #include "basements/graphics/mesh.h" // [NEW]
#include "basements/graphics/mesh.h" // [NEW]
#include "basements/engine/physics_world_gpu.h" // [NEW]
#include "basements/graphics/gizmo.h" // [NEW]
#include "basements/engine/scene_serializer.h" // [NEW]
#include "basements/io/urdf_parser.h"
#include "basements/io/urdf_converter.h"
#include "basements/physics/mpm/mpm_solver.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"
// #include "basements/editor/panels/script_panel.h" // [NEW]
#include <glm/gtx/matrix_decompose.hpp> // For decomposing transforms if needed
#include <glm/gtc/type_ptr.hpp>

namespace basements {
namespace editor {

// Helper: Screen to Ray
static Ray get_ray_from_mouse(float mouse_x, float mouse_y, float screen_w, float screen_h, const Camera& camera) {
    float x_ndc = (2.0f * mouse_x) / screen_w - 1.0f;
    float y_ndc = 1.0f - (2.0f * mouse_y) / screen_h; // OpenGL Y is up, mouse Y is down? Usually mouse Y is from top.
    
    // Check if mouse is within viewport? Assuming processed before call.
    
    glm::vec4 clip_coords(x_ndc, y_ndc, -1.0f, 1.0f);
    
    float p[16];
    camera.get_projection_matrix(p, screen_w / screen_h);
    glm::mat4 projection = glm::make_mat4(p);
    
    glm::vec4 eye_coords = glm::inverse(projection) * clip_coords;
    eye_coords = glm::vec4(eye_coords.x, eye_coords.y, -1.0f, 0.0f);
    
    float v[16];
    camera.get_view_matrix(v);
    glm::mat4 view = glm::make_mat4(v);
    
    glm::vec4 world_coords = glm::inverse(view) * eye_coords;
    glm::vec3 ray_dir = glm::normalize(glm::vec3(world_coords));
    
    glm::vec3 cam_pos;
    camera.get_position(&cam_pos.x);
    
    return Ray(Vec3(cam_pos.x, cam_pos.y, cam_pos.z), Vec3(ray_dir.x, ray_dir.y, ray_dir.z));
}


// ============================================================================
// Internal Implementation (PIMPL)
// ============================================================================

// Helpers
static Vec3 to_vec3(const glm::vec3& v) { return Vec3(v.x, v.y, v.z); }
static glm::vec3 to_glm(const Vec3& v) { return glm::vec3(v.x, v.y, v.z); }

struct EditorApp::Impl {
    EditorConfig config;
    GLFWwindow* window = nullptr;
    EditorMode mode = EditorMode::EDIT;
    
    // Rendering
    std::unique_ptr<Framebuffer> framebuffer;
    std::unique_ptr<Camera> editor_camera;
    
    // Scene
    std::unique_ptr<SceneGraph> scene_graph; // [NEW]
    std::unique_ptr<CommandHistory> command_history; // [NEW]
    std::unique_ptr<SelectionManager> selection_manager; // [NEW] Multi-Select Support

    struct NodeTransform {
        Vec3 position;
        Quaternion rotation;
        Vec3 scale;
    };
    std::unordered_map<NodeID, NodeTransform> drag_initial_transforms;

    // Full motion snapshot: transform + rigid-body kinematic state.
    // Captured at play() and (optionally) at user-driven checkpoints so that
    // a Stop→Play cycle can resume mid-simulation instead of losing velocity.
    struct NodeSnapshot {
        Vec3       position;
        Quaternion rotation;
        Vec3       scale;
        Vec3       linear_velocity   = Vec3::zero();
        Vec3       angular_velocity  = Vec3::zero();
        bool       is_sleeping       = false;
        bool       has_motion_state  = false;  // false → treat as pure pose snapshot
    };

    // Gizmo Drag State
    Vec3 drag_start_pos;
    Quaternion drag_start_rot;
    Vec3 drag_start_scale;
    
    // Box Selection State
    bool is_box_selecting = false;
    ImVec2 box_select_start;
    ImVec2 box_select_current;

    // Physics Runtime
    std::unique_ptr<engine::PhysicsWorldGPU> physics_world;
    std::unordered_map<NodeID, engine::BodyHandle> node_to_body_map;
    std::unordered_map<NodeID, NodeSnapshot> play_snapshot; // restored on Stop; optionally seeds next Play

    // User-facing capture slots (multi-slot snapshot system, JSON saveable).
    // Independent from play_snapshot — these are explicit checkpoints.
    std::array<std::unordered_map<NodeID, NodeSnapshot>, EditorApp::NUM_SNAPSHOT_SLOTS> snapshot_slots;
    std::array<std::string, EditorApp::NUM_SNAPSHOT_SLOTS>                              snapshot_labels;
    bool                                                                                open_label_editor = false;
    // SceneNode JOINT → PhysicsWorld constraint handle, populated on play().
    std::unordered_map<NodeID, engine::ConstraintHandle>                                node_to_constraint_map;
    // Last live slider value forwarded to the constraint, for derivative-based
    // velocity drive (see EditorApp::step). Indexed by JOINT node id.
    std::unordered_map<NodeID, float>                                                   last_live_position;

    // Panels
    std::unique_ptr<PhysicsPanel> physics_panel;
    std::unique_ptr<InspectorPanel> inspector_panel;
    std::unique_ptr<HierarchyPanel> hierarchy_panel; // [NEW] // [NEW]
    std::unique_ptr<ConsolePanel> console_panel; // [NEW]
    // std::unique_ptr<WorldGenPanel> world_gen_panel; // [NEW]
    // std::unique_ptr<ScriptPanel> script_panel; // [NEW]

    // State
    bool show_demo_window = false; 
    bool show_hierarchy = true;
    bool show_inspector = true;
    bool show_console = true;
    bool show_viewport = true;
    bool show_physics_panel = true; 
    bool show_world_gen = true; // [NEW] 
    bool reset_layout = true; 

    // Time
    float last_frame_time = 0.0f;
    float delta_time = 0.0f;
    
    // Viewport State
    bool viewport_focused = false;
    bool viewport_hovered = false;
    ImVec2 viewport_size = {0, 0};

    Impl(const EditorConfig& cfg) : config(cfg) {}
};

// ... (Rest of global/static/lifecycle code remains same until initialize) ...
// Logic: will use logic to find where to inject initialization.

// In EditorApp::initialize:
// impl_->physics_panel = std::make_unique<PhysicsPanel>();


// Global instance
static EditorApp* g_editor_app_instance = nullptr;

EditorApp* get_editor_app() {
    return g_editor_app_instance;
}

static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

EditorApp::EditorApp() : impl_(std::make_unique<Impl>(EditorConfig())) {
    g_editor_app_instance = this;
}

EditorApp::~EditorApp() {
    if (g_editor_app_instance == this) {
        g_editor_app_instance = nullptr;
    }
}

// ============================================================================
// Lifecycle
// ============================================================================

bool EditorApp::initialize(const EditorConfig& config) {
    impl_->config = config;
    
    // 1. Initialize GLFW
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // OpenGL 3.3 Core Profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    
    // Window creation
    impl_->window = glfwCreateWindow(
        config.window_width, 
        config.window_height, 
        config.window_title.c_str(), 
        nullptr, 
        nullptr
    );
    
    if (!impl_->window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    
    glfwSetWindowSizeLimits(impl_->window, 800, 600, GLFW_DONT_CARE, GLFW_DONT_CARE);
    glfwMakeContextCurrent(impl_->window);
    glfwSwapInterval(config.enable_vsync ? 1 : 0);

    // 2. Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking!
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
    
    // Style
    if (config.dark_theme) {
        ImGui::StyleColorsDark();
    } else {
        ImGui::StyleColorsLight();
    }

    // Font Loading (Korean Support)
    // Try multiple paths to find the font asset
    const char* font_names[] = {
        "assets/fonts/Pretendard-Regular.otf",
        "../assets/fonts/Pretendard-Regular.otf",
        "../../assets/fonts/Pretendard-Regular.otf",
        "../../../assets/fonts/Pretendard-Regular.otf",
        "c:\\Windows\\Fonts\\malgun.ttf" // System Fallback
    };

    bool font_loaded = false;
    for (const char* path : font_names) {
        if (std::filesystem::exists(path)) {
            // Load with Korean glyph ranges
            // ranges: Default + Korean + basic symbols
            ImFontConfig font_cfg;
            font_cfg.OversampleH = 2;
            font_cfg.OversampleV = 2;
            
            io.Fonts->AddFontFromFileTTF(path, 18.0f, &font_cfg, io.Fonts->GetGlyphRangesKorean());
            std::cout << "[Editor] Loaded font: " << path << std::endl;
            font_loaded = true;
            break;
        }
    }
    
    if (!font_loaded) {
        std::cerr << "[Editor] Warning: Failed to load custom font. Korean characters may not display correctly." << std::endl;
        // Fallback to default, scale it a bit
        io.Fonts->AddFontDefault(); 
    }


    
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
    
    // Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(impl_->window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    // Initialize GL Loader
    if (!init_opengl_loader()) {
        std::cerr << "Failed to initialize OpenGL loader!" << std::endl;
        return false;
    }

    // 3. Initialize Renderer
    Renderer::initialize();
    
    // 4. Initialize Python Scripting [NEW]
    // PythonBridge::initialize();
    // PythonBridge::execute_string("import basements; basements.log('🐍 Embedded Python Active!')");

    FramebufferSpecification fb_spec;
    fb_spec.width = 1280;
    fb_spec.height = 720;
    impl_->framebuffer = std::make_unique<Framebuffer>(fb_spec);
    
    impl_->editor_camera = std::make_unique<Camera>(math::Vec3(5.0f, 5.0f, 5.0f));

    // Initialize Panels
    impl_->physics_panel = std::make_unique<PhysicsPanel>();
    impl_->inspector_panel = std::make_unique<InspectorPanel>();
    impl_->hierarchy_panel = std::make_unique<HierarchyPanel>(); // [NEW] // [NEW]
    impl_->console_panel = std::make_unique<ConsolePanel>(); // [NEW]
    // impl_->world_gen_panel = std::make_unique<WorldGenPanel>(); // [NEW]
    // impl_->script_panel = std::make_unique<ScriptPanel>(); // [NEW]
    impl_->scene_graph = std::make_unique<SceneGraph>(); // [NEW]
    impl_->command_history = std::make_unique<CommandHistory>(); // [NEW]
    gizmo_manager_ = std::make_unique<GizmoManager>(); // [NEW]

    // Create default scene
    auto ground = impl_->scene_graph->create_rigid_body("Ground", ShapeType::BOX);
    auto* ground_node = impl_->scene_graph->get_node(ground);
    if(ground_node) {
        ground_node->physics.body_type = BodyType::STATIC;
        ground_node->physics.shape_size = Vec3(20.0f, 0.1f, 20.0f);
        ground_node->local_position = Vec3(0.0f, -0.1f, 0.0f);
        ground_node->physics.use_gravity = false;
    }

    auto cube = impl_->scene_graph->create_rigid_body("Cube", ShapeType::BOX);
    auto* cube_node = impl_->scene_graph->get_node(cube);
    if(cube_node) {
        cube_node->local_position = Vec3(0.0f, 2.0f, 0.0f);
    }
    
    std::cout << "✅ Editor initialized successfully." << std::endl;
    return true;
}

void EditorApp::run() {
    if (!impl_->window) return;

    while (!glfwWindowShouldClose(impl_->window)) {
        float time = (float)glfwGetTime();
        impl_->delta_time = time - impl_->last_frame_time;
        impl_->last_frame_time = time;

        // MPM step (independent of rigid body simulation mode)
        if (impl_->physics_panel) impl_->physics_panel->step(impl_->delta_time);

        // Rigid body Physics Step
        if (impl_->mode == EditorMode::SIMULATE) {
            step();
        }

        glfwPollEvents();

        // ------------------------------------------------
        // 1. Process Input (Camera)
        // ------------------------------------------------
        if (impl_->viewport_focused) {
            if (ImGui::IsKeyDown(ImGuiKey_W)) impl_->editor_camera->process_keyboard(CameraMovement::FORWARD, impl_->delta_time);
            if (ImGui::IsKeyDown(ImGuiKey_S)) impl_->editor_camera->process_keyboard(CameraMovement::BACKWARD, impl_->delta_time);
            if (ImGui::IsKeyDown(ImGuiKey_A)) impl_->editor_camera->process_keyboard(CameraMovement::LEFT, impl_->delta_time);
            if (ImGui::IsKeyDown(ImGuiKey_D)) impl_->editor_camera->process_keyboard(CameraMovement::RIGHT, impl_->delta_time);
            if (ImGui::IsKeyDown(ImGuiKey_Space)) impl_->editor_camera->position.y += impl_->delta_time * impl_->editor_camera->movement_speed;
            if (ImGui::IsKeyDown(ImGuiKey_LeftShift)) impl_->editor_camera->position.y -= impl_->delta_time * impl_->editor_camera->movement_speed;
            
            // Mouse rotation only when right mouse button is held
            if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
                impl_->editor_camera->process_mouse_movement(mouse_delta.x, -mouse_delta.y); // Y inverted for OpenGL
            }
        }
        
        // Gizmo Input Handling
        if (impl_->viewport_focused && gizmo_manager_ && impl_->scene_graph) {
            // Mode Switching
            if (!ImGui::GetIO().WantTextInput) {
                if (ImGui::IsKeyPressed(ImGuiKey_W)) gizmo_manager_->set_mode(GizmoMode::TRANSLATE);
                if (ImGui::IsKeyPressed(ImGuiKey_E)) gizmo_manager_->set_mode(GizmoMode::ROTATE);
                if (ImGui::IsKeyPressed(ImGuiKey_R)) gizmo_manager_->set_mode(GizmoMode::SCALE);
            }
            
            // Mouse Interaction
            ImVec2 mouse_pos = ImGui::GetMousePos();
            // Need to offset mouse pos by viewport position if viewport is docked/windowed
            // But we render to framebuffer, so mouse coordinates need to be relative to viewport
            // We'll use the viewport window position if available, or just global if fullscreen.
            // Simplified: Assume global mouse for now (Works if Viewport is fullscreen-ish or we map strictly).
            // Actually, we should use ImGui::GetMousePos() and subtract Viewport Window Pos.
            // But we don't have viewport pos easily accessible here (it's in render_ui).
            // Let's rely on standard Input logic or pass mouse coords via state.
            // Since `run` calls `render_ui` later, we don't know viewport rect here.
            
            // BETTER: Handle Gizmo interaction INSIDE `render_ui` when Viewport is drawn.
            // But `render_ui` is strictly ImGui.
            // We can calculate ray here if we assume full window or store viewport rect from last frame.
            // Impl stores `viewport_size` but not `viewport_pos`.
            // Let's assume input handling happens here using generic logic or we move it to `render_ui`.
            // Moving to `render_ui` is cleaner for coordinates.
            
            // However, to keep structure:
            // Let's calculate Ray assuming mouse is valid (we only check if viewport_focused).
        }
        
        // 1.5 Process Input (Shortcuts)
        // Global shortcuts checking (Ctrl+Z, Ctrl+Y, Ctrl+S)
        bool ctrl_pressed = ImGui::GetIO().KeyCtrl;
        if (ctrl_pressed && ImGui::IsKeyPressed(ImGuiKey_Z) && !ImGui::IsKeyDown(ImGuiKey_LeftShift)) {
            if (impl_->command_history) impl_->command_history->undo();
        }
        // Ctrl+Y or Ctrl+Shift+Z for Redo
        if (ctrl_pressed && (ImGui::IsKeyPressed(ImGuiKey_Y) || (ImGui::IsKeyDown(ImGuiKey_LeftShift) && ImGui::IsKeyPressed(ImGuiKey_Z)))) {
            if (impl_->command_history) impl_->command_history->redo();
        }

        // Simulation state capture/restore.
        //   F5/F6/F7/F8        → capture into slot 0/1/2/3
        //   Shift+F5..F8       → restore slot 0..3
        //   F9 / F10           → legacy single-slot alias (== slot 0)
        if (!ImGui::GetIO().WantTextInput) {
            const bool shift = ImGui::IsKeyDown(ImGuiKey_LeftShift) || ImGui::IsKeyDown(ImGuiKey_RightShift);
            ImGuiKey slot_keys[4] = { ImGuiKey_F5, ImGuiKey_F6, ImGuiKey_F7, ImGuiKey_F8 };
            for (int s = 0; s < 4; ++s) {
                if (ImGui::IsKeyPressed(slot_keys[s])) {
                    if (shift) restore_runtime_state(s);
                    else       capture_runtime_state(s);
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_F9))  capture_runtime_state(0);
            if (ImGui::IsKeyPressed(ImGuiKey_F10)) restore_runtime_state(0);
        }
        
        // Editing Shortcuts
            // Duplicate (Ctrl+D)
            if (ctrl_pressed && ImGui::IsKeyPressed(ImGuiKey_D)) {
                if (impl_->command_history && impl_->selection_manager && impl_->selection_manager->has_selection()) {
                    // Multi-Duplicate
                    // For now, let's keep it simple: Duplicate primary or iterate?
                    // Let's duplicate ALL selected nodes
                    const auto& selected = impl_->selection_manager->get_selected();
                    // We need a CompositeCommand to group them, but CommandHistory might not support it yet.
                    // Loop for MVP:
                    for (NodeID id : selected) {
                        impl_->command_history->execute_command(std::make_unique<DuplicateNodeCommand>(
                             impl_->scene_graph.get(), id));
                    }
                }
            }
            
            // Delete (Delete Key)
            if (ImGui::IsKeyPressed(ImGuiKey_Delete)) {
                 if (impl_->command_history && impl_->selection_manager && impl_->selection_manager->has_selection()) {
                     const auto& selected = impl_->selection_manager->get_selected();
                     // Copy list because deletion will modify it?
                     // DeleteNodeCommand likely removes from graph.
                     std::vector<NodeID> to_delete = selected; 
                     
                     for (NodeID id : to_delete) {
                        impl_->command_history->execute_command(std::make_unique<DeleteNodeCommand>(
                            impl_->scene_graph.get(), id));
                     }
                     impl_->selection_manager->clear();
                }
            }
            
            // Focus (F Key)
            if (ImGui::IsKeyPressed(ImGuiKey_F)) {
                if (impl_->selection_manager && impl_->selection_manager->has_selection()) {
                    glm::vec3 center = impl_->selection_manager->get_selection_center(impl_->scene_graph.get());
                    
                    // Move camera to look at center
                    glm::vec3 cam_target = center + glm::vec3(0.0f, 2.0f, 5.0f);
                    impl_->editor_camera->position = math::Vec3(cam_target.x, cam_target.y, cam_target.z);
                    impl_->editor_camera->pitch = -20.0f;
                    impl_->editor_camera->yaw = -90.0f;
                    impl_->editor_camera->update_camera_vectors();
                }
            }

        // ------------------------------------------------
        // 2. Render Scene to Framebuffer
        // ------------------------------------------------
        // 2. Render Scene to Framebuffer
        // ------------------------------------------------
        impl_->framebuffer->bind();
        Renderer::set_clear_color({0.15f, 0.15f, 0.17f}); // Dark Gray-Blue
        Renderer::clear();
        
        Renderer::begin_scene(*impl_->editor_camera);
        
        // Draw Grid
        Renderer::draw_grid(20, 1.0f);
        
        // Draw Scene Nodes
        if (impl_->scene_graph) {
            auto bodies = impl_->scene_graph->get_all_bodies();
            for (auto id : bodies) {
                auto* node = impl_->scene_graph->get_node(id);
                if (!node) continue;
                
                // Base color: type-based during simulation, neutral in edit mode
                glm::vec3 color = glm::vec3(0.8f, 0.8f, 0.8f);
                if (impl_->mode == EditorMode::SIMULATE) {
                    if (node->physics.body_type == BodyType::STATIC)
                        color = glm::vec3(0.35f, 0.35f, 0.35f);
                    else if (node->physics.body_type == BodyType::KINEMATIC)
                        color = glm::vec3(0.4f, 0.8f, 0.4f);
                    else
                        color = glm::vec3(0.4f, 0.65f, 1.0f); // dynamic = blue
                } else {
                    if (node->name == "Ground") color = glm::vec3(0.3f, 0.3f, 0.3f);
                }
                // Selection always overrides
                if (impl_->selection_manager && impl_->selection_manager->is_selected(id)) {
                    color = glm::vec3(1.0f, 0.5f, 0.0f);
                }
                
                // Convert Vec3 to glm::vec3
                glm::vec3 pos(node->local_position.x, node->local_position.y, node->local_position.z);
                glm::vec3 scale(node->physics.shape_size.x * 2.0f, node->physics.shape_size.y * 2.0f, node->physics.shape_size.z * 2.0f); // Size is full extent
                
                if (node->physics.shape_type == ShapeType::BOX) {
                    Renderer::draw_cube(pos, scale, color);
                }
                
                // Draw Mesh if available
                if (node->mesh) {
                    Renderer::draw_mesh(*node->mesh, pos, color);
                }
            }
            
            // Gizmo Rendering
            // Gizmo Rendering & Selection Bounding Box
            if (impl_->selection_manager && impl_->selection_manager->has_selection()) {
                glm::vec3 center = impl_->selection_manager->get_selection_center(impl_->scene_graph.get());
                
                // Render Gizmo at center
                if (gizmo_manager_) {
                    // For rotation, we might just use identity if multiple selected, 
                    // or use the primary selected node's rotation. 
                    // Let's use primary for now, or Identity if multiple?
                    // Blender uses "Active Element" rotation.
                    Quaternion rot = Quaternion::identity();
                    if (impl_->selection_manager->count() == 1) {
                         auto* node = impl_->scene_graph->get_node(impl_->selection_manager->get_primary());
                         if(node) rot = node->local_rotation;
                    }
                    
                    gizmo_manager_->render(to_vec3(center), rot, *impl_->editor_camera);
                }
                
                // Draw Individual AABBs
                const auto& selected = impl_->selection_manager->get_selected();
                for (NodeID id : selected) {
                    auto* node = impl_->scene_graph->get_node(id);
                    if (!node) continue;
                    
                    glm::vec3 pos(node->local_position.x, node->local_position.y, node->local_position.z);
                    glm::vec3 half_size(node->physics.shape_size.x, node->physics.shape_size.y, node->physics.shape_size.z);
                    glm::vec3 min = pos - half_size;
                    glm::vec3 max = pos + half_size;
                    Renderer::draw_aabb(min, max, glm::vec3(1.0f, 0.6f, 0.0f)); // Orange
                }
            }
        }

        // MPM Particles (rendered inside framebuffer with correct GL state)
        if (impl_->physics_panel) impl_->physics_panel->render_particles();

        Renderer::end_scene();
        impl_->framebuffer->unbind();

        // ------------------------------------------------
        // 3. Render UI
        // ------------------------------------------------
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Render DockSpace and UI
        render_dockspace();
        render_ui();
        
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(impl_->window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        
        // We don't clear default framebuffer color here because docking fills the screen, 
        // but it's good practice in case docking is undocked.
        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        // Multi-viewport
        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }
        
        glfwSwapBuffers(impl_->window);
    }

    shutdown();
}

void EditorApp::shutdown() {
    // PythonBridge::shutdown(); // [NEW]

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    if (impl_->window) {
        glfwDestroyWindow(impl_->window);
        impl_->window = nullptr;
    }
    glfwTerminate();
}

// ============================================================================
// UI & Docking
// ============================================================================

void EditorApp::render_dockspace() {
    static bool opt_fullscreen = true;
    static bool opt_padding = false;
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

    // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
    // because it would be confusing to have two docking targets within each others.
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    if (opt_fullscreen) {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    }

    if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
        window_flags |= ImGuiWindowFlags_NoBackground;

    if (!opt_padding)
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGui::Begin("DockSpace Demo", nullptr, window_flags);

    if (!opt_padding)
        ImGui::PopStyleVar();

    if (opt_fullscreen)
        ImGui::PopStyleVar(2);

    // Submit the DockSpace
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable) {
        ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

        // [NEW] Default Layout Setup
        if (impl_->reset_layout) {
            impl_->reset_layout = false;
            
            ImGui::DockBuilderRemoveNode(dockspace_id); // Clear existing layout
            ImGui::DockBuilderAddNode(dockspace_id, dockspace_flags | ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspace_id, ImGui::GetMainViewport()->Size);

            ImGuiID dock_main_id = dockspace_id;
            ImGuiID dock_id_left = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Left, 0.2f, nullptr, &dock_main_id);
            ImGuiID dock_id_right = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Right, 0.25f, nullptr, &dock_main_id);
            ImGuiID dock_id_down = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Down, 0.25f, nullptr, &dock_main_id);
            
            ImGui::DockBuilderDockWindow("Hierarchy", dock_id_left);
            ImGui::DockBuilderDockWindow("Inspector", dock_id_right);
            ImGui::DockBuilderDockWindow("Physics Settings", dock_id_right); // Dock Physics Settings with Inspector
            ImGui::DockBuilderDockWindow("Console", dock_id_down);
            ImGui::DockBuilderDockWindow("Viewport", dock_main_id);
            ImGui::DockBuilderDockWindow("ImGui Demo", dock_main_id); // Hidden by default

            ImGui::DockBuilderFinish(dockspace_id);
        }
    }

    render_main_menu();
    render_toolbar();

    ImGui::End();
}

void EditorApp::render_main_menu() {
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New Scene", "Ctrl+N")) {
                std::cerr << "[trace] menu File>New Scene\n"; std::cerr.flush();
                new_scene();
            }
            if (ImGui::MenuItem("Open Scene...", "Ctrl+O")) {
                std::cerr << "[trace] menu File>Open Scene\n"; std::cerr.flush();
                // Guarded: missing scene.json no longer crashes the editor.
                if (std::filesystem::exists("scene.json")) {
                    open_scene("scene.json");
                } else {
                    std::cerr << "[Editor] scene.json not found in CWD — "
                                 "drop a .json onto the window or pick via "
                                 "future file dialog.\n";
                }
            }
            if (ImGui::MenuItem("Save Scene", "Ctrl+S")) {
                std::cerr << "[trace] menu File>Save Scene\n"; std::cerr.flush();
                save_scene();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Undo", "Ctrl+Z")) {
                std::cerr << "[trace] menu File>Undo\n"; std::cerr.flush();
                if (impl_->command_history) impl_->command_history->undo();
            }
            if (ImGui::MenuItem("Redo", "Ctrl+Y")) {
                std::cerr << "[trace] menu File>Redo\n"; std::cerr.flush();
                if (impl_->command_history) impl_->command_history->redo();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                std::cerr << "[trace] menu File>Exit\n"; std::cerr.flush();
                glfwSetWindowShouldClose(impl_->window, true);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem("Play",  nullptr, impl_->mode == EditorMode::SIMULATE)) {
                std::cerr << "[trace] menu Simulation>Play\n"; std::cerr.flush();
                play();
            }
            if (ImGui::MenuItem("Pause", nullptr, impl_->mode == EditorMode::PAUSED)) {
                std::cerr << "[trace] menu Simulation>Pause\n"; std::cerr.flush();
                pause();
            }
            if (ImGui::MenuItem("Stop",  nullptr, impl_->mode == EditorMode::EDIT)) {
                std::cerr << "[trace] menu Simulation>Stop\n"; std::cerr.flush();
                stop();
            }
            ImGui::Separator();
            const bool can_capture =
                (impl_->mode == EditorMode::SIMULATE || impl_->mode == EditorMode::PAUSED);
            const char* slot_keys[4]    = {"F5", "F6", "F7", "F8"};
            const char* restore_keys[4] = {"Shift+F5", "Shift+F6", "Shift+F7", "Shift+F8"};
            auto slot_label = [this](int s, const char* verb) {
                static char buf[128];
                const std::string& lbl = impl_->snapshot_labels[s];
                if (lbl.empty()) std::snprintf(buf, sizeof(buf), "%s Slot %d", verb, s);
                else             std::snprintf(buf, sizeof(buf), "%s Slot %d  [%s]", verb, s, lbl.c_str());
                return buf;
            };
            for (int s = 0; s < NUM_SNAPSHOT_SLOTS; ++s) {
                if (ImGui::MenuItem(slot_label(s, "Capture"), slot_keys[s], false, can_capture))
                    capture_runtime_state(s);
            }
            ImGui::Separator();
            for (int s = 0; s < NUM_SNAPSHOT_SLOTS; ++s) {
                const bool slot_filled = !impl_->snapshot_slots[s].empty();
                if (ImGui::MenuItem(slot_label(s, "Restore"), restore_keys[s], false, slot_filled))
                    restore_runtime_state(s);
            }
            ImGui::Separator();
            for (int s = 0; s < NUM_SNAPSHOT_SLOTS; ++s) {
                const bool slot_filled = !impl_->snapshot_slots[s].empty();
                if (ImGui::MenuItem(slot_label(s, "Save"), nullptr, false, slot_filled))
                    save_snapshot(default_snapshot_path(s), s);
            }
            for (int s = 0; s < NUM_SNAPSHOT_SLOTS; ++s) {
                if (ImGui::MenuItem(slot_label(s, "Load")))
                    load_snapshot(default_snapshot_path(s), s);
            }
            ImGui::Separator();
            // Label editor runs in a dedicated modal popup so the InputText
            // widgets are not nested inside a menu (which is a fragile pattern
            // in ImGui — observed to misbehave on some Windows configs).
            if (ImGui::MenuItem("Edit Labels...")) {
                impl_->open_label_editor = true;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Window")) {
            if (ImGui::MenuItem("Reset Layout")) impl_->reset_layout = true; // [NEW]
            ImGui::Separator();
            ImGui::MenuItem("Physics Settings", nullptr, &impl_->show_physics_panel); // [NEW]
            ImGui::Separator();
            ImGui::MenuItem("Hierarchy", nullptr, &impl_->show_hierarchy);
            ImGui::MenuItem("Inspector", nullptr, &impl_->show_inspector);
            ImGui::MenuItem("Viewport", nullptr, &impl_->show_viewport);
            ImGui::MenuItem("Console", nullptr, &impl_->show_console);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &impl_->show_demo_window);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }
}

void EditorApp::render_toolbar() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 2));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemInnerSpacing, ImVec2(0, 0));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    if (ImGui::Begin("##toolbar", nullptr, flags)) {
        float size = ImGui::GetWindowHeight() - 4.0f;
        
        if (impl_->mode == EditorMode::EDIT) {
            if (ImGui::Button("Play", ImVec2(size, size))) {
                std::cerr << "[trace] toolbar Play\n"; std::cerr.flush();
                play();
            }
        } else {
            if (ImGui::Button("Stop", ImVec2(size, size))) {
                std::cerr << "[trace] toolbar Stop\n"; std::cerr.flush();
                stop();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Pause", ImVec2(size, size))) {
            std::cerr << "[trace] toolbar Pause\n"; std::cerr.flush();
            pause();
        }
        
        ImGui::SameLine();
        ImGui::Separator();
        ImGui::SameLine();
        
        // Snapping UI
        if (gizmo_manager_) {
            bool snap = gizmo_manager_->is_snap_enabled();
            if (ImGui::Checkbox("Snap", &snap)) {
                gizmo_manager_->set_snap_enabled(snap);
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            float grid = gizmo_manager_->get_snap_translation();
            if (ImGui::DragFloat("##Grid", &grid, 0.1f, 0.1f, 10.0f, "%.1fm")) {
                gizmo_manager_->set_snap_translation(grid);
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60);
            float angle = gizmo_manager_->get_snap_rotation();
            if (ImGui::DragFloat("##Angle", &angle, 1.0f, 1.0f, 90.0f, "%.0f deg")) {
                gizmo_manager_->set_snap_rotation(angle);
            }
        }
        
        ImGui::SameLine();
        ImGui::Separator();
        ImGui::SameLine();
        
        // World/Local Space Toggle [NEW]
        if (gizmo_manager_) {
            static bool is_local = false;
            if (ImGui::Button(is_local ? "Local" : "World", ImVec2(60, size))) {
                is_local = !is_local;
                // TODO: Pass to gizmo_manager when we add set_space method
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Toggle between World and Local coordinate space");
            }
        }
    }
    ImGui::End();
    
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(1);
}

void EditorApp::render_ui() {
    if (impl_->show_demo_window) ImGui::ShowDemoWindow(&impl_->show_demo_window);

    // Snapshot label editor — modal popup. Opens when the Simulation menu
    // sets impl_->open_label_editor = true; closes via the Close button or
    // by pressing Escape.
    if (impl_->open_label_editor) {
        ImGui::OpenPopup("Snapshot Slot Labels");
        impl_->open_label_editor = false;
    }
    if (ImGui::BeginPopupModal("Snapshot Slot Labels", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Set a human-readable name for each capture slot.");
        ImGui::TextDisabled("Labels appear in the Simulation menu and become part");
        ImGui::TextDisabled("of the default save path (snapshots/<label>_slot<N>.json).");
        ImGui::Separator();
        for (int s = 0; s < NUM_SNAPSHOT_SLOTS; ++s) {
            char buf[64] = {0};
            const std::string& cur = impl_->snapshot_labels[s];
            std::strncpy(buf, cur.c_str(), sizeof(buf) - 1);
            char widget_id[40];
            std::snprintf(widget_id, sizeof(widget_id), "Slot %d##label", s);
            ImGui::SetNextItemWidth(260);
            if (ImGui::InputText(widget_id, buf, sizeof(buf))) {
                impl_->snapshot_labels[s] = buf;
            }
        }
        ImGui::Separator();
        if (ImGui::Button("Close", ImVec2(120, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // Physics Panel [NEW]
    if (impl_->show_physics_panel && impl_->physics_panel) {
        impl_->physics_panel->on_render(&impl_->show_physics_panel);
    }

    // 1. Hierarchy Panel

    if (impl_->show_hierarchy && impl_->hierarchy_panel) {
        impl_->hierarchy_panel->on_render(&impl_->show_hierarchy, impl_->scene_graph.get(), impl_->selection_manager.get(), impl_->command_history.get());
    }
    
    // 2. Inspector Panel
    if (impl_->show_inspector && impl_->inspector_panel) {
        SceneNode* selected = nullptr;
        NodeID primary_id = INVALID_NODE_ID;
        if (impl_->selection_manager && impl_->selection_manager->has_selection()) {
            primary_id = impl_->selection_manager->get_primary();
            selected = impl_->scene_graph->get_node(primary_id);
        }

        SimulationBodyState sim_state;
        if (impl_->physics_world && primary_id != INVALID_NODE_ID) {
            auto it = impl_->node_to_body_map.find(primary_id);
            if (it != impl_->node_to_body_map.end() && impl_->physics_world->is_body_valid(it->second)) {
                sim_state.linear_velocity  = impl_->physics_world->get_body_linear_velocity(it->second);
                sim_state.angular_velocity = impl_->physics_world->get_body_angular_velocity(it->second);
                sim_state.is_sleeping      = impl_->physics_world->is_body_sleeping(it->second);
                sim_state.valid            = true;
            }
        }

        impl_->inspector_panel->on_render(&impl_->show_inspector, selected,
                                          impl_->scene_graph.get(), impl_->command_history.get(),
                                          sim_state.valid ? &sim_state : nullptr);
    }

    // Console Panel [NEW]
    if (impl_->show_console && impl_->console_panel) {
        impl_->console_panel->on_render(&impl_->show_console);
    }
    
    // World Gen Panel [NEW]
    // if (impl_->show_world_gen && impl_->world_gen_panel) {
    //    impl_->world_gen_panel->on_render(impl_->scene_graph.get());
    // }

    // Script Panel [NEW]
    static bool show_script_panel = true; // Use a persistent bool or add to Impl? 
    // Let's add 'show_script_panel' to Impl later for menu consistency, or just use static for now as MVP.
    // Actually, user wants "Do it well". Let's add to Impl properly.
    // But I can't edit Impl struct easily again without context shift.
    // For now, I will use a local static or just handle it if I can access impl member if I added it?
    // I haven't added `show_script_panel` boolean to Impl yet.
    // I'll add it to Impl now in the "Adding ScriptPanel member" step? Too late, simultaneous calls.
    // I'll assume I can just use a static here, OR I'll add the bool in a separate step.
    // Let's use static for this step to ensure it compiles, then I can refactor to Impl if I want menu control.
    // Wait, I want menu control.
    // I'll add `bool show_script_panel = true;` to Impl in a separate call if needed.
    // For now, let's render it if `impl_->script_panel` exists.
    // if (impl_->script_panel) {
    //     static bool s_show_script = true;
    //     if(s_show_script) impl_->script_panel->on_render(&s_show_script);
    // }

    // 3. Viewport Panel
    if (impl_->show_viewport) {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Viewport", &impl_->show_viewport);
        
        // Update viewport navigation status
        impl_->viewport_focused = ImGui::IsWindowFocused();
        impl_->viewport_hovered = ImGui::IsWindowHovered();
        
        // Calculate viewport size for frame buffer resizing (future)
        ImVec2 size = ImGui::GetContentRegionAvail();
        impl_->viewport_size = size;

        // Draw Framebuffer Texture
        ImGui::Image((ImTextureID)(intptr_t)impl_->framebuffer->get_color_attachment_id(), size, ImVec2(0, 1), ImVec2(1, 0));
        
        if (impl_->viewport_focused && gizmo_manager_) {
            // Updated condition: don't check single selected_node_id
            ImVec2 win_pos = ImGui::GetWindowPos();
            ImVec2 cursor_pos = ImGui::GetCursorScreenPos(); // Top-left of where image started
            // Actually `Image` uses cursor pos. `GetWindowPos` + TitleBar etc.
            // Let's use `ImGui::GetItemRectMin()` after Image to get top-left of image.
            ImVec2 image_min = ImGui::GetItemRectMin();
            ImVec2 mouse_pos = ImGui::GetMousePos();
            
            float mx = mouse_pos.x - image_min.x;
            float my = mouse_pos.y - image_min.y;
            
            if (mx >= 0 && my >= 0 && mx < size.x && my < size.y) {
                 // Valid mouse over viewport
                 Ray ray = get_ray_from_mouse(mx, my, size.x, size.y, *impl_->editor_camera);
                 
                 // Handle Input
                 if (impl_->scene_graph) { // Check scene graph existence
                     if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && gizmo_manager_) {
                         glm::vec3 cam_pos; impl_->editor_camera->get_position(&cam_pos.x);
                         
                         glm::vec3 center = impl_->selection_manager->get_selection_center(impl_->scene_graph.get());
                         Quaternion rot = Quaternion::identity();
                         if (impl_->selection_manager->count() == 1) {
                             auto* node = impl_->scene_graph->get_node(impl_->selection_manager->get_primary());
                             if(node) rot = node->local_rotation;
                         }

                         auto axis = gizmo_manager_->begin_drag(ray, to_vec3(cam_pos), to_vec3(center), rot);
                          
                         if (axis != GizmoAxis::NONE) {
                             impl_->drag_start_pos = to_vec3(center); 
                             impl_->drag_start_rot = rot; 
                             impl_->drag_start_scale = Vec3(1.0f);
                             
                             // Capture initial state of all selected nodes
                             impl_->drag_initial_transforms.clear();
                             const auto& selected = impl_->selection_manager->get_selected();
                             for (NodeID id : selected) {
                                 SceneNode* n = impl_->scene_graph->get_node(id);
                                 if (n) {
                                     impl_->drag_initial_transforms[id] = {n->local_position, n->local_rotation, n->local_scale};
                                 }
                             }
                         } else {
                             // Not clicking Gizmo -> Raycast for Object Selection
                             // If not dragging gizmo, maybe we are clicking an object?
                             // Implement Raycast against Scene BVH or brute force logic here?
                             // For MVP: Brute force check all bodies.
                             
                             float closest_dist = std::numeric_limits<float>::max();
                             NodeID hit_id = INVALID_NODE_ID;
                             
                             if (impl_->scene_graph) {
                                 // Simple sphere/box raycast against all physics bodies
                                 // Reuse physics world raycast if available, or visual raycast.
                                 // Visual Raycast is safer for Editor.
                                 // Let's iterate all nodes.
                                 const auto& bodies = impl_->scene_graph->get_all_bodies();
                                 for(NodeID id : bodies) {
                                     SceneNode* n = impl_->scene_graph->get_node(id);
                                     if(!n) continue;
                                     
                                     // Simple Sphere Check for selection
                                     // TODO: Proper OBB/Mesh Raycast
                                     glm::vec3 pos(n->local_position.x, n->local_position.y, n->local_position.z);
                                     float r = std::max({n->physics.shape_size.x, n->physics.shape_size.y, n->physics.shape_size.z});
                                     
                                     // Ray Sphere Intersection
                                     //... (Simplest check)
                                     // Use engine::Intersection::RaySphere?
                                     // Let's assume we have a helper or implement simple one.
                                     glm::vec3 dir(ray.direction.x, ray.direction.y, ray.direction.z);
                                     glm::vec3 origin(ray.origin.x, ray.origin.y, ray.origin.z);
                                     glm::vec3 L = pos - origin;
                                     float tca = glm::dot(L, dir);
                                     if (tca < 0) continue;
                                     float d2 = glm::dot(L, L) - tca * tca;
                                     if (d2 > r*r) continue;
                                     
                                     float thc = sqrt(r*r - d2);
                                     float t0 = tca - thc;
                                     float t1 = tca + thc;
                                     
                                     if (t0 < closest_dist) {
                                         closest_dist = t0;
                                         hit_id = id;
                                     }
                                 }
                             }
                             
                             // Apply Selection Logic
                             bool ctrl = ImGui::GetIO().KeyCtrl;
                             bool shift = ImGui::GetIO().KeyShift;
                             
                             if (hit_id != INVALID_NODE_ID) {
                                 if (ctrl || shift) {
                                     impl_->selection_manager->toggle(hit_id);
                                 } else {
                                     // If already selected, don't clear (might be starting a drag)
                                     if (!impl_->selection_manager->is_selected(hit_id)) {
                                         impl_->selection_manager->select(hit_id);
                                     }
                                     // If clicking already selected node without modifiers, keep selection (allow drag).
                                     // If we clicked a different selected node, it remains selected.
                                 }
                             } else {
                                // Background Click -> Clear Selection (unless Box Select start)
                                if (!ctrl && !shift) {
                                    impl_->selection_manager->clear();
                                }
                                // Start Box Select Logic
                                if (!ctrl && !shift) {
                                    impl_->selection_manager->clear();
                                }
                                impl_->is_box_selecting = true;
                                impl_->box_select_start = mouse_pos;
                                impl_->box_select_current = mouse_pos;
                             }
                         }
                     }
                     else if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                         if (impl_->is_box_selecting) {
                             impl_->box_select_current = mouse_pos;
                         }
                         else if (gizmo_manager_->is_dragging()) {
                            Vec3 d_pos, d_scale; Quaternion d_rot;
                            gizmo_manager_->update_drag(ray, impl_->drag_start_pos, impl_->drag_start_rot, d_pos, d_rot, d_scale);
                            
                            // Apply Delta
                            for (auto& [id, init_trans] : impl_->drag_initial_transforms) {
                                 SceneNode* n = impl_->scene_graph->get_node(id);
                                 if(!n) continue;

                                 // Position: Simply add delta
                                 n->local_position = init_trans.position + d_pos;
                                 
                                 // Rotation: Apply delta rotation
                                 // Note: For pivoting around center, we would need:
                                 // pos = center + d_rot * (init_pos - center)
                                 // But here doing local addition for simplicity as requested/planned.
                                 // Wait, d_pos handles the pivot movement!
                                 // So we just need to handle local rotation?
                                 // If we rotate the GIZMO (at center), the objects should orbit.
                                 // Orbit logic:
                                 // Vec3 rel_pos = init_trans.position - impl_->drag_start_pos; // Vector from center to obj
                                 // Vec3 rotated_rel = d_rot * rel_pos;
                                 // n->local_position = impl_->drag_start_pos + rotated_rel + d_pos; // Orbit + Translate
                                 // n->local_rotation = d_rot * init_trans.rotation;
                                 
                                 // Let's try Orbit Logic! It's much better.
                                 // d_rot is the rotation of the GIZMO from start.
                                 // impl_->drag_start_pos is the CENTER at start.
                                 
                                 Vec3 rel_pos = init_trans.position - impl_->drag_start_pos;
                                 Vec3 rotated_rel = d_rot.rotate(rel_pos); // rotated vector
                                 n->local_position = impl_->drag_start_pos + d_pos + rotated_rel;
                                 n->local_rotation = d_rot * init_trans.rotation;
                                 n->local_scale = init_trans.scale + d_scale;
                            }
                         }
                     }
                     else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                         if (impl_->is_box_selecting) {
                             impl_->is_box_selecting = false;
                             
                             // Perform Box Query
                             ImVec2 min = ImVec2(std::min(impl_->box_select_start.x, impl_->box_select_current.x), std::min(impl_->box_select_start.y, impl_->box_select_current.y));
                             ImVec2 max = ImVec2(std::max(impl_->box_select_start.x, impl_->box_select_current.x), std::max(impl_->box_select_start.y, impl_->box_select_current.y));
                             
                             // Convert to viewport-relative
                             min.x -= image_min.x; min.y -= image_min.y;
                             max.x -= image_min.x; max.y -= image_min.y;
                             
                             // Iterate and project
                             // Need to project world pos to screen pos?
                             // Yes, Project World -> Screen.
                             if (impl_->scene_graph) {
                                 const auto& bodies = impl_->scene_graph->get_all_bodies();
                                 bool shift = ImGui::GetIO().KeyShift;
                                 
                                 glm::mat4 view, proj;
                                 float v[16], p[16];
                                 impl_->editor_camera->get_view_matrix(v);
                                 impl_->editor_camera->get_projection_matrix(p, size.x/size.y);
                                 view = glm::make_mat4(v);
                                 proj = glm::make_mat4(p);
                                 glm::mat4 view_proj = proj * view;
                                 
                                 for(NodeID id : bodies) {
                                     SceneNode* n = impl_->scene_graph->get_node(id);
                                     if(!n) continue;
                                     
                                     glm::vec4 pos(n->local_position.x, n->local_position.y, n->local_position.z, 1.0f);
                                     glm::vec4 clip = view_proj * pos;
                                     if(clip.w <= 0) continue; // Behind camera
                                     
                                     // NDC -> Screen
                                     glm::vec3 ndc = glm::vec3(clip) / clip.w;
                                     float sx = (ndc.x + 1.0f) * 0.5f * size.x;
                                     float sy = (1.0f - ndc.y) * 0.5f * size.y; // Inverted Y
                                     
                                     if (sx >= min.x && sx <= max.x && sy >= min.y && sy <= max.y) {
                                         impl_->selection_manager->add_to_selection(id);
                                     }
                                 }
                             }
                         }
                         else if (gizmo_manager_->is_dragging()) {
                             gizmo_manager_->end_drag();
                             
                             // Commit Command (Group Support)
                             if (impl_->command_history && !impl_->drag_initial_transforms.empty()) {
                                 auto batch_cmd = std::make_unique<CompositeCommand>();
                                 bool changed = false;

                                 for (auto& [id, init_trans] : impl_->drag_initial_transforms) {
                                     SceneNode* n = impl_->scene_graph->get_node(id);
                                     if (!n) continue;

                                     // Check for changes
                                     if (n->local_position != init_trans.position ||
                                         n->local_rotation != init_trans.rotation ||
                                         n->local_scale != init_trans.scale) 
                                     {
                                         auto cmd = std::make_unique<ChangeTransformCommand>(
                                             impl_->scene_graph.get(), 
                                             id,
                                             init_trans.position, n->local_position,
                                             init_trans.rotation, n->local_rotation,
                                             init_trans.scale, n->local_scale
                                         );
                                         // Note: Command is already "executed" by us manually updating the node state during drag.
                                         // But Command::execute() sets it again.
                                         // CommandHistory::execute_command calls execute().
                                         // It's idempotent so it's fine.
                                         batch_cmd->add_command(std::move(cmd));
                                         changed = true;
                                     }
                                 }

                                 if (changed) {
                                     impl_->command_history->execute_command(std::move(batch_cmd));
                                 }
                             }
                             
                             impl_->drag_initial_transforms.clear();
                         }
                     }
                     
                     // Draw Box Selection
                     if (impl_->is_box_selecting) {
                         ImDrawList* draw_list = ImGui::GetWindowDrawList();
                         draw_list->AddRectFilled(impl_->box_select_start, impl_->box_select_current, IM_COL32(255, 255, 255, 30));
                         draw_list->AddRect(impl_->box_select_start, impl_->box_select_current, IM_COL32(255, 255, 255, 200));
                     }
                 }
            }
        }
        
        ImGui::End();
        ImGui::PopStyleVar();
    }

}

// ============================================================================
// Stubs
// ============================================================================
// ============================================================================
// Simulation Control
// ============================================================================


// ============================================================================
// Simulation Control
// ============================================================================
void EditorApp::set_mode(EditorMode mode) { impl_->mode = mode; }
EditorMode EditorApp::get_mode() const { return impl_->mode; }

void EditorApp::editor_trace(const std::string& msg) {
#ifdef BASEMENTS_EDITOR_TRACE
    std::cerr << "[trace] " << msg << "\n"; std::cerr.flush();
    if (impl_ && impl_->console_panel) {
        impl_->console_panel->log("[trace] " + msg);
    }
#else
    (void)msg;
#endif
}

// Streaming-style trace macro. Compiled out entirely when
// BASEMENTS_EDITOR_TRACE is not defined (Release builds without diagnostics).
#ifdef BASEMENTS_EDITOR_TRACE
#define EDIT_TRACE(...) do { \
    std::ostringstream _oss; _oss << __VA_ARGS__; \
    this->editor_trace(_oss.str()); \
} while (0)
#else
#define EDIT_TRACE(...) ((void)0)
#endif

void EditorApp::play() {
    EDIT_TRACE("play() begin");
    if (impl_->mode == EditorMode::SIMULATE) return;

    std::cout << "[Editor] Starting Simulation..." << std::endl;
    
    // 0. Save snapshot so Stop can restore positions.
    //    Existing snapshots with has_motion_state==true are preserved so a prior
    //    capture_runtime_state() can seed body velocities below.
    decltype(impl_->play_snapshot) preserved;
    for (auto& [id, snap] : impl_->play_snapshot) {
        if (snap.has_motion_state) preserved.emplace(id, snap);
    }
    impl_->play_snapshot.clear();
    for (auto id : impl_->scene_graph->get_all_nodes()) {
        auto* node = impl_->scene_graph->get_node(id);
        if (!node) continue;
        Impl::NodeSnapshot snap;
        snap.position = node->local_position;
        snap.rotation = node->local_rotation;
        snap.scale    = node->local_scale;
        auto it = preserved.find(id);
        if (it != preserved.end()) {
            snap.linear_velocity  = it->second.linear_velocity;
            snap.angular_velocity = it->second.angular_velocity;
            snap.is_sleeping      = it->second.is_sleeping;
            snap.has_motion_state = true;
        }
        impl_->play_snapshot[id] = snap;
    }

    // 1. Initialize Physics World
    engine::PhysicsWorldConfig config;
    config.gravity = math::Vec3(0.0f, -9.81f, 0.0f);
    impl_->physics_world = std::make_unique<engine::PhysicsWorldGPU>(config);
    impl_->node_to_body_map.clear();
    
    // 2. Sync Scene to Physics
    if (impl_->scene_graph) {
        auto bodies = impl_->scene_graph->get_all_bodies();
        for (auto id : bodies) {
            auto* node = impl_->scene_graph->get_node(id);
            if (!node) continue;
            
            engine::RigidBodyDesc desc;
            desc.position = node->local_position;
            desc.orientation = node->local_rotation;

            // Seed kinematic state from a previous capture_runtime_state() if any.
            auto snap_it = impl_->play_snapshot.find(id);
            if (snap_it != impl_->play_snapshot.end() && snap_it->second.has_motion_state) {
                desc.linear_velocity  = snap_it->second.linear_velocity;
                desc.angular_velocity = snap_it->second.angular_velocity;
            }

            // Map types
            if (node->physics.body_type == BodyType::DYNAMIC) desc.type = engine::BodyType::DYNAMIC;
            else if (node->physics.body_type == BodyType::STATIC) desc.type = engine::BodyType::STATIC;
            else desc.type = engine::BodyType::KINEMATIC;

            desc.mass         = node->physics.mass;
            desc.half_extents = node->physics.shape_size;
            desc.restitution  = node->physics.restitution;
            desc.friction     = node->physics.friction;

            // Box inertia tensor: I = (m/12) * diag(h²+d², w²+d², w²+h²)
            {
                float w = node->physics.shape_size.x * 2.0f;
                float h = node->physics.shape_size.y * 2.0f;
                float d = node->physics.shape_size.z * 2.0f;
                float f = desc.mass / 12.0f;
                desc.inertia_tensor = math::Matrix3(
                    f*(h*h+d*d), 0.0f,        0.0f,
                    0.0f,        f*(w*w+d*d), 0.0f,
                    0.0f,        0.0f,        f*(w*w+h*h)
                );
            }
            
            auto handle = impl_->physics_world->create_body(desc);
            impl_->node_to_body_map[id] = handle;
        }
    }

    // ---- JOINT wiring: each NodeType::JOINT becomes a PhysicsWorld constraint.
    //      Bodies referenced by the joint must already be in node_to_body_map.
    impl_->node_to_constraint_map.clear();
    if (impl_->scene_graph) {
        for (auto id : impl_->scene_graph->get_all_nodes()) {
            auto* node = impl_->scene_graph->get_node(id);
            if (!node || node->type != NodeType::JOINT) continue;

            auto ait = impl_->node_to_body_map.find(node->joint.connected_body_a);
            auto bit = impl_->node_to_body_map.find(node->joint.connected_body_b);
            if (ait == impl_->node_to_body_map.end() ||
                bit == impl_->node_to_body_map.end()) {
                std::cerr << "[Editor] joint '" << node->name
                          << "' references missing body — skipping.\n";
                continue;
            }

            engine::JointDescriptor jd;
            jd.type                      = node->joint.joint_type;
            jd.body_a                    = ait->second;
            jd.body_b                    = bit->second;
            jd.local_anchor_a            = node->joint.anchor_a;
            jd.local_anchor_b            = node->joint.anchor_b;
            jd.local_axis_a              = node->joint.axis_a;
            jd.local_axis_b              = node->joint.axis_b;
            jd.enable_limits             = node->joint.enable_limits;
            jd.lower_limit               = node->joint.lower_limit;
            jd.upper_limit               = node->joint.upper_limit;
            jd.enable_motor              = node->joint.enable_motor;
            jd.motor_target_velocity     = node->joint.motor_velocity;
            jd.motor_max_force_or_torque = node->joint.motor_max_force;

            engine::ConstraintHandle ch = impl_->physics_world->create_joint(jd);
            if (impl_->physics_world->is_joint_valid(ch)) {
                impl_->node_to_constraint_map[id] = ch;
                // Seed live-position tracking so the first Play frame doesn't
                // produce a velocity spike from 0 → current slider value.
                impl_->last_live_position[id] = node->joint.live_position;
            }
        }
    }
    std::cout << "[Editor] Wired "
              << impl_->node_to_constraint_map.size()
              << " joint constraint(s).\n";

    impl_->mode = EditorMode::SIMULATE;
}

void EditorApp::pause() {
    impl_->mode = EditorMode::PAUSED;
}

void EditorApp::stop() {
    EDIT_TRACE("stop() begin");
    if (impl_->mode == EditorMode::EDIT) return;

    std::cout << "[Editor] Stopping Simulation..." << std::endl;
    impl_->mode = EditorMode::EDIT;

    // Restore pre-play poses. Motion state is intentionally discarded on Stop
    // (Stop = "reset to authored state"); use capture_runtime_state() before
    // Stop if you want to resume from this exact frame on the next Play.
    for (auto& [id, snap] : impl_->play_snapshot) {
        auto* node = impl_->scene_graph->get_node(id);
        if (node) {
            node->local_position = snap.position;
            node->local_rotation = snap.rotation;
            node->local_scale    = snap.scale;
        }
    }
    impl_->play_snapshot.clear();

    // Clear physics world
    impl_->physics_world.reset();
    impl_->node_to_body_map.clear();
    impl_->node_to_constraint_map.clear();
    impl_->last_live_position.clear();
}


void EditorApp::capture_runtime_state(int slot, bool auto_label) {
    EDIT_TRACE("capture_runtime_state(slot=" << slot << ")");
    if (slot < 0 || slot >= NUM_SNAPSHOT_SLOTS) return;
    if (!impl_->physics_world) return;
    if (impl_->mode != EditorMode::SIMULATE && impl_->mode != EditorMode::PAUSED) return;

    if (auto_label) {
        char ts[32];
        std::time_t now = std::time(nullptr);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &now);
#else
        localtime_r(&now, &tm_buf);
#endif
        std::strftime(ts, sizeof(ts), "auto_%Y-%m-%dT%H-%M-%S", &tm_buf);
        impl_->snapshot_labels[slot] = ts;
    }

    auto& snap_map = impl_->snapshot_slots[slot];
    snap_map.clear();
    for (auto& [node_id, body_handle] : impl_->node_to_body_map) {
        auto* node = impl_->scene_graph->get_node(node_id);
        if (!node) continue;
        Impl::NodeSnapshot snap;
        snap.position         = impl_->physics_world->get_body_position(body_handle);
        snap.rotation         = impl_->physics_world->get_body_orientation(body_handle);
        snap.scale            = node->local_scale;
        snap.linear_velocity  = impl_->physics_world->get_body_linear_velocity(body_handle);
        snap.angular_velocity = impl_->physics_world->get_body_angular_velocity(body_handle);
        snap.is_sleeping      = impl_->physics_world->is_body_sleeping(body_handle);
        snap.has_motion_state = true;
        snap_map[node_id] = snap;
    }

    // Slot 0 is the "play seed" slot — also mirror into play_snapshot so that
    // Stop→Play uses these velocities (back-compat with single-slot workflow).
    if (slot == 0) {
        for (auto& [id, snap] : snap_map) {
            auto it = impl_->play_snapshot.find(id);
            if (it != impl_->play_snapshot.end()) {
                it->second.linear_velocity  = snap.linear_velocity;
                it->second.angular_velocity = snap.angular_velocity;
                it->second.is_sleeping      = snap.is_sleeping;
                it->second.has_motion_state = true;
            }
        }
    }
    std::cout << "[Editor] Captured runtime state of " << snap_map.size()
              << " nodes into slot " << slot << ".\n";
}

void EditorApp::restore_runtime_state(int slot) {
    EDIT_TRACE("restore_runtime_state(slot=" << slot << ")");
    if (slot < 0 || slot >= NUM_SNAPSHOT_SLOTS) return;
    auto& snap_map = impl_->snapshot_slots[slot];
    if (snap_map.empty()) {
        std::cout << "[Editor] Slot " << slot << " is empty.\n";
        return;
    }

    if (impl_->mode == EditorMode::SIMULATE || impl_->mode == EditorMode::PAUSED) {
        if (!impl_->physics_world) return;
        for (auto& [node_id, snap] : snap_map) {
            auto bit = impl_->node_to_body_map.find(node_id);
            if (bit == impl_->node_to_body_map.end()) continue;
            engine::BodyHandle h = bit->second;
            impl_->physics_world->set_body_position(h, snap.position);
            impl_->physics_world->set_body_orientation(h, snap.rotation);
            if (snap.has_motion_state) {
                impl_->physics_world->set_body_linear_velocity(h, snap.linear_velocity);
                impl_->physics_world->set_body_angular_velocity(h, snap.angular_velocity);
                if (!snap.is_sleeping) impl_->physics_world->wake_body(h);
            }
        }
    } else {
        for (auto& [node_id, snap] : snap_map) {
            auto* node = impl_->scene_graph->get_node(node_id);
            if (!node) continue;
            node->local_position = snap.position;
            node->local_rotation = snap.rotation;
            node->local_scale    = snap.scale;
        }
    }
    std::cout << "[Editor] Restored slot " << slot << " (" << snap_map.size() << " nodes).\n";
}

namespace {
nlohmann::json vec3_to_json(const Vec3& v) {
    return nlohmann::json::array({v.x, v.y, v.z});
}
nlohmann::json quat_to_json(const Quaternion& q) {
    return nlohmann::json::array({q.w, q.x, q.y, q.z});
}
Vec3 vec3_from_json(const nlohmann::json& j) {
    return Vec3(j.at(0).get<float>(), j.at(1).get<float>(), j.at(2).get<float>());
}
Quaternion quat_from_json(const nlohmann::json& j) {
    return Quaternion(j.at(0).get<float>(), j.at(1).get<float>(),
                      j.at(2).get<float>(), j.at(3).get<float>());
}
}

void EditorApp::set_snapshot_label(int slot, const std::string& label) {
    if (slot < 0 || slot >= NUM_SNAPSHOT_SLOTS) return;
    impl_->snapshot_labels[slot] = label;
}

const std::string& EditorApp::get_snapshot_label(int slot) const {
    static const std::string empty;
    if (slot < 0 || slot >= NUM_SNAPSHOT_SLOTS) return empty;
    return impl_->snapshot_labels[slot];
}

std::string EditorApp::default_snapshot_path(int slot) const {
    if (slot < 0 || slot >= NUM_SNAPSHOT_SLOTS) return std::string();
    const std::string& label = impl_->snapshot_labels[slot];
    std::string fname;
    if (label.empty()) {
        fname = "snapshot_slot" + std::to_string(slot) + ".json";
    } else {
        // Sanitize label for use as a filename — strip path separators.
        std::string sane = label;
        for (char& c : sane) if (c == '/' || c == '\\' || c == ':') c = '_';
        fname = sane + "_slot" + std::to_string(slot) + ".json";
    }
    return "snapshots/" + fname;
}

bool EditorApp::save_snapshot(const std::string& path, int slot) const {
    if (slot < 0 || slot >= NUM_SNAPSHOT_SLOTS) return false;
    const auto& snap_map = impl_->snapshot_slots[slot];

    // Ensure parent directory exists.
    std::filesystem::path fpath(path);
    if (fpath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(fpath.parent_path(), ec);
    }

    nlohmann::json doc;
    doc["version"] = 1;
    doc["slot"]    = slot;
    doc["label"]   = impl_->snapshot_labels[slot];
    doc["nodes"]   = nlohmann::json::array();
    for (const auto& [id, snap] : snap_map) {
        nlohmann::json n;
        n["id"]               = (uint64_t)id;
        n["position"]         = vec3_to_json(snap.position);
        n["rotation"]         = quat_to_json(snap.rotation);
        n["scale"]            = vec3_to_json(snap.scale);
        n["linear_velocity"]  = vec3_to_json(snap.linear_velocity);
        n["angular_velocity"] = vec3_to_json(snap.angular_velocity);
        n["is_sleeping"]      = snap.is_sleeping;
        n["has_motion_state"] = snap.has_motion_state;
        doc["nodes"].push_back(n);
    }
    std::ofstream f(path);
    if (!f) return false;
    f << doc.dump(2);
    std::cout << "[Editor] Saved snapshot slot " << slot << " (" << snap_map.size()
              << " nodes) → " << path << "\n";
    return true;
}

bool EditorApp::load_snapshot(const std::string& path, int slot) {
    if (slot < 0 || slot >= NUM_SNAPSHOT_SLOTS) return false;
    std::ifstream f(path);
    if (!f) return false;
    nlohmann::json doc;
    try { f >> doc; } catch (...) { return false; }
    if (!doc.contains("nodes")) return false;

    if (doc.contains("label")) impl_->snapshot_labels[slot] = doc["label"].get<std::string>();
    auto& snap_map = impl_->snapshot_slots[slot];
    snap_map.clear();
    for (const auto& n : doc["nodes"]) {
        NodeID id = (NodeID)n.at("id").get<uint64_t>();
        Impl::NodeSnapshot s;
        s.position         = vec3_from_json(n.at("position"));
        s.rotation         = quat_from_json(n.at("rotation"));
        s.scale            = vec3_from_json(n.at("scale"));
        s.linear_velocity  = vec3_from_json(n.at("linear_velocity"));
        s.angular_velocity = vec3_from_json(n.at("angular_velocity"));
        s.is_sleeping      = n.at("is_sleeping").get<bool>();
        s.has_motion_state = n.at("has_motion_state").get<bool>();
        snap_map[id] = s;
    }
    std::cout << "[Editor] Loaded snapshot slot " << slot << " (" << snap_map.size()
              << " nodes) ← " << path << "\n";
    return true;
}

void EditorApp::step() {
    if (!impl_) return;
    if (!impl_->physics_world) return;
    if (!impl_->scene_graph)   return;

    // Step Physics
    float dt = 1.0f / 60.0f; // Fixed step

    // MPM ↔ rigid Dirichlet BC: before the rigid solver advances, push each
    // dynamic body's OBB into the MPM grid (if any) as a velocity boundary,
    // and accumulate the reaction force/torque back on the body. PhysicsPanel's
    // MPM step has already run in run()'s main loop, so the grid is populated.
    // Guarded throughout — a missing body or node simply skips this slot.
    if (impl_->physics_panel) {
        basements::mpm::MPMSolver* solver = nullptr;
        try { solver = impl_->physics_panel->get_solver(); }
        catch (...) { solver = nullptr; }
        if (solver) {
            auto& grid = solver->get_grid_mutable();
            for (auto& kv : impl_->node_to_body_map) {
                const NodeID node_id = kv.first;
                const engine::BodyHandle body_handle = kv.second;
                if (!impl_->physics_world->is_body_valid(body_handle)) continue;
                if (impl_->physics_world->get_body_type(body_handle) != engine::BodyType::DYNAMIC) continue;
                auto* node = impl_->scene_graph->get_node(node_id);
                if (!node) continue;

                basements::physics::coupling::RigidColliderState col;
                col.center           = impl_->physics_world->get_body_position(body_handle);
                col.orientation      = impl_->physics_world->get_body_orientation(body_handle);
                col.half_extents     = node->physics.shape_size;
                col.linear_velocity  = impl_->physics_world->get_body_linear_velocity(body_handle);
                col.angular_velocity = impl_->physics_world->get_body_angular_velocity(body_handle);
                col.friction         = node->physics.friction;
                col.mass             = node->physics.mass;  // for PIC-Rigid (M1)

                auto r = basements::physics::coupling::MPMRigidCoupler::apply(grid, col, dt);
                if (r.nodes_inside > 0) {
                    impl_->physics_world->apply_force (body_handle, r.force);
                    impl_->physics_world->apply_torque(body_handle, r.torque);
                }
            }
        }
    }

    // Apply Inspector live_position to wired joints as a *velocity* command,
    // not a position warp: qdot = Δlive / dt. Slider stationary → qdot = 0 →
    // joint coasts under its own dynamics (gravity, motor, etc). Slider
    // dragged → derived velocity is applied via set_joint_velocity for one
    // step. Avoids the per-frame velocity-zeroing zigzag of pure position
    // warping at 60 Hz.
    for (auto& kv : impl_->node_to_constraint_map) {
        auto* node = impl_->scene_graph->get_node(kv.first);
        if (!node) continue;
        if (!impl_->physics_world->is_joint_valid(kv.second)) continue;

        float prev = impl_->last_live_position[kv.first];
        float curr = node->joint.live_position;
        float diff = curr - prev;
        if (std::abs(diff) > 1e-6f) {
            impl_->physics_world->set_joint_velocity(kv.second, diff / dt);
            impl_->last_live_position[kv.first] = curr;
        }
    }

    impl_->physics_world->step(dt);
    
    // Sync Physics to Scene
    for (auto& [node_id, body_handle] : impl_->node_to_body_map) {
        auto* node = impl_->scene_graph->get_node(node_id);
        if (!node) continue;
        
        // Only update dynamic bodies
        if (node->physics.body_type == BodyType::DYNAMIC) {
            node->local_position = impl_->physics_world->get_body_position(body_handle);
            node->local_rotation = impl_->physics_world->get_body_orientation(body_handle);
        }
    }
}
bool EditorApp::new_scene() {
    stop(); // Stop simulation
    if (impl_->scene_graph) {
        impl_->scene_graph->clear();
        // Create default ground?
        auto ground = impl_->scene_graph->create_rigid_body("Ground", ShapeType::BOX);
        auto* ground_node = impl_->scene_graph->get_node(ground);
        if(ground_node) {
            ground_node->physics.body_type = BodyType::STATIC;
            ground_node->physics.shape_size = Vec3(20.0f, 0.1f, 20.0f);
            ground_node->local_position = Vec3(0.0f, -0.1f, 0.0f);
            ground_node->physics.use_gravity = false;
        }
        if (impl_->selection_manager) impl_->selection_manager->clear();
    }
    return true; 
}

bool EditorApp::open_scene(const std::string& path) {
    stop();
    if (impl_->scene_graph) {
        if (impl_->selection_manager) impl_->selection_manager->clear();
        // If path is empty, open dialog? For MVP just hardcode or return false if empty.
        // If called from Menu without path, we might want "scene.json" default or dialog.
        // Assuming path provided.
        std::string load_path = path.empty() ? "scene.json" : path;
        
        std::cout << "[Editor] Loading scene from " << load_path << "..." << std::endl;
        return SceneSerializer::load_scene(*impl_->scene_graph, load_path);
    }
    return false;
}

bool EditorApp::save_scene() {
    // Save to current path or default
    return save_scene_as("scene.json");
}

bool EditorApp::save_scene_as(const std::string& path) {
    if (impl_->scene_graph) {
        std::cout << "[Editor] Saving scene to " << path << "..." << std::endl;
        return SceneSerializer::save_scene(*impl_->scene_graph, path);
    }
    return false;
}
void EditorApp::on_drop_callback(GLFWwindow* /*window*/, int count, const char** paths) {
    std::cerr << "[trace] on_drop_callback count=" << count << "\n"; std::cerr.flush();
    if (!g_editor_app_instance) return;
    for (int i = 0; i < count; ++i) {
        if (!paths[i]) continue;
        try {
            g_editor_app_instance->import_file(paths[i]);
        } catch (const std::exception& e) {
            std::cerr << "[Editor] drop import threw: " << e.what() << "\n";
        }
    }
}

bool EditorApp::import_file(const std::string& path) {
    EDIT_TRACE("import_file: " << path);
    // Dispatch by extension. URDF goes through the parser → SceneGraph converter
    // so link nodes appear in the hierarchy; mesh files fall through to the
    // OBJ/STL/glTF importer.
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });

    if (ext == ".urdf") {
        basements::io::URDFParser parser;
        auto result = parser.parse_file(path);
        if (!result.success) {
            std::cerr << "URDF parse failed: " << result.error_message << std::endl;
            return false;
        }
        NodeID root_id = INVALID_NODE_ID;
        std::string err;
        if (!basements::io::URDFConverter::to_scene_graph(
                result.robot, *impl_->scene_graph, root_id, err)) {
            std::cerr << "URDF→Scene conversion failed: " << err << std::endl;
            return false;
        }
        std::cout << "[Editor] Imported URDF '" << result.robot.name << "': "
                  << result.robot.links.size() << " links, "
                  << result.robot.joints.size() << " joints "
                  << "(joint constraints currently informational only — "
                     "Play simulates link bodies as independent rigid bodies; "
                     "see URDFPhysicsBridge for full constraint wiring)\n";
        return true;
    }

    auto mesh = std::make_shared<basements::editor::Mesh>();
    if (MeshImporter::load_mesh(path, *mesh)) {
        // Create Node
        std::string name = std::filesystem::path(path).stem().string();
        auto id = impl_->scene_graph->create_rigid_body(name, ShapeType::BOX);
        auto* node = impl_->scene_graph->get_node(id);
        if (node) {
            node->mesh = mesh;
            node->mesh_path = path; // [NEW]
            node->physics.body_type = BodyType::STATIC;
            node->physics.shape_type = ShapeType::MESH;
        }
        std::cout << "Imported mesh: " << path << std::endl;
        return true;
    }
    std::cerr << "Failed to import mesh: " << path << std::endl;
    return false;
}
bool EditorApp::export_scene(const std::string&, const std::string&) { return true; }

SceneGraph& EditorApp::get_scene() { 
    if (!impl_->scene_graph) throw std::runtime_error("SceneGraph not initialized");
    return *impl_->scene_graph; 
}

SelectionManager& EditorApp::get_selection() { throw std::runtime_error("Not impl"); }
CommandHistory& EditorApp::get_command_history() { throw std::runtime_error("Not impl"); }
void EditorApp::set_update_callback(UpdateCallback) {}

} // namespace editor
} // namespace basements

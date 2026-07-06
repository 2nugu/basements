#ifndef BASEMENTS_EDITOR_APP_H
#define BASEMENTS_EDITOR_APP_H

#ifdef _WIN32
#define NOMINMAX
#endif

/**
 * @file editor_app.h
 * @brief Main application class for Basements Scene Editor
 * 
 * ImGui-based visual editor for designing physics scenes.
 * 
 * Features:
 *   - Dockable panels (Hierarchy, Inspector, Viewport, Console)
 *   - Real-time physics simulation (Play/Pause/Step)
 *   - Multiple file format support (bscene, URDF, OBJ, glTF)
 *   - Undo/Redo command history
 * 
 * @see implementation_plan.md for full roadmap
 */

#include <memory>
#include <string>
#include <functional>

struct GLFWwindow; // Global forward declaration

namespace basements {
namespace editor {

// Forward declarations
class Window;
class Renderer;
class ImGuiLayer;
class SceneGraph;
class SelectionManager;
class CommandHistory;
class GizmoManager; // [NEW]

// Panel forward declarations
class HierarchyPanel;
class InspectorPanel;
class ViewportPanel;
class ConsolePanel;

// ============================================================================
// Editor Configuration
// ============================================================================

struct EditorConfig {
    std::string window_title    = "Basements Scene Editor";
    int window_width            = 1920;
    int window_height           = 1080;
    bool enable_vsync           = true;
    bool dark_theme             = true;
    float ui_scale              = 1.0f;
    std::string last_project_path;
};

// ============================================================================
// Editor State
// ============================================================================

enum class EditorMode {
    EDIT,       // Scene editing mode
    SIMULATE,   // Physics simulation running
    PAUSED      // Simulation paused
};

enum class GizmoOperation {
    NONE,
    TRANSLATE,
    ROTATE,
    SCALE
};

// ============================================================================
// Editor Application
// ============================================================================

/**
 * @brief Main editor application
 * 
 * Usage:
 * @code
 *   EditorApp app;
 *   app.initialize();
 *   app.run();
 * @endcode
 */
class EditorApp {
public:
    EditorApp();
    ~EditorApp();
    
    // Lifecycle
    bool initialize(const EditorConfig& config = EditorConfig());
    void run();
    void shutdown();
    
    // Mode control
    void set_mode(EditorMode mode);
    EditorMode get_mode() const;
    
    // Simulation control
    void play();
    void pause();
    void stop();
    void step();

    // Capture the current PhysicsWorld kinematic state (linear/angular velocity,
    // sleep state) into snapshot slot @p slot (0..NUM_SNAPSHOT_SLOTS-1).
    // If @p auto_label is true, replaces the slot's label with an
    // "auto_<ISO8601>" timestamp so the user can see when each slot was taken
    // without having to manage labels manually.
    // The next play() also reads snapshot slot 0 as seed (back-compat path).
    // No-op if not currently in SIMULATE/PAUSED mode or slot is out of range.
    void capture_runtime_state(int slot = 0, bool auto_label = false);

    // Push slot @p slot back into the world. In SIMULATE/PAUSED, this directly
    // writes body position/orientation/velocity. In EDIT, only node transforms
    // are restored (no PhysicsWorld). No-op if the slot is empty.
    void restore_runtime_state(int slot = 0);

    // Persist / load snapshot slot @p slot as JSON. Returns false on I/O error.
    // Slots are independent: save_snapshot("a.json", 0) and (".../b.json", 1)
    // produce separate files.
    bool save_snapshot(const std::string& path, int slot = 0) const;
    bool load_snapshot(const std::string& path, int slot = 0);

    // User-supplied label for a slot ("pre_explosion", "near_goal", etc.).
    // Stored in the JSON and shown in the Simulation menu. Empty = anonymous.
    void               set_snapshot_label(int slot, const std::string& label);
    const std::string& get_snapshot_label(int slot) const;

    // Default disk path used by the Simulation → Save / Load Slot menu items.
    // Pattern: snapshots/<label>_slot<N>.json (or snapshot_slot<N>.json when
    // the slot has no label). Created relative to the working directory.
    std::string default_snapshot_path(int slot) const;

    static constexpr int NUM_SNAPSHOT_SLOTS = 4;

    // Diagnostic — write a "[trace] msg" line to both stderr (captured by
    // run_editor_console.bat into editor_log.txt) and the Console Panel
    // (visible inside the GUI without a console window).
    void editor_trace(const std::string& msg);
    
    // File operations
    bool new_scene();
    bool open_scene(const std::string& path);
    bool save_scene();
    bool save_scene_as(const std::string& path);
    bool import_file(const std::string& path);  // URDF, OBJ, STL, glTF
    bool export_scene(const std::string& path, const std::string& format);
    
    // Accessors
    SceneGraph& get_scene();
    SelectionManager& get_selection();
    CommandHistory& get_command_history();
    
    // Event callbacks
    using UpdateCallback = std::function<void(float delta_time)>;
    void set_update_callback(UpdateCallback callback);
    
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    // Internal update loop
    void update(float delta_time);
    void render();
    void render_ui();
    
    // Panel rendering
    // Panel rendering
    void render_main_menu();
    void render_toolbar();
    void render_dockspace();

    // Internal Callbacks
    static void on_drop_callback(GLFWwindow* window, int count, const char** paths);
    
    // Gizmo
    std::unique_ptr<GizmoManager> gizmo_manager_; // [NEW]
};

// ============================================================================
// Global Editor Access (Optional)
// ============================================================================

/**
 * @brief Get the global editor instance (if running)
 */
EditorApp* get_editor_app();

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_APP_H

#ifndef BASEMENTS_EDITOR_WORKSPACE_H
#define BASEMENTS_EDITOR_WORKSPACE_H

#include <string>
#include <memory>
#include <vector>
#include <filesystem>
#include <map>
#include "basements/engine/scene_graph.h"
#include "basements/editor/core/component.h"

namespace basements {
namespace editor {

/**
 * @brief Project Metadata
 */
struct ProjectMetadata {
    std::string name;
    std::filesystem::path path;
    std::string version = "1.0.0";
    std::string author;
    std::string description;
};

/**
 * @brief Simulation Settings
 */
struct SimulationSettings {
    float time_step = 0.016f;
    int sub_steps = 1;
    float gravity = -9.81f;
    bool enable_gpu_acceleration = true;
};

/**
 * @brief Workspace handles the entire project context.
 * 
 * Instead of just a single scene, a workspace manages:
 *   - The scene graph
 *   - Project-wide settings
 *   - Assets and metadata
 *   - Real-time and offline simulation configurations
 */
class Workspace {
public:
    Workspace(const std::string& name = "Untitled Project");
    ~Workspace();

    // Lifecycle
    static std::unique_ptr<Workspace> create_new(const std::string& name, const std::filesystem::path& path);
    static std::unique_ptr<Workspace> load(const std::filesystem::path& path);
    bool save();
    bool save_as(const std::filesystem::path& new_path);

    // Accessors
    SceneGraph& get_scene() { return *scene_graph_; }
    ProjectMetadata& get_metadata() { return metadata_; }
    SimulationSettings& get_settings() { return settings_; }

    // Layer Management
    void set_layer_name(int layer, const std::string& name) { layer_names_[layer] = name; }
    std::string get_layer_name(int layer) const { 
        auto it = layer_names_.find(layer);
        return (it != layer_names_.end()) ? it->second : "Layer " + std::to_string(layer);
    }
    const std::map<int, std::string>& get_all_layers() const { return layer_names_; }

    // Global Parameters (For Decomposition/Setup)
    void set_parameter(const std::string& name, const Property& prop) { global_parameters_[name] = prop; }
    const std::map<std::string, Property>& get_all_parameters() const { return global_parameters_; }

    // State
    bool is_dirty() const { return dirty_; }
    void mark_dirty(bool dirty = true) { dirty_ = dirty; }

private:
    ProjectMetadata metadata_;
    SimulationSettings settings_;
    std::unique_ptr<SceneGraph> scene_graph_;
    std::map<int, std::string> layer_names_;
    std::map<std::string, Property> global_parameters_;
    bool dirty_ = false;
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_WORKSPACE_H

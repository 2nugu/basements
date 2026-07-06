#include "basements/engine/workspace.h"
#include <iostream>

namespace basements {
namespace editor {

Workspace::Workspace(const std::string& name) {
    metadata_.name = name;
    scene_graph_ = std::make_unique<SceneGraph>();
}

Workspace::~Workspace() = default;

std::unique_ptr<Workspace> Workspace::create_new(const std::string& name, const std::filesystem::path& path) {
    auto ws = std::make_unique<Workspace>(name);
    ws->metadata_.path = path;
    
    // Create directory if it doesn't exist
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directories(path);
    }
    
    std::cout << "[Workspace] Created new project: " << name << " at " << path << std::endl;
    return ws;
}

std::unique_ptr<Workspace> Workspace::load(const std::filesystem::path& path) {
    // Basic implementation: Just create a workspace with the directory name for now
    std::string name = path.filename().string();
    auto ws = std::make_unique<Workspace>(name);
    ws->metadata_.path = path;
    
    // TODO: Actually load from a project file (e.g., .bproj)
    
    std::cout << "[Workspace] Loaded project: " << name << " from " << path << std::endl;
    return ws;
}

bool Workspace::save() {
    if (metadata_.path.empty()) return false;
    
    // TODO: Serialize metadata and settings to a project file
    // SceneGraph is saved separately or as part of the project
    
    std::cout << "[Workspace] Saved project: " << metadata_.name << std::endl;
    dirty_ = false;
    return true;
}

bool Workspace::save_as(const std::filesystem::path& new_path) {
    metadata_.path = new_path;
    return save();
}

} // namespace editor
} // namespace basements

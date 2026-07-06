#include "basements/engine/module_manager.h"
#include "basements/engine/workspace.h"
#include <iostream>

namespace basements {
namespace editor {

ModuleManager::ModuleManager(Workspace& workspace) : workspace_(workspace) {}

ModuleManager::~ModuleManager() {
    shutdown();
}

void ModuleManager::update(float delta_time) {
    for (auto& [name, module] : modules_) {
        module->on_update(delta_time);
    }
}

void ModuleManager::render_ui() {
    for (auto& [name, module] : modules_) {
        module->on_render_ui();
    }
}

void ModuleManager::shutdown() {
    for (auto& [name, module] : modules_) {
        module->on_shutdown();
    }
    modules_.clear();
}

IModule* ModuleManager::get_module(const std::string& name) {
    auto it = modules_.find(name);
    return (it != modules_.end()) ? it->second.get() : nullptr;
}

} // namespace editor
} // namespace basements

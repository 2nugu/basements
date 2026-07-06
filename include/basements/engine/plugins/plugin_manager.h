#ifndef BASEMENTS_ENGINE_PLUGIN_MANAGER_H
#define BASEMENTS_ENGINE_PLUGIN_MANAGER_H

#include "plugin_interface.h"
#include "basements/engine/utils/error_handling.h"
#include <unordered_map>
#include <vector>
#include <memory>

namespace basements {
namespace engine {

/// Manages physics plugins
class PluginManager {
public:
    PluginManager() = default;
    ~PluginManager() = default;

    /// Register a new plugin
    void register_plugin(std::unique_ptr<PhysicsPlugin> plugin) {
        if (!plugin) {
            ErrorManager::throw_error(
                EngineError::PLUGIN_ERROR,
                "Cannot register null plugin"
            );
        }

        std::string plugin_name = plugin->name();
        
        if (plugins_.find(plugin_name) != plugins_.end()) {
            ErrorManager::throw_error(
                EngineError::PLUGIN_ERROR,
                "Plugin already registered: " + plugin_name
            );
        }

        plugins_[plugin_name] = std::move(plugin);
    }

    /// Unregister a plugin by name
    void unregister_plugin(const std::string& plugin_name) {
        auto it = plugins_.find(plugin_name);
        if (it != plugins_.end()) {
            it->second->shutdown();
            plugins_.erase(it);
        }
    }

    /// Get plugin by name
    PhysicsPlugin* get_plugin(const std::string& plugin_name) {
        auto it = plugins_.find(plugin_name);
        return (it != plugins_.end()) ? it->second.get() : nullptr;
    }

    /// Check if plugin exists
    bool has_plugin(const std::string& plugin_name) const {
        return plugins_.find(plugin_name) != plugins_.end();
    }

    /// Get list of all plugin names
    std::vector<std::string> list_plugins() const {
        std::vector<std::string> names;
        names.reserve(plugins_.size());
        for (const auto& [name, plugin] : plugins_) {
            names.push_back(name);
        }
        return names;
    }

    /// Initialize all plugins
    void initialize_all() {
        for (auto& [name, plugin] : plugins_) {
            plugin->initialize();
        }
    }

    /// Update all plugins
    void update_all(float delta_time_seconds) {
        for (auto& [name, plugin] : plugins_) {
            plugin->update(delta_time_seconds);
        }
    }

    /// Shutdown all plugins
    void shutdown_all() {
        for (auto& [name, plugin] : plugins_) {
            plugin->shutdown();
        }
    }

    /// Get plugin count
    size_t plugin_count() const {
        return plugins_.size();
    }

private:
    std::unordered_map<std::string, std::unique_ptr<PhysicsPlugin>> plugins_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_PLUGIN_MANAGER_H

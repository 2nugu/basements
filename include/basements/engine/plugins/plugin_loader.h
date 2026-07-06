/**
 * @file plugin_loader.h
 * @brief Dynamic Plugin Loader (DLL/SO)
 * 
 * Loads plugins at runtime without engine recompilation.
 * Cross-platform support for Windows (.dll) and Linux (.so).
 */

#ifndef BASEMENTS_PLUGIN_LOADER_H
#define BASEMENTS_PLUGIN_LOADER_H

#include "basements/engine/plugins/plugin_capi.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#ifdef _WIN32
    #include <windows.h>
    typedef HMODULE PluginHandle;
#else
    #include <dlfcn.h>
    typedef void* PluginHandle;
#endif

namespace basements {
namespace plugin {

/**
 * @brief Loaded Plugin Instance
 */
struct LoadedPlugin {
    std::string path;
    PluginHandle handle;
    BasementsPluginCallbacks callbacks;
    BasementsPluginInfo info;
    bool active;
    
    LoadedPlugin() : handle(nullptr), callbacks{}, info{}, active(false) {}
};

/**
 * @brief Plugin Loader & Manager
 */
class PluginLoader {
public:
    static PluginLoader& instance() {
        static PluginLoader loader;
        return loader;
    }
    
    /**
     * @brief Load a plugin from file path
     * @return true on success
     */
    bool load(const std::string& path) {
        // Open library
        PluginHandle handle = open_library(path);
        if (!handle) {
            last_error_ = "Failed to open library: " + path;
            return false;
        }
        
        // Find register function
        auto register_func = (BasementsPluginRegisterFunc)get_symbol(handle, BASEMENTS_PLUGIN_REGISTER_NAME);
        if (!register_func) {
            last_error_ = "Plugin missing entry point: " BASEMENTS_PLUGIN_REGISTER_NAME;
            close_library(handle);
            return false;
        }
        
        // Get callbacks and info
        BasementsPluginCallbacks cb = register_func();
        if (!cb.get_info || !cb.on_load || !cb.on_unload) {
            last_error_ = "Plugin has NULL required callbacks";
            close_library(handle);
            return false;
        }
        
        BasementsPluginInfo info = cb.get_info();
        
        // Check API version
        if (info.api_version != BASEMENTS_PLUGIN_API_VERSION) {
            last_error_ = "Plugin API version mismatch";
            close_library(handle);
            return false;
        }
        
        // Initialize plugin
        if (cb.on_load(engine_context_) != 0) {
            last_error_ = "Plugin on_load failed";
            close_library(handle);
            return false;
        }
        
        // Store plugin
        LoadedPlugin plugin;
        plugin.path = path;
        plugin.handle = handle;
        plugin.callbacks = cb;
        plugin.info = info;
        plugin.active = true;
        
        plugins_[info.name] = plugin;
        return true;
    }
    
    /**
     * @brief Unload a plugin by name
     */
    void unload(const std::string& name) {
        auto it = plugins_.find(name);
        if (it == plugins_.end()) return;
        
        LoadedPlugin& plugin = it->second;
        if (plugin.active && plugin.callbacks.on_unload) {
            plugin.callbacks.on_unload();
        }
        
        close_library(plugin.handle);
        plugins_.erase(it);
    }
    
    /**
     * @brief Call pre_step on all active plugins
     */
    void broadcast_pre_step(float dt) {
        for (auto& [name, plugin] : plugins_) {
            if (plugin.active && plugin.callbacks.on_pre_step) {
                plugin.callbacks.on_pre_step(dt);
            }
        }
    }
    
    /**
     * @brief Call post_step on all active plugins
     */
    void broadcast_post_step(float dt) {
        for (auto& [name, plugin] : plugins_) {
            if (plugin.active && plugin.callbacks.on_post_step) {
                plugin.callbacks.on_post_step(dt);
            }
        }
    }
    
    /**
     * @brief Get list of loaded plugin names
     */
    std::vector<std::string> get_loaded_plugins() const {
        std::vector<std::string> names;
        for (const auto& [name, plugin] : plugins_) {
            names.push_back(name);
        }
        return names;
    }
    
    const std::string& get_last_error() const { return last_error_; }
    
    void set_engine_context(BasementsEngineContext ctx) { engine_context_ = ctx; }
    
private:
    PluginLoader() : engine_context_(nullptr) {}
    ~PluginLoader() {
        for (auto& [name, plugin] : plugins_) {
            if (plugin.active && plugin.callbacks.on_unload) {
                plugin.callbacks.on_unload();
            }
            close_library(plugin.handle);
        }
    }
    
    PluginLoader(const PluginLoader&) = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;
    
    // Platform-specific library functions
    PluginHandle open_library(const std::string& path) {
#ifdef _WIN32
        return LoadLibraryA(path.c_str());
#else
        return dlopen(path.c_str(), RTLD_NOW);
#endif
    }
    
    void close_library(PluginHandle handle) {
        if (!handle) return;
#ifdef _WIN32
        FreeLibrary(handle);
#else
        dlclose(handle);
#endif
    }
    
    void* get_symbol(PluginHandle handle, const char* name) {
#ifdef _WIN32
        return (void*)GetProcAddress(handle, name);
#else
        return dlsym(handle, name);
#endif
    }
    
    std::unordered_map<std::string, LoadedPlugin> plugins_;
    std::string last_error_;
    BasementsEngineContext engine_context_;
};

} // namespace plugin
} // namespace basements

#endif // BASEMENTS_PLUGIN_LOADER_H

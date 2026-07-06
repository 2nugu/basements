/**
 * @file plugin_capi.h
 * @brief C-ABI Stable Plugin Interface for DLL/SO Loading
 * 
 * This header provides a C-compatible API for plugin development.
 * Plugins compiled with different compilers can be loaded safely.
 */

#ifndef BASEMENTS_PLUGIN_CAPI_H
#define BASEMENTS_PLUGIN_CAPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Export Macros
// ============================================================================
#ifdef _WIN32
    #ifdef BASEMENTS_PLUGIN_EXPORT
        #define BASEMENTS_PLUGIN_API __declspec(dllexport)
    #else
        #define BASEMENTS_PLUGIN_API __declspec(dllimport)
    #endif
#else
    #define BASEMENTS_PLUGIN_API __attribute__((visibility("default")))
#endif

// ============================================================================
// Plugin Info Structure
// ============================================================================
typedef struct {
    const char* name;
    const char* version;
    const char* author;
    const char* description;
    uint32_t api_version;  // For compatibility checking
} BasementsPluginInfo;

#define BASEMENTS_PLUGIN_API_VERSION 1

// ============================================================================
// Engine Context (Opaque Handle)
// ============================================================================
typedef void* BasementsEngineContext;

// ============================================================================
// Plugin Callbacks
// ============================================================================
typedef struct {
    // Required
    BasementsPluginInfo (*get_info)(void);
    int (*on_load)(BasementsEngineContext ctx);
    void (*on_unload)(void);
    
    // Optional (can be NULL)
    void (*on_pre_step)(float dt);
    void (*on_post_step)(float dt);
    void (*on_render)(void);
} BasementsPluginCallbacks;

// ============================================================================
// Plugin Entry Point (Must be exported by every plugin DLL)
// ============================================================================
/**
 * @brief Main entry point for plugin registration
 * 
 * Every plugin DLL must export this function with exact signature:
 *   BASEMENTS_PLUGIN_API BasementsPluginCallbacks basements_plugin_register(void);
 */
typedef BasementsPluginCallbacks (*BasementsPluginRegisterFunc)(void);

#define BASEMENTS_PLUGIN_REGISTER_NAME "basements_plugin_register"

// ============================================================================
// Helper Macros for Plugin Authors
// ============================================================================
#define BASEMENTS_PLUGIN_DEFINE(info_func, load_func, unload_func) \
    BASEMENTS_PLUGIN_API BasementsPluginCallbacks basements_plugin_register(void) { \
        BasementsPluginCallbacks cb = {0}; \
        cb.get_info = info_func; \
        cb.on_load = load_func; \
        cb.on_unload = unload_func; \
        cb.on_pre_step = NULL; \
        cb.on_post_step = NULL; \
        cb.on_render = NULL; \
        return cb; \
    }

#define BASEMENTS_PLUGIN_DEFINE_FULL(info_func, load_func, unload_func, pre_step, post_step, render) \
    BASEMENTS_PLUGIN_API BasementsPluginCallbacks basements_plugin_register(void) { \
        BasementsPluginCallbacks cb = {0}; \
        cb.get_info = info_func; \
        cb.on_load = load_func; \
        cb.on_unload = unload_func; \
        cb.on_pre_step = pre_step; \
        cb.on_post_step = post_step; \
        cb.on_render = render; \
        return cb; \
    }

#ifdef __cplusplus
}
#endif

#endif // BASEMENTS_PLUGIN_CAPI_H

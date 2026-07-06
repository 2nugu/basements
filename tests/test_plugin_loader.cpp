/**
 * @file test_plugin_loader.cpp
 * @brief Unit tests for Plugin Loader
 */

#include <gtest/gtest.h>
#include "basements/engine/plugins/plugin_loader.h"

using namespace basements::plugin;

TEST(PluginLoaderTest, Singleton) {
    auto& loader1 = PluginLoader::instance();
    auto& loader2 = PluginLoader::instance();
    
    EXPECT_EQ(&loader1, &loader2);
}

TEST(PluginLoaderTest, LoadNonExistent) {
    auto& loader = PluginLoader::instance();
    
    bool result = loader.load("nonexistent_plugin.dll");
    EXPECT_FALSE(result);
    EXPECT_FALSE(loader.get_last_error().empty());
}

TEST(PluginLoaderTest, BroadcastEmpty) {
    auto& loader = PluginLoader::instance();
    
    // Should not crash with no plugins
    loader.broadcast_pre_step(0.016f);
    loader.broadcast_post_step(0.016f);
    
    EXPECT_TRUE(true); // If we reach here, no crash
}

TEST(PluginLoaderTest, GetLoadedPlugins) {
    auto& loader = PluginLoader::instance();
    
    auto plugins = loader.get_loaded_plugins();
    // Should return empty or whatever is currently loaded
    EXPECT_GE(plugins.size(), 0u);
}

// ============================================================
// Basements Scene Editor - Main Entry Point
// ============================================================

#include "basements/editor/editor_app.h"
#include <iostream>

int main() {
    using namespace basements::editor;

    // 1. Create Application
    EditorApp app;
    
    // 2. Configure
    EditorConfig config;
    config.window_title = "Basements Physics Environment Editor";
    config.window_width = 1600;
    config.window_height = 900;
    config.dark_theme = true;

    // 3. Initialize
    if (!app.initialize(config)) {
        std::cerr << "Fatal Error: Failed to initialize EditorApp." << std::endl;
        return -1;
    }
    
    // 4. Run Main Loop
    app.run();
    
    return 0;
}

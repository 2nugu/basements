#include "basements/editor/scripting/python_bridge.h"
#include <iostream>

// Stub implementation used when Python/pybind11 is not linked.
// console_panel.cpp calls execute_string; this no-op prevents linker errors.

namespace basements {
namespace editor {

void PythonBridge::initialize() {
    std::cout << "[PythonBridge] Python scripting is not enabled in this build.\n";
}

void PythonBridge::shutdown() {}

void PythonBridge::execute_string(const std::string& code) {
    std::cout << "[PythonBridge] Python not available. Code: " << code << "\n";
}

void PythonBridge::bind_api() {}

} // namespace editor
} // namespace basements

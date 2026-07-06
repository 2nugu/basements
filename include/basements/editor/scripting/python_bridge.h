#ifndef BASEMENTS_EDITOR_SCRIPTING_PYTHON_BRIDGE_H
#define BASEMENTS_EDITOR_SCRIPTING_PYTHON_BRIDGE_H

#include <string>

namespace basements {
namespace editor {

class PythonBridge {
public:
    static void initialize();
    static void shutdown();
    
    // Execute raw python string
    static void execute_string(const std::string& code);
    
    // Bind engine types (called during init)
    static void bind_api();
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_SCRIPTING_PYTHON_BRIDGE_H

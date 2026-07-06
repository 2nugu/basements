#ifndef BASEMENTS_EDITOR_SCRIPT_PANEL_H
#define BASEMENTS_EDITOR_SCRIPT_PANEL_H

#include <string>

namespace basements {
namespace editor {

class ScriptPanel {
public:
    ScriptPanel();
    ~ScriptPanel() = default;

    void on_render(bool* open);

private:
    std::string script_buffer_;
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_SCRIPT_PANEL_H

#ifndef BASEMENTS_EDITOR_PANELS_CONSOLE_PANEL_H
#define BASEMENTS_EDITOR_PANELS_CONSOLE_PANEL_H

#include <vector>
#include <string>

namespace basements {
namespace editor {

class ConsolePanel {
public:
    ConsolePanel();
    ~ConsolePanel() = default;

    void on_render(bool* open);
    
    void clear();
    void log(const std::string& message);

private:
    std::vector<std::string> items_;
    bool auto_scroll_ = true;
    bool scroll_to_bottom_ = false;
    
    // Command input
    char input_buffer_[256] = "";
    
    void execute_command(const std::string& command);
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_PANELS_CONSOLE_PANEL_H

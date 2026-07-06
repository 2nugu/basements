#include "basements/editor/ui/console_panel.h"
#include "basements/editor/scripting/python_bridge.h"
#include "imgui.h"
#include <iostream>

namespace basements {
namespace editor {

ConsolePanel::ConsolePanel() {
    clear();
}

void ConsolePanel::clear() {
    items_.clear();
}

void ConsolePanel::log(const std::string& message) {
    items_.push_back(message);
    if (auto_scroll_) scroll_to_bottom_ = true;
}

void ConsolePanel::execute_command(const std::string& command) {
    log("> " + command);
    
    // Execute Python
    // We should capture stdout/stderr from python if possible, 
    // but for now we rely on explicit logs or just side effects.
    PythonBridge::execute_string(command);
    
    // Clear input
    memset(input_buffer_, 0, sizeof(input_buffer_));
}

void ConsolePanel::on_render(bool* open) {
    if (!ImGui::Begin("Console", open)) {
        ImGui::End();
        return;
    }

    // Top Bar
    if (ImGui::Button("Clear")) clear();
    ImGui::SameLine();
    bool copy_to_clipboard = ImGui::Button("Copy");
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &auto_scroll_);

    ImGui::Separator();

    // Log Region
    const float footer_height_to_reserve = ImGui::GetStyle().ItemSpacing.y + ImGui::GetFrameHeightWithSpacing();
    ImGui::BeginChild("ScrollingRegion", ImVec2(0, -footer_height_to_reserve), false, ImGuiWindowFlags_HorizontalScrollbar);
    
    if (ImGui::BeginPopupContextWindow()) {
        if (ImGui::Selectable("Clear")) clear();
        ImGui::EndPopup();
    }

    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 1)); // Tighten spacing
    for (const auto& item : items_) {
        // Colorize simple errors
        bool has_color = false;
        if (item.find("[Error]") != std::string::npos || item.find("Error:") != std::string::npos) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
            has_color = true;
        } else if (item.find("> ") == 0) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.6f, 1.0f)); // Command echo
            has_color = true;
        }

        ImGui::TextUnformatted(item.c_str());
        
        if (has_color) ImGui::PopStyleColor();
    }
    ImGui::PopStyleVar();

    if (scroll_to_bottom_ || (auto_scroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()))
        ImGui::SetScrollHereY(1.0f);
    scroll_to_bottom_ = false;

    ImGui::EndChild();

    ImGui::Separator();

    // Command Input
    bool reclaim_focus = false;
    ImGuiInputTextFlags input_flags = ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackCompletion | ImGuiInputTextFlags_CallbackHistory;
    
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Input:");
    ImGui::SameLine();
    
    ImGui::PushItemWidth(-1);
    if (ImGui::InputText("##Input", input_buffer_, sizeof(input_buffer_), input_flags)) {
        std::string cmd = input_buffer_;
        if (!cmd.empty()) {
            execute_command(cmd);
        }
        reclaim_focus = true;
    }
    ImGui::PopItemWidth();

    // Auto-focus on window click
    ImGui::SetItemDefaultFocus();
    if (reclaim_focus)
        ImGui::SetKeyboardFocusHere(-1); // Focus previous widget

    ImGui::End();
}

} // namespace editor
} // namespace basements

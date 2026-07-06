#ifndef BASEMENTS_EDITOR_LOCALIZATION_H
#define BASEMENTS_EDITOR_LOCALIZATION_H

#include <string>
#include <unordered_map>

namespace basements {
namespace editor {

enum class Language {
    ENGLISH,
    KOREAN
};

class Localization {
public:
    static void set_language(Language lang) {
        current_lang = lang;
    }

    static Language get_language() {
        return current_lang;
    }

    static const char* get(const std::string& key) {
        if (current_lang == Language::ENGLISH) {
            return key.c_str(); // Default to key if English
        }
        
        // Simple Korean map
        static std::unordered_map<std::string, std::string> ko_map = {
            {"Scene Graph", "장면 그래프 (Scene Graph)"},
            {"World", "월드 (World)"},
            {"Inspector", "속성 (Inspector)"},
            {"Name", "이름"},
            {"Transform", "변형 (Transform)"},
            {"Position", "위치"},
            {"Scale", "크기"},
            {"Physics", "물리 (Physics)"},
            {"Use Gravity", "중력 사용"},
            {"Body Type", "바디 타입"},
            {"Static", "정적 (Static)"},
            {"Kinematic", "키네마틱 (Kinematic)"},
            {"Dynamic", "동적 (Dynamic)"},
            {"Mass", "질량 (Mass)"},
            {"Friction", "마찰력 (Friction)"},
            {"Restitution", "반발계수 (Restitution)"},
            {"Shape", "형태"},
            {"Box", "상자 (Box)"},
            {"Sphere", "구 (Sphere)"},
            {"Capsule", "캡슐 (Capsule)"},
            {"Mesh", "메쉬 (Mesh)"},
            {"Half-Extents", "반크기 (Half-Extents)"},
            {"Selected: None", "선택된 객체 없음"},
            {"Settings", "설정"},
            {"Global Settings", "전역 설정"},
            {"Group Settings", "그룹 설정"},
            {"Object Settings", "객체 설정"},
            {"Mechanics", "역학"},
            {"Thermodynamics", "열역학"},
            {"Electromagnetism", "전자기학"},
            {"Universal Constants", "보편 상수"},
            {"Features", "기능"},
            {"Gravity", "중력"},
            {"Air Density", "공기 밀도"},
            {"Air Viscosity", "공기 점성"},
            {"Ambient Temp", "주변 온도"},
            {"Enable Gravity", "중력 활성화"},
            {"Enable Drag", "항력 활성화"},
            {"Search", "검색"},
            {"Search Results:", "검색 결과:"},
            {"Reset Layout", "레이아웃 초기화"},
            {"File", "파일"},
            {"Window", "창"},
            {"View", "보기"},
        };

        auto it = ko_map.find(key);
        if (it != ko_map.end()) {
            return it->second.c_str();
        }
        return key.c_str();
    }

private:
    static Language current_lang;
};

// Convenience macro
#define LOC(key) Localization::get(key)

} // namespace editor
} // namespace basements

#endif

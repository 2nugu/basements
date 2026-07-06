#ifndef BASEMENTS_EDITOR_LOCALIZATION_H
#define BASEMENTS_EDITOR_LOCALIZATION_H

#include <string>
#include <unordered_map>

namespace basements {
namespace editor {

enum class Language {
    English,
    Korean
};

class Localization {
public:
    static Language current_lang;

    static void set_language(Language lang) {
        current_lang = lang;
    }

    static const char* get(const std::string& key) {
        if (current_lang == Language::English) {
            return key.c_str(); // Default is key itself if English
        }
        
        // Korean Map
        static std::unordered_map<std::string, std::string> ko_map = {
            // General
            {"Search", "검색"},
            {"Settings", "설정"},
            {"Reset", "초기화"},
            {"Presets", "프리셋"},
            
            // Physics Categories
            {"Mechanics", "역학"},
            {"Thermodynamics", "열역학"},
            {"Electromagnetism", "전자기학"},
            {"Universal Constants", "보편 상수"},
            {"Features", "기능 활성화"},
            
            // Parameters
            {"Gravity", "중력 가속도"},
            {"Air Density", "공기 밀도"},
            {"Air Viscosity", "공기 점성"},
            {"Ambient Temp", "주변 온도"},
            {"Stefan-Boltzmann", "슈테판-볼츠만 상수"},
            {"Vacuum Permittivity", "진공 유전율"},
            {"Vacuum Permeability", "진공 투자율"}, // [NEW]
            {"Speed of Light", "빛의 속도"},
            {"Gravitational Constant", "중력 상수"}, // [NEW]
            {"Planck Constant", "플랑크 상수"}, // [NEW]
            {"Enable Gravity", "중력 활성화"},
            {"Enable Drag", "항력(Drag) 활성화"},
            
            // Presets
            {"Earth", "지구"},
            {"Moon", "달"},
            {"Mars", "화성"},
            {"Space", "우주"}
        };

        if (current_lang == Language::Korean) {
            auto it = ko_map.find(key);
            if (it != ko_map.end()) {
                return it->second.c_str();
            }
        }
        
        return key.c_str();
    }
};

// Global Instance for header-only simple usage
// (In C++17 inline variables allow this in headers)
inline Language Localization::current_lang = Language::Korean; // Default to Korean as per user context

} // namespace editor
} // namespace basements

#define LOC(key) basements::editor::Localization::get(key)

#endif // BASEMENTS_EDITOR_LOCALIZATION_H

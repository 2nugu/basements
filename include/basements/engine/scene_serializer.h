#ifndef BASEMENTS_EDITOR_SCENE_SERIALIZER_H
#define BASEMENTS_EDITOR_SCENE_SERIALIZER_H

#include "basements/engine/scene_graph.h"
#include <string>

namespace basements {
namespace editor {

class SceneSerializer {
public:
    static bool save_scene(const SceneGraph& scene, const std::string& filepath);
    static bool load_scene(SceneGraph& scene, const std::string& filepath);
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_SCENE_SERIALIZER_H

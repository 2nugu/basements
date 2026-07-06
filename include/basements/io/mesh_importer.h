#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>
#include "basements/graphics/mesh.h"

namespace basements {
namespace editor {

class MeshImporter {
public:
    static bool load_obj(const std::string& path, Mesh& out_mesh);
    static bool load_gltf(const std::string& path, Mesh& out_mesh); // [NEW]
    static bool load_mesh(const std::string& path, Mesh& out_mesh); // Auto-detect extension
};

} // namespace editor
} // namespace basements

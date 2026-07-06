#ifndef BASEMENTS_EDITOR_RENDERING_MESH_H
#define BASEMENTS_EDITOR_RENDERING_MESH_H

#include <vector>
#include <string>
#include <glm/glm.hpp>

namespace basements {
namespace editor {

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texcoord;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int vao = 0;
    unsigned int vbo = 0;
    unsigned int ebo = 0;

    void setup_mesh();
    void draw();
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_RENDERING_MESH_H

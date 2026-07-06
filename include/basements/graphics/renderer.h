#ifndef BASEMENTS_EDITOR_RENDERER_H
#define BASEMENTS_EDITOR_RENDERER_H

#include "basements/graphics/shader.h"
#include "basements/graphics/camera.h"
#include "basements/io/mesh_importer.h"
#include <glm/glm.hpp>

namespace basements {
namespace editor {

class Renderer {
public:
    static void initialize();
    static void shutdown();
    
    static void set_clear_color(const glm::vec3& color);
    static void clear();
    
    static void begin_scene(const Camera& camera);
    static void end_scene();
    
    static void draw_cube(const glm::vec3& position, const glm::vec3& size, const glm::vec3& color);
    static void draw_grid(int size, float step);
    static void draw_mesh(Mesh& mesh, const glm::vec3& position, const glm::vec3& color);
    static void draw_line(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color);
    static void draw_aabb(const glm::vec3& min, const glm::vec3& max, const glm::vec3& color); // [NEW]
    
private:
    struct Impl;
    // Renderer data (Shaders, VATs, etc)
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_RENDERER_H

#ifndef BASEMENTS_EDITOR_PRIMITIVE_FACTORY_H
#define BASEMENTS_EDITOR_PRIMITIVE_FACTORY_H

#include "basements/graphics/mesh.h"
#include <memory>

namespace basements {
namespace editor {

class PrimitiveFactory {
public:
    static Mesh create_plane(float width = 10.0f, float depth = 10.0f, int sub_x = 10, int sub_z = 10);
    static Mesh create_cube(float size = 1.0f);
    static Mesh create_sphere(float radius = 0.5f, int segments = 32, int rings = 16);
    static Mesh create_cylinder(float top_radius = 0.5f, float bottom_radius = 0.5f, float height = 1.0f, int segments = 32);
    static Mesh create_cone(float radius = 0.5f, float height = 1.0f, int segments = 32);
    static Mesh create_torus(float major_radius = 0.8f, float minor_radius = 0.2f, int major_segments = 32, int minor_segments = 16);
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_PRIMITIVE_FACTORY_H

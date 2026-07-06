#ifndef BASEMENTS_COLLISION_SHAPE_H
#define BASEMENTS_COLLISION_SHAPE_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"

namespace basements {
namespace collision {

using namespace basements::math;

/**
 * @brief Types of collision shapes
 */
enum class ShapeType : int {
    SPHERE = 0,
    BOX = 1,
    CAPSULE = 2,
    CONVEX_HULL = 3,
    MESH = 4 // Fallback, likely not for GJK directly
};

/**
 * @brief Generic Shape data structure (Union-like for GPU SoA/AoS)
 * 
 * To keep things simple and GPU-friendly (standard layout), we might use a specialized 
 * struct for each, but for a general "Shape" pointer in kernels, we often use indices 
 * into specific arrays. However, for GJK, we need a unified way to pass shapes.
 * 
 * Here, we define the *Interface* concept for GJK.
 * Any struct T passed to GJK must implement:
 *   HOST_DEVICE Vec3 support(const Vec3& dir) const;
 */

/**
 * @brief Base struct for tagged unions if needed, 
 * but GJK will likely use templates to avoid virtual overhead.
 */
struct ShapeBase {
    ShapeType type;
};

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_SHAPE_H

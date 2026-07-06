#ifndef BASEMENTS_COLLISION_AABB_H
#define BASEMENTS_COLLISION_AABB_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include <limits>

namespace basements {
namespace collision {

using namespace basements::math;

struct alignas(16) AABB {
    Vec3 min;
    Vec3 max;

    HOST_DEVICE AABB() 
        #ifdef __CUDA_ARCH__
        : min(Vec3(FLT_MAX)), 
          max(Vec3(-FLT_MAX)) {}
        #else
        : min(Vec3(std::numeric_limits<float>::max())), 
          max(Vec3(std::numeric_limits<float>::lowest())) {}
        #endif

    HOST_DEVICE AABB(const Vec3& p) : min(p), max(p) {}

    HOST_DEVICE AABB(const Vec3& min_val, const Vec3& max_val) 
        : min(min_val), max(max_val) {}

    // Expand AABB to include a point
    HOST_DEVICE void expand(const Vec3& p) {
        min = Vec3::min(min, p);
        max = Vec3::max(max, p);
    }

    // Expand AABB to include another AABB
    HOST_DEVICE void expand(const AABB& other) {
        min = Vec3::min(min, other.min);
        max = Vec3::max(max, other.max);
    }

    // Check if overlaps with another AABB
    HOST_DEVICE bool overlaps(const AABB& other) const {
        if (max.x < other.min.x || min.x > other.max.x) return false;
        if (max.y < other.min.y || min.y > other.max.y) return false;
        if (max.z < other.min.z || min.z > other.max.z) return false;
        return true;
    }

    // Check if contains a point
    HOST_DEVICE bool contains(const Vec3& p) const {
        return (p.x >= min.x && p.x <= max.x &&
                p.y >= min.y && p.y <= max.y &&
                p.z >= min.z && p.z <= max.z);
    }

    HOST_DEVICE Vec3 center() const {
        return (min + max) * 0.5f;
    }

    HOST_DEVICE Vec3 extents() const {
        return (max - min) * 0.5f;
    }

    // Surface Area Heuristic (SAH) cost (mostly for BVH, but useful)
    HOST_DEVICE float surface_area() const {
        Vec3 d = max - min;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
};

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_AABB_H

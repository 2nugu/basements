#ifndef BASEMENTS_COLLISION_PRIMITIVES_H
#define BASEMENTS_COLLISION_PRIMITIVES_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/collision/shape.h"

namespace basements {
namespace collision {

using namespace basements::math;

// ============================================================================
// SPHERE
// ============================================================================

struct alignas(16) Sphere {
    Vec3 center;
    float radius;
    float _pad[3]; // Padding to 16 bytes alignment if needed, though Vec3 is 16

    HOST_DEVICE Sphere() : center(0.0f), radius(0.5f) {}
    HOST_DEVICE Sphere(const Vec3& c, float r) : center(c), radius(r) {}

    /**
     * @brief Support function for GJK
     * For a sphere, the support point is the center + radius in the direction of d.
     * BUT, typically GJK works on the Minkowski Difference. 
     * The standard support(d) returns the furthest point in direction d.
     */
    HOST_DEVICE Vec3 support(const Vec3& dir) const {
        if (dir.is_zero()) {
            return center + Vec3(radius, 0, 0); // Arbitrary fallback
        }
        return center + dir.normalized() * radius;
    }
};

// ============================================================================
// BOX
// ============================================================================

struct alignas(16) Box {
    Vec3 half_extents; // Half width/height/depth
    
    // Note: Position/Orientation usually handled by Transform in GJK, 
    // but we can assume this Box is in local space or include transform.
    // Let's assume Local Space for defining the shape dimensions.

    HOST_DEVICE Box() : half_extents(0.5f) {}
    HOST_DEVICE Box(const Vec3& half) : half_extents(half) {}
    HOST_DEVICE Box(float x, float y, float z) : half_extents(x, y, z) {}

    /**
     * @brief Support function for Box in Local Space
     * S(d) = (sign(d.x)*hx, sign(d.y)*hy, sign(d.z)*hz)
     */
    HOST_DEVICE Vec3 support(const Vec3& dir) const {
        return Vec3(
            (dir.x >= 0.0f) ? half_extents.x : -half_extents.x,
            (dir.y >= 0.0f) ? half_extents.y : -half_extents.y,
            (dir.z >= 0.0f) ? half_extents.z : -half_extents.z
        );
    }
};

// ============================================================================
// CAPSULE
// ============================================================================

/**
 * @brief Capsule aligned along the Y-axis (common convention)
 */
struct alignas(16) Capsule {
    float radius;
    float half_height; // Half of the cylinder segments height
    // Total height = 2 * half_height + 2 * radius
    
    // We can store centers of two spheres for easier support calc, 
    // or just mathematically compute it.
    // Center is origin (0,0,0) in local space. Points A=(0, -h, 0), B=(0, h, 0).
    
    HOST_DEVICE Capsule() : radius(0.5f), half_height(0.5f) {}
    HOST_DEVICE Capsule(float r, float hh) : radius(r), half_height(hh) {}

    /**
     * @brief Support function for Capsule in Local Space (Y-up)
     */
    HOST_DEVICE Vec3 support(const Vec3& dir) const {
        // 1. Find support of the segment (line between A and B)
        // A = (0, -h, 0), B = (0, h, 0)
        // Dot(A, d) = -h * d.y
        // Dot(B, d) = h * d.y
        // If d.y > 0, B is further. Else A is further.
        
        Vec3 segment_support = (dir.y > 0.0f) 
             ? Vec3(0.0f, half_height, 0.0f) 
             : Vec3(0.0f, -half_height, 0.0f);
             
        // 2. Add sphere support (radius * normalized(d))
        if (dir.is_zero()) {
            return segment_support + Vec3(radius, 0, 0);
        }
        
        return segment_support + dir.normalized() * radius;
    }
};

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_PRIMITIVES_H

#ifndef BASEMENTS_PHYSICS_FORMULAS_GEOMETRY_H
#define BASEMENTS_PHYSICS_FORMULAS_GEOMETRY_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace geometry {

using Vec3 = basements::math::Vec3;
constexpr float EPSILON = 1e-6f;

// ============================================================
// Distance and Projection
// ============================================================

/// Calculate distance from point to line (3D)
/// Formula: d = |(p - a) x (p - b)| / |b - a| where line is defined by points a and b
/// Alternatively with point and direction: d = |(p - a) x n| where n is normalized direction
/// @param point_vec Point P
/// @param line_origin_vec Point on line A
/// @param line_direction_normalized Direction vector n
/// @return Distance
HOST_DEVICE inline float calculate_point_to_line_distance(
    const Vec3& point_vec,
    const Vec3& line_origin_vec,
    const Vec3& line_direction_normalized
) {
    Vec3 diff = point_vec - line_origin_vec;
    Vec3 cross_prod = diff.cross(line_direction_normalized);
    return cross_prod.length();
}

/// Calculate distance from point to plane
/// Formula: d = |(p - p0) . n|
/// @param point_vec Point P
/// @param plane_point_vec Point on plane p0
/// @param plane_normal_vec Normalized plane normal n
/// @return Distance
HOST_DEVICE inline float calculate_point_to_plane_distance(
    const Vec3& point_vec,
    const Vec3& plane_point_vec,
    const Vec3& plane_normal_vec
) {
    Vec3 diff = point_vec - plane_point_vec;
    return std::abs(diff.dot(plane_normal_vec));
}

/// Project point onto plane
/// Formula: p_proj = p - dist * n
/// @param point_vec Point P
/// @param plane_point_vec Point on plane p0
/// @param plane_normal_vec Normalized plane normal n
/// @return Projected point
HOST_DEVICE inline Vec3 calculate_project_point_onto_plane(
    const Vec3& point_vec,
    const Vec3& plane_point_vec,
    const Vec3& plane_normal_vec
) {
    Vec3 diff = point_vec - plane_point_vec;
    float dist = diff.dot(plane_normal_vec);
    return point_vec - plane_normal_vec * dist;
}

/// Calculate angle between two vectors
/// Formula: θ = arccos((a . b) / (|a| |b|))
/// @param vector_a
/// @param vector_b
/// @return Angle in radians
HOST_DEVICE inline float calculate_angle_between_vectors_radians(
    const Vec3& vector_a,
    const Vec3& vector_b
) {
    float mag_product = vector_a.length() * vector_b.length();
    if (mag_product < EPSILON) return 0.0f;
    float dot_prod = vector_a.dot(vector_b);
    float clamped_dot = std::fmax(-1.0f, std::fmin(1.0f, dot_prod / mag_product));
    return std::acos(clamped_dot);
}

// ============================================================
// Primitives
// ============================================================

/// Plane structure defined by Normal and Distance from origin (Hessian normal form: n.p + d = 0)
struct Plane {
    Vec3 normal;
    float d; // Distance from origin along normal (usually negative if normal points away)

    HOST_DEVICE Plane() : normal(Vec3::unit_y()), d(0.0f) {}
    
    /// Construct plane from normal and point
    HOST_DEVICE Plane(const Vec3& n, const Vec3& p) {
        normal = n.normalized();
        d = -normal.dot(p);
    }
    
    /// Construct plane from 3 points
    HOST_DEVICE Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3) {
        Vec3 v1 = p2 - p1;
        Vec3 v2 = p3 - p1;
        normal = v1.cross(v2).normalized();
        d = -normal.dot(p1);
    }

    /// Calculate distance from point to plane (signed)
    HOST_DEVICE float distance_signed(const Vec3& p) const {
        return normal.dot(p) + d;
    }
};

/// Ray structure defined by Origin and Direction
struct Ray {
    Vec3 origin;
    Vec3 direction;

    HOST_DEVICE Ray() : origin(Vec3::zero()), direction(Vec3::unit_z()) {}
    HOST_DEVICE Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalized()) {}

    HOST_DEVICE Vec3 get_point_at(float t) const {
        return origin + direction * t;
    }
};

// ============================================================
// Intersections
// ============================================================

/// Calculate Ray-Plane intersection
/// @param ray Ray
/// @param plane Plane
/// @param t_out Output parameter for distance along ray
/// @return True if intersection exists (and is forward of ray origin)
HOST_DEVICE inline bool calculate_ray_plane_intersection(
    const Ray& ray,
    const Plane& plane,
    float& t_out
) {
    float denom = plane.normal.dot(ray.direction);
    if (std::abs(denom) < EPSILON) {
        return false; // Parallel
    }
    float t = -(plane.normal.dot(ray.origin) + plane.d) / denom;
    if (t >= 0.0f) {
        t_out = t;
        return true;
    }
    return false;
}

} // namespace geometry
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_GEOMETRY_H

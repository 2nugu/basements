#pragma once

#include "vec3.h"
#include "quaternion.h"
#include "matrix4.h"

#ifdef __CUDACC__
    #define HOST_DEVICE __host__ __device__
#else
    #define HOST_DEVICE
#endif

namespace basements {
namespace math {

/**
 * @brief Transform class combining position, rotation, and scale
 * 
 * Provides a convenient way to represent 3D transformations.
 * Commonly used in scene graphs and object hierarchies.
 */
struct Transform {
    Vec3 position;
    Quaternion rotation;
    Vec3 scale;

    // ============================================================
    // Constructors
    // ============================================================
    
    HOST_DEVICE Transform()
        : position(Vec3::zero())
        , rotation(Quaternion::identity())
        , scale(Vec3::one())
    {}

    HOST_DEVICE Transform(const Vec3& pos, const Quaternion& rot, const Vec3& scl)
        : position(pos)
        , rotation(rot)
        , scale(scl)
    {}

    HOST_DEVICE Transform(const Vec3& pos, const Quaternion& rot)
        : position(pos)
        , rotation(rot)
        , scale(Vec3::one())
    {}

    HOST_DEVICE explicit Transform(const Vec3& pos)
        : position(pos)
        , rotation(Quaternion::identity())
        , scale(Vec3::one())
    {}

    // ============================================================
    // Static Factory Methods
    // ============================================================
    
    HOST_DEVICE static Transform identity() {
        return Transform();
    }

    HOST_DEVICE static Transform from_position(const Vec3& pos) {
        return Transform(pos);
    }

    HOST_DEVICE static Transform from_rotation(const Quaternion& rot) {
        return Transform(Vec3::zero(), rot);
    }

    HOST_DEVICE static Transform from_scale(const Vec3& scl) {
        Transform t;
        t.scale = scl;
        return t;
    }

    // ============================================================
    // Transformation Methods
    // ============================================================
    
    /// Transform a point (applies scale, rotation, translation)
    HOST_DEVICE Vec3 transform_point(const Vec3& point) const {
        Vec3 scaled = Vec3(point.x * scale.x, point.y * scale.y, point.z * scale.z);
        Vec3 rotated = rotation.rotate(scaled);
        return rotated + position;
    }

    /// Transform a direction (applies rotation only, no translation)
    HOST_DEVICE Vec3 transform_direction(const Vec3& direction) const {
        return rotation.rotate(direction);
    }

    /// Inverse transform a point
    HOST_DEVICE Vec3 inverse_transform_point(const Vec3& point) const {
        Vec3 translated = point - position;
        Vec3 rotated = rotation.inverse().rotate(translated);
        return Vec3(rotated.x / scale.x, rotated.y / scale.y, rotated.z / scale.z);
    }

    /// Inverse transform a direction
    HOST_DEVICE Vec3 inverse_transform_direction(const Vec3& direction) const {
        return rotation.inverse().rotate(direction);
    }

    // ============================================================
    // Matrix Conversion
    // ============================================================
    
    /// Convert to 4x4 transformation matrix
    HOST_DEVICE basements::math::Matrix4 to_matrix() const {
        basements::math::Matrix4 mat = basements::math::Matrix4::identity();
        
        // Scale
        mat.m[0][0] = scale.x;
        mat.m[1][1] = scale.y;
        mat.m[2][2] = scale.z;
        
        // Rotation (convert quaternion to matrix)
        float xx = rotation.x * rotation.x;
        float yy = rotation.y * rotation.y;
        float zz = rotation.z * rotation.z;
        float xy = rotation.x * rotation.y;
        float xz = rotation.x * rotation.z;
        float yz = rotation.y * rotation.z;
        float wx = rotation.w * rotation.x;
        float wy = rotation.w * rotation.y;
        float wz = rotation.w * rotation.z;

        basements::math::Matrix4 rot_mat;
        rot_mat.m[0][0] = 1.0f - 2.0f * (yy + zz);
        rot_mat.m[0][1] = 2.0f * (xy - wz);
        rot_mat.m[0][2] = 2.0f * (xz + wy);
        rot_mat.m[0][3] = 0.0f;

        rot_mat.m[1][0] = 2.0f * (xy + wz);
        rot_mat.m[1][1] = 1.0f - 2.0f * (xx + zz);
        rot_mat.m[1][2] = 2.0f * (yz - wx);
        rot_mat.m[1][3] = 0.0f;

        rot_mat.m[2][0] = 2.0f * (xz - wy);
        rot_mat.m[2][1] = 2.0f * (yz + wx);
        rot_mat.m[2][2] = 1.0f - 2.0f * (xx + yy);
        rot_mat.m[2][3] = 0.0f;

        rot_mat.m[3][0] = 0.0f;
        rot_mat.m[3][1] = 0.0f;
        rot_mat.m[3][2] = 0.0f;
        rot_mat.m[3][3] = 1.0f;

        // Combine scale and rotation
        mat = rot_mat * mat;
        
        // Translation
        mat.m[0][3] = position.x;
        mat.m[1][3] = position.y;
        mat.m[2][3] = position.z;
        
        return mat;
    }

    // ============================================================
    // Combination
    // ============================================================
    
    /// Combine two transforms (this * other)
    HOST_DEVICE Transform operator*(const Transform& other) const {
        Transform result;
        result.position = transform_point(other.position);
        result.rotation = rotation * other.rotation;
        result.scale = Vec3(scale.x * other.scale.x, scale.y * other.scale.y, scale.z * other.scale.z);
        return result;
    }

    /// Inverse transform
    HOST_DEVICE Transform inverse() const {
        Transform result;
        result.rotation = rotation.inverse();
        result.scale = Vec3(1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z);
        result.position = result.rotation.rotate(Vec3(
            -position.x * result.scale.x,
            -position.y * result.scale.y,
            -position.z * result.scale.z
        ));
        return result;
    }

    // ============================================================
    // Interpolation
    // ============================================================
    
    /// Linear interpolation
    HOST_DEVICE static Transform lerp(const Transform& a, const Transform& b, float t) {
        Transform result;
        result.position = Vec3::lerp(a.position, b.position, t);
        result.rotation = Quaternion::slerp(a.rotation, b.rotation, t);
        result.scale = Vec3::lerp(a.scale, b.scale, t);
        return result;
    }

    // ============================================================
    // Utility
    // ============================================================
    
    /// Get forward direction (local +Z axis)
    HOST_DEVICE Vec3 forward() const {
        return rotation.rotate(Vec3(0, 0, 1));
    }

    /// Get right direction (local +X axis)
    HOST_DEVICE Vec3 right() const {
        return rotation.rotate(Vec3(1, 0, 0));
    }

    /// Get up direction (local +Y axis)
    HOST_DEVICE Vec3 up() const {
        return rotation.rotate(Vec3(0, 1, 0));
    }

    /// Look at target
    HOST_DEVICE void look_at(const Vec3& target, const Vec3& up_vector = Vec3(0, 1, 0)) {
        Vec3 forward = (target - position).normalized();
        Vec3 right = up_vector.cross(forward).normalized();
        Vec3 up = forward.cross(right);

        // Use Quaternion::look_rotation directly
        rotation = Quaternion::look_rotation(forward, up);
    }
};

} // namespace math
} // namespace basements

#pragma once

#include "common.h"
#include "vec3.h"
#include <cmath>

#ifdef __CUDACC__
    #define HOST_DEVICE __host__ __device__
#else
    #define HOST_DEVICE
#endif

namespace basements {
namespace math {

/**
 * @brief 4D Vector class with SIMD and GPU support
 * 
 * Provides 4D vector operations for homogeneous coordinates, quaternions,
 * and color (RGBA). All methods are HOST_DEVICE compatible for CUDA kernels.
 */
struct Vec4 {
    float x, y, z, w;

    // ============================================================
    // Constructors
    // ============================================================
    
    HOST_DEVICE Vec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
    HOST_DEVICE Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    HOST_DEVICE explicit Vec4(float scalar) : x(scalar), y(scalar), z(scalar), w(scalar) {}
    HOST_DEVICE Vec4(const Vec3& v, float w_) : x(v.x), y(v.y), z(v.z), w(w_) {}

    // ============================================================
    // Static Factory Methods
    // ============================================================
    
    HOST_DEVICE static Vec4 zero() { return Vec4(0.0f, 0.0f, 0.0f, 0.0f); }
    HOST_DEVICE static Vec4 one() { return Vec4(1.0f, 1.0f, 1.0f, 1.0f); }
    HOST_DEVICE static Vec4 unit_x() { return Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
    HOST_DEVICE static Vec4 unit_y() { return Vec4(0.0f, 1.0f, 0.0f, 0.0f); }
    HOST_DEVICE static Vec4 unit_z() { return Vec4(0.0f, 0.0f, 1.0f, 0.0f); }
    HOST_DEVICE static Vec4 unit_w() { return Vec4(0.0f, 0.0f, 0.0f, 1.0f); }

    // ============================================================
    // Conversion
    // ============================================================
    
    /// Convert to Vec3 (drop w component)
    HOST_DEVICE Vec3 xyz() const {
        return Vec3(x, y, z);
    }

    /// Homogeneous divide (perspective divide)
    HOST_DEVICE Vec3 to_vec3() const {
        if (fabsf(w) < EPSILON) return Vec3(x, y, z);
        float inv_w = 1.0f / w;
        return Vec3(x * inv_w, y * inv_w, z * inv_w);
    }

    // ============================================================
    // Arithmetic Operators
    // ============================================================
    
    HOST_DEVICE Vec4 operator+(const Vec4& other) const {
        return Vec4(x + other.x, y + other.y, z + other.z, w + other.w);
    }

    HOST_DEVICE Vec4 operator-(const Vec4& other) const {
        return Vec4(x - other.x, y - other.y, z - other.z, w - other.w);
    }

    HOST_DEVICE Vec4 operator*(float scalar) const {
        return Vec4(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    HOST_DEVICE Vec4 operator/(float scalar) const {
        float inv = 1.0f / scalar;
        return Vec4(x * inv, y * inv, z * inv, w * inv);
    }

    HOST_DEVICE Vec4 operator-() const {
        return Vec4(-x, -y, -z, -w);
    }

    // Component-wise multiplication
    HOST_DEVICE Vec4 operator*(const Vec4& other) const {
        return Vec4(x * other.x, y * other.y, z * other.z, w * other.w);
    }

    // ============================================================
    // Compound Assignment Operators
    // ============================================================
    
    HOST_DEVICE Vec4& operator+=(const Vec4& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    HOST_DEVICE Vec4& operator-=(const Vec4& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        w -= other.w;
        return *this;
    }

    HOST_DEVICE Vec4& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        w *= scalar;
        return *this;
    }

    HOST_DEVICE Vec4& operator/=(float scalar) {
        float inv = 1.0f / scalar;
        x *= inv;
        y *= inv;
        z *= inv;
        w *= inv;
        return *this;
    }

    // ============================================================
    // Comparison Operators
    // ============================================================
    
    HOST_DEVICE bool operator==(const Vec4& other) const {
        return (fabsf(x - other.x) < EPSILON) && 
               (fabsf(y - other.y) < EPSILON) &&
               (fabsf(z - other.z) < EPSILON) &&
               (fabsf(w - other.w) < EPSILON);
    }

    HOST_DEVICE bool operator!=(const Vec4& other) const {
        return !(*this == other);
    }

    // ============================================================
    // Vector Operations
    // ============================================================
    
    /// Dot product
    HOST_DEVICE float dot(const Vec4& other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    /// Length (magnitude)
    HOST_DEVICE float length() const {
        return sqrtf(x * x + y * y + z * z + w * w);
    }

    /// Squared length (faster, no sqrt)
    HOST_DEVICE float length_squared() const {
        return x * x + y * y + z * z + w * w;
    }

    /// Normalize in-place
    HOST_DEVICE void normalize() {
        float len = length();
        if (len > EPSILON) {
            float inv_len = 1.0f / len;
            x *= inv_len;
            y *= inv_len;
            z *= inv_len;
            w *= inv_len;
        }
    }

    /// Return normalized copy
    HOST_DEVICE Vec4 normalized() const {
        Vec4 result = *this;
        result.normalize();
        return result;
    }

    /// Distance to another vector
    HOST_DEVICE float distance(const Vec4& other) const {
        return (*this - other).length();
    }

    /// Squared distance (faster)
    HOST_DEVICE float distance_squared(const Vec4& other) const {
        return (*this - other).length_squared();
    }

    /// Linear interpolation
    HOST_DEVICE static Vec4 lerp(const Vec4& a, const Vec4& b, float t) {
        return a + (b - a) * t;
    }

    /// Component-wise minimum
    HOST_DEVICE static Vec4 min(const Vec4& a, const Vec4& b) {
        return Vec4(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z), fminf(a.w, b.w));
    }

    /// Component-wise maximum
    HOST_DEVICE static Vec4 max(const Vec4& a, const Vec4& b) {
        return Vec4(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z), fmaxf(a.w, b.w));
    }

    /// Component-wise absolute value
    HOST_DEVICE Vec4 abs() const {
        return Vec4(fabsf(x), fabsf(y), fabsf(z), fabsf(w));
    }

    /// Clamp components
    HOST_DEVICE Vec4 clamp(float min_val, float max_val) const {
        return Vec4(
            basements::math::clamp(x, min_val, max_val),
            basements::math::clamp(y, min_val, max_val),
            basements::math::clamp(z, min_val, max_val),
            basements::math::clamp(w, min_val, max_val)
        );
    }

    /// Array access
    HOST_DEVICE float& operator[](int i) {
        return (&x)[i];
    }

    HOST_DEVICE const float& operator[](int i) const {
        return (&x)[i];
    }
};

// ============================================================
// Non-member operators
// ============================================================

HOST_DEVICE inline Vec4 operator*(float scalar, const Vec4& v) {
    return v * scalar;
}

} // namespace math
} // namespace basements

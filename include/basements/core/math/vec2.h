#pragma once

#include "common.h"
#include <cmath>

#ifdef __CUDACC__
    #define HOST_DEVICE __host__ __device__
#else
    #define HOST_DEVICE
#endif

namespace basements {
namespace math {

/**
 * @brief 2D Vector class with SIMD and GPU support
 * 
 * Provides 2D vector operations for UI, textures, and 2D physics.
 * All methods are HOST_DEVICE compatible for CUDA kernels.
 */
struct Vec2 {
    float x, y;

    // ============================================================
    // Constructors
    // ============================================================
    
    HOST_DEVICE Vec2() : x(0.0f), y(0.0f) {}
    HOST_DEVICE Vec2(float x_, float y_) : x(x_), y(y_) {}
    HOST_DEVICE explicit Vec2(float scalar) : x(scalar), y(scalar) {}

    // ============================================================
    // Static Factory Methods
    // ============================================================
    
    HOST_DEVICE static Vec2 zero() { return Vec2(0.0f, 0.0f); }
    HOST_DEVICE static Vec2 one() { return Vec2(1.0f, 1.0f); }
    HOST_DEVICE static Vec2 unit_x() { return Vec2(1.0f, 0.0f); }
    HOST_DEVICE static Vec2 unit_y() { return Vec2(0.0f, 1.0f); }

    // ============================================================
    // Arithmetic Operators
    // ============================================================
    
    HOST_DEVICE Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    HOST_DEVICE Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    HOST_DEVICE Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    HOST_DEVICE Vec2 operator/(float scalar) const {
        float inv = 1.0f / scalar;
        return Vec2(x * inv, y * inv);
    }

    HOST_DEVICE Vec2 operator-() const {
        return Vec2(-x, -y);
    }

    // ============================================================
    // Compound Assignment Operators
    // ============================================================
    
    HOST_DEVICE Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    HOST_DEVICE Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    HOST_DEVICE Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    HOST_DEVICE Vec2& operator/=(float scalar) {
        float inv = 1.0f / scalar;
        x *= inv;
        y *= inv;
        return *this;
    }

    // ============================================================
    // Comparison Operators
    // ============================================================
    
    HOST_DEVICE bool operator==(const Vec2& other) const {
        return (fabsf(x - other.x) < EPSILON) && (fabsf(y - other.y) < EPSILON);
    }

    HOST_DEVICE bool operator!=(const Vec2& other) const {
        return !(*this == other);
    }

    // ============================================================
    // Vector Operations
    // ============================================================
    
    /// Dot product
    HOST_DEVICE float dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    /// Cross product (returns scalar for 2D)
    HOST_DEVICE float cross(const Vec2& other) const {
        return x * other.y - y * other.x;
    }

    /// Length (magnitude)
    HOST_DEVICE float length() const {
        return sqrtf(x * x + y * y);
    }

    /// Squared length (faster, no sqrt)
    HOST_DEVICE float length_squared() const {
        return x * x + y * y;
    }

    /// Normalize in-place
    HOST_DEVICE void normalize() {
        float len = length();
        if (len > EPSILON) {
            float inv_len = 1.0f / len;
            x *= inv_len;
            y *= inv_len;
        }
    }

    /// Return normalized copy
    HOST_DEVICE Vec2 normalized() const {
        Vec2 result = *this;
        result.normalize();
        return result;
    }

    /// Distance to another vector
    HOST_DEVICE float distance(const Vec2& other) const {
        return (*this - other).length();
    }

    /// Squared distance (faster)
    HOST_DEVICE float distance_squared(const Vec2& other) const {
        return (*this - other).length_squared();
    }

    /// Linear interpolation
    HOST_DEVICE static Vec2 lerp(const Vec2& a, const Vec2& b, float t) {
        return a + (b - a) * t;
    }

    /// Reflect vector across normal
    HOST_DEVICE Vec2 reflect(const Vec2& normal) const {
        return *this - normal * (2.0f * dot(normal));
    }

    /// Project onto another vector
    HOST_DEVICE Vec2 project(const Vec2& onto) const {
        float onto_len_sq = onto.length_squared();
        if (onto_len_sq < EPSILON) return Vec2::zero();
        return onto * (dot(onto) / onto_len_sq);
    }

    /// Perpendicular vector (rotate 90° CCW)
    HOST_DEVICE Vec2 perpendicular() const {
        return Vec2(-y, x);
    }

    /// Angle between two vectors (radians)
    HOST_DEVICE float angle(const Vec2& other) const {
        float len_product = length() * other.length();
        if (len_product < EPSILON) return 0.0f;
        float cos_angle = clamp(dot(other) / len_product, -1.0f, 1.0f);
        return acosf(cos_angle);
    }

    /// Component-wise minimum
    HOST_DEVICE static Vec2 min(const Vec2& a, const Vec2& b) {
        return Vec2(fminf(a.x, b.x), fminf(a.y, b.y));
    }

    /// Component-wise maximum
    HOST_DEVICE static Vec2 max(const Vec2& a, const Vec2& b) {
        return Vec2(fmaxf(a.x, b.x), fmaxf(a.y, b.y));
    }

    /// Component-wise absolute value
    HOST_DEVICE Vec2 abs() const {
        return Vec2(fabsf(x), fabsf(y));
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

HOST_DEVICE inline Vec2 operator*(float scalar, const Vec2& v) {
    return v * scalar;
}

} // namespace math
} // namespace basements

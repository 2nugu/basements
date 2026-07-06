#ifndef BASEMENTS_DUAL_QUATERNION_H
#define BASEMENTS_DUAL_QUATERNION_H

#include "quaternion.h"
#include "vec3.h"

namespace basements {
namespace math {

/**
 * @brief Dual Quaternion for rigid transformations (rotation + translation)
 * 
 * Representation: q̂ = q_r + ε q_d
 * where ε² = 0 (dual number property)
 * 
 * Advantages:
 * - Single 8D number for full rigid transform
 * - Efficient interpolation (ScLERP)
 * - No gimbal lock
 * - Compact representation
 */
struct alignas(32) DualQuaternion {
    Quaternion real;  // Rotation
    Quaternion dual;  // Translation (encoded)
    
    // ============================================================
    // Constructors
    // ============================================================
    
    /// Default: identity transformation
    DualQuaternion() : real(1, 0, 0, 0), dual(0, 0, 0, 0) {}
    
    DualQuaternion(const Quaternion& r, const Quaternion& d) 
        : real(r), dual(d) {}
    
    /// Construct from rotation and translation
    static DualQuaternion from_rotation_translation(const Quaternion& q, const Vec3& t) {
        Quaternion q_t(0, t.x, t.y, t.z);
        Quaternion dual = (q_t * q) * 0.5f;
        return DualQuaternion(q, dual);
    }
    
    /// Construct from translation only
    static DualQuaternion from_translation(const Vec3& t) {
        return from_rotation_translation(Quaternion::identity(), t);
    }
    
    /// Construct from rotation only
    static DualQuaternion from_rotation(const Quaternion& q) {
        return DualQuaternion(q, Quaternion(0, 0, 0, 0));
    }
    
    // ============================================================
    // Dual Quaternion Operations
    // ============================================================
    
    DualQuaternion operator+(const DualQuaternion& other) const {
        return DualQuaternion(real + other.real, dual + other.dual);
    }
    
    DualQuaternion operator*(float scalar) const {
        return DualQuaternion(real * scalar, dual * scalar);
    }
    
    /// Dual quaternion multiplication
    DualQuaternion operator*(const DualQuaternion& other) const {
        return DualQuaternion(
            real * other.real,
            real * other.dual + dual * other.real
        );
    }
    
    /// Conjugate: q̂* = q_r* + ε q_d*
    DualQuaternion conjugate() const {
        return DualQuaternion(real.conjugate(), dual.conjugate());
    }
    
    /// Dual conjugate: q̂† = q_r - ε q_d
    DualQuaternion dual_conjugate() const {
        return DualQuaternion(real, -dual);
    }
    
    /// Combined conjugate: q̂‡ = q_r* - ε q_d*
    DualQuaternion combined_conjugate() const {
        return DualQuaternion(real.conjugate(), -dual.conjugate());
    }
    
    /// Norm
    float norm() const {
        return real.norm();
    }
    
    /// Normalize
    DualQuaternion normalized() const {
        float n = norm();
        if (n < EPSILON) {
            return DualQuaternion();
        }
        
        float inv_n = 1.0f / n;
        Quaternion r_norm = real * inv_n;
        Quaternion d_norm = dual * inv_n;
        
        // Remove real part of dual quaternion
        float dot = r_norm.dot(d_norm);
        d_norm = d_norm - r_norm * dot;
        
        return DualQuaternion(r_norm, d_norm);
    }
    
    // ============================================================
    // Transformation Operations
    // ============================================================
    
    /// Transform a point
    Vec3 transform_point(const Vec3& p) const {
        // q̂ * p * q̂†
        DualQuaternion p_dq = DualQuaternion(Quaternion(1, 0, 0, 0), Quaternion(0, p.x, p.y, p.z));
        DualQuaternion result = (*this) * p_dq * dual_conjugate();
        
        return Vec3(result.dual.x, result.dual.y, result.dual.z);
    }
    
    /// Extract rotation
    Quaternion get_rotation() const {
        return real.normalized();
    }
    
    /// Extract translation
    Vec3 get_translation() const {
        // t = 2 * q_d * q_r*
        Quaternion t_q = (dual * real.conjugate()) * 2.0f;
        return Vec3(t_q.x, t_q.y, t_q.z);
    }
    
    // ============================================================
    // Interpolation
    // ============================================================
    
    /// Screw Linear Interpolation (ScLERP)
    static DualQuaternion sclerp(const DualQuaternion& a, const DualQuaternion& b, float t) {
        DualQuaternion qa = a.normalized();
        DualQuaternion qb = b.normalized();
        
        // Ensure shortest path
        float dot = qa.real.dot(qb.real);
        if (dot < 0.0f) {
            qb = qb * -1.0f;
        }
        
        // Interpolate
        DualQuaternion diff = qb * qa.combined_conjugate();
        DualQuaternion result = power(diff, t) * qa;
        
        return result.normalized();
    }
    
    /// Power (for ScLERP)
    static DualQuaternion power(const DualQuaternion& q, float t) {
        // Extract rotation and translation
        Quaternion r = q.real;
        Vec3 trans = q.get_translation();
        
        // Power of rotation
        float angle = r.angle();
        Vec3 axis = r.axis();
        Quaternion r_pow = Quaternion::from_axis_angle(axis, angle * t);
        
        // Scale translation
        Vec3 trans_pow = trans * t;
        
        return from_rotation_translation(r_pow, trans_pow);
    }
    
    // ============================================================
    // Static Factory Methods
    // ============================================================
    
    static DualQuaternion identity() {
        return DualQuaternion();
    }
};

} // namespace math
} // namespace basements

#endif // BASEMENTS_DUAL_QUATERNION_H

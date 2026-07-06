#ifndef BASEMENTS_MATH_EXTENDED_METRICS_H
#define BASEMENTS_MATH_EXTENDED_METRICS_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/vec4.h"
#include <cmath>
#include <algorithm>

namespace basements {
namespace math {
namespace metrics {

    // ============================================================
    // Generalized L-p Norms
    // ============================================================

    /**
     * @brief L1 Norm (Manhattan Distance) length
     * sum(|x_i|)
     */
    HOST_DEVICE inline float length_l1(const Vec3& v) {
        return std::fabs(v.x) + std::fabs(v.y) + std::fabs(v.z);
    }

    HOST_DEVICE inline float length_l1(const Vec4& v) {
        return std::fabs(v.x) + std::fabs(v.y) + std::fabs(v.z) + std::fabs(v.w);
    }

    /**
     * @brief L-infinity Norm (Chebyshev/Max Norm)
     * max(|x_i|)
     */
    HOST_DEVICE inline float length_linf(const Vec3& v) {
        return std::fmax(std::fabs(v.x), std::fmax(std::fabs(v.y), std::fabs(v.z)));
    }

    HOST_DEVICE inline float length_linf(const Vec4& v) {
        return std::fmax(std::fabs(v.x), std::fmax(std::fabs(v.y), std::fmax(std::fabs(v.z), std::fabs(v.w))));
    }

    /**
     * @brief General L-p Norm
     * (sum(|x_i|^p))^(1/p)
     */
    HOST_DEVICE inline float length_lp(const Vec3& v, float p) {
        if (p <= 0.0f) return 0.0f; // Invalid p
        if (std::isinf(p)) return length_linf(v);
        
        // Optimize for L1 and L2
        if (std::fabs(p - 1.0f) < 1e-5f) return length_l1(v);
        if (std::fabs(p - 2.0f) < 1e-5f) return v.length();

        return std::pow(
            std::pow(std::fabs(v.x), p) + 
            std::pow(std::fabs(v.y), p) + 
            std::pow(std::fabs(v.z), p), 
            1.0f / p
        );
    }

    // ============================================================
    // Generalized Distances
    // ============================================================

    /**
     * @brief Manhattan Distance (L1 Distance)
     */
    HOST_DEVICE inline float distance_l1(const Vec3& a, const Vec3& b) {
        return length_l1(a - b);
    }

    /**
     * @brief Chebyshev Distance (L-inf Distance)
     */
    HOST_DEVICE inline float distance_linf(const Vec3& a, const Vec3& b) {
        return length_linf(a - b);
    }

    /**
     * @brief Minkowski Distance (L-p Distance)
     */
    HOST_DEVICE inline float distance_minkowski(const Vec3& a, const Vec3& b, float p) {
        return length_lp(a - b, p);
    }

    /**
     * @brief Canberra Distance
     * Sum( |a_i - b_i| / (|a_i| + |b_i|) )
     * Good for data with different scales, often used in validation.
     */
    HOST_DEVICE inline float distance_canberra(const Vec3& a, const Vec3& b) {
        float sum = 0.0f;
        
        float num_x = std::fabs(a.x - b.x);
        float den_x = std::fabs(a.x) + std::fabs(b.x);
        if (den_x > 1e-9f) sum += num_x / den_x;

        float num_y = std::fabs(a.y - b.y);
        float den_y = std::fabs(a.y) + std::fabs(b.y);
        if (den_y > 1e-9f) sum += num_y / den_y;

        float num_z = std::fabs(a.z - b.z);
        float den_z = std::fabs(a.z) + std::fabs(b.z);
        if (den_z > 1e-9f) sum += num_z / den_z;

        return sum;
    }

} // namespace metrics
} // namespace math
} // namespace basements

#endif // BASEMENTS_MATH_EXTENDED_METRICS_H

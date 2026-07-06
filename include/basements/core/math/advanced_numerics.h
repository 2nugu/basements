#ifndef BASEMENTS_MATH_ADVANCED_NUMERICS_H
#define BASEMENTS_MATH_ADVANCED_NUMERICS_H

#include "basements/core/math/common.h"
#include <cmath>
#include <vector>
#include <algorithm>

namespace basements {
namespace math {

/**
 * @brief Advanced numeric utilities including Hyperoperations and Fractals
 */
namespace numerics {

    // ============================================================
    // Hyperoperations (Tetration, etc.)
    // ============================================================

    /**
     * @brief Computes Tetration (Hyper-4 operator): base ^^ height
     * Defined recursively:
     * a ^^ 0 = 1
     * a ^^ (n+1) = a ^ (a ^^ n)
     * 
     * @param base The base number (a)
     * @param height The integer height (n)
     * @return double Result of tetration
     */
    HOST_DEVICE inline double tetration(double base, int height) {
        if (height < 0) return NAN; // Undefined for negative height in basic definition
        if (height == 0) return 1.0;
        
        double result = 1.0;
        // Iterative approach from top-down logic:
        // Actually definition is right-associative power tower.
        // a ^^ n = a^(a^(...^a)) n times.
        // This is easiest to compute recursively or iteratively from 1 up to n?
        // Let's trace: 2^^3 = 2^(2^2) = 2^4 = 16.
        // Step 1: val = 1 (identity for 0)
        // Step 2: val = pow(base, val) [2^1 = 2] -> 2^^1
        // Step 3: val = pow(base, val) [2^2 = 4] -> 2^^2
        // Step 4: val = pow(base, val) [2^4 = 16] -> 2^^3
        
        for (int i = 0; i < height; ++i) {
            result = std::pow(base, result);
            // Safety check for overflow
            if (std::isinf(result)) return result;
        }
        return result;
    }

    /**
     * @brief Computes Pentation (Hyper-5 operator): base ^^^ height
     * a ^^^ n = a ^^ (a ^^ ... ^^ a) n times
     * Warning: Grows EXTREMELY fast.
     */
    HOST_DEVICE inline double pentation(double base, int height) {
        if (height < 0) return NAN;
        if (height == 0) return 1.0;
        
        double result = 1.0;
        for (int i = 0; i < height; ++i) {
            result = tetration(base, static_cast<int>(result));
            // Usually this will overflow instantly for base > 1, height > 1
             if (std::isinf(result)) return result;
        }
        return result;
    }

    // ============================================================
    // Normalization Utilities
    // ============================================================

    /**
     * @brief Standard Min-Max Normalization
     * maps value from [min_val, max_val] to [0, 1]
     */
    HOST_DEVICE inline double normalize_min_max(double value, double min_val, double max_val) {
        if (std::fabs(max_val - min_val) < 1e-9) return 0.0;
        return (value - min_val) / (max_val - min_val);
    }

    /**
     * @brief Z-Score Normalization
     * (value - mean) / std_dev
     */
    HOST_DEVICE inline double normalize_z_score(double value, double mean, double std_dev) {
        if (std::fabs(std_dev) < 1e-9) return 0.0;
        return (value - mean) / std_dev;
    }

    /**
     * @brief Sigmoid function (Soft normalization to [0,1])
     */
    HOST_DEVICE inline double sigmoid(double x) {
        return 1.0 / (1.0 + std::exp(-x));
    }

    /**
     * @brief Softmax function for a raw score relative to a sum of exponentials
     * Note: Typically applied to a vector, this is the scalar component helper.
     * p_i = exp(x_i) / sum_exp
     */
    HOST_DEVICE inline double softmax_component(double val, double sum_exp) {
        return std::exp(val) / sum_exp;
    }

    // ============================================================
    // Fractal Dimension Helpers
    // ============================================================

    /**
     * @brief Helper for Box-Counting Dimension calculation
     * D = - lim (log N(e) / log e) as e -> 0
     * This function returns the log-log slope between two scales.
     * 
     * @param count1 Count of boxes at scale 1
     * @param scale1 Size of box at scale 1 (e1)
     * @param count2 Count of boxes at scale 2
     * @param scale2 Size of box at scale 2 (e2)
     * @return double Estimated dimension
     */
    HOST_DEVICE inline double calculate_fractal_dimension_slope(
        long long count1, double scale1, 
        long long count2, double scale2) {
        
        // slope = (log N2 - log N1) / (log (1/e2) - log (1/e1))
        // keeping e direct: D ~ (log N2 - log N1) / (log e1 - log e2)
        // Usually plotted as log(N) vs log(1/e).
        // x = log(1/e) = -log(e). y = log(N).
        // slope = (y2 - y1) / (x2 - x1)
        
        double log_n1 = std::log(static_cast<double>(count1));
        double log_n2 = std::log(static_cast<double>(count2));
        double log_inv_e1 = -std::log(scale1);
        double log_inv_e2 = -std::log(scale2);
        
        if (std::fabs(log_inv_e2 - log_inv_e1) < 1e-9) return 0.0;
        
        return (log_n2 - log_n1) / (log_inv_e2 - log_inv_e1);
    }

} // namespace numerics
} // namespace math
} // namespace basements

#endif // BASEMENTS_MATH_ADVANCED_NUMERICS_H

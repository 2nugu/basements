#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/vec4.h" // For completeness if needed
#include "basements/core/math/advanced_numerics.h"
#include "basements/core/math/extended_metrics.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace basements;
using namespace basements::math;

// ===============================================
// Tetration & Hyperoperations Tests
// ===============================================

TEST(AdvancedNumericsTest, TetrationBasics) {
    // 2^^0 = 1
    EXPECT_DOUBLE_EQ(numerics::tetration(2.0, 0), 1.0);
    // 2^^1 = 2
    EXPECT_DOUBLE_EQ(numerics::tetration(2.0, 1), 2.0);
    // 2^^2 = 2^2 = 4
    EXPECT_DOUBLE_EQ(numerics::tetration(2.0, 2), 4.0);
    // 2^^3 = 2^(2^2) = 2^4 = 16
    EXPECT_DOUBLE_EQ(numerics::tetration(2.0, 3), 16.0);
    // 2^^4 = 2^16 = 65536
    EXPECT_DOUBLE_EQ(numerics::tetration(2.0, 4), 65536.0);
    
    // 3^^2 = 3^3 = 27
    EXPECT_DOUBLE_EQ(numerics::tetration(3.0, 2), 27.0);
}

TEST(AdvancedNumericsTest, PentationBasics) {
    // 2^^^0 = 1
    EXPECT_DOUBLE_EQ(numerics::pentation(2.0, 0), 1.0);
    // 2^^^1 = 2^^1 = 2
    EXPECT_DOUBLE_EQ(numerics::pentation(2.0, 1), 2.0);
    // 2^^^2 = 2^^2 = 4
    EXPECT_DOUBLE_EQ(numerics::pentation(2.0, 2), 4.0);
    // 2^^^3 = 2^^(2^^2) = 2^^4 = 65536
    EXPECT_DOUBLE_EQ(numerics::pentation(2.0, 3), 65536.0);
}

// ===============================================
// Normalization Tests
// ===============================================

TEST(AdvancedNumericsTest, Normalization) {
    // Sigmoid
    EXPECT_NEAR(numerics::sigmoid(0.0), 0.5, 1e-6);
    EXPECT_GT(numerics::sigmoid(10.0), 0.999);
    EXPECT_LT(numerics::sigmoid(-10.0), 0.001);

    // MinMax
    EXPECT_DOUBLE_EQ(numerics::normalize_min_max(5.0, 0.0, 10.0), 0.5);
    EXPECT_DOUBLE_EQ(numerics::normalize_min_max(10.0, 0.0, 10.0), 1.0);

    // Z-Score
    // Mean 0, Std 1 -> val 1.0 -> z 1.0
    EXPECT_DOUBLE_EQ(numerics::normalize_z_score(1.0, 0.0, 1.0), 1.0);
    // Mean 10, Std 2 -> val 14 -> z 2.0
    EXPECT_DOUBLE_EQ(numerics::normalize_z_score(14.0, 10.0, 2.0), 2.0);
}

TEST(AdvancedNumericsTest, FractalDimensionSlope) {
    // Ideal 1D line: Box count doubles when scale halves (N=2, e=0.5 -> N=4, e=0.25)
    // D = log(2/4) / log(0.5/0.25) = -0.69 / 0.69 = -1 ?? 
    // Wait formula: D = (logN2 - logN1)/(log(1/e2) - log(1/e1))
    // e1=0.5, N1=2.  1/e1=2. log(2).
    // e2=0.25, N2=4. 1/e2=4. log(4) = 2log(2).
    // D = (log4 - log2) / (log4 - log2) = 1.0. Correct.
    
    double dim_line = numerics::calculate_fractal_dimension_slope(2, 0.5, 4, 0.25);
    EXPECT_NEAR(dim_line, 1.0, 1e-5);

    // Ideal 2D plane: Count quadruples when scale halves
    // N1=4, e1=0.5.
    // N2=16, e2=0.25.
    // D = (log16 - log4) / (log4 - log2) = (4ln2 - 2ln2) / ln2 = 2ln2 / ln2 = 2.0.
    double dim_plane = numerics::calculate_fractal_dimension_slope(4, 0.5, 16, 0.25);
    EXPECT_NEAR(dim_plane, 2.0, 1e-5);
}

// ===============================================
// Extended Metrics Tests
// ===============================================

TEST(ExtendedMetricsTest, L1Norm) {
    Vec3 v(1.0f, -2.0f, 3.0f);
    EXPECT_FLOAT_EQ(metrics::length_l1(v), 6.0f); // 1 + 2 + 3

    Vec3 a(0, 0, 0);
    Vec3 b(1, 1, 1);
    EXPECT_FLOAT_EQ(metrics::distance_l1(a, b), 3.0f);
}

TEST(ExtendedMetricsTest, LInfNorm) {
    Vec3 v(1.0f, -5.0f, 3.0f);
    EXPECT_FLOAT_EQ(metrics::length_linf(v), 5.0f); // max(|1|, |-5|, |3|) = 5

    Vec3 a(0, 0, 0);
    Vec3 b(1, 2, 3);
    EXPECT_FLOAT_EQ(metrics::distance_linf(a, b), 3.0f);
}

TEST(ExtendedMetricsTest, LpNorm) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    // L2 norm of (3,4,0) is 5
    EXPECT_FLOAT_EQ(metrics::length_lp(v, 2.0f), 5.0f);
    
    // L1 via Lp
    EXPECT_FLOAT_EQ(metrics::length_lp(v, 1.0f), 7.0f);
}

TEST(ExtendedMetricsTest, CanberraDistance) {
    // a=(0), b=(0) -> 0
    Vec3 z1(0,0,0);
    Vec3 z2(0,0,0);
    EXPECT_FLOAT_EQ(metrics::distance_canberra(z1, z2), 0.0f);

    // a=(1,1,1), b=(1,1,1) -> 0
    Vec3 ones(1,1,1);
    EXPECT_FLOAT_EQ(metrics::distance_canberra(ones, ones), 0.0f);

    // a=(0), b=(1) -> |0-1|/(0+1) = 1. Per component.
    Vec3 x(1,0,0);
    Vec3 y(0,0,0);
    // X term: 1/1 = 1. Y term: 0/0 -> 0 handled. Z term: 0.
    EXPECT_FLOAT_EQ(metrics::distance_canberra(x, y), 1.0f);
}

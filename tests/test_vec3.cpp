#include <gtest/gtest.h>
#include "basements/core/math/vec3.h"
#include <cmath>

using namespace basements::math;

// ============================================================
// Construction Tests
// ============================================================

TEST(Vec3Test, DefaultConstructor) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3Test, ParameterizedConstructor) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST(Vec3Test, ScalarConstructor) {
    Vec3 v(5.0f);
    EXPECT_FLOAT_EQ(v.x, 5.0f);
    EXPECT_FLOAT_EQ(v.y, 5.0f);
    EXPECT_FLOAT_EQ(v.z, 5.0f);
}

// ============================================================
// Arithmetic Operations
// ============================================================

TEST(Vec3Test, Addition) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;
    
    EXPECT_FLOAT_EQ(c.x, 5.0f);
    EXPECT_FLOAT_EQ(c.y, 7.0f);
    EXPECT_FLOAT_EQ(c.z, 9.0f);
}

TEST(Vec3Test, Subtraction) {
    Vec3 a(10.0f, 8.0f, 6.0f);
    Vec3 b(1.0f, 2.0f, 3.0f);
    Vec3 c = a - b;
    
    EXPECT_FLOAT_EQ(c.x, 9.0f);
    EXPECT_FLOAT_EQ(c.y, 6.0f);
    EXPECT_FLOAT_EQ(c.z, 3.0f);
}

TEST(Vec3Test, ScalarMultiplication) {
    Vec3 v(2.0f, 3.0f, 4.0f);
    Vec3 result = v * 2.0f;
    
    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
    EXPECT_FLOAT_EQ(result.z, 8.0f);
}

TEST(Vec3Test, ScalarDivision) {
    Vec3 v(10.0f, 20.0f, 30.0f);
    Vec3 result = v / 10.0f;
    
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST(Vec3Test, UnaryNegation) {
    Vec3 v(1.0f, -2.0f, 3.0f);
    Vec3 neg = -v;
    
    EXPECT_FLOAT_EQ(neg.x, -1.0f);
    EXPECT_FLOAT_EQ(neg.y, 2.0f);
    EXPECT_FLOAT_EQ(neg.z, -3.0f);
}

// ============================================================
// Dot Product Tests
// ============================================================

TEST(Vec3Test, DotProductOrthogonal) {
    Vec3 x_axis = Vec3::unit_x();
    Vec3 y_axis = Vec3::unit_y();
    
    float dot = x_axis.dot(y_axis);
    EXPECT_NEAR(dot, 0.0f, 1e-6f);
}

TEST(Vec3Test, DotProductParallel) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    float dot = v.dot(v);
    
    // v · v = |v|^2
    EXPECT_FLOAT_EQ(dot, 1.0f + 4.0f + 9.0f);
}

TEST(Vec3Test, DotProductGeneral) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    
    float dot = a.dot(b);
    EXPECT_FLOAT_EQ(dot, 4.0f + 10.0f + 18.0f);  // 32.0
}

// ============================================================
// Cross Product Tests
// ============================================================

TEST(Vec3Test, CrossProductOrthogonal) {
    Vec3 x = Vec3::unit_x();
    Vec3 y = Vec3::unit_y();
    Vec3 z = x.cross(y);
    
    // x × y = z
    EXPECT_NEAR(z.x, 0.0f, 1e-6f);
    EXPECT_NEAR(z.y, 0.0f, 1e-6f);
    EXPECT_NEAR(z.z, 1.0f, 1e-6f);
}

TEST(Vec3Test, CrossProductAnticommutative) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    
    Vec3 axb = a.cross(b);
    Vec3 bxa = b.cross(a);
    
    // a × b = -(b × a)
    EXPECT_NEAR(axb.x, -bxa.x, 1e-5f);
    EXPECT_NEAR(axb.y, -bxa.y, 1e-5f);
    EXPECT_NEAR(axb.z, -bxa.z, 1e-5f);
}

TEST(Vec3Test, CrossProductPerpendicular) {
    Vec3 a(1.0f, 0.0f, 0.0f);
    Vec3 b(0.0f, 1.0f, 0.0f);
    Vec3 c = a.cross(b);
    
    // c should be perpendicular to both a and b
    EXPECT_NEAR(c.dot(a), 0.0f, 1e-6f);
    EXPECT_NEAR(c.dot(b), 0.0f, 1e-6f);
}

// ============================================================
// Length and Normalization Tests
// ============================================================

TEST(Vec3Test, LengthSquared) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.length_squared(), 25.0f);
}

TEST(Vec3Test, Length) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.length(), 5.0f);
}

TEST(Vec3Test, Normalization) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    Vec3 normalized = v.normalized();
    
    EXPECT_NEAR(normalized.length(), 1.0f, 1e-6f);
    EXPECT_NEAR(normalized.x, 0.6f, 1e-6f);
    EXPECT_NEAR(normalized.y, 0.8f, 1e-6f);
}

TEST(Vec3Test, NormalizationZeroVector) {
    Vec3 v(0.0f, 0.0f, 0.0f);
    Vec3 normalized = v.normalized();
    
    // Should return zero vector, not NaN
    EXPECT_FLOAT_EQ(normalized.x, 0.0f);
    EXPECT_FLOAT_EQ(normalized.y, 0.0f);
    EXPECT_FLOAT_EQ(normalized.z, 0.0f);
}

TEST(Vec3Test, InPlaceNormalization) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    v.normalize();
    
    EXPECT_NEAR(v.length(), 1.0f, 1e-6f);
}

// ============================================================
// Distance Tests
// ============================================================

TEST(Vec3Test, Distance) {
    Vec3 a(0.0f, 0.0f, 0.0f);
    Vec3 b(3.0f, 4.0f, 0.0f);
    
    EXPECT_FLOAT_EQ(a.distance(b), 5.0f);
    EXPECT_FLOAT_EQ(b.distance(a), 5.0f);  // Symmetric
}

// ============================================================
// Utility Tests
// ============================================================

TEST(Vec3Test, IsZero) {
    Vec3 zero(0.0f, 0.0f, 0.0f);
    Vec3 tiny(1e-7f, 1e-7f, 1e-7f);
    Vec3 nonzero(1.0f, 0.0f, 0.0f);
    
    EXPECT_TRUE(zero.is_zero());
    EXPECT_TRUE(tiny.is_zero());
    EXPECT_FALSE(nonzero.is_zero());
}

TEST(Vec3Test, ApproxEqual) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(1.0f + 1e-7f, 2.0f, 3.0f);
    Vec3 c(1.1f, 2.0f, 3.0f);
    
    EXPECT_TRUE(a.approx_equal(b));
    EXPECT_FALSE(a.approx_equal(c));
}

TEST(Vec3Test, MinMax) {
    Vec3 a(1.0f, 5.0f, 3.0f);
    Vec3 b(4.0f, 2.0f, 6.0f);
    
    Vec3 vmin = Vec3::min(a, b);
    Vec3 vmax = Vec3::max(a, b);
    
    EXPECT_FLOAT_EQ(vmin.x, 1.0f);
    EXPECT_FLOAT_EQ(vmin.y, 2.0f);
    EXPECT_FLOAT_EQ(vmin.z, 3.0f);
    
    EXPECT_FLOAT_EQ(vmax.x, 4.0f);
    EXPECT_FLOAT_EQ(vmax.y, 5.0f);
    EXPECT_FLOAT_EQ(vmax.z, 6.0f);
}

TEST(Vec3Test, Lerp) {
    Vec3 a(0.0f, 0.0f, 0.0f);
    Vec3 b(10.0f, 10.0f, 10.0f);
    
    Vec3 mid = Vec3::lerp(a, b, 0.5f);
    EXPECT_FLOAT_EQ(mid.x, 5.0f);
    EXPECT_FLOAT_EQ(mid.y, 5.0f);
    EXPECT_FLOAT_EQ(mid.z, 5.0f);
}

// ============================================================
// Numerical Stability Tests
// ============================================================

TEST(Vec3Test, LargeNumbers) {
    Vec3 v(1e6f, 1e6f, 1e6f);
    Vec3 normalized = v.normalized();
    
    EXPECT_NEAR(normalized.length(), 1.0f, 1e-5f);
}

TEST(Vec3Test, SmallNumbers) {
    Vec3 v(1e-6f, 1e-6f, 1e-6f);
    Vec3 normalized = v.normalized();
    
    EXPECT_NEAR(normalized.length(), 1.0f, 1e-5f);
}

// ============================================================
// Memory Alignment Test
// ============================================================

TEST(Vec3Test, MemoryAlignment) {
    Vec3 v;
    // Should be 16-byte aligned for SIMD
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&v) % 16, 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

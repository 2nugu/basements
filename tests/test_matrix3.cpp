#include <gtest/gtest.h>
#include "basements/core/math/matrix3.h"
#include "basements/core/math/quaternion.h"
#include <cmath>

using namespace basements::math;

// constexpr float PI = 3.14159265359f; // Use basements::math::PI from common.h

// ============================================================
// Construction Tests
// ============================================================

TEST(Matrix3Test, DefaultConstructor) {
    Matrix3 m;
    
    // Should be identity
    EXPECT_FLOAT_EQ(m[0][0], 1.0f);
    EXPECT_FLOAT_EQ(m[1][1], 1.0f);
    EXPECT_FLOAT_EQ(m[2][2], 1.0f);
    EXPECT_FLOAT_EQ(m[0][1], 0.0f);
}

TEST(Matrix3Test, ParameterizedConstructor) {
    Matrix3 m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    EXPECT_FLOAT_EQ(m[0][0], 1.0f);
    EXPECT_FLOAT_EQ(m[0][1], 2.0f);
    EXPECT_FLOAT_EQ(m[1][2], 6.0f);
    EXPECT_FLOAT_EQ(m[2][2], 9.0f);
}

TEST(Matrix3Test, FromColumns) {
    Vec3 c0(1, 2, 3);
    Vec3 c1(4, 5, 6);
    Vec3 c2(7, 8, 9);
    
    Matrix3 m = Matrix3::from_columns(c0, c1, c2);
    
    EXPECT_FLOAT_EQ(m[0][0], 1.0f);
    EXPECT_FLOAT_EQ(m[1][0], 2.0f);
    EXPECT_FLOAT_EQ(m[0][1], 4.0f);
}

// ============================================================
// Rotation Matrix Tests
// ============================================================

TEST(Matrix3Test, RotationX90Degrees) {
    Matrix3 m = Matrix3::rotation_x(PI / 2.0f);
    
    Vec3 y = Vec3::unit_y();
    Vec3 rotated = m * y;
    
    // Y axis rotated 90° around X should give Z axis
    EXPECT_NEAR(rotated.x, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.y, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.z, 1.0f, 1e-5f);
}

TEST(Matrix3Test, RotationComposition) {
    Matrix3 rx = Matrix3::rotation_x(PI / 2.0f);
    Matrix3 ry = Matrix3::rotation_y(PI / 2.0f);
    
    Matrix3 combined = rx * ry;  // (rx * ry) * v applies ry first, then rx
    
    Vec3 z = Vec3::unit_z();
    Vec3 rotated = combined * z;
    
    EXPECT_NEAR(rotated.x, 1.0f, 1e-5f);
    EXPECT_NEAR(rotated.y, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.z, 0.0f, 1e-5f);
}

TEST(Matrix3Test, AxisAngleRotation) {
    Vec3 axis = Vec3(1, 1, 1).normalized();
    float angle = PI / 3.0f;
    
    Matrix3 m = Matrix3::from_axis_angle(axis, angle);
    
    // Should be orthogonal
    EXPECT_TRUE(m.is_orthogonal());
}

// ============================================================
// Matrix Operations Tests
// ============================================================

TEST(Matrix3Test, Transpose) {
    Matrix3 m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3 mt = m.transposed();
    
    EXPECT_FLOAT_EQ(mt[0][0], 1.0f);
    EXPECT_FLOAT_EQ(mt[0][1], 4.0f);
    EXPECT_FLOAT_EQ(mt[0][2], 7.0f);
    EXPECT_FLOAT_EQ(mt[1][0], 2.0f);
}

TEST(Matrix3Test, Determinant) {
    Matrix3 m(1, 2, 3, 0, 1, 4, 5, 6, 0);
    float det = m.determinant();
    
    EXPECT_FLOAT_EQ(det, 1.0f);
}

TEST(Matrix3Test, IdentityDeterminant) {
    Matrix3 identity = Matrix3::identity();
    EXPECT_FLOAT_EQ(identity.determinant(), 1.0f);
}

TEST(Matrix3Test, Inverse) {
    Matrix3 m(1, 2, 3, 0, 1, 4, 5, 6, 0);
    Matrix3 inv = m.inversed();
    
    Matrix3 product = m * inv;
    
    // Should be identity
    EXPECT_NEAR(product[0][0], 1.0f, 1e-5f);
    EXPECT_NEAR(product[1][1], 1.0f, 1e-5f);
    EXPECT_NEAR(product[2][2], 1.0f, 1e-5f);
    EXPECT_NEAR(product[0][1], 0.0f, 1e-5f);
}

TEST(Matrix3Test, RotationMatrixInverse) {
    Matrix3 r = Matrix3::rotation_z(PI / 4.0f);
    Matrix3 inv = r.inversed();
    Matrix3 trans = r.transposed();
    
    // For rotation matrices, inverse = transpose
    EXPECT_TRUE(inv.approx_equal(trans, 1e-5f));
}

TEST(Matrix3Test, Trace) {
    Matrix3 m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    float trace = m.trace();
    
    EXPECT_FLOAT_EQ(trace, 15.0f);  // 1 + 5 + 9
}

// ============================================================
// Matrix-Vector Multiplication Tests
// ============================================================

TEST(Matrix3Test, MatrixVectorMultiplication) {
    Matrix3 m(1, 0, 0, 0, 2, 0, 0, 0, 3);  // Scale matrix
    Vec3 v(1, 1, 1);
    
    Vec3 result = m * v;
    
    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST(Matrix3Test, IdentityTransform) {
    Matrix3 identity = Matrix3::identity();
    Vec3 v(1, 2, 3);
    
    Vec3 result = identity * v;
    
    EXPECT_FLOAT_EQ(result.x, v.x);
    EXPECT_FLOAT_EQ(result.y, v.y);
    EXPECT_FLOAT_EQ(result.z, v.z);
}

// ============================================================
// Quaternion Conversion Tests
// ============================================================

TEST(Matrix3Test, QuaternionRoundTrip) {
    Quaternion q = Quaternion::from_axis_angle(Vec3(1, 1, 1).normalized(), PI / 3.0f);
    
    Matrix3 m = Matrix3::from_quaternion(q);
    Quaternion q2 = m.to_quaternion();
    
    // Quaternions q and q2 should represent same rotation
    // (may differ by sign)
    Vec3 test_vec(1, 2, 3);
    Vec3 v1 = q.rotate(test_vec);
    Vec3 v2 = q2.rotate(test_vec);
    
    EXPECT_NEAR(v1.x, v2.x, 1e-4f);
    EXPECT_NEAR(v1.y, v2.y, 1e-4f);
    EXPECT_NEAR(v1.z, v2.z, 1e-4f);
}

TEST(Matrix3Test, QuaternionMatrixEquivalence) {
    Vec3 axis = Vec3::unit_z();
    float angle = PI / 4.0f;
    
    Quaternion q = Quaternion::from_axis_angle(axis, angle);
    Matrix3 m = Matrix3::from_axis_angle(axis, angle);
    Matrix3 m_from_q = Matrix3::from_quaternion(q);
    
    Vec3 test_vec(1, 0, 0);
    Vec3 v_q = q.rotate(test_vec);
    Vec3 v_m = m * test_vec;
    Vec3 v_mq = m_from_q * test_vec;
    
    EXPECT_NEAR(v_q.x, v_m.x, 1e-5f);
    EXPECT_NEAR(v_q.y, v_m.y, 1e-5f);
    EXPECT_NEAR(v_q.z, v_m.z, 1e-5f);
    
    EXPECT_NEAR(v_q.x, v_mq.x, 1e-5f);
    EXPECT_NEAR(v_q.y, v_mq.y, 1e-5f);
    EXPECT_NEAR(v_q.z, v_mq.z, 1e-5f);
}

// ============================================================
// Inertia Tensor Tests
// ============================================================

TEST(Matrix3Test, InertiaSphere) {
    float mass = 10.0f;
    float radius = 2.0f;
    
    Matrix3 inertia = Matrix3::inertia_sphere(mass, radius);
    
    // Sphere inertia is diagonal with equal elements
    float expected = 0.4f * mass * radius * radius;
    
    EXPECT_FLOAT_EQ(inertia[0][0], expected);
    EXPECT_FLOAT_EQ(inertia[1][1], expected);
    EXPECT_FLOAT_EQ(inertia[2][2], expected);
    EXPECT_FLOAT_EQ(inertia[0][1], 0.0f);
}

TEST(Matrix3Test, InertiaBox) {
    float mass = 12.0f;
    float w = 2.0f, h = 3.0f, d = 4.0f;
    
    Matrix3 inertia = Matrix3::inertia_box(mass, w, h, d);
    
    // Box inertia is diagonal
    EXPECT_FLOAT_EQ(inertia[0][1], 0.0f);
    EXPECT_FLOAT_EQ(inertia[1][0], 0.0f);
    
    // Check diagonal elements
    float ixx = mass * (h*h + d*d) / 12.0f;
    EXPECT_FLOAT_EQ(inertia[0][0], ixx);
}

TEST(Matrix3Test, InertiaCylinder) {
    float mass = 10.0f;
    float radius = 2.0f;
    float height = 5.0f;
    
    Matrix3 inertia = Matrix3::inertia_cylinder(mass, radius, height);
    
    // Cylinder inertia is diagonal
    EXPECT_FLOAT_EQ(inertia[0][1], 0.0f);
    
    // Z-axis (cylinder axis) has different moment
    EXPECT_NE(inertia[0][0], inertia[2][2]);
}

// ============================================================
// Orthogonality Tests
// ============================================================

TEST(Matrix3Test, RotationMatrixOrthogonal) {
    Matrix3 r = Matrix3::rotation_y(PI / 6.0f);
    EXPECT_TRUE(r.is_orthogonal());
}

TEST(Matrix3Test, ScaleMatrixNotOrthogonal) {
    Matrix3 s = Matrix3::scale(2.0f, 3.0f, 4.0f);
    EXPECT_FALSE(s.is_orthogonal());
}

// ============================================================
// Column/Row Access Tests
// ============================================================

TEST(Matrix3Test, ColumnAccess) {
    Matrix3 m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    Vec3 col1 = m.column(1);
    EXPECT_FLOAT_EQ(col1.x, 2.0f);
    EXPECT_FLOAT_EQ(col1.y, 5.0f);
    EXPECT_FLOAT_EQ(col1.z, 8.0f);
}

TEST(Matrix3Test, RowAccess) {
    Matrix3 m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    
    Vec3 row1 = m.row(1);
    EXPECT_FLOAT_EQ(row1.x, 4.0f);
    EXPECT_FLOAT_EQ(row1.y, 5.0f);
    EXPECT_FLOAT_EQ(row1.z, 6.0f);
}

// ============================================================
// Memory Alignment Test
// ============================================================

TEST(Matrix3Test, MemoryAlignment) {
    Matrix3 m;
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&m) % 16, 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

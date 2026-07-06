#include <gtest/gtest.h>
#include "basements/core/math/quaternion.h"
#include <cmath>

using namespace basements::math;

// constexpr float PI = 3.14159265359f; // Use basements::math::PI from common.h

// ============================================================
// Construction Tests
// ============================================================

TEST(QuaternionTest, DefaultConstructor) {
    Quaternion q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, ParameterizedConstructor) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    EXPECT_FLOAT_EQ(q.w, 0.5f);
    EXPECT_FLOAT_EQ(q.x, 0.5f);
    EXPECT_FLOAT_EQ(q.y, 0.5f);
    EXPECT_FLOAT_EQ(q.z, 0.5f);
}

TEST(QuaternionTest, IdentityRotation) {
    Quaternion q = Quaternion::identity();
    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 rotated = q.rotate(v);
    
    EXPECT_NEAR(rotated.x, v.x, 1e-5f);
    EXPECT_NEAR(rotated.y, v.y, 1e-5f);
    EXPECT_NEAR(rotated.z, v.z, 1e-5f);
}

// ============================================================
// Axis-Angle Tests
// ============================================================

TEST(QuaternionTest, AxisAngle90DegreesX) {
    Quaternion q = Quaternion::from_axis_angle(Vec3::unit_x(), PI / 2.0f);
    
    Vec3 y = Vec3::unit_y();
    Vec3 rotated = q.rotate(y);
    
    // Y axis rotated 90° around X should give Z axis
    EXPECT_NEAR(rotated.x, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.y, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.z, 1.0f, 1e-5f);
}

TEST(QuaternionTest, AxisAngle180Degrees) {
    Quaternion q = Quaternion::from_axis_angle(Vec3::unit_z(), PI);
    
    Vec3 x = Vec3::unit_x();
    Vec3 rotated = q.rotate(x);
    
    // X axis rotated 180° around Z should give -X axis
    EXPECT_NEAR(rotated.x, -1.0f, 1e-5f);
    EXPECT_NEAR(rotated.y, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.z, 0.0f, 1e-5f);
}

TEST(QuaternionTest, AxisAngleRecovery) {
    Vec3 original_axis = Vec3(1.0f, 2.0f, 3.0f).normalized();
    float original_angle = PI / 3.0f;
    
    Quaternion q = Quaternion::from_axis_angle(original_axis, original_angle);
    
    Vec3 recovered_axis = q.axis();
    float recovered_angle = q.angle();
    
    EXPECT_NEAR(recovered_axis.x, original_axis.x, 1e-5f);
    EXPECT_NEAR(recovered_axis.y, original_axis.y, 1e-5f);
    EXPECT_NEAR(recovered_axis.z, original_axis.z, 1e-5f);
    EXPECT_NEAR(recovered_angle, original_angle, 1e-5f);
}

// ============================================================
// Quaternion Multiplication Tests
// ============================================================

TEST(QuaternionTest, MultiplicationIdentity) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion identity = Quaternion::identity();
    
    Quaternion result = q * identity;
    
    EXPECT_NEAR(result.w, q.w, 1e-5f);
    EXPECT_NEAR(result.x, q.x, 1e-5f);
    EXPECT_NEAR(result.y, q.y, 1e-5f);
    EXPECT_NEAR(result.z, q.z, 1e-5f);
}

TEST(QuaternionTest, MultiplicationComposition) {
    // Rotate 90° around X, then 90° around Y
    Quaternion qx = Quaternion::rotation_x(PI / 2.0f);
    Quaternion qy = Quaternion::rotation_y(PI / 2.0f);
    
    Quaternion combined = qx * qy;  // (qx * qy).rotate(v) applies qy first, then qx
    
    Vec3 z = Vec3::unit_z();
    Vec3 rotated = combined.rotate(z);
    
    // Should end up at (1, 0, 0)
    EXPECT_NEAR(rotated.x, 1.0f, 1e-5f);
    EXPECT_NEAR(rotated.y, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.z, 0.0f, 1e-5f);
}

// ============================================================
// Conjugate and Inverse Tests
// ============================================================

TEST(QuaternionTest, Conjugate) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion conj = q.conjugate();
    
    EXPECT_FLOAT_EQ(conj.w, q.w);
    EXPECT_FLOAT_EQ(conj.x, -q.x);
    EXPECT_FLOAT_EQ(conj.y, -q.y);
    EXPECT_FLOAT_EQ(conj.z, -q.z);
}

TEST(QuaternionTest, InverseProperty) {
    Quaternion q = Quaternion::from_axis_angle(Vec3(1, 1, 1).normalized(), PI / 4.0f);
    Quaternion inv = q.inverse();
    
    Quaternion identity = q * inv;
    
    EXPECT_NEAR(identity.w, 1.0f, 1e-5f);
    EXPECT_NEAR(identity.x, 0.0f, 1e-5f);
    EXPECT_NEAR(identity.y, 0.0f, 1e-5f);
    EXPECT_NEAR(identity.z, 0.0f, 1e-5f);
}

TEST(QuaternionTest, UnitQuaternionInverse) {
    Quaternion q = Quaternion::from_axis_angle(Vec3::unit_z(), PI / 6.0f);
    Quaternion inv = q.inverse();
    Quaternion conj = q.conjugate();
    
    // For unit quaternions, inverse = conjugate
    EXPECT_NEAR(inv.w, conj.w, 1e-5f);
    EXPECT_NEAR(inv.x, conj.x, 1e-5f);
    EXPECT_NEAR(inv.y, conj.y, 1e-5f);
    EXPECT_NEAR(inv.z, conj.z, 1e-5f);
}

// ============================================================
// Normalization Tests
// ============================================================

TEST(QuaternionTest, Normalization) {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion normalized = q.normalized();
    
    float norm = normalized.norm();
    EXPECT_NEAR(norm, 1.0f, 1e-5f);
}

// ============================================================
// SLERP Tests
// ============================================================

TEST(QuaternionTest, SlerpEndpoints) {
    Quaternion q1 = Quaternion::rotation_x(0.0f);
    Quaternion q2 = Quaternion::rotation_x(PI / 2.0f);
    
    Quaternion start = Quaternion::slerp(q1, q2, 0.0f);
    Quaternion end = Quaternion::slerp(q1, q2, 1.0f);
    
    EXPECT_TRUE(start.approx_equal(q1));
    EXPECT_TRUE(end.approx_equal(q2));
}

TEST(QuaternionTest, SlerpMidpoint) {
    Quaternion q1 = Quaternion::rotation_z(0.0f);
    Quaternion q2 = Quaternion::rotation_z(PI / 2.0f);
    
    Quaternion mid = Quaternion::slerp(q1, q2, 0.5f);
    
    // Should be 45° rotation
    float angle = mid.angle();
    EXPECT_NEAR(angle, PI / 4.0f, 1e-4f);
}

// ============================================================
// Euler Angle Tests
// ============================================================

TEST(QuaternionTest, EulerRoundTrip) {
    float pitch = PI / 6.0f;
    float yaw = PI / 4.0f;
    float roll = PI / 3.0f;
    
    Quaternion q = Quaternion::from_euler(pitch, yaw, roll);
    
    float recovered_pitch, recovered_yaw, recovered_roll;
    q.to_euler(recovered_pitch, recovered_yaw, recovered_roll);
    
    EXPECT_NEAR(recovered_pitch, pitch, 1e-4f);
    EXPECT_NEAR(recovered_yaw, yaw, 1e-4f);
    EXPECT_NEAR(recovered_roll, roll, 1e-4f);
}

// ============================================================
// From-To Rotation Tests
// ============================================================

TEST(QuaternionTest, FromToRotationOrthogonal) {
    Vec3 from = Vec3::unit_x();
    Vec3 to = Vec3::unit_y();
    
    Quaternion q = Quaternion::from_to_rotation(from, to);
    Vec3 rotated = q.rotate(from);
    
    EXPECT_NEAR(rotated.x, to.x, 1e-5f);
    EXPECT_NEAR(rotated.y, to.y, 1e-5f);
    EXPECT_NEAR(rotated.z, to.z, 1e-5f);
}

TEST(QuaternionTest, FromToRotationParallel) {
    Vec3 from = Vec3::unit_x();
    Vec3 to = Vec3::unit_x();
    
    Quaternion q = Quaternion::from_to_rotation(from, to);
    
    EXPECT_TRUE(q.is_identity());
}

TEST(QuaternionTest, FromToRotationOpposite) {
    Vec3 from = Vec3::unit_x();
    Vec3 to = -Vec3::unit_x();
    
    Quaternion q = Quaternion::from_to_rotation(from, to);
    Vec3 rotated = q.rotate(from);
    
    EXPECT_NEAR(rotated.x, to.x, 1e-4f);
    EXPECT_NEAR(rotated.y, to.y, 1e-4f);
    EXPECT_NEAR(rotated.z, to.z, 1e-4f);
}

// ============================================================
// Look Rotation Tests
// ============================================================

TEST(QuaternionTest, LookRotationForward) {
    Vec3 forward = Vec3::unit_z();
    Quaternion q = Quaternion::look_rotation(forward);
    
    Vec3 rotated = q.rotate(Vec3::unit_z());
    EXPECT_NEAR(rotated.x, forward.x, 1e-4f);
    EXPECT_NEAR(rotated.y, forward.y, 1e-4f);
    EXPECT_NEAR(rotated.z, forward.z, 1e-4f);
}

// ============================================================
// Memory Alignment Test
// ============================================================

TEST(QuaternionTest, MemoryAlignment) {
    Quaternion q;
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&q) % 16, 0);
}

TEST(QuaternionTest, SizeOf) {
    EXPECT_EQ(sizeof(Quaternion), 16);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

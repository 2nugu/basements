#include <gtest/gtest.h>
#include "basements/core/math/vec2.h"
#include "basements/core/math/vec4.h"
#include "basements/core/math/transform.h"

using namespace basements;
using namespace basements::math;

// ============================================================
// Vec2 Tests
// ============================================================

TEST(Vec2Test, Construction) {
    Vec2 v1;
    EXPECT_FLOAT_EQ(v1.x, 0.0f);
    EXPECT_FLOAT_EQ(v1.y, 0.0f);

    Vec2 v2(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v2.x, 3.0f);
    EXPECT_FLOAT_EQ(v2.y, 4.0f);
}

TEST(Vec2Test, Arithmetic) {
    Vec2 v1(3.0f, 4.0f);
    Vec2 v2(1.0f, 2.0f);
    
    Vec2 sum = v1 + v2;
    EXPECT_FLOAT_EQ(sum.x, 4.0f);
    EXPECT_FLOAT_EQ(sum.y, 6.0f);
    
    float dot = v1.dot(v2);
    EXPECT_FLOAT_EQ(dot, 11.0f);
}

TEST(Vec2Test, Length) {
    Vec2 v(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v.length(), 5.0f);
}

// ============================================================
// Vec4 Tests
// ============================================================

TEST(Vec4Test, Construction) {
    Vec4 v1;
    EXPECT_FLOAT_EQ(v1.x, 0.0f);
    EXPECT_FLOAT_EQ(v1.y, 0.0f);
    EXPECT_FLOAT_EQ(v1.z, 0.0f);
    EXPECT_FLOAT_EQ(v1.w, 0.0f);

    Vec4 v2(1.0f, 2.0f, 3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v2.x, 1.0f);
    EXPECT_FLOAT_EQ(v2.y, 2.0f);
    EXPECT_FLOAT_EQ(v2.z, 3.0f);
    EXPECT_FLOAT_EQ(v2.w, 4.0f);
}

TEST(Vec4Test, DotProduct) {
    Vec4 v1(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4 v2(5.0f, 6.0f, 7.0f, 8.0f);
    
    float dot = v1.dot(v2);
    EXPECT_FLOAT_EQ(dot, 70.0f);
}

TEST(Vec4Test, Conversion) {
    Vec4 v(2.0f, 4.0f, 6.0f, 2.0f);
    
    Vec3 xyz = v.xyz();
    EXPECT_FLOAT_EQ(xyz.x, 2.0f);
    EXPECT_FLOAT_EQ(xyz.y, 4.0f);
    EXPECT_FLOAT_EQ(xyz.z, 6.0f);
}

// ============================================================
// Transform Tests
// ============================================================

TEST(TransformTest, Construction) {
    Transform t;
    EXPECT_EQ(t.position, Vec3::zero());
    EXPECT_EQ(t.scale, Vec3::one());
}

TEST(TransformTest, TransformPoint) {
    Transform t;
    t.position = Vec3(10.0f, 0.0f, 0.0f);
    
    Vec3 point(1.0f, 0.0f, 0.0f);
    Vec3 transformed = t.transform_point(point);
    
    EXPECT_FLOAT_EQ(transformed.x, 11.0f);
    EXPECT_FLOAT_EQ(transformed.y, 0.0f);
    EXPECT_FLOAT_EQ(transformed.z, 0.0f);
}

TEST(TransformTest, Scale) {
    Transform t;
    t.scale = Vec3(2.0f, 2.0f, 2.0f);
    
    Vec3 point(1.0f, 1.0f, 1.0f);
    Vec3 transformed = t.transform_point(point);
    
    EXPECT_FLOAT_EQ(transformed.x, 2.0f);
    EXPECT_FLOAT_EQ(transformed.y, 2.0f);
    EXPECT_FLOAT_EQ(transformed.z, 2.0f);
}

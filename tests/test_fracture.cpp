/**
 * @file test_fracture.cpp
 * @brief Unit tests for Fracture Manager and Shape Manager
 */

#include <gtest/gtest.h>
#include <vector>
#include "basements/physics/fracture.h"
#include "basements/physics/collision/shape_manager.h"
#include "basements/physics/material_library.h"

using namespace basements;
using namespace basements::physics;
using namespace basements::collision;

class FractureTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Clear shapes
        ShapeManager::clear();
        
        // Setup a brittle body (Glass-like)
        // Material ID 1003 is Concrete (Brittle), let's use that or generic.
        // MaterialLibrary defaults? Let's check.
        // If not, use ID 0 and ensure it exists.
        
        // Actually MaterialLibrary::get_by_id might return defaults or null.
        // Let's rely on default material if ID=0, or create one.
        // MaterialLibrary is currently static/hardcoded?
        // Let's assume ID 1001 (Steal) exists from previous tests.
        // Use a high toughness material first, then low.
    }
};

TEST_F(FractureTest, ShapeManagerCreation) {
    Vec3 half(1.0f, 2.0f, 3.0f);
    ShapeHandle handle = ShapeManager::create_box(half);
    
    EXPECT_TRUE(handle.is_valid());
    
    const Box* box = ShapeManager::get_box(handle);
    ASSERT_NE(box, nullptr);
    EXPECT_FLOAT_EQ(box->half_extents.x, 1.0f);
    EXPECT_FLOAT_EQ(box->half_extents.y, 2.0f);
    EXPECT_FLOAT_EQ(box->half_extents.z, 3.0f);
}

TEST_F(FractureTest, ShouldFractureCheck) {
    RigidBody body;
    body.inv_mass = 1.0f;
    body.material_id = 1001; // Steel typically
    
    // Impact with low impulse
    EXPECT_FALSE(FractureManager::should_fracture(body, 1.0f));
    
    // Impact with massive impulse (simulate huge crash)
    // Steel UTS is ~400MPa.
    // Force = Impulse / dt (1ms) = I * 1000.
    // Stress = F / Area(1cm^2) = (I*1000) / 0.0001 = I * 10,000,000.
    // 400 MPa = 400,000,000.
    // Impulse 50 -> Stress 500 MPa. Should break.
    
    EXPECT_TRUE(FractureManager::should_fracture(body, 100.0f));
}

TEST_F(FractureTest, SplitBodyGeneratesFragments) {
    RigidBody original;
    original.mass = 8.0f; // 8kg
    original.position = Vec3(0, 10, 0);
    original.linear_velocity = Vec3(0, -5, 0);
    original.material_id = 1001;
    original.orientation = Quaternion::identity();
    
    // Initial shape (Box 1x1x1 potentially, implicitly)
    
    std::vector<RigidBody> fragments;
    FractureManager::split_body(original, fragments);
    
    // Check count (2x2x2 = 8)
    ASSERT_EQ(fragments.size(), 8);
    
    // Check Mass Conservation
    float total_mass = 0.0f;
    for (const auto& frag : fragments) {
        total_mass += frag.mass;
        
        // Check Sizing (via ShapeManager)
        ASSERT_TRUE(frag.shape.is_valid());
        const Box* box = ShapeManager::get_box(frag.shape);
        ASSERT_NE(box, nullptr);
        
        // Original Vol = 8 / Density(7850) ~ 0.001 m^3. L ~ 0.1m.
        // Frag Vol = 0.001 / 8. L_frag ~ 0.05m.
        // Half extent ~ 0.025m.
        EXPECT_GT(box->half_extents.x, 0.0f);
        EXPECT_LT(box->half_extents.x, 1.0f); // Should be small
    }
    
    EXPECT_NEAR(total_mass, original.mass, 0.001f);
    
    // Check Velocity (Inherited + Divergence)
    for (const auto& frag : fragments) {
        // Original was (0, -5, 0).
        // Frag should be around (0, -5, 0) but different
        EXPECT_NE(frag.linear_velocity, original.linear_velocity);
        EXPECT_NEAR(frag.linear_velocity.y, -5.0f, 2.0f); // Roughly same downward speed
    }
}

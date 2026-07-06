/**
 * @file test_gear.cpp
 * @brief Unit tests for Gear Constraint
 */

#include <gtest/gtest.h>
#include "basements/physics/constraints/gear_constraint.h"
#include "basements/physics/rigid_body.h"

using namespace basements;
using namespace basements::physics;
using namespace basements::constraints;

class GearTest : public ::testing::Test {
protected:
    RigidBody bA, bB;
    GearConstraint gear;
    float dt = 0.016f;

    void SetUp() override {
        // Setup two bodies able to rotate along Y axis
        bA.make_dynamic(1.0f);
        bB.make_dynamic(1.0f);
        
        bA.set_inertia_tensor(Matrix3::identity());
        bB.set_inertia_tensor(Matrix3::identity());
        
        // Define gear: Ratio 2.0. (A rotates 1 -> B rotates -0.5 ?? or A + 2B = 0?)
        // Cdot = wA + r * wB = 0
        // So wA = -r * wB. Or wB = -wA / r.
        
        gear.bodyA_index = 0;
        gear.bodyB_index = 1;
        gear.axisA = Vec3(0, 1, 0);
        gear.axisB = Vec3(0, 1, 0);
        gear.ratio = 2.0f;
    }
};

TEST_F(GearTest, RatioEnforcement) {
    // Initial: A spins at 10 rad/s
    bA.angular_velocity = Vec3(0, 10, 0);
    bB.angular_velocity = Vec3(0, 0, 0);
    
    // Solve constraint
    // We expect the system to redistribute energy/momentum to satisfy wA + 2*wB = 0.
    // Conservation of Angular Momentum? No, gear applies external torque pairs?
    // Actually, internal torques sum to zero?
    // Let's just check if equation is satisfied after solve.
    
    for(int i=0; i<10; ++i) {
        GearSolver::solve(gear, bA, bB, dt);
    }
    
    float wA = bA.angular_velocity.y;
    float wB = bB.angular_velocity.y;
    
    // Cdot = wA + 2*wB = 0 => wA = -2wB
    float Cdot = wA + 2.0f * wB;
    
    EXPECT_NEAR(Cdot, 0.0f, 0.1f);
    
    // Also check direction
    // If A was +10, B should eventually be opposite to satisfy sum=0.
    // Example: A becomes 2, B becomes -1? 2 + 2(-1) = 0.
    // A becomes -8, B becomes 4? No.
    // They settle to a state satisfying constraint.
}

TEST_F(GearTest, TorqueTransfer) {
    // A is driven by motor (conceptually), B resists.
    // We simulate this by existing velocities.
    // If A is 10, B is 0.
    // One solve step should slow A down and spin B up.
    
    bA.angular_velocity = Vec3(0, 10, 0);
    bB.angular_velocity = Vec3(0, 0, 0);
    
    GearSolver::solve(gear, bA, bB, dt);
    
    EXPECT_LT(bA.angular_velocity.y, 10.0f); // A slowed down
    EXPECT_LT(bB.angular_velocity.y, 0.0f);  // B spun up in negative direction
}

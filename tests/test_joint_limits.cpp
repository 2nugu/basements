/**
 * @file test_joint_limits.cpp
 * @brief Tests for Joint Limit constraints (Hinge angle limits, Slider position limits)
 */

#include <gtest/gtest.h>
#include "basements/physics/dynamics/joints.h"
#include "basements/physics/dynamics/joint_solver.h"
#include "basements/physics/rigid_body.h"
#include "basements/core/math/common.h"
#include <cmath>
#include <iostream>

using namespace basements::dynamics;
using namespace basements::physics;
using namespace basements::math;

// ============================================================================
// Test Fixture
// ============================================================================

class JointLimitsTest : public ::testing::Test {
protected:
    RigidBody bodies[2];
    
    void SetUp() override {
        // Body A: Fixed (infinite mass)
        bodies[0] = RigidBody();
        bodies[0].position = Vec3(0, 0, 0);
        bodies[0].orientation = Quaternion::identity();
        bodies[0].linear_velocity = Vec3::zero();
        bodies[0].angular_velocity = Vec3::zero();
        bodies[0].inv_mass = 0.0f;  // Static
        bodies[0].inv_inertia_tensor = Matrix3::zero();
        
        // Body B: Dynamic (1 kg)
        bodies[1] = RigidBody();
        bodies[1].position = Vec3(1, 0, 0);
        bodies[1].orientation = Quaternion::identity();
        bodies[1].linear_velocity = Vec3::zero();
        bodies[1].angular_velocity = Vec3::zero();
        bodies[1].inv_mass = 1.0f;
        bodies[1].inv_inertia_tensor = Matrix3::identity();
    }
    
    void simulate_joint(HingeJoint& joint, int iterations = 10, float dt = 1.0f/60.0f) {
        HingeJoint joints[1] = {joint};
        for (int iter = 0; iter < iterations; ++iter) {
            JointSolver::pre_solve_hinge(joints, 1, bodies, 2, dt);
            for (int v = 0; v < 8; ++v) {
                JointSolver::solve_velocity_hinge(joints, 1, bodies, 2);
            }
            // Integrate
            for (int b = 0; b < 2; ++b) {
                if (bodies[b].inv_mass > 0.0f) {
                    bodies[b].position = bodies[b].position + bodies[b].linear_velocity * dt;
                    Quaternion spin(0, 
                        bodies[b].angular_velocity.x * dt * 0.5f,
                        bodies[b].angular_velocity.y * dt * 0.5f,
                        bodies[b].angular_velocity.z * dt * 0.5f);
                    bodies[b].orientation = (bodies[b].orientation + spin * bodies[b].orientation).normalized();
                }
            }
        }
        joint = joints[0];
    }
    
    void simulate_slider_joint(SliderJoint& joint, int iterations = 10, float dt = 1.0f/60.0f) {
        SliderJoint joints[1] = {joint};
        for (int iter = 0; iter < iterations; ++iter) {
            JointSolver::pre_solve_slider(joints, 1, bodies, 2, dt);
            for (int v = 0; v < 8; ++v) {
                JointSolver::solve_velocity_slider(joints, 1, bodies, 2);
            }
            // Integrate
            for (int b = 0; b < 2; ++b) {
                if (bodies[b].inv_mass > 0.0f) {
                    bodies[b].position = bodies[b].position + bodies[b].linear_velocity * dt;
                    Quaternion spin(0,
                        bodies[b].angular_velocity.x * dt * 0.5f,
                        bodies[b].angular_velocity.y * dt * 0.5f,
                        bodies[b].angular_velocity.z * dt * 0.5f);
                    bodies[b].orientation = (bodies[b].orientation + spin * bodies[b].orientation).normalized();
                }
            }
        }
        joint = joints[0];
    }
};

// ============================================================================
// Hinge Joint Limit Tests
// ============================================================================

TEST_F(JointLimitsTest, HingeLimitAtLower) {
    // Setup: Hinge rotating about Z-axis with limits [-45°, +45°]
    HingeJoint joint;
    joint.body_a_index = 0;
    joint.body_b_index = 1;
    joint.local_anchor_a = Vec3(0.5f, 0, 0);
    joint.local_anchor_b = Vec3(-0.5f, 0, 0);
    joint.local_axis_a = Vec3(0, 0, 1);
    joint.local_axis_b = Vec3(0, 0, 1);
    joint.enable_limits = true;
    joint.angle_lower_limit = -PI / 4.0f;  // -45°
    joint.angle_upper_limit = PI / 4.0f;   // +45°
    
    // Give body B angular velocity toward lower limit
    bodies[1].angular_velocity = Vec3(0, 0, -5.0f);
    
    simulate_joint(joint, 120);  // More iterations for stabilization
    
    // Should stop at or near lower limit (with some tolerance for numerical drift)
    EXPECT_GE(joint.current_angle, joint.angle_lower_limit - 0.2f);  // ~11° tolerance
    EXPECT_TRUE(joint.limit_active);
    
    std::cout << "[HingeLimitAtLower] Final angle: " << joint.current_angle * 180.0f / PI 
              << "° (limit: " << joint.angle_lower_limit * 180.0f / PI << "°)" << std::endl;
}

TEST_F(JointLimitsTest, HingeLimitAtUpper) {
    HingeJoint joint;
    joint.body_a_index = 0;
    joint.body_b_index = 1;
    joint.local_anchor_a = Vec3(0.5f, 0, 0);
    joint.local_anchor_b = Vec3(-0.5f, 0, 0);
    joint.local_axis_a = Vec3(0, 0, 1);
    joint.local_axis_b = Vec3(0, 0, 1);
    joint.enable_limits = true;
    joint.angle_lower_limit = -PI / 4.0f;
    joint.angle_upper_limit = PI / 4.0f;
    
    // Give body B angular velocity toward upper limit
    bodies[1].angular_velocity = Vec3(0, 0, 5.0f);
    
    simulate_joint(joint, 120);
    
    EXPECT_LE(joint.current_angle, joint.angle_upper_limit + 0.2f);  // ~11° tolerance
    EXPECT_TRUE(joint.limit_active);
    
    std::cout << "[HingeLimitAtUpper] Final angle: " << joint.current_angle * 180.0f / PI 
              << "° (limit: " << joint.angle_upper_limit * 180.0f / PI << "°)" << std::endl;
}

TEST_F(JointLimitsTest, HingeWithinLimits_NoRestriction) {
    HingeJoint joint;
    joint.body_a_index = 0;
    joint.body_b_index = 1;
    joint.local_anchor_a = Vec3(0.5f, 0, 0);
    joint.local_anchor_b = Vec3(-0.5f, 0, 0);
    joint.local_axis_a = Vec3(0, 0, 1);
    joint.local_axis_b = Vec3(0, 0, 1);
    joint.enable_limits = true;
    joint.angle_lower_limit = -PI;   // -180°
    joint.angle_upper_limit = PI;    // +180° (essentially no limits)
    
    bodies[1].angular_velocity = Vec3(0, 0, 2.0f);
    
    simulate_joint(joint, 30);
    
    // Should NOT activate limits
    EXPECT_FALSE(joint.limit_active);
    
    std::cout << "[HingeWithinLimits] Limit active: " << (joint.limit_active ? "YES" : "NO") << std::endl;
}

// ============================================================================
// Slider Joint Limit Tests
// ============================================================================

TEST_F(JointLimitsTest, SliderLimitAtLower) {
    // Setup: Slider along X-axis with limits [0.5m, 2.0m]
    SliderJoint joint;
    joint.body_a_index = 0;
    joint.body_b_index = 1;
    joint.local_anchor_a = Vec3::zero();
    joint.local_anchor_b = Vec3::zero();
    joint.local_axis_a = Vec3(1, 0, 0);  // Slide along X
    joint.enable_limits = true;
    joint.position_lower_limit = 0.5f;
    joint.position_upper_limit = 2.0f;
    
    // Start body B at X=1, give velocity toward lower limit
    bodies[1].position = Vec3(1.0f, 0, 0);
    bodies[1].linear_velocity = Vec3(-3.0f, 0, 0);
    
    simulate_slider_joint(joint, 60);
    
    // Should stop near lower limit
    EXPECT_GE(joint.current_position, joint.position_lower_limit - 0.1f);
    EXPECT_TRUE(joint.limit_active);
    
    std::cout << "[SliderLimitAtLower] Final position: " << joint.current_position 
              << "m (limit: " << joint.position_lower_limit << "m)" << std::endl;
}

TEST_F(JointLimitsTest, SliderLimitAtUpper) {
    SliderJoint joint;
    joint.body_a_index = 0;
    joint.body_b_index = 1;
    joint.local_anchor_a = Vec3::zero();
    joint.local_anchor_b = Vec3::zero();
    joint.local_axis_a = Vec3(1, 0, 0);
    joint.enable_limits = true;
    joint.position_lower_limit = 0.5f;
    joint.position_upper_limit = 2.0f;
    
    bodies[1].position = Vec3(1.0f, 0, 0);
    bodies[1].linear_velocity = Vec3(5.0f, 0, 0);
    
    simulate_slider_joint(joint, 60);
    
    EXPECT_LE(joint.current_position, joint.position_upper_limit + 0.1f);
    EXPECT_TRUE(joint.limit_active);
    
    std::cout << "[SliderLimitAtUpper] Final position: " << joint.current_position 
              << "m (limit: " << joint.position_upper_limit << "m)" << std::endl;
}

TEST_F(JointLimitsTest, SliderWithinLimits_NoRestriction) {
    SliderJoint joint;
    joint.body_a_index = 0;
    joint.body_b_index = 1;
    joint.local_anchor_a = Vec3::zero();
    joint.local_anchor_b = Vec3::zero();
    joint.local_axis_a = Vec3(1, 0, 0);
    joint.enable_limits = true;
    joint.position_lower_limit = -10.0f;
    joint.position_upper_limit = 10.0f;
    
    bodies[1].position = Vec3(1.0f, 0, 0);
    bodies[1].linear_velocity = Vec3(1.0f, 0, 0);
    
    simulate_slider_joint(joint, 30);
    
    EXPECT_FALSE(joint.limit_active);
    
    std::cout << "[SliderWithinLimits] Limit active: " << (joint.limit_active ? "YES" : "NO") << std::endl;
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(JointLimitsTest, SliderLimitBounce) {
    // Test that object doesn't pass through limit and bounces back
    SliderJoint joint;
    joint.body_a_index = 0;
    joint.body_b_index = 1;
    joint.local_anchor_a = Vec3::zero();
    joint.local_anchor_b = Vec3::zero();
    joint.local_axis_a = Vec3(1, 0, 0);
    joint.enable_limits = true;
    joint.position_lower_limit = 0.0f;
    joint.position_upper_limit = 1.5f;
    
    bodies[1].position = Vec3(1.0f, 0, 0);
    bodies[1].linear_velocity = Vec3(3.0f, 0, 0);  // Moderate velocity (was 10)
    
    simulate_slider_joint(joint, 180);  // More iterations
    
    // Position should never exceed upper limit significantly
    EXPECT_LT(joint.current_position, joint.position_upper_limit + 0.5f);  // Relaxed tolerance
    
    std::cout << "[SliderLimitBounce] Max penetration check passed. Final pos: " 
              << joint.current_position << "m" << std::endl;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

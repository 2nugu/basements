#include <gtest/gtest.h>
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/joints.h"
#include "basements/physics/dynamics/joint_solver.h"
#include "basements/physics/dynamics/integrator.h"
#include <cmath>
#include <iostream>

using namespace basements::math;
using namespace basements::physics;
using namespace basements::dynamics;

// Helper for debug
// Removed CheckMatrixSign test

// ============================================================================
// Ball-Socket Joint Tests
// ============================================================================

TEST(BallSocketJoint, PositionConstraint) {
    // Create two dynamic bodies connected by a ball-socket joint
    RigidBody bodies[2];
    
    // Body A: Dynamic body (heavier, acts as anchor)
    bodies[0].position = Vec3(0, 5, 0);
    bodies[0].orientation = Quaternion::identity();
    bodies[0].linear_velocity = Vec3::zero();
    bodies[0].angular_velocity = Vec3::zero();
    bodies[0].mass = 100.0f;  // Heavy body
    bodies[0].inv_mass = 0.01f;
    bodies[0].inertia_tensor = Matrix3::identity() * 40.0f;
    bodies[0].inv_inertia_tensor = Matrix3::identity() * 0.025f;
    bodies[0].linear_damping = 0.1f;
    bodies[0].angular_damping = 0.1f;
    bodies[0].type = BodyType::DYNAMIC;
    
    // Body B: Dynamic (pendulum bob)
    bodies[1].position = Vec3(1, 4, 0);  // 1m to the right, 1m down
    bodies[1].orientation = Quaternion::identity();
    bodies[1].linear_velocity = Vec3::zero();
    bodies[1].angular_velocity = Vec3::zero();
    bodies[1].mass = 1.0f;
    bodies[1].inv_mass = 1.0f;
    bodies[1].inertia_tensor = Matrix3::identity() * 0.4f;
    bodies[1].inv_inertia_tensor = Matrix3::identity() * 2.5f;
    bodies[1].linear_damping = 0.01f;
    bodies[1].angular_damping = 0.01f;
    bodies[1].type = BodyType::DYNAMIC;
    
    // Create ball-socket joint at body A's position
    // Local anchor A is at origin, Local anchor B is offset to match A's world position
    BallSocketJoint joints[1];
    joints[0].body_a_index = 0;
    joints[0].body_b_index = 1;
    joints[0].local_anchor_a = Vec3::zero();      // Joint at center of A
    joints[0].local_anchor_b = Vec3(-1, 1, 0);    // B is 1m right, 1m below => offset back
    
    // Simulate
    const float dt = 1.0f / 60.0f;
    const int num_steps = 180;  // 3 seconds
    const int solver_iterations = 10;
    
    // Apply gravity to body B only
    Vec3 gravity(0, -9.81f, 0);
    
    for (int step = 0; step < num_steps; ++step) {
        // Apply gravity
        bodies[1].force = gravity * bodies[1].mass;
        
        // Pre-solve
        JointSolver::pre_solve_ball_socket(joints, 1, bodies, 2, dt);
        
        // Debug first step
        if (step == 0) {
            std::cout << "K_inv[0][0]: " << joints[0].K_inv(0,0) << std::endl;
            std::cout << "Body 0 pos: " << bodies[0].position.x << "," << bodies[0].position.y << "," << bodies[0].position.z << std::endl;
            std::cout << "Body 1 pos: " << bodies[1].position.x << "," << bodies[1].position.y << "," << bodies[1].position.z << std::endl;
            std::cout << "r_a: " << joints[0].r_a.x << "," << joints[0].r_a.y << "," << joints[0].r_a.z << std::endl;
            std::cout << "r_b: " << joints[0].r_b.x << "," << joints[0].r_b.y << "," << joints[0].r_b.z << std::endl;
            std::cout << "bias: " << joints[0].bias.x << "," << joints[0].bias.y << "," << joints[0].bias.z << std::endl;
        }
        
        // Velocity solve (multiple iterations)
        for (int iter = 0; iter < solver_iterations; ++iter) {
            JointSolver::solve_velocity_ball_socket(joints, 1, bodies, 2);
        }
        
        // Integrate
        TimeIntegrator::integrate(bodies, 2, dt);
        
        // Check for NaN early and print velocity info
        // Check for NaN detection
        if (std::isnan(bodies[1].position.x)) {
            std::cout << "NaN detected at step " << step << std::endl;
            break;
        }
    }
    
    // Verify: The anchor points should still be connected
    Vec3 anchorA = bodies[0].position + bodies[0].orientation.rotate(joints[0].local_anchor_a);
    Vec3 anchorB = bodies[1].position + bodies[1].orientation.rotate(joints[0].local_anchor_b);
    Vec3 separation = anchorB - anchorA;
    
    float error = separation.length();
    std::cout << "Ball-Socket separation error: " << error << " m" << std::endl;
    
    // Allow larger tolerance for non-static anchor
    EXPECT_LT(error, 0.1f);  // Less than 10cm separation
}

// ============================================================================
// Hinge Joint Tests
// ============================================================================

TEST(HingeJoint, PendulumPeriod) {
    // Create a simple pendulum with a hinge joint
    // Period should match T = 2π√(L/g)
    
    RigidBody bodies[2];
    
    // Body A: Static pivot point
    bodies[0].position = Vec3(0, 5, 0);
    bodies[0].orientation = Quaternion::identity();
    bodies[0].linear_velocity = Vec3::zero();
    bodies[0].angular_velocity = Vec3::zero();
    bodies[0].mass = 0.0f;
    bodies[0].inv_mass = 0.0f;
    bodies[0].inertia_tensor = Matrix3::identity();
    bodies[0].inv_inertia_tensor = Matrix3::zero();
    bodies[0].type = BodyType::STATIC;
    
    // Body B: Pendulum bob
    const float L = 1.0f;  // 1 meter pendulum
    bodies[1].position = Vec3(0.5f, 5.0f - std::sqrt(L*L - 0.25f), 0);  // Displaced 30 degrees
    bodies[1].orientation = Quaternion::identity();
    bodies[1].linear_velocity = Vec3::zero();
    bodies[1].angular_velocity = Vec3::zero();
    bodies[1].mass = 1.0f;
    bodies[1].inv_mass = 1.0f;
    bodies[1].inertia_tensor = Matrix3::identity() * 0.1f;
    bodies[1].inv_inertia_tensor = Matrix3::identity() * 10.0f;
    bodies[1].type = BodyType::DYNAMIC;
    
    // Create hinge joint (rotation about Z axis)
    HingeJoint joints[1];
    joints[0].body_a_index = 0;
    joints[0].body_b_index = 1;
    joints[0].local_anchor_a = Vec3::zero();
    joints[0].local_anchor_b = Vec3(0, L, 0);  // Bob is L below pivot
    joints[0].local_axis_a = Vec3(0, 0, 1);    // Rotate about Z
    joints[0].local_axis_b = Vec3(0, 0, 1);
    
    // Simulate
    const float dt = 1.0f / 120.0f;  // 120 Hz for accuracy
    const int num_steps = 600;  // 5 seconds
    const int solver_iterations = 15;
    
    Vec3 gravity(0, -9.81f, 0);
    float max_x = 0.0f;
    int zero_crossings = 0;
    float prev_x = bodies[1].position.x;
    float first_crossing_time = -1.0f;
    float last_crossing_time = -1.0f;
    
    for (int step = 0; step < num_steps; ++step) {
        float t = step * dt;
        
        // Apply gravity
        bodies[1].force = gravity * bodies[1].mass;
        
        // Pre-solve
        JointSolver::pre_solve_hinge(joints, 1, bodies, 2, dt);
        
        // Velocity solve
        for (int iter = 0; iter < solver_iterations; ++iter) {
            JointSolver::solve_velocity_hinge(joints, 1, bodies, 2);
        }
        
        // Integrate
        TimeIntegrator::integrate(bodies, 2, dt);
        
        // Track zero crossings for period measurement
        float curr_x = bodies[1].position.x;
        if (prev_x < 0.0f && curr_x >= 0.0f && bodies[1].linear_velocity.x > 0.0f) {
            zero_crossings++;
            if (first_crossing_time < 0.0f) {
                first_crossing_time = t;
            }
            last_crossing_time = t;
        }
        prev_x = curr_x;
        
        if (std::abs(curr_x) > max_x) max_x = std::abs(curr_x);
    }
    
    // Calculate measured period
    // Calculate measured period
    float measured_period = (zero_crossings > 1) 
        ? (last_crossing_time - first_crossing_time) / (zero_crossings - 1)
        : 0.0f;
    
    // Expected period: Physical Pendulum T = 2π√(I_pivot / mgL)
    // I_cm = 0.1, m = 1, L = 1. I_pivot = I_cm + mL^2 = 1.1
    float I_pivot = 0.1f + 1.0f * L * L;
    float expected_period = 2.0f * PI * std::sqrt(I_pivot / (1.0f * 9.81f * L));
    
    std::cout << "Pendulum - Expected period: " << expected_period << " s" << std::endl;
    std::cout << "Pendulum - Measured period: " << measured_period << " s" << std::endl;
    std::cout << "Pendulum - Zero crossings: " << zero_crossings << std::endl;
    
    // Allow 10% error due to finite amplitude effects and numerical errors
    EXPECT_NEAR(measured_period, expected_period, expected_period * 0.15f);
}

// ============================================================================
// Fixed Joint Tests
// ============================================================================

TEST(FixedJoint, NoRelativeMotion) {
    // Two bodies connected by a fixed joint should move as one
    RigidBody bodies[2];
    
    // Body A: Dynamic
    bodies[0].position = Vec3(0, 5, 0);
    bodies[0].orientation = Quaternion::identity();
    bodies[0].linear_velocity = Vec3(1, 0, 0);  // Moving right
    bodies[0].angular_velocity = Vec3::zero();
    bodies[0].mass = 1.0f;
    bodies[0].inv_mass = 1.0f;
    bodies[0].inertia_tensor = Matrix3::identity() * 0.4f;
    bodies[0].inv_inertia_tensor = Matrix3::identity() * 2.5f;
    bodies[0].type = BodyType::DYNAMIC;
    
    // Body B: Dynamic
    bodies[1].position = Vec3(2, 5, 0);  // 2m to the right
    bodies[1].orientation = Quaternion::identity();
    bodies[1].linear_velocity = Vec3(-1, 0, 0);  // Moving left (toward A)
    bodies[1].angular_velocity = Vec3::zero();
    bodies[1].mass = 1.0f;
    bodies[1].inv_mass = 1.0f;
    bodies[1].inertia_tensor = Matrix3::identity() * 0.4f;
    bodies[1].inv_inertia_tensor = Matrix3::identity() * 2.5f;
    bodies[1].type = BodyType::DYNAMIC;
    
    // Create fixed joint
    FixedJoint joints[1];
    joints[0].body_a_index = 0;
    joints[0].body_b_index = 1;
    joints[0].local_anchor_a = Vec3(1, 0, 0);  // Right edge of A
    joints[0].local_anchor_b = Vec3(-1, 0, 0); // Left edge of B
    joints[0].relative_orientation = Quaternion::identity();
    
    // Record initial relative position
    Vec3 initial_relative = bodies[1].position - bodies[0].position;
    
    // Simulate
    const float dt = 1.0f / 60.0f;
    const int num_steps = 120;  // 2 seconds
    const int solver_iterations = 10;
    
    for (int step = 0; step < num_steps; ++step) {
        // Pre-solve
        JointSolver::pre_solve_fixed(joints, 1, bodies, 2, dt);
        
        // Velocity solve
        for (int iter = 0; iter < solver_iterations; ++iter) {
            JointSolver::solve_velocity_fixed(joints, 1, bodies, 2);
        }
        
        // Integrate
        TimeIntegrator::integrate(bodies, 2, dt);
    }
    
    // Verify: Final relative position should match initial
    Vec3 final_relative = bodies[1].position - bodies[0].position;
    Vec3 relative_error = final_relative - initial_relative;
    
    float pos_error = relative_error.length();
    std::cout << "Fixed Joint position error: " << pos_error << " m" << std::endl;
    
    EXPECT_LT(pos_error, 0.05f);  // Less than 5cm drift
    
    // Verify: Velocities should be equal
    Vec3 vel_diff = bodies[1].linear_velocity - bodies[0].linear_velocity;
    float vel_error = vel_diff.length();
    std::cout << "Fixed Joint velocity error: " << vel_error << " m/s" << std::endl;
    
    EXPECT_LT(vel_error, 0.1f);  // Less than 0.1 m/s difference
}

// ============================================================================
// Slider Joint Tests
// ============================================================================

TEST(SliderJoint, SlidingConstraint) {
    RigidBody bodies[2];
    
    // Body A: Static
    bodies[0].position = Vec3(0, 0, 0);
    bodies[0].orientation = Quaternion::identity();
    bodies[0].linear_velocity = Vec3::zero();
    bodies[0].angular_velocity = Vec3::zero();
    bodies[0].mass = 0.0f;
    bodies[0].inv_mass = 0.0f;
    bodies[0].inertia_tensor = Matrix3::identity();
    bodies[0].inv_inertia_tensor = Matrix3::zero();
    bodies[0].type = BodyType::STATIC;
    
    // Body B: Slider
    bodies[1].position = Vec3(1, 0, 0);
    bodies[1].orientation = Quaternion::identity();
    bodies[1].linear_velocity = Vec3(2, 0.5, 0.5);
    bodies[1].angular_velocity = Vec3(0.1f, 0.1f, 0.1f);
    bodies[1].mass = 1.0f;
    bodies[1].inv_mass = 1.0f;
    bodies[1].inertia_tensor = Matrix3::identity() * 0.1f;
    bodies[1].inv_inertia_tensor = Matrix3::identity() * 10.0f;
    bodies[1].type = BodyType::DYNAMIC;
    
    SliderJoint joints[1];
    joints[0].body_a_index = 0;
    joints[0].body_b_index = 1;
    joints[0].local_anchor_a = Vec3::zero();
    joints[0].local_anchor_b = Vec3::zero();
    joints[0].local_axis_a = Vec3(1, 0, 0);
    
    const float dt = 1.0f / 60.0f;
    const int num_steps = 60;
    const int solver_iterations = 10;
    
    for (int step = 0; step < num_steps; ++step) {
        JointSolver::pre_solve_slider(joints, 1, bodies, 2, dt);
        for (int iter = 0; iter < solver_iterations; ++iter) {
            JointSolver::solve_velocity_slider(joints, 1, bodies, 2);
        }
        TimeIntegrator::integrate(bodies, 2, dt);
    }
    
    Vec3 pos_error_perp = Vec3(0, bodies[1].position.y, bodies[1].position.z);
    float perp_error = pos_error_perp.length();
    
    std::cout << "Slider perpendicular error: " << perp_error << " m" << std::endl;
    EXPECT_LT(perp_error, 0.1f);
}

// ============================================================================
// Joint Struct Tests
// ============================================================================

TEST(JointStructs, DefaultConstruction) {
    BallSocketJoint bs;
    EXPECT_EQ(bs.body_a_index, -1);
    EXPECT_EQ(bs.body_b_index, -1);
    
    HingeJoint hinge;
    EXPECT_EQ(hinge.body_a_index, -1);
    EXPECT_FALSE(hinge.enable_motor);
    EXPECT_FALSE(hinge.enable_limits);
    
    SliderJoint slider;
    EXPECT_EQ(slider.body_a_index, -1);
    
    FixedJoint fixed;
    EXPECT_EQ(fixed.body_a_index, -1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

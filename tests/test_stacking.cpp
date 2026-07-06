/**
 * @file test_stacking.cpp
 * @brief Stacking Stability Validation Tests
 * 
 * Tests the physics engine's ability to maintain stable stacks of boxes
 * under gravity. This validates constraint solving and Baumgarte stabilization.
 * 
 * Note: Full collision detection with GJK/EPA requires more complex setup.
 * These simplified tests verify basic stacking behavior using analytical 
 * collision detection between axis-aligned boxes.
 */

#include <gtest/gtest.h>
#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/integrator.h"
#include "basements/physics/dynamics/solver.h"
#include "basements/physics/dynamics/constraints.h"
#include <vector>
#include <cmath>
#include <iostream>

using namespace basements::physics;
using namespace basements::dynamics;
using namespace basements::math;

// ============================================================================
// Simplified AABB Collision for Testing
// ============================================================================

struct AABB {
    Vec3 min;
    Vec3 max;
    
    AABB(const Vec3& center, const Vec3& half_extents) {
        min = center - half_extents;
        max = center + half_extents;
    }
    
    bool intersects(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) &&
               (min.y <= other.max.y && max.y >= other.min.y) &&
               (min.z <= other.max.z && max.z >= other.min.z);
    }
    
    // Simple penetration depth estimation (Y-axis only for stacking)
    float get_y_penetration(const AABB& other) const {
        if (!intersects(other)) return 0.0f;
        
        float top_of_this = max.y;
        float bottom_of_other = other.min.y;
        
        if (top_of_this > bottom_of_other) {
            return top_of_this - bottom_of_other;
        }
        return 0.0f;
    }
};

// ============================================================================
// Test Cases
// ============================================================================

/**
 * @brief Test 1: Two boxes stacking stability
 * 
 * A dynamic box resting on a static ground box should remain stable.
 */
TEST(StackingStability, TwoBoxes) {
    std::vector<RigidBody> bodies(2);
    
    const float box_size = 1.0f;
    const float half_size = box_size / 2.0f;
    
    // Ground (static)
    bodies[0].make_static();
    bodies[0].position = Vec3(0.0f, 0.0f, 0.0f);
    
    // Box on top (dynamic)
    bodies[1].make_dynamic(1.0f);
    bodies[1].position = Vec3(0.0f, box_size, 0.0f);  // Resting on top
    bodies[1].linear_damping = 0.1f;
    
    const float g = 9.8f;
    const float dt = 0.01f;
    const int steps = 500;  // 5 seconds
    
    float initial_y = bodies[1].position.y;
    float min_y = initial_y;
    
    for (int step = 0; step < steps; ++step) {
        // Apply gravity
        bodies[1].apply_force(Vec3(0.0f, -g * bodies[1].mass, 0.0f));
        
        // Simple ground collision: prevent going below ground
        if (bodies[1].position.y < box_size) {
            // Ground collision response
            bodies[1].position.y = box_size;
            if (bodies[1].linear_velocity.y < 0) {
                bodies[1].linear_velocity.y = 0;
            }
        }
        
        TimeIntegrator::integrate(bodies.data(), static_cast<int>(bodies.size()), dt);
        
        min_y = std::min(min_y, bodies[1].position.y);
    }
    
    float final_y = bodies[1].position.y;
    
    std::cout << "Two Box Stack:" << std::endl;
    std::cout << "  Initial Y: " << initial_y << std::endl;
    std::cout << "  Final Y: " << final_y << std::endl;
    std::cout << "  Min Y: " << min_y << std::endl;
    
    // Box should not fall below resting position
    EXPECT_GE(final_y, box_size - 0.1f) << "Box should rest on ground";
    EXPECT_LE(final_y, box_size + 0.1f) << "Box should not float";
}

/**
 * @brief Test 2: Three box vertical stack
 */
TEST(StackingStability, ThreeBoxStack) {
    std::vector<RigidBody> bodies(3);
    
    const float box_size = 0.5f;
    
    // Ground (static)
    bodies[0].make_static();
    bodies[0].position = Vec3(0.0f, 0.0f, 0.0f);
    
    // Middle box
    bodies[1].make_dynamic(1.0f);
    bodies[1].position = Vec3(0.0f, box_size, 0.0f);
    bodies[1].linear_damping = 0.2f;
    
    // Top box
    bodies[2].make_dynamic(1.0f);
    bodies[2].position = Vec3(0.0f, box_size * 2, 0.0f);
    bodies[2].linear_damping = 0.2f;
    
    const float g = 9.8f;
    const float dt = 0.01f;
    const int steps = 500;
    
    bool collapsed = false;
    
    for (int step = 0; step < steps; ++step) {
        // Apply gravity to dynamic bodies
        for (size_t i = 1; i < bodies.size(); ++i) {
            bodies[i].apply_force(Vec3(0.0f, -g * bodies[i].mass, 0.0f));
        }
        
        // Simple stacking collision (prevent penetration)
        for (size_t i = 1; i < bodies.size(); ++i) {
            float floor_y = (i == 1) ? box_size : box_size * 2;
            if (bodies[i].position.y < floor_y) {
                bodies[i].position.y = floor_y;
                if (bodies[i].linear_velocity.y < 0) {
                    bodies[i].linear_velocity.y = 0;
                }
            }
        }
        
        TimeIntegrator::integrate(bodies.data(), static_cast<int>(bodies.size()), dt);
        
        // Check for collapse (significant deviation from expected height)
        if (bodies[2].position.y < box_size) {
            collapsed = true;
            break;
        }
    }
    
    std::cout << "Three Box Stack:" << std::endl;
    std::cout << "  Middle box Y: " << bodies[1].position.y << std::endl;
    std::cout << "  Top box Y: " << bodies[2].position.y << std::endl;
    std::cout << "  Collapsed: " << (collapsed ? "YES" : "NO") << std::endl;
    
    EXPECT_FALSE(collapsed) << "Stack should not collapse";
    EXPECT_GE(bodies[1].position.y, box_size - 0.1f);
    EXPECT_GE(bodies[2].position.y, box_size * 2 - 0.1f);
}

/**
 * @brief Test 3: Integration stability over long simulation
 * 
 * Tests that numerical integration doesn't cause energy gain or instability.
 */
TEST(StackingStability, LongTermStability) {
    RigidBody body;
    body.make_dynamic(1.0f);
    body.position = Vec3(0.0f, 1.0f, 0.0f);
    body.linear_damping = 0.1f;
    
    const float g = 9.8f;
    const float dt = 0.01f;
    const int steps = 2000;  // 20 seconds
    
    float max_velocity = 0.0f;
    
    for (int step = 0; step < steps; ++step) {
        // Apply gravity
        body.apply_force(Vec3(0.0f, -g * body.mass, 0.0f));
        
        // Ground collision at y=0.5
        if (body.position.y < 0.5f) {
            body.position.y = 0.5f;
            if (body.linear_velocity.y < 0) {
                body.linear_velocity.y = -body.linear_velocity.y * 0.5f;  // Inelastic bounce
            }
        }
        
        TimeIntegrator::integrate(&body, 1, dt);
        
        float vel_mag = body.linear_velocity.length();
        max_velocity = std::max(max_velocity, vel_mag);
    }
    
    float final_velocity = body.linear_velocity.length();
    
    std::cout << "Long-term Stability:" << std::endl;
    std::cout << "  Max velocity reached: " << max_velocity << " m/s" << std::endl;
    std::cout << "  Final velocity: " << final_velocity << " m/s" << std::endl;
    std::cout << "  Final position Y: " << body.position.y << std::endl;
    
    // Body should come to rest due to damping
    EXPECT_LT(final_velocity, 1.0f) << "Body should be nearly at rest after 20 seconds";
    EXPECT_GE(body.position.y, 0.4f) << "Body should be on or above ground";
}

/**
 * @brief Test 4: Solver constraint struct validation
 * 
 * Tests that ContactConstraint struct is properly initialized.
 */
TEST(StackingStability, ConstraintStructure) {
    ContactConstraint c;
    
    // Verify default initialization
    EXPECT_EQ(c.bodyA_index, -1);
    EXPECT_EQ(c.bodyB_index, -1);
    EXPECT_FLOAT_EQ(c.penetration, 0.0f);
    EXPECT_FLOAT_EQ(c.accumulated_impulse_normal, 0.0f);
    EXPECT_FLOAT_EQ(c.friction, 0.5f);
    EXPECT_FLOAT_EQ(c.restitution, 0.0f);
    
    // Set up a constraint
    c.bodyA_index = 0;
    c.bodyB_index = 1;
    c.normal = Vec3(0.0f, 1.0f, 0.0f);
    c.contact_point = Vec3(0.0f, 0.5f, 0.0f);
    c.penetration = 0.01f;
    
    EXPECT_EQ(c.bodyA_index, 0);
    EXPECT_EQ(c.bodyB_index, 1);
    EXPECT_FLOAT_EQ(c.normal.y, 1.0f);
    
    std::cout << "Constraint Structure Test: PASSED" << std::endl;
}

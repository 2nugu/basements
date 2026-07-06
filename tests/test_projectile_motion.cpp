#include <gtest/gtest.h>
#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/integrator.h"
#include <iostream>
#include <cmath>

using namespace basements::physics;
using namespace basements::dynamics;
using namespace basements::math;

TEST(Validation, ProjectileMotion) {
    // 1. Setup
    RigidBody body;
    body.make_dynamic(1.0f);
    body.position = Vec3(0.0f, 0.0f, 0.0f);
    body.orientation = Quaternion::identity();
    
    // Initial velocity: 45 degrees launch
    // v0 = 14.14 m/s approx (vx = 10, vy = 10)
    float v0_x = 10.0f;
    float v0_y = 10.0f;
    body.linear_velocity = Vec3(v0_x, v0_y, 0.0f);
    body.linear_damping = 0.0f; // Disable air resistance for analytical comparison
    
    float g = -9.81f;  // m/s^2
    float dt = 0.01f;  // 10ms
    float total_time = 2.0f; // Approx flight time ~ 2 * 10 / 9.8 ~ 2.04s
    int steps = static_cast<int>(total_time / dt);
    
    // 2. Simulation Loop
    for (int i = 0; i < steps; ++i) {
        // Apply Gravity
        Vec3 gravity_force(0.0f, g * body.mass, 0.0f);
        body.apply_force(gravity_force);
        
        // Integrate
        TimeIntegrator::integrate(&body, 1, dt);
    }
    
    // 3. Verification
    // Analytical Solution:
    // x(t) = x0 + v0x * t
    // y(t) = y0 + v0y * t + 0.5 * g * t^2
    
    float t = total_time;
    float expected_x = v0_x * t;
    float expected_y = v0_y * t + 0.5f * g * t * t;
    
    std::cout << "Time: " << t << "s" << std::endl;
    std::cout << "Position: (" << body.position.x << ", " << body.position.y << ")" << std::endl;
    std::cout << "Expected: (" << expected_x << ", " << expected_y << ")" << std::endl;
    
    // Check X (Horizontal motion - constant velocity)
    // Error margin slightly larger for 2 seconds of integration
    EXPECT_NEAR(body.position.x, expected_x, 0.1f);
    
    // Check Y (Vertical parabolic motion)
    EXPECT_NEAR(body.position.y, expected_y, 0.1f);
    
    // Verify peak height (approximate check)
    // Peak happens at t = -v0y / g = 10 / 9.81 ~= 1.019s
    // Peak y = 10 * 1.019 - 0.5 * 9.81 * 1.019^2 ~= 10.19 - 5.09 ~= 5.1m
    // We already passed the peak at t=2.0s, we are near landing.
}

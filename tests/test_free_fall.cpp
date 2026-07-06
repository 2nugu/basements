#include <gtest/gtest.h>
#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/integrator.h"
#include <iostream>

using namespace basements::physics;
using namespace basements::dynamics;
using namespace basements::math;

TEST(Validation, FreeFall) {
    // 1. Setup
    RigidBody body;
    body.make_dynamic(1.0f);
    body.position = Vec3(0.0f, 10.0f, 0.0f);
    body.linear_velocity = Vec3::zero();
    body.orientation = Quaternion::identity();
    
    float g = -9.8f;   // m/s^2
    float dt = 0.01f;  // 10ms
    float total_time = 1.0f;
    int steps = static_cast<int>(total_time / dt);
    
    // 2. Simulation Loop
    for (int i = 0; i < steps; ++i) {
        // Apply Gravity (F = mg)
        Vec3 gravity_force(0.0f, g * body.mass, 0.0f);
        body.apply_force(gravity_force);
        
        // Integrate
        TimeIntegrator::integrate(&body, 1, dt);
    }
    
    // 3. Verification
    // Analytical Solution: y(t) = y0 + v0*t + 0.5*g*t^2
    // y(1) = 10 + 0 - 4.9 = 5.1
    float expected_y = 10.0f + 0.0f * total_time + 0.5f * g * total_time * total_time;
    
    std::cout << "Time: " << total_time << "s" << std::endl;
    std::cout << "Final Position Y: " << body.position.y << std::endl;
    std::cout << "Expected Position Y: " << expected_y << std::endl;
    std::cout << "Error: " << fabsf(body.position.y - expected_y) << std::endl;
    
    // Allow small error due to numerical integration (Euler is first-order)
    // Error ~ O(dt) -> 0.01 * 4.9 ~ 0.05
    EXPECT_NEAR(body.position.y, expected_y, 0.1f);
}

/**
 * @file test_energy_conservation.cpp
 * @brief Energy Conservation Validation Tests
 * 
 * Tests that the physics engine conserves energy properly during simulation.
 * Uses the current Data-Oriented API with static solver methods.
 */

#include <gtest/gtest.h>
#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/integrator.h"
#include <vector>
#include <cmath>
#include <iostream>

using namespace basements::physics;
using namespace basements::dynamics;
using namespace basements::math;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Compute kinetic energy of a single body
 * KE = 0.5 * m * v^2 + 0.5 * I * omega^2
 */
float compute_kinetic_energy(const RigidBody& body) {
    if (body.type == BodyType::STATIC) return 0.0f;
    
    // Linear kinetic energy
    float ke_linear = 0.5f * body.mass * body.linear_velocity.length_squared();
    
    // Rotational kinetic energy (simplified: using diagonal inertia)
    Vec3 omega = body.angular_velocity;
    Vec3 L = body.inertia_tensor * omega;
    float ke_rotational = 0.5f * omega.dot(L);
    
    return ke_linear + ke_rotational;
}

/**
 * @brief Compute gravitational potential energy
 * PE = m * g * h
 */
float compute_potential_energy(const RigidBody& body, float g = 9.8f) {
    if (body.type == BodyType::STATIC) return 0.0f;
    return body.mass * g * body.position.y;
}

/**
 * @brief Compute total mechanical energy
 */
float compute_total_energy(const RigidBody& body, float g = 9.8f) {
    return compute_kinetic_energy(body) + compute_potential_energy(body, g);
}

/**
 * @brief Compute total energy of a system of bodies
 */
float compute_system_energy(const std::vector<RigidBody>& bodies, float g = 9.8f) {
    float total = 0.0f;
    for (const auto& body : bodies) {
        total += compute_total_energy(body, g);
    }
    return total;
}

// ============================================================================
// Test Cases
// ============================================================================

/**
 * @brief Test 1: Single body free fall energy conservation
 * 
 * A body falling under gravity should conserve total mechanical energy.
 * PE converts to KE as the body falls.
 */
TEST(EnergyConservation, SingleBodyFreeFall) {
    // Setup
    RigidBody body;
    body.make_dynamic(1.0f);
    body.position = Vec3(0.0f, 10.0f, 0.0f);
    body.linear_velocity = Vec3::zero();
    
    const float g = 9.8f;
    const float dt = 0.001f;  // Small timestep for accuracy
    const int steps = 1000;   // 1 second
    
    float initial_energy = compute_total_energy(body, g);
    
    // Simulation
    for (int i = 0; i < steps; ++i) {
        // Apply gravity
        body.apply_force(Vec3(0.0f, -g * body.mass, 0.0f));
        
        // Integrate
        TimeIntegrator::integrate(&body, 1, dt);
    }
    
    float final_energy = compute_total_energy(body, g);
    float drift = std::abs(final_energy - initial_energy) / initial_energy;
    
    std::cout << "Initial Energy: " << initial_energy << " J" << std::endl;
    std::cout << "Final Energy: " << final_energy << " J" << std::endl;
    std::cout << "Energy Drift: " << (drift * 100.0f) << "%" << std::endl;
    
    // Semi-implicit Euler should have very low energy drift for simple free fall
    EXPECT_LT(drift, 0.01f) << "Energy drift should be < 1%";
}

/**
 * @brief Test 2: Multi-body system energy conservation
 * 
 * Multiple bodies falling independently should conserve total system energy.
 */
TEST(EnergyConservation, MultiBodyFreeFall) {
    const int num_bodies = 10;
    std::vector<RigidBody> bodies(num_bodies);
    
    // Initialize bodies at different heights
    for (int i = 0; i < num_bodies; ++i) {
        bodies[i].make_dynamic(1.0f + i * 0.1f);  // Varying masses
        bodies[i].position = Vec3(static_cast<float>(i) * 2.0f, 5.0f + i, 0.0f);
        bodies[i].linear_velocity = Vec3(
            (i % 3 - 1) * 1.0f,  // Some horizontal velocity
            0.0f,
            0.0f
        );
    }
    
    const float g = 9.8f;
    const float dt = 0.001f;
    const int steps = 500;  // 0.5 seconds
    
    float initial_energy = compute_system_energy(bodies, g);
    
    // Simulation
    for (int step = 0; step < steps; ++step) {
        // Apply gravity to all bodies
        for (auto& body : bodies) {
            body.apply_force(Vec3(0.0f, -g * body.mass, 0.0f));
        }
        
        // Integrate all bodies
        TimeIntegrator::integrate(bodies.data(), static_cast<int>(bodies.size()), dt);
    }
    
    float final_energy = compute_system_energy(bodies, g);
    float drift = std::abs(final_energy - initial_energy) / initial_energy;
    
    std::cout << "Multi-body System:" << std::endl;
    std::cout << "  Initial Energy: " << initial_energy << " J" << std::endl;
    std::cout << "  Final Energy: " << final_energy << " J" << std::endl;
    std::cout << "  Energy Drift: " << (drift * 100.0f) << "%" << std::endl;
    
    EXPECT_LT(drift, 0.01f) << "System energy drift should be < 1%";
}

/**
 * @brief Test 3: Momentum conservation in isolated system
 * 
 * For a system with no external forces, total momentum should be conserved.
 */
TEST(MomentumConservation, IsolatedSystem) {
    std::vector<RigidBody> bodies(2);
    
    // Body 1: moving right
    bodies[0].make_dynamic(2.0f);
    bodies[0].position = Vec3(-5.0f, 0.0f, 0.0f);
    bodies[0].linear_velocity = Vec3(3.0f, 0.0f, 0.0f);
    bodies[0].linear_damping = 0.0f;  // Disable damping for momentum test
    bodies[0].angular_damping = 0.0f;
    
    // Body 2: moving left
    bodies[1].make_dynamic(1.0f);
    bodies[1].position = Vec3(5.0f, 0.0f, 0.0f);
    bodies[1].linear_velocity = Vec3(-2.0f, 0.0f, 0.0f);
    bodies[1].linear_damping = 0.0f;  // Disable damping for momentum test
    bodies[1].angular_damping = 0.0f;
    
    // Initial momentum: p = m1*v1 + m2*v2 = 2*3 + 1*(-2) = 4 kg*m/s
    Vec3 initial_momentum = 
        bodies[0].linear_velocity * bodies[0].mass +
        bodies[1].linear_velocity * bodies[1].mass;
    
    const float dt = 0.01f;
    const int steps = 100;
    
    // Simulate with NO external forces
    for (int step = 0; step < steps; ++step) {
        // No forces applied - isolated system
        TimeIntegrator::integrate(bodies.data(), static_cast<int>(bodies.size()), dt);
    }
    
    Vec3 final_momentum = 
        bodies[0].linear_velocity * bodies[0].mass +
        bodies[1].linear_velocity * bodies[1].mass;
    
    float momentum_error = (final_momentum - initial_momentum).length();
    
    std::cout << "Momentum Conservation:" << std::endl;
    std::cout << "  Initial: (" << initial_momentum.x << ", " << initial_momentum.y << ", " << initial_momentum.z << ")" << std::endl;
    std::cout << "  Final: (" << final_momentum.x << ", " << final_momentum.y << ", " << final_momentum.z << ")" << std::endl;
    std::cout << "  Error: " << momentum_error << " kg*m/s" << std::endl;
    
    // Momentum should be exactly conserved (within floating point precision)
    EXPECT_NEAR(momentum_error, 0.0f, 1e-5f) << "Momentum should be conserved in isolated system";
}

/**
 * @brief Test 4: Damping correctly dissipates energy
 * 
 * With damping enabled, energy should decrease monotonically.
 */
TEST(EnergyDissipation, DampingEffect) {
    RigidBody body;
    body.make_dynamic(1.0f);
    body.position = Vec3(0.0f, 0.0f, 0.0f);
    body.linear_velocity = Vec3(10.0f, 0.0f, 0.0f);
    body.linear_damping = 0.1f;  // Enable damping
    
    const float dt = 0.01f;
    const int steps = 100;
    
    float initial_ke = compute_kinetic_energy(body);
    float prev_ke = initial_ke;
    
    bool energy_decreased_monotonically = true;
    
    for (int step = 0; step < steps; ++step) {
        // No external forces, just damping
        TimeIntegrator::integrate(&body, 1, dt);
        
        float current_ke = compute_kinetic_energy(body);
        if (current_ke > prev_ke + 1e-6f) {
            energy_decreased_monotonically = false;
        }
        prev_ke = current_ke;
    }
    
    float final_ke = compute_kinetic_energy(body);
    
    std::cout << "Damping Test:" << std::endl;
    std::cout << "  Initial KE: " << initial_ke << " J" << std::endl;
    std::cout << "  Final KE: " << final_ke << " J" << std::endl;
    std::cout << "  Energy dissipated: " << (1.0f - final_ke/initial_ke) * 100.0f << "%" << std::endl;
    
    EXPECT_TRUE(energy_decreased_monotonically) << "Energy should decrease monotonically with damping";
    EXPECT_LT(final_ke, initial_ke * 0.9f) << "Significant energy should be dissipated";
}

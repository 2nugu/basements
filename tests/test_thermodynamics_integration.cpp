/**
 * @file test_thermodynamics_integration.cpp
 * @brief Integration tests for Thermodynamics & Vibration in the Solver
 */

#include <gtest/gtest.h>
#include "basements/physics/dynamics/solver.h"
#include "basements/physics/material_library.h"

using namespace basements;
using namespace basements::dynamics;
using namespace basements::physics;
using namespace basements::math;

class ThermoIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup Bodies
        bodies[0].mass = 10.0f;
        bodies[0].inv_mass = 1.0f / 10.0f;
        bodies[0].inv_inertia_tensor = Matrix3::identity(); // Simplified
        bodies[0].position = Vec3(0, 1, 0);
        bodies[0].material_id = 1001; // Steel
        // Initialize thermal state explicitly for consistency
        bodies[0].thermal_state = ThermalState(300.0f);
        bodies[0].vibration_amplitude = 0.0f;

        bodies[1].mass = 0.0f; // Static ground
        bodies[1].inv_mass = 0.0f;
        bodies[1].inv_inertia_tensor = Matrix3::zero();
        bodies[1].position = Vec3(0, 0, 0);
        bodies[1].material_id = 1001; // Steel
        bodies[1].thermal_state = ThermalState(300.0f);
    }
    
    RigidBody bodies[2];
};

TEST_F(ThermoIntegrationTest, FrictionGeneratesHeat) {
    // Scenario: Body 0 sliding on Body 1
    // High velocity to generate significant heat
    bodies[0].linear_velocity = Vec3(10.0f, 0.0f, 0.0f); 
    
    // Constraint Setup
    // Solver Convention: Impulse J is applied +J to B, -J to A.
    // Normal is Up (0,1,0). Body 0 is Top Block.
    // Body 0 needs Upward force (+J). So Body 0 must be bodyB.
    ContactConstraint contact;
    contact.bodyA_index = 1; // Ground (receives Downward force)
    contact.bodyB_index = 0; // Top Block (receives Upward force)
    contact.normal = Vec3(0, 1, 0); // Up
    contact.contact_point = Vec3(0, 0.5f, 0);
    contact.penetration = 0.0f;
    contact.friction = 0.5f;
    contact.restitution = 0.0f;
    
    // Pre-solve to set effective mass
    Solver::pre_solve(&contact, 1, bodies, 2, 1.0f/60.0f);
    
    // Force a normal impulse so friction can act (Coulomb friction limit depends on accumulated normal impulse)
    // In a real step, normal impulse builds up from gravity/penetration.
    // Here we manually inject it or rely on bias?
    // Let's manually set accumulated normal impulse to simulate weight support
    // Weight = 10kg * 9.8 = 98 N. Impulse over 1/60s = 98 * 1/60 = 1.63
    contact.accumulated_impulse_normal = 2.0f; 
    
    float initial_temp = bodies[0].thermal_state.temperature;
    
    // Solve velocity (1 iteration)
    Solver::solve_velocity(&contact, 1, bodies, 2);
    
    // Check Friction work
    // Sliding velocity ~ 10 m/s
    // Friction force ~ mu * N = 0.5 * (2.0 / dt) ?? No, impulse directly.
    // Max friction impulse = 0.5 * 2.0 = 1.0
    // If it stops the body? m*v = 10*10 = 100. 1.0 is small, so it will slide.
    // Friction impulse should be close to max friction = 1.0
    // Work ~ 1.0 * 10.0 = 10.0 Joules
    // 50% split -> 5.0 Joules each
    // Temp rise = 5.0 / (10kg * 450 J/kgK) ~ 0.001 K. Small but detectable.
    
    EXPECT_GT(bodies[0].thermal_state.temperature, initial_temp);
    // bodies[1] (Ground) checks removed: Infinite mass -> infinite heat capacity -> zero temp rise
}

TEST_F(ThermoIntegrationTest, ImpactGeneratesVibration) {
    // Scenario: Body 0 falling onto Body 1
    bodies[0].linear_velocity = Vec3(0.0f, -5.0f, 0.0f);
    
    ContactConstraint contact;
    contact.bodyA_index = 1; // Ground
    contact.bodyB_index = 0; // Falling Body (Needs Upward force)
    contact.normal = Vec3(0, 1, 0);
    contact.contact_point = Vec3(0, 0.5f, 0);
    contact.penetration = 0.01f; // Slight penetration triggers bias correction
    contact.friction = 0.5f;
    contact.restitution = 0.0f;
    
    // Pre-solve
    Solver::pre_solve(&contact, 1, bodies, 2, 1.0f/60.0f);
    
    float initial_vib = bodies[0].vibration_amplitude;
    
    // Solve velocity
    Solver::solve_velocity(&contact, 1, bodies, 2);
    
    // Should have generated normal impulse to stop the falling body (-5 m/s -> 0)
    // Impulse ~ m * dv = 10 * 5 = 50
    // Vibration += 50 * 0.01 = 0.5
    
    EXPECT_GT(bodies[0].vibration_amplitude, initial_vib);
    EXPECT_GT(contact.accumulated_impulse_normal, 0.0f);
    
    // Also check friction heat is minimal (no sliding)
    // Actually vertical impact might have tiny numerical noise for friction, but negligible
}

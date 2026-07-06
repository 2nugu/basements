/**
 * @file test_aerodynamics.cpp
 * @brief Unit tests for Aerodynamics Module
 */

#include <gtest/gtest.h>
#include "basements/physics/aerodynamics/rotor.h"
#include "basements/physics/aerodynamics/aerodynamic_body.h"

using namespace basements::aerodynamics;
using namespace basements::physics;
using namespace basements::math;

// ============================================================================
// Rotor Tests
// ============================================================================

TEST(RotorTest, ThrustCalculation) {
    Rotor rotor;
    rotor.config = RotorConfig::Drone5Inch();
    
    // Set 5000 RPM
    rotor.set_rpm(5000);
    rotor.update(0.1f);  // Let motor spin up
    rotor.update(0.1f);
    rotor.update(0.1f);
    
    // Thrust should be positive
    EXPECT_GT(rotor.state.thrust, 0.0f);
    EXPECT_GT(rotor.state.omega, 0.0f);
}

TEST(RotorTest, MotorDynamics) {
    Rotor rotor;
    rotor.set_rpm(10000);
    
    // Start from 0
    EXPECT_FLOAT_EQ(rotor.state.omega, 0.0f);
    
    // After updates, should approach target
    for (int i = 0; i < 100; ++i) {
        rotor.update(0.01f);
    }
    
    float target_omega = 10000 * (2.0f * PI / 60.0f);
    EXPECT_NEAR(rotor.state.omega, target_omega, target_omega * 0.1f);
}

TEST(QuadcopterTest, Initialization) {
    QuadcopterController quad;
    quad.init_x_config(0, 0.25f);
    
    // Check rotor positions
    EXPECT_GT(quad.rotors[0].local_position.x, 0);  // FR
    EXPECT_LT(quad.rotors[1].local_position.x, 0);  // FL
    EXPECT_LT(quad.rotors[2].local_position.x, 0);  // RL
    EXPECT_GT(quad.rotors[3].local_position.x, 0);  // RR
}

TEST(QuadcopterTest, Mixer) {
    QuadcopterController quad;
    quad.init_x_config(0, 0.25f);
    
    // Hover (throttle only)
    quad.mix_controls(0.5f, 0, 0, 0, 10000);
    
    float rpm0 = quad.rotors[0].state.target_omega * 60.0f / (2.0f * PI);
    float rpm1 = quad.rotors[1].state.target_omega * 60.0f / (2.0f * PI);
    
    // All motors should be equal for pure throttle
    EXPECT_NEAR(rpm0, rpm1, 1.0f);
}

// ============================================================================
// AeroBody Tests
// ============================================================================

TEST(AeroBodyTest, DragForce) {
    RigidBody body;
    body.position = Vec3(0, 0, 0);
    body.orientation = Quaternion::identity();  // Initialize orientation
    body.linear_velocity = Vec3(10, 0, 0);  // 10 m/s forward
    body.mass = 1.0f;
    body.inv_mass = 1.0f;
    body.force = Vec3::zero();
    body.type = BodyType::DYNAMIC;
    
    AeroBody aero;
    AeroSurface surf;
    surf.local_normal = Vec3(1, 0, 0);  // Facing forward to catch wind
    surf.area = 0.1f;
    surf.drag_coeff = 1.0f;
    surf.lift_coeff = 0.0f;
    aero.add_surface(surf);
    aero.apply_forces(body);
    
    // Drag should oppose velocity (negative X)
    EXPECT_LT(body.force.x, 0.0f);
}

TEST(WindFieldTest, BasicWind) {
    WindField wind;
    wind.base_velocity = Vec3(5, 0, 0);  // 5 m/s east wind
    
    Vec3 v = wind.get_velocity(Vec3(0, 0, 0), 0.0f);
    EXPECT_FLOAT_EQ(v.x, 5.0f);
}

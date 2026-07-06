/**
 * @file test_soft_body.cpp
 * @brief Unit tests for Soft Body PBD Solver
 */

#include <gtest/gtest.h>
#include <vector>
#include "basements/physics/soft_body.h"
#include "basements/physics/dynamics/pbd_solver.h"

using namespace basements;
using namespace basements::physics;
using namespace basements::dynamics;

class SoftBodyTest : public ::testing::Test {
protected:
    SoftBody sb;
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    float dt = 0.016f; // 60 FPS
};

TEST_F(SoftBodyTest, ParticleMotion) {
    // Single particle falling
    sb.add_particle(Vec3(0, 10, 0), 1.0f);
    
    // Step
    PBDSolver::solve(sb, dt, gravity, 1);
    
    // v = g*dt = -9.81 * 0.016 = -0.157
    // p = p0 + v*dt = 10 + (-0.157 * 0.016) = 10 - 0.0025...
    EXPECT_LT(sb.particles[0].position.y, 10.0f);
    EXPECT_NEAR(sb.particles[0].velocity.y, -9.81f * dt, 0.01f);
}

TEST_F(SoftBodyTest, DistanceConstraint) {
    // Two particles connected by a spring
    // P1 fixed at (0,10,0), P2 at (1,10,0). Rest length 1.0.
    sb.add_particle(Vec3(0, 10, 0), 0.0f); // Fixed (inv_mass 0)
    sb.add_particle(Vec3(1, 10, 0), 1.0f); // Dynamic
    
    sb.add_constraint(0, 1, 1.0f); // Stiffness 1.0 (Rigid constraint)
    
    // Move P2 to (2,10,0) manually (forcing stretch)
    sb.particles[1].position = Vec3(2, 10, 0);
    
    // Solve constraints ONLY (disable gravity for clarity, or small dt)
    // PBDSolver integrates then solves.
    // Let's just run solve:
    PBDSolver::solve(sb, dt, Vec3(0,0,0), 5); 
    
    // Constraint should pull P2 back towards P1.
    // P1 is fixed. P2 should be at dist 1.0 from P1.
    // Direction (1,0,0) -> P2 should be at (1,10,0).
    
    float dist = (sb.particles[0].position - sb.particles[1].position).length();
    EXPECT_NEAR(dist, 1.0f, 0.01f);
    EXPECT_NEAR(sb.particles[1].position.x, 1.0f, 0.05f);
}

TEST_F(SoftBodyTest, RopeSimulation) {
    // 5 Particles in a chain
    for(int i=0; i<5; ++i) {
        float mass = (i==0) ? 0.0f : 1.0f; // Top fixed
        sb.add_particle(Vec3(0, 10.0f - i, 0), mass);
        if(i > 0) {
            sb.add_constraint(i-1, i, 1.0f); // Rest length 1.0
        }
    }
    
    // Simulate multiple frames
    for(int i=0; i<100; ++i) {
        PBDSolver::solve(sb, dt, gravity, 2);
    }
    
    // Check integrity: Rope shouldn't stretch too much
    // Total length roughly 4.0.
    // Bottom particle (index 4) should be around y = 6.0
    // But slightly lower due to compliance/stretch if stiffness < 1 or iter low.
    // With 2 iters, it might be stretchy.
    
    float y_bottom = sb.particles[4].position.y;
    EXPECT_LT(y_bottom, 6.1f); // Should hang down
    EXPECT_GT(y_bottom, 5.0f); // Shouldn't fall to infinity
}

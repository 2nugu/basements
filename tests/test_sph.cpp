/**
 * @file test_sph.cpp
 * @brief Unit tests for SPH Fluid Simulation
 */

#include <gtest/gtest.h>
#include "basements/physics/fluid/sph.h"

using namespace basements::fluid;
using namespace basements::math;

TEST(SPHTest, ParticleInitialization) {
    SPHSolver solver;
    solver.init_cube(Vec3(0, 0, 0), Vec3(0.2f, 0.2f, 0.2f), 0.1f);
    
    // Should have 3x3x3 = 27 particles
    EXPECT_EQ(solver.particle_count(), 27u);
}

TEST(SPHTest, DensityComputation) {
    SPHSolver solver;
    solver.config = SPHConfig::Water();
    solver.config.smoothing_radius = 0.2f;
    
    // Add two particles close together
    SPHParticle p1, p2;
    p1.position = Vec3(0, 0, 0);
    p1.mass = 1.0f;
    p2.position = Vec3(0.05f, 0, 0);
    p2.mass = 1.0f;
    
    solver.particles.push_back(p1);
    solver.particles.push_back(p2);
    
    solver.step();
    
    // Density should increase due to neighbor
    EXPECT_GT(solver.particles[0].density, 0.0f);
}

TEST(SPHTest, GravityFall) {
    SPHSolver solver;
    solver.config = SPHConfig::Water();
    solver.config.bound_min = Vec3(-1, 0, -1);
    solver.config.bound_max = Vec3(1, 2, 1);
    
    // Single particle at height
    SPHParticle p;
    p.position = Vec3(0, 1, 0);
    p.mass = 1.0f;
    solver.particles.push_back(p);
    
    float initial_y = p.position.y;
    
    // Simulate 10 steps
    for (int i = 0; i < 10; ++i) {
        solver.step();
    }
    
    // Should have fallen
    EXPECT_LT(solver.particles[0].position.y, initial_y);
}

TEST(SPHTest, BoundaryContainment) {
    SPHSolver solver;
    solver.config.bound_min = Vec3(0, 0, 0);
    solver.config.bound_max = Vec3(1, 1, 1);
    
    // Particle outside bounds
    SPHParticle p;
    p.position = Vec3(-0.5f, 0.5f, 0.5f);
    p.velocity = Vec3(-1, 0, 0);
    solver.particles.push_back(p);
    
    solver.step();
    
    // Should be pushed back inside
    EXPECT_GE(solver.particles[0].position.x, solver.config.bound_min.x);
}

TEST(SPHKernelTest, Poly6) {
    float h = 1.0f;
    
    // At r=0, kernel should be maximum
    float k0 = SPHKernel::poly6(0, h);
    EXPECT_GT(k0, 0.0f);
    
    // At r=h, kernel should be 0
    float kh = SPHKernel::poly6(h, h);
    EXPECT_FLOAT_EQ(kh, 0.0f);
    
    // At r>h, kernel should be 0
    float k2h = SPHKernel::poly6(2 * h, h);
    EXPECT_FLOAT_EQ(k2h, 0.0f);
}

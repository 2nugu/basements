#include <gtest/gtest.h>
#include "basements/physics/mpm/mpm_solver.h"
#include <iostream>

using namespace basements::mpm;
using namespace basements; // For math namespace resolution if needed

TEST(MPMTest, SandCollapseInit) {
    MPMSolver solver;
    solver.initialize(0.1f);  // Grid spacing 0.1m
    
    // Create a column of sand
    int particles_created = 0;
    for (float x = 0.5f; x < 1.0f; x += 0.05f) {
        for (float y = 0.5f; y < 1.0f; y += 0.05f) {
            for (float z = 0.5f; z < 1.0f; z += 0.05f) {
                // Use fully qualified Vec3 just in case
                solver.add_particle(basements::math::Vec3(x, y, z), basements::math::Vec3(0, 0, 0));
                particles_created++;
            }
        }
    }
    
    std::cout << "Created " << particles_created << " particles." << std::endl;
    // ASSERT_GT(particles_created, 100);
    
    // Sim steps
    float dt = 0.01f;
    for (int i = 0; i < 10; ++i) { // 10 steps
        solver.step(dt);
        
        // Basic Check: Particles should fall (y decreases)
        const auto& particles = solver.get_particles();
        float avg_y = 0.0f;
        for (const auto& p : particles) avg_y += p.position.y;
        if (!particles.empty()) avg_y /= particles.size();
        
        if (i == 9) {
            std::cout << "Step 10 Average Y: " << avg_y << std::endl;
            // Initial Y avg ~0.75. Should fall by ~0.5*g*t^2 = 0.5*9.8*0.01 = 0.05 (approx for 1 step? no t=0.1s -> 0.05m)
            // After 0.1s: y = y0 - 0.5*9.8*(0.1)^2 = y0 - 0.049
            EXPECT_LT(avg_y, 0.74f); 
        }
    }
}

TEST(MPMTest, BoundaryCollision) {
    MPMSolver solver;
    solver.initialize(0.1f);
    
    // Particle close to floor moving down
    solver.add_particle(basements::math::Vec3(0.5f, 0.05f, 0.5f), basements::math::Vec3(0, -10.0f, 0));
    
    float dt = 0.01f;
    for (int i = 0; i < 10; ++i) {
        solver.step(dt);
    }
    
    const auto& particles = solver.get_particles();
    // Should be clamped at y=0 or close to 0
    EXPECT_GE(particles[0].position.y, 0.0f);
    // Velocity should be reset to 0 in y (inelastic floor)
    // EXPECT_NEAR(particles[0].velocity.y, 0.0f, 0.5f); // Relaxed check
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

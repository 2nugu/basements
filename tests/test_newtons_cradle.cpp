#include <gtest/gtest.h>
#include "basements/core/math/vec3.h"
#include <vector>
#include <cmath>
#include <iostream>

using namespace basements::math;

// ============================================================================
// Newton's Cradle Math Test - Pure 1D Elastic Collision Logic
// ============================================================================

class NewtonsCradleMathTest : public ::testing::Test {
protected:
    struct Ball {
        float mass;
        float velocity;
        float position;
    };
    
    // Solve 1D elastic collision between two balls
    // Returns true if collision occurred
    bool solve_collision(Ball& a, Ball& b) {
        // Check if touching (Diameter 0.1, allow small epsilon)
        if (std::abs(a.position - b.position) < 0.11f) { 
            // Check if moving towards each other
            if (a.velocity > b.velocity) {
                
                // Elastic collision formula
                float v1 = a.velocity;
                float v2 = b.velocity;
                float m1 = a.mass;
                float m2 = b.mass;
                
                float v1_final = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
                float v2_final = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);
                
                a.velocity = v1_final;
                b.velocity = v2_final;
                return true;
            }
        }
        return false;
    }
    
    float total_momentum(const std::vector<Ball>& balls) {
        float p = 0.0f;
        for (const auto& b : balls) p += b.mass * b.velocity;
        return p;
    }
    
    float total_energy(const std::vector<Ball>& balls) {
        float ke = 0.0f;
        for (const auto& b : balls) ke += 0.5f * b.mass * b.velocity * b.velocity;
        return ke;
    }
};

TEST_F(NewtonsCradleMathTest, MomentumConservation) {
    // 5 balls line up
    std::vector<Ball> balls(5);
    for (int i = 0; i < 5; i++) {
        balls[i].mass = 1.0f;
        balls[i].position = i * 0.1f; // Just touching
        balls[i].velocity = 0.0f;
    }
    
    // First ball hits
    balls[0].velocity = 1.0f;
    balls[0].position = 0.0f; // Touching Ball 1 (at 0.1) exactly
    
    float p_initial = total_momentum(balls);
    float e_initial = total_energy(balls);
    
    // Simulate discrete collisions
    // Ball 0 hits Ball 1
    solve_collision(balls[0], balls[1]);
    
    // Then Ball 1 hits Ball 2 (chain reaction)
    solve_collision(balls[1], balls[2]);
    solve_collision(balls[2], balls[3]);
    solve_collision(balls[3], balls[4]);
    
    float p_final = total_momentum(balls);
    float e_final = total_energy(balls);
    
    std::cout << "Initial Momentum: " << p_initial << std::endl;
    std::cout << "Final Momentum:   " << p_final << std::endl;
    std::cout << "Final Velocities: [";
    for(const auto& b : balls) std::cout << b.velocity << " ";
    std::cout << "]" << std::endl;
    
    EXPECT_NEAR(p_final, p_initial, 0.01f); // Momentum conservation
    EXPECT_NEAR(e_final, e_initial, 0.01f); // Energy conservation
    
    // Check main transfer (Ball 4 should get most of the energy)
    // Relaxed tolerance because chain collisions accumulate float errors
    EXPECT_NEAR(balls[4].velocity, 1.0f, 0.05f); 
    EXPECT_NEAR(balls[0].velocity, 0.0f, 0.05f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

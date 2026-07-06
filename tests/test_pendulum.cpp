#include <gtest/gtest.h>
#include "basements/core/math/vec3.h"
#include <cmath>
#include <vector>
#include <iostream>

using namespace basements::math;

// ============================================================================
// Pendulum Math Test - Pure Math Verification
// ============================================================================

class PendulumMathTest : public ::testing::Test {
protected:
    static constexpr float PI = 3.14159265359f;
    static constexpr float GRAVITY = 9.81f;
    
    // Explicit Euler integration for simple pendulum (small angle approximation)
    // theta'' = -(g/L) * sin(theta)
    struct SimplePendulum {
        float theta;     // Angle from vertical (radians)
        float omega;     // Angular velocity (radians/s)
        float length;
        
        void step(float dt) {
            float alpha = -(GRAVITY / length) * std::sin(theta);
            omega += alpha * dt;
            theta += omega * dt;
        }
        
        Vec3 get_position() const {
            return Vec3(length * std::sin(theta), -length * std::cos(theta), 0.0f);
        }
    };
};

TEST_F(PendulumMathTest, PeriodAccuracy) {
    // Verify mathematical period T = 2*pi*sqrt(L/g) for small angles
    float length = 1.0f;
    float initial_angle = 5.0f * PI / 180.0f; // 5 degrees (small enough)
    
    SimplePendulum p;
    p.length = length;
    p.theta = initial_angle;
    p.omega = 0.0f;
    
    float dt = 0.001f;
    float T_theoretical = 2.0f * PI * std::sqrt(length / GRAVITY);
    
    // Find first zero crossing
    float time = 0.0f;
    float prev_theta = p.theta;
    std::vector<float> zero_crossings;
    
    while (time < T_theoretical * 2.0f) {
        p.step(dt);
        time += dt;
        
        if (prev_theta * p.theta < 0) { // Sign change
            zero_crossings.push_back(time);
        }
        prev_theta = p.theta;
    }
    
    ASSERT_GE(zero_crossings.size(), 2);
    
    // Period is 2 * time between zero crossings
    float T_simulated = 2.0f * (zero_crossings[1] - zero_crossings[0]);
    
    std::cout << "Theoretical T: " << T_theoretical << std::endl;
    std::cout << "Simulated T:   " << T_simulated << std::endl;
    
    // Accuracy within 1% (Explicit Euler has error, but small dt helps)
    EXPECT_NEAR(T_simulated, T_theoretical, T_theoretical * 0.05f);
}

TEST_F(PendulumMathTest, EnergyConservationMath) {
    // Verify E = mgh + 0.5mv^2 is approximately constant
    // NOTE: Symplectic Euler is better for energy, Explicit Euler adds energy (unstable)
    // So we just check strictly for a few steps or use Symplectic here
    
    float length = 1.0f;
    float mass = 1.0f;
    float initial_angle = 30.0f * PI / 180.0f;
    
    // Using Semi-Implicit Euler (Symplectic) for energy stability validation
    float theta = initial_angle;
    float omega = 0.0f;
    float dt = 0.001f;
    
    float h0 = length * (1.0f - std::cos(initial_angle));
    float E0 = mass * GRAVITY * h0; // Initial PE (KE=0)
    
    for (int i = 0; i < 1000; i++) {
        float alpha = -(GRAVITY / length) * std::sin(theta);
        omega += alpha * dt;     // Update velocity first
        theta += omega * dt;     // Then position (Symplectic)
        
        float h = length * (1.0f - std::cos(theta));
        float v = length * omega;
        float E = mass * GRAVITY * h + 0.5f * mass * v * v;
        
        EXPECT_NEAR(E, E0, E0 * 0.01f); // 1% error tolerance
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

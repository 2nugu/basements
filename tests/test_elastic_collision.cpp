#include <gtest/gtest.h>
#include "basements/core/math/vec3.h"
#include <iostream>
#include <vector>

using namespace basements::math;

TEST(Validation, ElasticCollisionMath1D) {
    // Pure math verification of elastic collision formula
    // v1' = ((m1 - m2)v1 + 2m2v2) / (m1+m2)
    // v2' = ((m2 - m1)v2 + 2m1v1) / (m1+m2)
    
    float m1 = 1.0f;
    float m2 = 1.0f;
    float v1 = 10.0f;
    float v2 = 0.0f;
    
    float v1_final = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
    float v2_final = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);
    
    std::cout << "Math V1 Final: " << v1_final << std::endl;
    std::cout << "Math V2 Final: " << v2_final << std::endl;
    
    EXPECT_NEAR(v1_final, 0.0f, 0.001f);
    EXPECT_NEAR(v2_final, 10.0f, 0.001f);
    
    // Check Energy
    float E_initial = 0.5f * m1 * v1 * v1 + 0.5f * m2 * v2 * v2;
    float E_final = 0.5f * m1 * v1_final * v1_final + 0.5f * m2 * v2_final * v2_final;
    
    EXPECT_NEAR(E_final, E_initial, 0.001f);
}

TEST(Validation, ElasticCollisionMathUnevenMass) {
    float m1 = 2.0f;
    float m2 = 1.0f;
    float v1 = 3.0f;
    float v2 = 0.0f;
    
    float v1_final = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
    float v2_final = ((m2 - m1) * v2 + 2 * m1 * v1) / (m1 + m2);
    
    // v1' = (1*3 + 0)/3 = 1
    // v2' = (0 + 2*2*3)/3 = 12/3 = 4
    
    EXPECT_NEAR(v1_final, 1.0f, 0.001f);
    EXPECT_NEAR(v2_final, 4.0f, 0.001f);
    
    float E_initial = 0.5f * m1 * v1 * v1; // 0.5*2*9 = 9
    float E_final = 0.5f * m1 * v1_final * v1_final + 0.5f * m2 * v2_final * v2_final; 
    // 0.5*2*1 + 0.5*1*16 = 1 + 8 = 9
    
    EXPECT_NEAR(E_final, E_initial, 0.001f);
}

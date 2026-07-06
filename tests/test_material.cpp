/**
 * @file test_material.cpp
 * @brief Tests for Material System and Material Library
 */

#include <gtest/gtest.h>
#include "basements/physics/material.h"
#include "basements/physics/material_library.h"
#include <cmath>
#include <iostream>

using namespace basements::physics;

// ============================================================================
// Material Property Tests
// ============================================================================

TEST(MaterialTest, DefaultConstructor) {
    Material m;
    EXPECT_STREQ(m.name, "Unknown");
    EXPECT_GT(m.density, 0.0f);
    EXPECT_GT(m.youngs_modulus, 0.0f);
    EXPECT_GT(m.poisson_ratio, 0.0f);
    EXPECT_LT(m.poisson_ratio, 0.5f);  // Must be < 0.5 for stability
}

TEST(MaterialTest, DerivedProperties) {
    // Steel-like material
    Material m("TestSteel", 7850.0f, 200e9f, 0.3f, 250e6f);
    
    // G = E / (2(1+ν)) = 200e9 / (2 * 1.3) ≈ 76.9e9
    float expected_G = 200e9f / (2.0f * 1.3f);
    EXPECT_NEAR(m.shear_modulus, expected_G, expected_G * 0.01f);
    
    // K = E / (3(1-2ν)) = 200e9 / (3 * 0.4) ≈ 166.7e9
    float expected_K = 200e9f / (3.0f * 0.4f);
    EXPECT_NEAR(m.bulk_modulus, expected_K, expected_K * 0.01f);
    
    std::cout << "[DerivedProperties] G = " << m.shear_modulus / 1e9f 
              << " GPa, K = " << m.bulk_modulus / 1e9f << " GPa" << std::endl;
}

TEST(MaterialTest, StressStrainCalculation) {
    Material m("TestSteel", 7850.0f, 200e9f, 0.3f, 250e6f);
    
    // Apply 0.1% strain
    float strain = 0.001f;
    float stress = m.stress_from_strain(strain);
    
    // σ = E * ε = 200e9 * 0.001 = 200e6 Pa = 200 MPa
    EXPECT_NEAR(stress, 200e6f, 1e6f);
    
    // Reverse calculation
    float recovered_strain = m.strain_from_stress(stress);
    EXPECT_NEAR(recovered_strain, strain, 1e-6f);
    
    std::cout << "[StressStrain] ε = " << strain * 100.0f 
              << "%, σ = " << stress / 1e6f << " MPa" << std::endl;
}

TEST(MaterialTest, YieldCheck) {
    Material m("TestSteel", 7850.0f, 200e9f, 0.3f, 250e6f);
    
    // Below yield
    EXPECT_FALSE(m.is_yielding(200e6f));
    
    // At yield
    EXPECT_TRUE(m.is_yielding(250e6f));
    
    // Above yield
    EXPECT_TRUE(m.is_yielding(300e6f));
}

TEST(MaterialTest, WaveSpeed) {
    Material steel = MaterialLibrary::Steel_Structural();
    
    // Steel longitudinal wave speed ≈ 5000-6000 m/s
    float c_L = steel.wave_speed_longitudinal();
    EXPECT_GT(c_L, 4000.0f);
    EXPECT_LT(c_L, 7000.0f);
    
    // Shear wave speed ≈ 3000 m/s
    float c_T = steel.wave_speed_transverse();
    EXPECT_GT(c_T, 2500.0f);
    EXPECT_LT(c_T, 4000.0f);
    
    std::cout << "[WaveSpeed] Steel: c_L = " << c_L << " m/s, c_T = " << c_T << " m/s" << std::endl;
}

// ============================================================================
// Material Library Tests
// ============================================================================

TEST(MaterialLibraryTest, SteelProperties) {
    Material steel = MaterialLibrary::Steel_Structural();
    
    EXPECT_STREQ(steel.name, "Steel_Structural");
    EXPECT_NEAR(steel.density, 7850.0f, 50.0f);
    EXPECT_NEAR(steel.youngs_modulus, 200e9f, 10e9f);
    EXPECT_NEAR(steel.poisson_ratio, 0.29f, 0.05f);
    EXPECT_GT(steel.yield_strength, 200e6f);
}

TEST(MaterialLibraryTest, AluminumProperties) {
    Material al = MaterialLibrary::Aluminum_6061();
    
    EXPECT_STREQ(al.name, "Aluminum_6061_T6");
    EXPECT_NEAR(al.density, 2700.0f, 50.0f);
    EXPECT_NEAR(al.youngs_modulus, 68.9e9f, 5e9f);
}

TEST(MaterialLibraryTest, RubberProperties) {
    Material rubber = MaterialLibrary::Rubber();
    
    // Rubber should be very soft (low E) and bouncy (high restitution)
    EXPECT_LT(rubber.youngs_modulus, 1e9f);  // << 1 GPa
    EXPECT_GT(rubber.restitution, 0.7f);
    EXPECT_NEAR(rubber.poisson_ratio, 0.49f, 0.01f);  // Nearly incompressible
}

TEST(MaterialLibraryTest, GlassProperties) {
    Material glass = MaterialLibrary::Glass();
    
    // Glass is brittle: low tensile strength, high compressive
    EXPECT_LT(glass.ultimate_strength, 100e6f);
    EXPECT_GT(glass.compressive_strength, 500e6f);
    EXPECT_LT(glass.fracture_toughness, 2e6f);
}

TEST(MaterialLibraryTest, LookupById) {
    Material steel = MaterialLibrary::get_by_id(1001);
    EXPECT_STREQ(steel.name, "Steel_Structural");
    
    Material rubber = MaterialLibrary::get_by_id(3001);
    EXPECT_STREQ(rubber.name, "Rubber_Natural");
    
    Material unknown = MaterialLibrary::get_by_id(99999);
    EXPECT_STREQ(unknown.name, "Unknown");
}

TEST(MaterialLibraryTest, AllMaterialsValid) {
    // Test that all predefined materials have valid derived properties
    uint32_t ids[] = {1001, 1002, 1003, 1010, 1011, 1020, 1030,
                      2001, 2010, 2020,
                      3001, 3010, 3011, 3012,
                      4001, 4002, 4010,
                      9001, 9010};
    
    for (uint32_t id : ids) {
        Material m = MaterialLibrary::get_by_id(id);
        EXPECT_GT(m.shear_modulus, 0.0f) << "Invalid G for material " << m.name;
        EXPECT_GT(m.bulk_modulus, 0.0f) << "Invalid K for material " << m.name;
    }
    
    std::cout << "[AllMaterialsValid] " << sizeof(ids)/sizeof(ids[0]) 
              << " materials validated" << std::endl;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

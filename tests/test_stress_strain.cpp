/**
 * @file test_stress_strain.cpp
 * @brief Tests for Stress/Strain Tensor calculations
 */

#include <gtest/gtest.h>
#include "basements/physics/stress_strain.h"
#include "basements/physics/material.h"
#include "basements/physics/material_library.h"
#include <cmath>
#include <iostream>

using namespace basements::physics;
using namespace basements::math;

// ============================================================================
// SymmetricTensor3 Basic Tests
// ============================================================================

TEST(StressStrainTest, TensorInvariants) {
    // Hydrostatic stress state: σ = p·I
    StressTensor hydrostatic(100.0f, 100.0f, 100.0f, 0.0f, 0.0f, 0.0f);
    
    EXPECT_FLOAT_EQ(hydrostatic.I1(), 300.0f);
    EXPECT_FLOAT_EQ(hydrostatic.hydrostatic(), 100.0f);
    
    // Von Mises should be zero for hydrostatic
    EXPECT_NEAR(hydrostatic.von_mises(), 0.0f, 1e-3f);
    
    std::cout << "[TensorInvariants] Hydrostatic I1 = " << hydrostatic.I1() 
              << ", Von Mises = " << hydrostatic.von_mises() << std::endl;
}

TEST(StressStrainTest, UniaxialStress) {
    // Uniaxial tension: σ_xx = 100 MPa
    StressTensor uniaxial(100e6f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
    // Von Mises for uniaxial = σ_xx
    EXPECT_NEAR(uniaxial.von_mises(), 100e6f, 1e4f);
    
    // Principal stresses
    Vec3 principals = uniaxial.principal_stresses();
    float max_p = std::max({principals.x, principals.y, principals.z});
    EXPECT_NEAR(max_p, 100e6f, 1e4f);
    
    std::cout << "[UniaxialStress] σ_vm = " << uniaxial.von_mises() / 1e6f << " MPa" << std::endl;
}

TEST(StressStrainTest, PureShear) {
    // Pure shear: τ_xy = 50 MPa
    StressTensor shear(0.0f, 0.0f, 0.0f, 50e6f, 0.0f, 0.0f);
    
    // Von Mises for pure shear = sqrt(3) * τ
    float expected_vm = std::sqrt(3.0f) * 50e6f;
    EXPECT_NEAR(shear.von_mises(), expected_vm, 1e4f);
    
    std::cout << "[PureShear] σ_vm = " << shear.von_mises() / 1e6f << " MPa" << std::endl;
}

TEST(StressStrainTest, DeviatoricTensor) {
    StressTensor stress(100.0f, 50.0f, 30.0f, 10.0f, 5.0f, 3.0f);
    
    StressTensor dev = stress.deviatoric();
    
    // Deviatoric trace should be zero
    EXPECT_NEAR(dev.I1(), 0.0f, 1e-3f);
    
    std::cout << "[DeviatoricTensor] Original I1 = " << stress.I1() 
              << ", Deviatoric I1 = " << dev.I1() << std::endl;
}

// ============================================================================
// Hooke's Law Tests
// ============================================================================

TEST(StressStrainTest, HookesLaw_UniaxialTension) {
    Material steel = MaterialLibrary::Steel_Structural();
    
    // Apply 0.1% strain in x-direction
    StrainTensor strain(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
    StressTensor stress = stress_from_strain(strain, steel);
    
    // For constrained uniaxial: σ_xx ≈ E * ε_xx (approximately)
    // With Poisson effect, should be close to 200 MPa
    EXPECT_GT(stress.xx, 150e6f);
    EXPECT_LT(stress.xx, 300e6f);
    
    std::cout << "[HookesLaw_Uniaxial] ε_xx = 0.1%, σ_xx = " 
              << stress.xx / 1e6f << " MPa" << std::endl;
}

TEST(StressStrainTest, HookesLaw_Roundtrip) {
    Material steel = MaterialLibrary::Steel_Structural();
    
    // Create arbitrary strain
    StrainTensor original_strain(0.001f, -0.0003f, -0.0003f, 0.0002f, 0.0001f, 0.00005f);
    
    // Convert to stress and back
    StressTensor stress = stress_from_strain(original_strain, steel);
    StrainTensor recovered_strain = strain_from_stress(stress, steel);
    
    // Should match original
    EXPECT_NEAR(recovered_strain.xx, original_strain.xx, 1e-8f);
    EXPECT_NEAR(recovered_strain.yy, original_strain.yy, 1e-8f);
    EXPECT_NEAR(recovered_strain.zz, original_strain.zz, 1e-8f);
    EXPECT_NEAR(recovered_strain.xy, original_strain.xy, 1e-8f);
    
    std::cout << "[HookesLaw_Roundtrip] Strain recovered successfully" << std::endl;
}

// ============================================================================
// Yield Criteria Tests
// ============================================================================

TEST(StressStrainTest, VonMisesYieldCriterion) {
    Material steel = MaterialLibrary::Steel_Structural();
    
    // Below yield: 200 MPa uniaxial
    StressTensor below_yield(200e6f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    float ratio_below = von_mises_yield_ratio(below_yield, steel);
    EXPECT_LT(ratio_below, 1.0f);
    
    // Above yield: 300 MPa uniaxial
    StressTensor above_yield(300e6f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    float ratio_above = von_mises_yield_ratio(above_yield, steel);
    EXPECT_GT(ratio_above, 1.0f);
    
    std::cout << "[VonMisesYield] 200MPa ratio = " << ratio_below 
              << ", 300MPa ratio = " << ratio_above << std::endl;
}

TEST(StressStrainTest, TrescaYieldCriterion) {
    Material steel = MaterialLibrary::Steel_Structural();
    
    // Pure shear at yield: τ = σ_y / 2
    float yield_shear = steel.yield_strength / 2.0f;
    StressTensor at_yield(0.0f, 0.0f, 0.0f, yield_shear, 0.0f, 0.0f);
    
    float ratio = tresca_yield_ratio(at_yield, steel);
    EXPECT_NEAR(ratio, 1.0f, 0.05f);
    
    std::cout << "[TrescaYield] At yield shear ratio = " << ratio << std::endl;
}

// ============================================================================
// Principal Stresses Test
// ============================================================================

TEST(StressStrainTest, PrincipalStresses) {
    // Known diagonal tensor - principals should equal diagonal
    StressTensor diagonal(100.0f, 50.0f, 25.0f, 0.0f, 0.0f, 0.0f);
    
    Vec3 principals = diagonal.principal_stresses();
    
    // Sort and compare
    float vals[] = {principals.x, principals.y, principals.z};
    std::sort(vals, vals + 3);
    
    EXPECT_NEAR(vals[0], 25.0f, 1.0f);
    EXPECT_NEAR(vals[1], 50.0f, 1.0f);
    EXPECT_NEAR(vals[2], 100.0f, 1.0f);
    
    std::cout << "[PrincipalStresses] σ1=" << vals[2] 
              << ", σ2=" << vals[1] << ", σ3=" << vals[0] << std::endl;
}

// ============================================================================
// Damage Model Test
// ============================================================================

TEST(StressStrainTest, DamageAccumulation) {
    DamageState state;
    
    EXPECT_FLOAT_EQ(state.damage, 0.0f);
    EXPECT_FALSE(state.is_failed());
    
    // Apply strain below threshold
    state.update(0.001f, 0.01f);  // 10% of failure strain
    EXPECT_FLOAT_EQ(state.damage, 0.0f);
    
    // Apply strain above 50% threshold
    state.update(0.006f, 0.01f);  // 60% of failure strain
    EXPECT_GT(state.damage, 0.0f);
    EXPECT_LT(state.damage, 1.0f);
    
    // Apply strain at failure
    state.update(0.01f, 0.01f);
    EXPECT_NEAR(state.damage, 1.0f, 0.01f);
    EXPECT_TRUE(state.is_failed());
    
    std::cout << "[DamageAccumulation] Final damage = " << state.damage << std::endl;
}

TEST(StressStrainTest, StrainEnergyDensity) {
    Material steel = MaterialLibrary::Steel_Structural();
    
    // 0.1% strain
    StrainTensor strain(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    StressTensor stress = stress_from_strain(strain, steel);
    
    float energy = strain_energy_density(stress, strain);
    
    // U ≈ (1/2) * E * ε^2 ≈ 0.5 * 200e9 * 0.001^2 ≈ 100000 J/m³
    EXPECT_GT(energy, 50000.0f);
    EXPECT_LT(energy, 200000.0f);
    
    std::cout << "[StrainEnergyDensity] U = " << energy << " J/m³" << std::endl;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

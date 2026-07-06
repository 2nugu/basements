/**
 * @file test_thermodynamics.cpp
 * @brief Tests for Thermodynamics system
 */

#include <gtest/gtest.h>
#include "basements/physics/thermodynamics.h"
#include "basements/physics/material_library.h"

using namespace basements::physics;

TEST(ThermodynamicsTest, InitialState) {
    ThermalState state;
    EXPECT_FLOAT_EQ(state.temperature, 293.15f); // 20 C
    EXPECT_FLOAT_EQ(state.thermal_energy, 0.0f);
    EXPECT_FLOAT_EQ(state.melting_percent, 0.0f);
    EXPECT_FALSE(state.is_melting);
    EXPECT_FALSE(state.is_fluid);
}

TEST(ThermodynamicsTest, HeatCapacity) {
    Material steel = MaterialLibrary::Steel_Structural(); // Cp = 490 J/kgK
    float mass = 10.0f; // 10 kg
    
    float C = calculate_heat_capacity(steel, mass);
    EXPECT_NEAR(C, 4900.0f, 1.0f);
}

TEST(ThermodynamicsTest, SimpleHeating) {
    Material steel = MaterialLibrary::Steel_Structural();
    float mass = 1.0f;
    ThermalState state(300.0f); // 300 K
    
    // Add 490 J of energy -> Should raise temp by 1 K (since Cp=490)
    apply_thermal_energy(state, steel, mass, 490.0f);
    
    EXPECT_NEAR(state.temperature, 301.0f, 0.1f);
    EXPECT_FLOAT_EQ(state.thermal_energy, 490.0f);
    EXPECT_FALSE(state.is_melting);
}

TEST(ThermodynamicsTest, PhaseChange_Melting) {
    Material ice = MaterialLibrary::Ice();
    float mass = 1.0f;
    // Start just below melting (273 K)
    ThermalState state(272.0f); 
    
    // Ice Cp ~ 2100 (approx from specific_heat), Fusion ~ 334000
    // MaterialLibrary::Ice has specific_heat? Wait, let's check. 
    // Material default specific_heat is 1000. Ice might not set it explicitly in library?
    // Let's rely on whatever is in struct. 
    // apply_thermal_energy uses constant 250000 for fusion.
    
    float C = calculate_heat_capacity(ice, mass);
    
    // 1. Heat to melting point
    // Need (273 - 272) * C energy
    float energy_to_melt_start = 1.0f * C; 
    apply_thermal_energy(state, ice, mass, energy_to_melt_start);
    
    EXPECT_NEAR(state.temperature, 273.0f, 0.1f);
    // Boundary condition: might satisfy >= melting_point due to float precision
    // so we check if melting percent is negligible instead of strictly is_melting == false
    EXPECT_NEAR(state.melting_percent, 0.0f, 1e-4f);
    
    // 2. Add energy for partial melting
    // Add 10% of latent heat (25000 J)
    apply_thermal_energy(state, ice, mass, 25000.0f);
    
    EXPECT_NEAR(state.temperature, 273.0f, 0.1f); // Temp should hold steady
    EXPECT_TRUE(state.is_melting);
    EXPECT_GT(state.melting_percent, 0.0f);
    EXPECT_LT(state.melting_percent, 1.0f);
    
    // 3. Complete melting
    // Add lots of energy
    apply_thermal_energy(state, ice, mass, 500000.0f); 
    
    // Should be fluid and temp > 273
    EXPECT_TRUE(state.is_fluid);
    EXPECT_FALSE(state.is_melting);
    EXPECT_GT(state.temperature, 273.0f);
}

TEST(ThermodynamicsTest, ThermalSoftening) {
    Material steel = MaterialLibrary::Steel_Structural();
    float Tm = steel.melting_point; // 1700 K
    
    // 1. Low temperature (room temp)
    float E_room = get_softened_modulus(steel, 300.0f);
    EXPECT_NEAR(E_room, steel.youngs_modulus, 1e5f); // No softening
    
    // 2. High temperature (80% of melting)
    // 0.8 * 1700 = 1360 K
    // Logic: soft = 1.0 - (0.8 - 0.5)*1.8 = 1 - 0.3*1.8 = 1 - 0.54 = 0.46
    float E_hot = get_softened_modulus(steel, Tm * 0.8f);
    float expected_factor = 1.0f - (0.8f - 0.5f) * 1.8f;
    EXPECT_NEAR(E_hot, steel.youngs_modulus * expected_factor, 1e5f);
    
    // 3. Melted
    float E_melted = get_softened_modulus(steel, Tm + 100.0f);
    EXPECT_LT(E_melted, steel.youngs_modulus * 0.01f); // Should be very soft
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/**
 * @file test_fatigue.cpp
 * @brief Unit tests for Fatigue Analysis based on Miner's Rule
 */

#include <gtest/gtest.h>
#include "basements/physics/fatigue.h"
#include "basements/physics/rigid_body.h"
#include "basements/physics/material_library.h"

using namespace basements;
using namespace basements::physics;

class FatigueTest : public ::testing::Test {
protected:
    RigidBody body;
    Material mat;

    void SetUp() override {
        // Create a custom material for testing
        mat = Material("TestMetal", 7800.0f, 200e9f, 0.3f, 250e6f); // Yield 250MPa
        mat.ultimate_strength = 500e6f; // 500 MPa
        mat.fatigue_limit = 100e6f;     // 100 MPa
        mat.fatigue_exponent = 3.0f;    // Easier calculation: D = (S/Ult)^3
        
        // Register material (Mocking Library behavior or just using ID if local?)
        // MaterialLibrary uses static storage. We might need to rely on it.
        // Or we can just modify body's material properties if we could? 
        // RigidBody hold material_id. FatigueManager calls MaterialLibrary::get_by_id.
        // We need to ensure we can retrieve this material.
        
        // Hack: MaterialLibrary doesn't have a "add_custom" easily exposed usually?
        // Let's check MaterialLibrary.
        // If not, we might need a mock-friendly FatigueManager or just rely on a standard material.
        
        // Alternative: Use a standard material and assert based on its visible properties.
        // MaterialLibrary::get_standard(MaterialCategory::METAL) -> Steel.
    }
};

TEST_F(FatigueTest, DamageAccumulation) {
    // We need a way to mock the material lookup for the body.
    // Since FatigueManager::apply_fatigue calls MaterialLibrary::get_by_id(body.material_id),
    // we must use a valid ID.
    // Let's look at what ID 1 is. Even if we can't change it, we can read it.
    
    // Use ID 1001 (Structural Steel)
    body.material_id = 1001; 
    Material steel = MaterialLibrary::get_by_id(1001);
    
    // Let's use steel's properties.
    // needed: ultimate_strength, fatigue_limit, fatigue_exponent.
    
    if (steel.id == 0) {
        GTEST_SKIP() << "Material ID 1 not found via Library.";
    }

    // 1. Test Below Limit (Infinite Life)
    // Stress < fatigue_limit
    float low_stress = steel.fatigue_limit * 0.9f;
    bool failed = FatigueManager::apply_fatigue(body, low_stress);
    
    EXPECT_FALSE(failed);
    EXPECT_FLOAT_EQ(body.fatigue_state.accumulated_damage, 0.0f);
    
    // 2. Test Above Limit (Accumulate Damage)
    // Stress = 0.5 * Ultimate. 
    // If exponent is e.g. 4.0, Damage = (0.5)^4 = 0.0625.
    float target_stress = steel.ultimate_strength * 0.5f;
    
    // Ensure this target is above limit
    if (target_stress < steel.fatigue_limit) {
        target_stress = steel.ultimate_strength * 0.9f; // Go higher
    }

    float expected_damage_per_hit = std::pow(target_stress / steel.ultimate_strength, steel.fatigue_exponent);
    
    failed = FatigueManager::apply_fatigue(body, target_stress);
    EXPECT_FALSE(failed);
    EXPECT_NEAR(body.fatigue_state.accumulated_damage, expected_damage_per_hit, 1e-5f);
    
    // 3. Test Failure
    // Add massive damage to near 1.0
    body.fatigue_state.accumulated_damage = 0.999f;
    
    // One small hit
    // Even a small hit (above limit) adds damage.
    failed = FatigueManager::apply_fatigue(body, target_stress); // Should tip over 1.0
    
    EXPECT_TRUE(failed);
    EXPECT_GT(body.fatigue_state.accumulated_damage, 1.0f);
}

TEST_F(FatigueTest, CycleCounting) {
    body.material_id = 1001;
    Material steel = MaterialLibrary::get_by_id(1001);
    if (steel.id == 0) GTEST_SKIP();
    
    float heavy_stress = steel.ultimate_strength * 0.8f;
    
    FatigueManager::apply_fatigue(body, heavy_stress);
    EXPECT_EQ(body.fatigue_state.cycle_count, 1);
    
    FatigueManager::apply_fatigue(body, heavy_stress);
    EXPECT_EQ(body.fatigue_state.cycle_count, 2);
}

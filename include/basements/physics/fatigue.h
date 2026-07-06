/**
 * @file fatigue.h
 * @brief Fatigue Analysis using Miner's Rule
 */

#ifndef BASEMENTS_PHYSICS_FATIGUE_H
#define BASEMENTS_PHYSICS_FATIGUE_H

#include "basements/physics/rigid_body.h"
#include "basements/physics/material_library.h"
#include "basements/physics/fracture.h"
#include <algorithm>
#include <cmath>

namespace basements {
namespace physics {

class FatigueManager {
public:
    /**
     * @brief Apply fatigue damage based on current stress cycle.
     * Should be called when a stress peak is detected (e.g., impact).
     * 
     * @param body The rigid body.
     * @param stress_peak Peak stress experienced (Pa).
     * @return true if body failed (fractured) due to fatigue.
     */
    static bool apply_fatigue(RigidBody& body, float stress_peak) {
        if (body.inv_mass == 0.0f) return false;

        Material mat = MaterialLibrary::get_by_id(body.material_id);
        if (mat.id == 0) return false;

        // 1. Check Endurance Limit/Fatigue Limit
        // If stress is below limit, theoretically infinite life (no damage).
        if (stress_peak < mat.fatigue_limit) {
            return false; 
        }

        // 2. Calculate Damage per Cycle (Miner's Rule / Basquin's Equation)
        // N * S^m = C
        // Damage D = 1/N = (S / C)^(1/b) ...
        // Simplified: D = (Stress / UltimateStrength)^m
        // If Stress = Ultimate, D = 1 (Instant failure).
        // If Stress = FatigueLimit, D should be very small (e.g. 1/1e7).
        
        float ratio = stress_peak / mat.ultimate_strength;
        if (ratio > 1.0f) ratio = 1.0f; // Cap at ultimate (handled by fracture check elsewhere usually)

        // Adjust model:
        // Use a simple power law scaled so that at Fatigue Limit, D is negligible.
        // Let's use: D = pow(stress / ultimate, m)
        // Check if this makes physical sense: 
        // If ratio = 0.5 (fatigue limit approx), 0.5^4 = 0.0625. Too high.
        // Real S-N curve: Log-Log linear.
        // N = (Stress / A)^(-m). D = 1/N.
        
        // Let A = Ultimate Strength.
        // D = (Stress / Ultimate)^m
        // If m=10 (steel), ratio=0.5 -> 0.001 (1000 cycles).
        // If m=4, ratio=0.5 -> 0.06 (16 cycles).
        // So m needs to be appropriate.
        // Let's use the material's exponent.
        
        float damage = std::pow(ratio, mat.fatigue_exponent);
        
        // 3. Accumulate
        body.fatigue_state.accumulated_damage += damage;
        body.fatigue_state.last_stress_peak = stress_peak;
        body.fatigue_state.cycle_count++;
        
        // 4. Check Failure
        if (body.fatigue_state.accumulated_damage >= 1.0f) {
            // Trigger Fracture!
            // We need to call FractureManager.
            // But we don't have fragments out vector here.
            // We return true, and caller handles splitting?
            // Or we mark body for destruction?
            return true;
        }
        
        return false;
    }
};

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FATIGUE_H

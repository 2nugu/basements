/**
 * @file thermodynamics.h
 * @brief Thermodynamics and heat transfer logic for physics simulation
 * 
 * Handles temperature tracking, heat capacity, phase changes (melting),
 * and thermal softening of materials.
 */

#ifndef BASEMENTS_PHYSICS_THERMODYNAMICS_H
#define BASEMENTS_PHYSICS_THERMODYNAMICS_H

#include "basements/core/math/common.h"
#include "basements/physics/material.h"
#include <cmath>
#include <algorithm>

namespace basements {
namespace physics {

/**
 * @brief Represents the thermal state of a rigid body
 */
struct ThermalState {
    float temperature;          ///< Current absolute temperature (Kelvin)
    float thermal_energy;       ///< Accumulated internal thermal energy (Joules)
    float melting_percent;      ///< Phase change progress 0.0 (solid) to 1.0 (fluid)
    
    bool is_melting;            ///< Flag: currently undergoing phase transition
    bool is_fluid;              ///< Flag: completely melted
    
    HOST_DEVICE ThermalState(float initial_temp_k = 293.15f) // 20°C
        : temperature(initial_temp_k)
        , thermal_energy(0.0f)
        , melting_percent(0.0f)
        , is_melting(false)
        , is_fluid(false)
    {}
};

// ============================================================================
// Core Calculation Functions
// ============================================================================

/**
 * @brief Calculate total heat capacity of an object
 * @param material Material properties
 * @param mass Mass of object (kg)
 * @return Heat capacity C (J/K)
 */
HOST_DEVICE inline float calculate_heat_capacity(const Material& material, float mass) {
    return material.specific_heat * mass;
}

/**
 * @brief Calculate degraded Young's Modulus due to high temperature
 * 
 * Uses a simplified model where stiffness decreases as temperature approaches melting point.
 * E(T) = E_0 * (1 - (T/T_m)^k) for T < T_m
 * 
 * @param material Material properties
 * @param temperature Current temperature (K)
 * @return Softened Young's Modulus (Pa)
 */
HOST_DEVICE inline float get_softened_modulus(const Material& material, float temperature) {
    if (temperature >= material.melting_point) {
        return material.youngs_modulus * 0.001f; // Residual stiffness (viscous fluid-like)
    }
    
    // Simple linear softening starting from 50% of melting point
    // This is a heuristic approximation for game physics
    float ratio = temperature / material.melting_point;
    float softening_factor = 1.0f;
    
    if (ratio > 0.5f) {
        // Soften from 1.0 at 0.5*Tm down to 0.1 at Tm
        // Linear interpolation: y = 1 - (x - 0.5) * 1.8
        // At x=0.5 -> y=1.0
        // At x=1.0 -> y=0.1
        softening_factor = 1.0f - (ratio - 0.5f) * 1.8f;
        softening_factor = std::max(0.1f, softening_factor);
    }
    
    return material.youngs_modulus * softening_factor;
}

/**
 * @brief Apply thermal energy change and update state (including phase change)
 * 
 * @param state Thermal state to update
 * @param material Material properties
 * @param mass Mass of object (kg)
 * @param energy_input Energy added (Joules), can be negative
 */
HOST_DEVICE inline void apply_thermal_energy(
    ThermalState& state,
    const Material& material,
    float mass,
    float energy_input
) {
    float C = calculate_heat_capacity(material, mass);
    if (C <= 0.0f) return;
    
    // Add to total tracking
    state.thermal_energy += energy_input;
    
    // Latent heat of fusion approximation (if not provided, assume ~200 kJ/kg for generic)
    // J/kg. Standard values: Ice=334k, Steel~270k, Al~390k
    constexpr float LATENT_HEAT_FUSION = 250000.0f; 
    float total_latent_heat = LATENT_HEAT_FUSION * mass;
    
    if (state.is_fluid) {
        // Already fluid, simple temp increase
        state.temperature += energy_input / C;
    } else if (state.is_melting) {
        // Phase change in progress
        // Energy goes into phase change, not temperature
        float percent_change = energy_input / total_latent_heat;
        state.melting_percent += percent_change;
        
        if (state.melting_percent >= 1.0f) {
            // Finished melting - calculate excess energy
            float excess_percent = state.melting_percent - 1.0f;
            float excess_energy = excess_percent * total_latent_heat;
            
            state.melting_percent = 1.0f;
            state.is_melting = false;
            state.is_fluid = true;
            
            // Apply excess energy to temperature
            state.temperature += excess_energy / C;
            
        } else if (state.melting_percent <= 0.0f) {
            // Re-froze
            state.melting_percent = 0.0f;
            state.is_melting = false;
            // Excess cooling lowers temp below melting point
        }
    } else {
        // Solid state
        float delta_T = energy_input / C;
        state.temperature += delta_T;
        
        // Check for melting start
        if (state.temperature >= material.melting_point) {
            float excess_temp = state.temperature - material.melting_point;
            state.temperature = material.melting_point;
            
            state.is_melting = true;
            float excess_energy = excess_temp * C;
            state.melting_percent += excess_energy / total_latent_heat;
        }
    }
    
    // Sanity check
    if (state.temperature < 0.0f) state.temperature = 0.0f; // No negative Kelvin
}

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_THERMODYNAMICS_H

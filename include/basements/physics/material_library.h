/**
 * @file material_library.h
 * @brief Predefined common materials with realistic physical properties
 * 
 * Contains factory functions and constants for common engineering materials.
 * All values are based on typical material data sheets.
 */

#ifndef BASEMENTS_PHYSICS_MATERIAL_LIBRARY_H
#define BASEMENTS_PHYSICS_MATERIAL_LIBRARY_H

#include "basements/physics/material.h"

namespace basements {
namespace physics {

/**
 * @brief Library of predefined materials
 */
namespace MaterialLibrary {

// ============================================================================
// Metals
// ============================================================================

/// Structural steel (A36 equivalent)
inline Material Steel_Structural() {
    Material m("Steel_Structural", 7850.0f, 200e9f, 0.29f, 250e6f);
    m.id = 1001;
    m.ultimate_strength = 400e6f;
    m.compressive_strength = 400e6f;
    m.fracture_toughness = 50e6f;
    m.hardness = 150.0f;  // Brinell ~150
    m.friction_static = 0.74f;
    m.friction_dynamic = 0.57f;
    m.restitution = 0.6f;
    m.thermal_conductivity = 50.0f;
    m.specific_heat = 490.0f;
    m.thermal_expansion = 12e-6f;
    m.melting_point = 1700.0f;
    return m;
}

/// Stainless steel (304)
inline Material Steel_Stainless() {
    Material m("Steel_Stainless_304", 8000.0f, 193e9f, 0.29f, 215e6f);
    m.id = 1002;
    m.ultimate_strength = 505e6f;
    m.hardness = 200.0f;
    m.friction_static = 0.6f;
    m.friction_dynamic = 0.4f;
    m.restitution = 0.55f;
    m.thermal_conductivity = 16.0f;
    m.melting_point = 1673.0f;
    return m;
}

/// Tool steel (hardened)
inline Material Steel_Tool() {
    Material m("Steel_Tool", 7700.0f, 210e9f, 0.30f, 1500e6f);
    m.id = 1003;
    m.ultimate_strength = 2000e6f;
    m.hardness = 600.0f;  // Vickers ~600
    m.restitution = 0.7f;
    return m;
}

/// Aluminum 6061-T6
inline Material Aluminum_6061() {
    Material m("Aluminum_6061_T6", 2700.0f, 68.9e9f, 0.33f, 276e6f);
    m.id = 1010;
    m.ultimate_strength = 310e6f;
    m.fracture_toughness = 29e6f;
    m.hardness = 95.0f;
    m.friction_static = 0.61f;
    m.friction_dynamic = 0.47f;
    m.restitution = 0.65f;
    m.thermal_conductivity = 167.0f;
    m.specific_heat = 896.0f;
    m.thermal_expansion = 23.6e-6f;
    m.melting_point = 855.0f;
    return m;
}

/// Aluminum 7075-T6 (aircraft grade)
inline Material Aluminum_7075() {
    Material m("Aluminum_7075_T6", 2810.0f, 71.7e9f, 0.33f, 503e6f);
    m.id = 1011;
    m.ultimate_strength = 572e6f;
    m.hardness = 175.0f;
    m.restitution = 0.6f;
    m.melting_point = 750.0f;
    return m;
}

/// Copper (pure)
inline Material Copper() {
    Material m("Copper_Pure", 8960.0f, 117e9f, 0.34f, 70e6f);
    m.id = 1020;
    m.ultimate_strength = 220e6f;
    m.hardness = 50.0f;
    m.thermal_conductivity = 401.0f;
    m.specific_heat = 385.0f;
    m.restitution = 0.4f;
    m.melting_point = 1358.0f;
    return m;
}

/// Titanium Ti-6Al-4V (Grade 5)
inline Material Titanium_Grade5() {
    Material m("Titanium_Ti6Al4V", 4430.0f, 113.8e9f, 0.342f, 880e6f);
    m.id = 1030;
    m.ultimate_strength = 950e6f;
    m.fracture_toughness = 75e6f;
    m.hardness = 350.0f;
    m.restitution = 0.5f;
    m.melting_point = 1933.0f;
    return m;
}

// ============================================================================
// Ceramics & Glass
// ============================================================================

/// Soda-lime glass
inline Material Glass() {
    Material m("Glass_SodaLime", 2500.0f, 70e9f, 0.22f, 33e6f);
    m.id = 2001;
    m.ultimate_strength = 33e6f;  // Very brittle
    m.compressive_strength = 1000e6f;  // Strong in compression
    m.fracture_toughness = 0.7e6f;  // Very low
    m.hardness = 550.0f;
    m.friction_static = 0.9f;
    m.friction_dynamic = 0.4f;
    m.restitution = 0.7f;
    m.thermal_conductivity = 1.0f;
    m.melting_point = 1673.0f;
    return m;
}

/// Concrete (typical)
inline Material Concrete() {
    Material m("Concrete", 2400.0f, 30e9f, 0.20f, 3e6f);
    m.id = 2010;
    m.ultimate_strength = 3e6f;
    m.compressive_strength = 30e6f;  // Much stronger in compression
    m.fracture_toughness = 1e6f;
    m.hardness = 400.0f;
    m.friction_static = 0.8f;
    m.friction_dynamic = 0.6f;
    m.restitution = 0.3f;
    return m;
}

/// Ceramic (alumina Al2O3)
inline Material Ceramic_Alumina() {
    Material m("Ceramic_Alumina", 3950.0f, 370e9f, 0.22f, 200e6f);
    m.id = 2020;
    m.ultimate_strength = 300e6f;
    m.compressive_strength = 2600e6f;
    m.fracture_toughness = 4e6f;
    m.hardness = 1500.0f;
    m.restitution = 0.5f;
    m.melting_point = 2345.0f;
    return m;
}

// ============================================================================
// Polymers
// ============================================================================

/// Rubber (natural)
inline Material Rubber() {
    Material m("Rubber_Natural", 1100.0f, 0.01e9f, 0.49f, 15e6f);
    m.id = 3001;
    m.ultimate_strength = 25e6f;
    m.hardness = 40.0f;  // Shore A
    m.friction_static = 1.0f;
    m.friction_dynamic = 0.8f;
    m.restitution = 0.85f;  // Very bouncy
    m.thermal_conductivity = 0.13f;
    return m;
}

/// ABS plastic
inline Material Plastic_ABS() {
    Material m("Plastic_ABS", 1050.0f, 2.3e9f, 0.35f, 45e6f);
    m.id = 3010;
    m.ultimate_strength = 50e6f;
    m.hardness = 100.0f;
    m.friction_static = 0.5f;
    m.friction_dynamic = 0.35f;
    m.restitution = 0.4f;
    m.melting_point = 378.0f;
    return m;
}

/// PLA (3D printing)
inline Material Plastic_PLA() {
    Material m("Plastic_PLA", 1240.0f, 3.5e9f, 0.36f, 60e6f);
    m.id = 3011;
    m.ultimate_strength = 65e6f;
    m.hardness = 80.0f;
    m.restitution = 0.35f;
    m.melting_point = 433.0f;
    return m;
}

/// Nylon (PA6)
inline Material Plastic_Nylon() {
    Material m("Nylon_PA6", 1140.0f, 2.7e9f, 0.40f, 75e6f);
    m.id = 3012;
    m.ultimate_strength = 80e6f;
    m.hardness = 75.0f;
    m.friction_static = 0.35f;
    m.friction_dynamic = 0.25f;
    m.restitution = 0.45f;
    return m;
}

// ============================================================================
// Natural Materials
// ============================================================================

/// Oak wood (along grain)
inline Material Wood_Oak() {
    Material m("Wood_Oak", 750.0f, 12e9f, 0.35f, 40e6f);
    m.id = 4001;
    m.ultimate_strength = 40e6f;
    m.compressive_strength = 50e6f;
    m.hardness = 1360.0f;  // Janka
    m.friction_static = 0.62f;
    m.friction_dynamic = 0.48f;
    m.restitution = 0.4f;
    return m;
}

/// Pine wood
inline Material Wood_Pine() {
    Material m("Wood_Pine", 510.0f, 9e9f, 0.35f, 30e6f);
    m.id = 4002;
    m.ultimate_strength = 30e6f;
    m.hardness = 380.0f;
    m.restitution = 0.35f;
    return m;
}

/// Ice
inline Material Ice() {
    Material m("Ice", 917.0f, 9.33e9f, 0.33f, 1e6f);
    m.id = 4010;
    m.ultimate_strength = 2e6f;
    m.compressive_strength = 5e6f;
    m.hardness = 1.5f;  // Mohs
    m.friction_static = 0.1f;
    m.friction_dynamic = 0.03f;
    m.restitution = 0.3f;
    m.melting_point = 273.0f;
    return m;
}

// ============================================================================
// Special Materials
// ============================================================================

/// Diamond (reference - extremely hard)
inline Material Diamond() {
    Material m("Diamond", 3520.0f, 1220e9f, 0.20f, 2800e6f);
    m.id = 9001;
    m.ultimate_strength = 2800e6f;
    m.hardness = 10000.0f;  // Vickers ~10000
    m.restitution = 0.8f;
    m.thermal_conductivity = 2200.0f;
    m.melting_point = 3820.0f;
    return m;
}

/// Ground/soil (for terrain)
inline Material Soil() {
    Material m("Soil", 1500.0f, 0.05e9f, 0.30f, 0.1e6f);
    m.id = 9010;
    m.friction_static = 0.7f;
    m.friction_dynamic = 0.5f;
    m.restitution = 0.1f;
    return m;
}

// ============================================================================
// Material Lookup
// ============================================================================

/**
 * @brief Get material by ID
 * @param id Material ID
 * @return Material with matching ID (or default if not found)
 */
inline Material get_by_id(uint32_t id) {
    switch (id) {
        case 1001: return Steel_Structural();
        case 1002: return Steel_Stainless();
        case 1003: return Steel_Tool();
        case 1010: return Aluminum_6061();
        case 1011: return Aluminum_7075();
        case 1020: return Copper();
        case 1030: return Titanium_Grade5();
        case 2001: return Glass();
        case 2010: return Concrete();
        case 2020: return Ceramic_Alumina();
        case 3001: return Rubber();
        case 3010: return Plastic_ABS();
        case 3011: return Plastic_PLA();
        case 3012: return Plastic_Nylon();
        case 4001: return Wood_Oak();
        case 4002: return Wood_Pine();
        case 4010: return Ice();
        case 9001: return Diamond();
        case 9010: return Soil();
        default: return Material();
    }
}

} // namespace MaterialLibrary
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_MATERIAL_LIBRARY_H

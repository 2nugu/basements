/**
 * @file material.h
 * @brief Material properties for physics simulation
 * 
 * Defines physical material characteristics including elastic properties,
 * strength limits, and surface friction for realistic deformation and fracture.
 */

#ifndef BASEMENTS_PHYSICS_MATERIAL_H
#define BASEMENTS_PHYSICS_MATERIAL_H

#include "basements/core/math/common.h"
#include <cmath>
#include <cstdint>

namespace basements {
namespace physics {

// Local epsilon for numerical stability
constexpr float MATERIAL_EPSILON = 1e-10f;

/**
 * @brief Physical material properties
 * 
 * Contains all properties needed for realistic material simulation:
 * - Density and mass calculation
 * - Elastic deformation (Hooke's law)
 * - Plastic deformation and fracture limits
 * - Surface interaction (friction, restitution)
 * - Thermal properties (optional)
 */
struct Material {
    // ========================================================================
    // Identification
    // ========================================================================
    
    uint32_t id;            ///< Unique material identifier
    const char* name;       ///< Human-readable name (e.g., "Steel_A36")
    
    // ========================================================================
    // Density & Mass
    // ========================================================================
    
    float density;          ///< Density (kg/m³)
    
    // ========================================================================
    // Elastic Properties (Linear Elasticity)
    // ========================================================================
    
    float youngs_modulus;   ///< Young's modulus E (Pa) - resistance to stretching
    float poisson_ratio;    ///< Poisson's ratio ν (dimensionless, typically 0.2-0.5)
    
    // Derived properties (computed from E and ν)
    float shear_modulus;    ///< Shear modulus G (Pa) = E / (2(1+ν))
    float bulk_modulus;     ///< Bulk modulus K (Pa) = E / (3(1-2ν))
    float lame_lambda;      ///< Lamé's first parameter λ = Eν / ((1+ν)(1-2ν))
    float lame_mu;          ///< Lamé's second parameter μ = G
    
    // ========================================================================
    // Strength Properties (Failure Limits)
    // ========================================================================
    
    float yield_strength;       ///< Yield strength σ_y (Pa) - plastic deformation starts
    float ultimate_strength;    ///< Ultimate tensile strength (Pa) - maximum stress
    float compressive_strength; ///< Compressive strength (Pa) - for brittle materials
    float fracture_toughness;   ///< Fracture toughness K_IC (Pa·√m) - crack resistance
    float fatigue_limit;        ///< Endurance limit (Pa) - infinite cycle stress
    float fatigue_exponent;     ///< Basquin's exponent (m) - typically 3-12 for metals
    
    // ========================================================================
    // Surface Properties
    // ========================================================================
    
    float hardness;             ///< Hardness (Vickers HV or similar)
    float friction_static;      ///< Static friction coefficient μ_s
    float friction_dynamic;     ///< Dynamic friction coefficient μ_k
    float restitution;          ///< Coefficient of restitution (0=plastic, 1=elastic)
    
    // ========================================================================
    // Thermal Properties (Optional)
    // ========================================================================
    
    float thermal_conductivity; ///< Thermal conductivity k (W/(m·K))
    float specific_heat;        ///< Specific heat capacity c_p (J/(kg·K))
    float thermal_expansion;    ///< Linear thermal expansion α (1/K)
    float melting_point;        ///< Melting temperature (K)
    
    // ========================================================================
    // Constructors
    // ========================================================================
    
    HOST_DEVICE Material()
        : id(0)
        , name("Unknown")
        , density(1000.0f)                  // Water density
        , youngs_modulus(1e9f)              // 1 GPa
        , poisson_ratio(0.3f)
        , shear_modulus(0.0f)
        , bulk_modulus(0.0f)
        , lame_lambda(0.0f)
        , lame_mu(0.0f)
        , yield_strength(1e6f)              // 1 MPa
        , ultimate_strength(2e6f)
        , compressive_strength(2e6f)
        , fracture_toughness(1e6f)
        , fatigue_limit(0.5e6f)
        , fatigue_exponent(4.0f)
        , hardness(100.0f)
        , friction_static(0.5f)
        , friction_dynamic(0.3f)
        , restitution(0.5f)
        , thermal_conductivity(1.0f)
        , specific_heat(1000.0f)
        , thermal_expansion(1e-5f)
        , melting_point(1000.0f)
    {
        compute_derived_properties();
    }
    
    /**
     * @brief Construct material with essential properties
     * @param material_name Display name
     * @param material_density Density (kg/m³)
     * @param E Young's modulus (Pa)
     * @param nu Poisson's ratio
     * @param sigma_y Yield strength (Pa)
     */
    HOST_DEVICE Material(
        const char* material_name,
        float material_density,
        float E,
        float nu,
        float sigma_y
    )
        : id(0)
        , name(material_name)
        , density(material_density)
        , youngs_modulus(E)
        , poisson_ratio(nu)
        , shear_modulus(0.0f)
        , bulk_modulus(0.0f)
        , lame_lambda(0.0f)
        , lame_mu(0.0f)
        , yield_strength(sigma_y)
        , ultimate_strength(sigma_y * 1.5f)
        , compressive_strength(sigma_y * 1.5f)
        , fracture_toughness(sigma_y * 0.001f)
        , fatigue_limit(sigma_y * 0.4f)
        , fatigue_exponent(4.0f) // Default for generic
        , hardness(100.0f)
        , friction_static(0.5f)
        , friction_dynamic(0.3f)
        , restitution(0.5f)
        , thermal_conductivity(50.0f)
        , specific_heat(500.0f)
        , thermal_expansion(1.2e-5f)
        , melting_point(1500.0f)
    {
        compute_derived_properties();
    }
    
    // ========================================================================
    // Property Computation
    // ========================================================================
    
    /**
     * @brief Compute derived elastic properties from E and ν
     */
    HOST_DEVICE void compute_derived_properties() {
        // Shear modulus: G = E / (2(1+ν))
        shear_modulus = youngs_modulus / (2.0f * (1.0f + poisson_ratio));
        
        // Bulk modulus: K = E / (3(1-2ν))
        float denom = 3.0f * (1.0f - 2.0f * poisson_ratio);
        bulk_modulus = (denom > MATERIAL_EPSILON) ? youngs_modulus / denom : 1e12f;
        
        // Lamé parameters
        lame_mu = shear_modulus;
        float nu_term = (1.0f + poisson_ratio) * (1.0f - 2.0f * poisson_ratio);
        lame_lambda = (nu_term > MATERIAL_EPSILON) ? 
            youngs_modulus * poisson_ratio / nu_term : 0.0f;
    }
    
    // ========================================================================
    // Stress-Strain Calculations
    // ========================================================================
    
    /**
     * @brief Calculate stress from strain (1D Hooke's Law)
     * @param strain Engineering strain ε = ΔL/L
     * @return Stress σ (Pa)
     */
    HOST_DEVICE float stress_from_strain(float strain) const {
        return youngs_modulus * strain;
    }
    
    /**
     * @brief Calculate strain from stress (1D Hooke's Law)
     * @param stress Engineering stress σ (Pa)
     * @return Strain ε (dimensionless)
     */
    HOST_DEVICE float strain_from_stress(float stress) const {
        return stress / youngs_modulus;
    }
    
    /**
     * @brief Check if stress exceeds yield strength
     * @param stress Von Mises equivalent stress (Pa)
     * @return true if material is yielding (plastic deformation)
     */
    HOST_DEVICE bool is_yielding(float stress) const {
        return std::abs(stress) >= yield_strength;
    }
    
    /**
     * @brief Check if stress exceeds ultimate strength
     * @param stress Von Mises equivalent stress (Pa)
     * @return true if material should fracture
     */
    HOST_DEVICE bool is_fractured(float stress) const {
        return std::abs(stress) >= ultimate_strength;
    }
    
    /**
     * @brief Calculate wave speed in material (for acoustics/impact)
     * @return Longitudinal wave speed (m/s)
     */
    HOST_DEVICE float wave_speed_longitudinal() const {
        // c_L = sqrt((K + 4G/3) / ρ) ≈ sqrt(E/ρ) for ν≈0.3
        float M = bulk_modulus + (4.0f / 3.0f) * shear_modulus;
        return std::sqrt(M / density);
    }
    
    /**
     * @brief Calculate shear wave speed
     * @return Transverse wave speed (m/s)
     */
    HOST_DEVICE float wave_speed_transverse() const {
        // c_T = sqrt(G / ρ)
        return std::sqrt(shear_modulus / density);
    }
};

// ============================================================================
// Material Category Enum
// ============================================================================

enum class MaterialCategory : uint8_t {
    METAL = 0,
    CERAMIC,
    POLYMER,
    COMPOSITE,
    NATURAL,
    FLUID,
    CUSTOM
};

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_MATERIAL_H

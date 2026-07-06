#ifndef BASEMENTS_PHYSICS_FORMULAS_RELATIVITY_H
#define BASEMENTS_PHYSICS_FORMULAS_RELATIVITY_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace relativity {

using namespace basements::math;

constexpr float SPEED_OF_LIGHT_METERS_PER_SECOND = 299792458.0f;  // m/s
constexpr float C_SQUARED = SPEED_OF_LIGHT_METERS_PER_SECOND * SPEED_OF_LIGHT_METERS_PER_SECOND;

// ============================================================
// Lorentz Factor
// ============================================================

/// Calculate Lorentz factor: γ = 1/√(1 - v²/c²)
/// @param velocity_meters_per_second Relative velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Lorentz factor (dimensionless, >= 1)
HOST_DEVICE inline float calculate_lorentz_factor(
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float beta_squared = (velocity_meters_per_second * velocity_meters_per_second) / (c_meters_per_second * c_meters_per_second);
    float denominator = 1.0f - beta_squared;
    return (denominator > EPSILON) ? (1.0f / std::sqrt(denominator)) : 1.0f;
}

/// Calculate Beta (velocity ratio): β = v/c
/// @param velocity_meters_per_second Relative velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Beta value (dimensionless, 0 <= beta < 1)
HOST_DEVICE inline constexpr float calculate_beta(
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return velocity_meters_per_second / c_meters_per_second;
}

// ============================================================
// Time Dilation
// ============================================================

/// Calculate dilated time interval
/// Formula: Δt' = γΔt
/// @param proper_time_seconds Proper time interval (s)
/// @param velocity_meters_per_second Relative velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Dilated time interval in seconds (s)
HOST_DEVICE inline float calculate_time_dilation_seconds(
    float proper_time_seconds,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return gamma * proper_time_seconds;
}

/// Calculate proper time from dilated time
/// Formula: Δt = Δt'/γ
/// @param dilated_time_seconds Dilated time interval (s)
/// @param velocity_meters_per_second Relative velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Proper time interval in seconds (s)
HOST_DEVICE inline float calculate_proper_time_seconds(
    float dilated_time_seconds,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return (gamma > EPSILON) ? (dilated_time_seconds / gamma) : 0.0f;
}

// ============================================================
// Length Contraction
// ============================================================

/// Calculate contracted length
/// Formula: L = L₀/γ = L₀√(1 - v²/c²)
/// @param proper_length_meters Proper length (m)
/// @param velocity_meters_per_second Relative velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Contracted length in meters (m)
HOST_DEVICE inline float calculate_length_contraction_meters(
    float proper_length_meters,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return (gamma > EPSILON) ? (proper_length_meters / gamma) : 0.0f;
}

/// Calculate proper length from contracted length
/// Formula: L₀ = γL
/// @param contracted_length_meters Contracted length (m)
/// @param velocity_meters_per_second Relative velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Proper length in meters (m)
HOST_DEVICE inline float calculate_proper_length_meters(
    float contracted_length_meters,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return gamma * contracted_length_meters;
}

// ============================================================
// Relativistic Momentum
// ============================================================

/// Calculate relativistic momentum
/// Formula: p = γmv
/// @param mass_kilograms Rest mass (kg)
/// @param velocity_meters_per_second Velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Momentum in kilogram meters per second (kg·m/s)
HOST_DEVICE inline float calculate_relativistic_momentum_kg_meters_per_second(
    float mass_kilograms,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return gamma * mass_kilograms * velocity_meters_per_second;
}

/// Calculate velocity from momentum
/// Formula: v = pc/√((pc)² + (mc²)²)
/// @param momentum_kg_meters_per_second Momentum (kg·m/s)
/// @param mass_kilograms Rest mass (kg)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_velocity_from_momentum_meters_per_second(
    float momentum_kg_meters_per_second,
    float mass_kilograms,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float pc = momentum_kg_meters_per_second * c_meters_per_second;
    float mc2 = mass_kilograms * c_meters_per_second * c_meters_per_second;
    float denominator = std::sqrt(pc * pc + mc2 * mc2);
    return (denominator > EPSILON) ? (pc * c_meters_per_second / denominator) : 0.0f;
}

// ============================================================
// Relativistic Energy
// ============================================================

/// Calculate rest mass energy
/// Formula: E₀ = mc²
/// @param mass_kilograms Rest mass (kg)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Rest energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_rest_energy_joules(
    float mass_kilograms,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return mass_kilograms * c_meters_per_second * c_meters_per_second;
}

/// Calculate total relativistic energy
/// Formula: E = γmc²
/// @param mass_kilograms Rest mass (kg)
/// @param velocity_meters_per_second Velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Total energy in Joules (J)
HOST_DEVICE inline float calculate_total_energy_joules(
    float mass_kilograms,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return gamma * mass_kilograms * c_meters_per_second * c_meters_per_second;
}

/// Calculate relativistic kinetic energy
/// Formula: K = (γ - 1)mc²
/// @param mass_kilograms Rest mass (kg)
/// @param velocity_meters_per_second Velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Kinetic energy in Joules (J)
HOST_DEVICE inline float calculate_relativistic_kinetic_energy_joules(
    float mass_kilograms,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return (gamma - 1.0f) * mass_kilograms * c_meters_per_second * c_meters_per_second;
}

/// Calculate total energy from momentum and mass
/// Formula: E² = (pc)² + (mc²)²
/// @param momentum_kg_meters_per_second Momentum (kg·m/s)
/// @param mass_kilograms Rest mass (kg)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Total energy in Joules (J)
HOST_DEVICE inline float calculate_energy_from_momentum_joules(
    float momentum_kg_meters_per_second,
    float mass_kilograms,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float pc = momentum_kg_meters_per_second * c_meters_per_second;
    float mc2 = mass_kilograms * c_meters_per_second * c_meters_per_second;
    return std::sqrt(pc * pc + mc2 * mc2);
}

/// Calculate momentum from total energy
/// Formula: p = √(E² - (mc²)²)/c
/// @param total_energy_joules Total energy (J)
/// @param mass_kilograms Rest mass (kg)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Momentum in kilogram meters per second (kg·m/s)
HOST_DEVICE inline float calculate_momentum_from_energy_kg_meters_per_second(
    float total_energy_joules,
    float mass_kilograms,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float mc2 = mass_kilograms * c_meters_per_second * c_meters_per_second;
    float p_squared_c_squared = total_energy_joules * total_energy_joules - mc2 * mc2;
    return (p_squared_c_squared >= 0.0f) ? 
           (std::sqrt(p_squared_c_squared) / c_meters_per_second) : 0.0f;
}

// ============================================================
// Velocity Addition
// ============================================================

/// Relativistic velocity addition
/// Formula: v = (v₁ + v₂)/(1 + v₁v₂/c²)
/// @param velocity1_meters_per_second First velocity (m/s)
/// @param velocity2_meters_per_second Second velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Resultant velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_relativistic_velocity_addition_meters_per_second(
    float velocity1_meters_per_second,
    float velocity2_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float numerator = velocity1_meters_per_second + velocity2_meters_per_second;
    float denominator = 1.0f + (velocity1_meters_per_second * velocity2_meters_per_second) / (c_meters_per_second * c_meters_per_second);
    return (constexpr_abs(denominator) > EPSILON) ? 
           (numerator / denominator) : 0.0f;
}

/// Relativistic velocity subtraction
/// Formula: v = (v₁ - v₂)/(1 - v₁v₂/c²)
/// @param velocity1_meters_per_second First velocity (m/s)
/// @param velocity2_meters_per_second Second velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Resultant velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_relativistic_velocity_subtraction_meters_per_second(
    float velocity1_meters_per_second,
    float velocity2_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return calculate_relativistic_velocity_addition_meters_per_second(velocity1_meters_per_second, -velocity2_meters_per_second, c_meters_per_second);
}

// ============================================================
// Relativistic Doppler Effect
// ============================================================

/// Calculate relativistic Doppler effect (longitudinal)
/// Formula: f' = f√[(1-β)/(1+β)] (for source moving away)
/// Note: Sign of velocity determines approaching vs receding. Positive v means receding.
/// @param source_frequency_hertz Source frequency (Hz)
/// @param velocity_meters_per_second Relative velocity (m/s), positive if receding
/// @param c_meters_per_second Speed of light (m/s)
/// @return Observed frequency in Hertz (Hz)
HOST_DEVICE inline float calculate_relativistic_doppler_frequency_hertz(
    float source_frequency_hertz,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float b = calculate_beta(velocity_meters_per_second, c_meters_per_second);
    float numerator = 1.0f - b;
    float denominator = 1.0f + b;
    return (denominator > EPSILON) ? 
           (source_frequency_hertz * std::sqrt(numerator / denominator)) : source_frequency_hertz;
}

/// Calculate transverse Doppler effect
/// Formula: f' = f/γ
/// @param source_frequency_hertz Source frequency (Hz)
/// @param velocity_meters_per_second Transverse velocity (m/s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Observed frequency in Hertz (Hz)
HOST_DEVICE inline float calculate_transverse_doppler_frequency_hertz(
    float source_frequency_hertz,
    float velocity_meters_per_second,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float gamma = calculate_lorentz_factor(velocity_meters_per_second, c_meters_per_second);
    return (gamma > EPSILON) ? (source_frequency_hertz / gamma) : 0.0f;
}

// ============================================================
// Mass-Energy Equivalence
// ============================================================

/// Calculate mass from energy
/// Formula: m = E/c²
/// @param energy_joules Energy (J)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Mass in kilograms (kg)
HOST_DEVICE inline constexpr float calculate_mass_from_energy_kilograms(
    float energy_joules,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return energy_joules / (c_meters_per_second * c_meters_per_second);
}

/// Calculate mass defect
/// Formula: Δm = ΔE/c²
/// @param energy_released_joules Energy released (J)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Mass defect in kilograms (kg)
HOST_DEVICE inline constexpr float calculate_mass_defect_kilograms(
    float energy_released_joules,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return energy_released_joules / (c_meters_per_second * c_meters_per_second);
}

// ============================================================
// Schwarzschild Radius (Black Hole)
// ============================================================

constexpr float GRAVITATIONAL_CONSTANT = 6.67430e-11f;

/// Calculate Schwarzschild radius
/// Formula: r_s = 2GM/c²
/// @param mass_kilograms Mass of the object (kg)
/// @param G_constant Gravitational constant
/// @param c_meters_per_second Speed of light (m/s)
/// @return Schwarzschild radius in meters (m)
HOST_DEVICE inline constexpr float calculate_schwarzschild_radius_meters(
    float mass_kilograms,
    float G_constant = GRAVITATIONAL_CONSTANT,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return 2.0f * G_constant * mass_kilograms / (c_meters_per_second * c_meters_per_second);
}

/// Calculate mass from Schwarzschild radius
/// Formula: M = r_s·c²/(2G)
/// @param radius_meters Schwarzschild radius (m)
/// @param G_constant Gravitational constant
/// @param c_meters_per_second Speed of light (m/s)
/// @return Mass in kilograms (kg)
HOST_DEVICE inline constexpr float calculate_mass_from_schwarzschild_kilograms(
    float radius_meters,
    float G_constant = GRAVITATIONAL_CONSTANT,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return (G_constant > EPSILON) ?
           (radius_meters * c_meters_per_second * c_meters_per_second / (2.0f * G_constant)) : 0.0f;
}

// ============================================================
// Gravitational Time Dilation
// ============================================================

/// Calculate gravitational time dilation (for stationary observer at radius r)
/// Formula: Δt' = Δt/√(1 - 2GM/(rc²)) = Δt/√(1 - r_s/r)
/// @param proper_time_seconds Time interval far from mass (s)
/// @param mass_kilograms Mass of the object (kg)
/// @param radius_meters Distance from center of mass (m)
/// @param G_constant Gravitational constant
/// @param c_meters_per_second Speed of light (m/s)
/// @return Dilated time interval at radius r in seconds (s)
HOST_DEVICE inline float calculate_gravitational_time_dilation_seconds(
    float proper_time_seconds,
    float mass_kilograms,
    float radius_meters,
    float G_constant = GRAVITATIONAL_CONSTANT,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    if (radius_meters < EPSILON) return proper_time_seconds;
    
    float schwarzschild_r = calculate_schwarzschild_radius_meters(mass_kilograms, G_constant, c_meters_per_second);
    float factor = schwarzschild_r / radius_meters;
    float denominator = 1.0f - factor;
    
    return (denominator > EPSILON) ? 
           (proper_time_seconds / std::sqrt(denominator)) : proper_time_seconds; // Returns proper_time if inside event horizon for safety
}

// ============================================================
// Redshift
// ============================================================

/// Calculate cosmological redshift
/// Formula: z = (λ_observed - λ_emitted)/λ_emitted
/// @param wavelength_observed_meters Observed wavelength (m)
/// @param wavelength_emitted_meters Emitted wavelength (m)
/// @return Redshift z (dimensionless)
HOST_DEVICE inline constexpr float calculate_redshift(
    float wavelength_observed_meters,
    float wavelength_emitted_meters
) {
    return (wavelength_emitted_meters > EPSILON) ? 
           ((wavelength_observed_meters - wavelength_emitted_meters) / wavelength_emitted_meters) : 0.0f;
}

/// Calculate recession velocity from redshift (non-relativistic approximation)
/// Formula: v ≈ cz
/// @param redshift_z Redshift z
/// @param c_meters_per_second Speed of light (m/s)
/// @return Recession velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_velocity_from_redshift_approx_meters_per_second(
    float redshift_z,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return c_meters_per_second * redshift_z;
}

/// Calculate recession velocity from redshift (relativistic)
/// Formula: v = c(z² + 2z)/(z² + 2z + 2)
/// @param redshift_z Redshift z
/// @param c_meters_per_second Speed of light (m/s)
/// @return Recession velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_velocity_from_redshift_meters_per_second(
    float redshift_z,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float z_sq_plus_2z = redshift_z * redshift_z + 2.0f * redshift_z;
    float denominator = z_sq_plus_2z + 2.0f;
    return (constexpr_abs(denominator) > EPSILON) ? 
           (c_meters_per_second * z_sq_plus_2z / denominator) : 0.0f;
}

} // namespace relativity
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_RELATIVITY_H

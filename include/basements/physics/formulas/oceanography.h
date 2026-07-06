#ifndef BASEMENTS_PHYSICS_FORMULAS_OCEANOGRAPHY_H
#define BASEMENTS_PHYSICS_FORMULAS_OCEANOGRAPHY_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace oceanography {

using namespace basements::math;

constexpr float SEAWATER_DENSITY_KG_PER_CUBIC_METER = 1025.0f;
constexpr float GRAVITY_ACCEL = 9.80665f;
constexpr float EARTH_ROTATION_RATE_RAD_PER_SEC = 7.2921159e-5f;

// ============================================================
// Tidal Forces
// ============================================================

/// Calculate tidal force magnitude (approximate)
/// Formula: F_tide ≈ 2GMm/r³ * R_earth
/// Note: This computes the differential force across Earth's radius.
/// @param mass_body_kilograms Mass of perturbing body (e.g. Moon) (kg)
/// @param distance_meters Distance to perturbing body (m)
/// @param earth_radius_meters Radius of the Earth (m)
/// @param G_constant Gravitational constant
/// @return Tidal acceleration term (m/s²)
HOST_DEVICE inline constexpr float calculate_tidal_acceleration_meters_per_second_squared(
    float mass_body_kilograms,
    float distance_meters,
    float earth_radius_meters,
    float G_constant = 6.67430e-11f
) {
    if (distance_meters < EPSILON) return 0.0f;
    float distance_cubed = distance_meters * distance_meters * distance_meters;
    return 2.0f * G_constant * mass_body_kilograms * earth_radius_meters / distance_cubed;
}

// ============================================================
// Wave Properties
// ============================================================

/// Calculate deep water wave speed
/// Formula: c = √(gλ / 2π)
/// @param wavelength_meters Wavelength (m)
/// @param g_accel Gravity (m/s²)
/// @return Wave speed (m/s)
HOST_DEVICE inline float calculate_deep_water_wave_speed_meters_per_second(
    float wavelength_meters,
    float g_accel = GRAVITY_ACCEL
) {
    return std::sqrt(g_accel * wavelength_meters / (2.0f * PI));
}

/// Calculate shallow water wave speed
/// Formula: c = √(gd)
/// @param depth_meters Water depth (m)
/// @param g_accel Gravity (m/s²)
/// @return Wave speed (m/s)
HOST_DEVICE inline float calculate_shallow_water_wave_speed_meters_per_second(
    float depth_meters,
    float g_accel = GRAVITY_ACCEL
) {
    return std::sqrt(g_accel * depth_meters);
}

/// Calculate wave period
/// Formula: T = λ / c
/// @param wavelength_meters Wavelength (m)
/// @param wave_speed_meters_per_second Wave speed (m/s)
/// @return Wave period (s)
HOST_DEVICE inline constexpr float calculate_wave_period_seconds(
    float wavelength_meters,
    float wave_speed_meters_per_second
) {
    return (wave_speed_meters_per_second > EPSILON) ? (wavelength_meters / wave_speed_meters_per_second) : 0.0f;
}

// ============================================================
// Ocean Currents
// ============================================================

/// Calculate Ekman depth
/// Formula: D_e = π * √(2 * Az / |f|)
/// @param eddy_viscosity_square_meters_per_second Vertical eddy viscosity Az (m²/s)
/// @param coriolis_parameter_per_second Coriolis parameter f (1/s)
/// @return Ekman depth (m)
HOST_DEVICE inline float calculate_ekman_depth_meters(
    float eddy_viscosity_square_meters_per_second,
    float coriolis_parameter_per_second
) {
    float f_abs = std::abs(coriolis_parameter_per_second);
    return (f_abs > EPSILON) 
           ? (PI * std::sqrt(2.0f * eddy_viscosity_square_meters_per_second / f_abs)) 
           : 0.0f;
}

/// Calculate Ekman transport magnitude
/// Formula: M_e = τ / |f|
/// @param wind_stress_newtons_per_square_meter Wind stress magnitude (N/m²)
/// @param coriolis_parameter_per_second Coriolis parameter f (1/s)
/// @return Ekman transport (kg/(m·s))
HOST_DEVICE inline constexpr float calculate_ekman_transport_kg_per_meter_second(
    float wind_stress_newtons_per_square_meter,
    float coriolis_parameter_per_second
) {
    float f_abs = constexpr_abs(coriolis_parameter_per_second);
    return (f_abs > EPSILON) 
           ? (wind_stress_newtons_per_square_meter / f_abs) 
           : 0.0f;
}

/// Calculate Geostrophic current speed
/// Formula: v_g = (g/f) * slope
/// @param surface_slope Slope of the sea surface (dimensionless, dh/dx)
/// @param coriolis_parameter_per_second Coriolis parameter (1/s)
/// @param g_accel Gravity (m/s²)
/// @return Current speed (m/s)
HOST_DEVICE inline constexpr float calculate_geostrophic_current_speed_meters_per_second(
    float surface_slope,
    float coriolis_parameter_per_second,
    float g_accel = GRAVITY_ACCEL
) {
    if (constexpr_abs(coriolis_parameter_per_second) < EPSILON) return 0.0f;
    return (g_accel / coriolis_parameter_per_second) * surface_slope;
}

// ============================================================
// Density & Pressure
// ============================================================

/// Calculate seawater density (Simplified Linear Equation of State)
/// Formula: ρ = ρ₀(1 - α(T - T₀) + β(S - S₀))
/// Note: Very simplified.
/// @param temperature_celsius Temperature (°C)
/// @param salinity_ppt Salinity (parts per thousand)
/// @param reference_density_kg_per_cubic_meter ρ₀
/// @return Density (kg/m³)
HOST_DEVICE inline constexpr float calculate_seawater_density_approx_kg_per_cubic_meter(
    float temperature_celsius,
    float salinity_ppt,
    float reference_density_kg_per_cubic_meter = 1025.0f
) {
    // Coefficients for approximation near standard ocean conditions
    const float alpha = 0.0002f; // Thermal expansion coefficient (1/K)
    const float beta = 0.0008f;  // Saline contraction coefficient (1/ppt)
    const float T0 = 10.0f;
    const float S0 = 35.0f;

    return reference_density_kg_per_cubic_meter * (1.0f - alpha * (temperature_celsius - T0) + beta * (salinity_ppt - S0));
}

/// Calculate hydrostatic pressure in ocean
/// Formula: P = P_atm + ρgz
/// @param depth_meters Depth (m)
/// @param density_kg_per_cubic_meter Avg density (kg/m³)
/// @param atmospheric_pressure_pascals Atmospheric pressure (Pa)
/// @param g_accel Gravity (m/s²)
/// @return Pressure (Pa)
HOST_DEVICE inline constexpr float calculate_ocean_hydrostatic_pressure_pascals(
    float depth_meters,
    float density_kg_per_cubic_meter = SEAWATER_DENSITY_KG_PER_CUBIC_METER,
    float atmospheric_pressure_pascals = 101325.0f,
    float g_accel = GRAVITY_ACCEL
) {
    return atmospheric_pressure_pascals + density_kg_per_cubic_meter * g_accel * depth_meters;
}

/// Calculate Buoyancy Frequency (Brunt-Vaisala Frequency) squared
/// Formula: N² = -(g/ρ)(dρ/dz)
/// @param density_gradient_per_meter Vertical density gradient dρ/dz (kg/m⁴) - usually negative for stability
/// @param density_kg_per_cubic_meter Reference density (kg/m³)
/// @param g_accel Gravity (m/s²)
/// @return N squared (rad²/s²)
HOST_DEVICE inline constexpr float calculate_buoyancy_frequency_squared_radians_per_second_squared(
    float density_gradient_per_meter,
    float density_kg_per_cubic_meter,
    float g_accel = GRAVITY_ACCEL
) {
    if (density_kg_per_cubic_meter <= EPSILON) return 0.0f;
    return -(g_accel / density_kg_per_cubic_meter) * density_gradient_per_meter;
}

} // namespace oceanography
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_OCEANOGRAPHY_H

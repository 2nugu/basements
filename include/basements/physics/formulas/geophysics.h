#ifndef BASEMENTS_PHYSICS_FORMULAS_GEOPHYSICS_H
#define BASEMENTS_PHYSICS_FORMULAS_GEOPHYSICS_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace geophysics {

using namespace basements::math;

constexpr float STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665f;
constexpr float EARTH_RADIUS_METERS = 6371000.0f;
constexpr float EARTH_MASS_KILOGRAMS = 5.972e24f;
constexpr float GRAVITATIONAL_CONSTANT = 6.67430e-11f;

// ============================================================
// Gravitational Variation
// ============================================================

/// Calculate gravity at altitude
/// Formula: g(h) = g₀(R/(R+h))²
/// @param altitude_meters Altitude above surface (m)
/// @param g0_meters_per_second_squared Surface gravity (m/s²)
/// @param earth_radius_meters Radius of Earth (m)
/// @return Gravity at altitude in m/s²
HOST_DEVICE inline constexpr float calculate_gravity_at_altitude_meters_per_second_squared(
    float altitude_meters,
    float g0_meters_per_second_squared = STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED,
    float earth_radius_meters = EARTH_RADIUS_METERS
) {
    float denominator = earth_radius_meters + altitude_meters;
    if (constexpr_abs(denominator) < EPSILON) return 0.0f;
    float ratio = earth_radius_meters / denominator;
    return g0_meters_per_second_squared * ratio * ratio;
}

/// Calculate altitude from gravity
/// Formula: h = R(√(g₀/g) - 1)
/// @param gravity_meters_per_second_squared Gravity at altitude (m/s²)
/// @param g0_meters_per_second_squared Surface gravity (m/s²)
/// @param earth_radius_meters Radius of Earth (m)
/// @return Altitude in meters (m)
HOST_DEVICE inline float calculate_altitude_from_gravity_meters(
    float gravity_meters_per_second_squared,
    float g0_meters_per_second_squared = STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED,
    float earth_radius_meters = EARTH_RADIUS_METERS
) {
    if (gravity_meters_per_second_squared < EPSILON) return 0.0f;
    return earth_radius_meters * (std::sqrt(g0_meters_per_second_squared / gravity_meters_per_second_squared) - 1.0f);
}

/// Calculate gravity at depth (assuming uniform density)
/// Formula: g(d) = g₀(1 - d/R)
/// @param depth_meters Depth below surface (m)
/// @param g0_meters_per_second_squared Surface gravity (m/s²)
/// @param earth_radius_meters Radius of Earth (m)
/// @return Gravity at depth in m/s²
HOST_DEVICE inline constexpr float calculate_gravity_at_depth_meters_per_second_squared(
    float depth_meters,
    float g0_meters_per_second_squared = STANDARD_GRAVITY_METERS_PER_SECOND_SQUARED,
    float earth_radius_meters = EARTH_RADIUS_METERS
) {
    if (earth_radius_meters < EPSILON) return 0.0f;
    return g0_meters_per_second_squared * (1.0f - depth_meters / earth_radius_meters);
}

// ============================================================
// Seismic Waves
// ============================================================

/// Calculate P-wave velocity
/// Formula: v_p = √((K + 4μ/3)/ρ)
/// @param bulk_modulus_pascals Bulk modulus (Pa)
/// @param shear_modulus_pascals Shear modulus (Pa)
/// @param density_kg_per_cubic_meter Density (kg/m³)
/// @return P-wave velocity in m/s
HOST_DEVICE inline float calculate_p_wave_velocity_meters_per_second(
    float bulk_modulus_pascals,
    float shear_modulus_pascals,
    float density_kg_per_cubic_meter
) {
    if (density_kg_per_cubic_meter < EPSILON) return 0.0f;
    float numerator = bulk_modulus_pascals + (4.0f / 3.0f) * shear_modulus_pascals;
    return std::sqrt(numerator / density_kg_per_cubic_meter);
}

/// Calculate S-wave velocity
/// Formula: v_s = √(μ/ρ)
/// @param shear_modulus_pascals Shear modulus (Pa)
/// @param density_kg_per_cubic_meter Density (kg/m³)
/// @return S-wave velocity in m/s
HOST_DEVICE inline float calculate_s_wave_velocity_meters_per_second(
    float shear_modulus_pascals,
    float density_kg_per_cubic_meter
) {
    if (density_kg_per_cubic_meter < EPSILON) return 0.0f;
    return std::sqrt(shear_modulus_pascals / density_kg_per_cubic_meter);
}

/// Calculate seismic distance from travel time
/// Formula: d = vt
/// @param velocity_meters_per_second Wave velocity (m/s)
/// @param time_seconds Travel time (s)
/// @return Distance in meters (m)
HOST_DEVICE inline constexpr float calculate_seismic_distance_meters(
    float velocity_meters_per_second,
    float time_seconds
) {
    return velocity_meters_per_second * time_seconds;
}

/// Calculate epicenter distance from S-P time difference (approximated)
/// Formula: d ≈ 8 * t_sp (km) -> converted to meters
/// @param sp_time_diff_seconds Time difference between S and P waves (s)
/// @return Distance in meters (m)
HOST_DEVICE inline constexpr float calculate_epicenter_distance_from_sp_time_meters(
    float sp_time_diff_seconds
) {
    return 8000.0f * sp_time_diff_seconds; // 8 km/s approximation
}

// ============================================================
// Earthquake Magnitude
// ============================================================

/// Calculate Richter magnitude
/// Formula: M_L = log₁₀(A) + f(Δ) roughly log10(A) + 3*log10(8*delta_t) - 2.92
/// simplified: M_L = log₁₀(A) + distance_correction
/// @param amplitude_micrometers Amplitude in micrometers
/// @param distance_correction Distance correction factor
/// @return Richter magnitude
HOST_DEVICE inline float calculate_richter_magnitude(
    float amplitude_micrometers,
    float distance_correction = 0.0f
) {
    if (amplitude_micrometers <= EPSILON) return 0.0f;
    return std::log10(amplitude_micrometers) + distance_correction;
}

/// Calculate Moment Magnitude
/// Formula: M_w = (2/3)log₁₀(M₀) - 10.7
/// @param seismic_moment_newton_meters Seismic moment (N·m)
/// @return Moment magnitude
HOST_DEVICE inline float calculate_moment_magnitude(
    float seismic_moment_newton_meters
) {
    if (seismic_moment_newton_meters <= EPSILON) return 0.0f;
    return (2.0f / 3.0f) * std::log10(seismic_moment_newton_meters) - 10.7f;
}

/// Calculate Seismic Moment
/// Formula: M₀ = μAD
/// @param shear_modulus_pascals Shear modulus (Pa)
/// @param rupture_area_square_meters Rupture area (m²)
/// @param average_slip_meters Average slip (m)
/// @return Seismic moment in Newton meters (N·m)
HOST_DEVICE inline constexpr float calculate_seismic_moment_newton_meters(
    float shear_modulus_pascals,
    float rupture_area_square_meters,
    float average_slip_meters
) {
    return shear_modulus_pascals * rupture_area_square_meters * average_slip_meters;
}

/// Calculate earthquake energy from magnitude
/// Formula: log₁₀(E) = 4.8 + 1.5M_s (Joules, classic Gutenberg-Richter)
/// Or M_w definition: log10(E) = 5.24 + 1.44M_w (Joules) -- Common approximation: E = 10^(11.8 + 1.5M)
/// @param magnitude Earthquake magnitude
/// @return Energy in Joules (J)
HOST_DEVICE inline float calculate_earthquake_energy_joules(
    float magnitude
) {
    return std::pow(10.0f, 11.8f + 1.5f * magnitude); // Energy in ergs is 11.8+1.5M, but typically converted
}

// ============================================================
// Plate Tectonics
// ============================================================

/// Calculate plate velocity
/// Formula: v = d/t
/// @param distance_meters Distance moved (m)
/// @param time_seconds Time elapsed (s)
/// @return Velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_plate_velocity_meters_per_second(
    float distance_meters,
    float time_seconds
) {
    return (time_seconds > EPSILON) ? (distance_meters / time_seconds) : 0.0f;
}

/// Calculate strain rate
/// Formula: ε̇ = Δv/L
/// @param velocity_diff_meters_per_second Velocity difference (m/s)
/// @param length_meters Length scale (m)
/// @return Strain rate in per second (1/s)
HOST_DEVICE inline constexpr float calculate_strain_rate_per_second(
    float velocity_diff_meters_per_second,
    float length_meters
) {
    return (length_meters > EPSILON) ? (velocity_diff_meters_per_second / length_meters) : 0.0f;
}

// ============================================================
// Geothermal
// ============================================================

/// Calculate temperature at depth
/// Formula: T(z) = T₀ + Γz
/// @param surface_temp_kelvin Surface temperature (K)
/// @param geothermal_gradient_kelvin_per_meter Geothermal gradient (K/m)
/// @param depth_meters Depth (m)
/// @return Temperature in Kelvin (K)
HOST_DEVICE inline constexpr float calculate_temperature_at_depth_kelvin(
    float surface_temp_kelvin,
    float geothermal_gradient_kelvin_per_meter,
    float depth_meters
) {
    return surface_temp_kelvin + geothermal_gradient_kelvin_per_meter * depth_meters;
}

/// Calculate geothermal heat flow
/// Formula: q = -kΓ
/// @param thermal_conductivity_watts_per_meter_kelvin Thermal conductivity (W/(m·K))
/// @param geothermal_gradient_kelvin_per_meter Geothermal gradient (K/m)
/// @return Heat flow in Watts per square meter (W/m²)
HOST_DEVICE inline constexpr float calculate_geothermal_heat_flow_watts_per_square_meter(
    float thermal_conductivity_watts_per_meter_kelvin,
    float geothermal_gradient_kelvin_per_meter
) {
    return -thermal_conductivity_watts_per_meter_kelvin * geothermal_gradient_kelvin_per_meter;
}

} // namespace geophysics
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_GEOPHYSICS_H

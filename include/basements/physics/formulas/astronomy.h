#ifndef BASEMENTS_PHYSICS_FORMULAS_ASTRONOMY_H
#define BASEMENTS_PHYSICS_FORMULAS_ASTRONOMY_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace astronomy {

using namespace basements::math;

constexpr float GRAVITATIONAL_CONSTANT = 6.67430e-11f;
constexpr float SOLAR_MASS_KILOGRAMS = 1.989e30f;
constexpr float SOLAR_LUMINOSITY_WATTS = 3.828e26f;
constexpr float STEFAN_BOLTZMANN_CONSTANT = 5.670374419e-8f; // W⋅m⁻²⋅K⁻⁴
constexpr float PARSEC_METERS = 3.0857e16f;
constexpr float LIGHT_YEAR_METERS = 9.4607e15f;
constexpr float ASTRONOMICAL_UNIT_METERS = 1.496e11f;

// ============================================================
// Kepler's Laws & Orbital Mechanics
// ============================================================

/// Calculate orbital period using Kepler's 3rd Law
/// Formula: T = 2π * √(a³ / (GM))
/// @param semi_major_axis_meters Semi-major axis (m)
/// @param central_mass_kilograms Mass of central body (kg)
/// @param G_constant Gravitational constant
/// @return Orbital period in seconds (s)
HOST_DEVICE inline float calculate_orbital_period_seconds(
    float semi_major_axis_meters,
    float central_mass_kilograms,
    float G_constant = GRAVITATIONAL_CONSTANT
) {
    if (central_mass_kilograms <= EPSILON) return 0.0f;
    float a_cubed = semi_major_axis_meters * semi_major_axis_meters * semi_major_axis_meters;
    return TWO_PI * std::sqrt(a_cubed / (G_constant * central_mass_kilograms));
}

/// Calculate orbital velocity (Vis-viva equation)
/// Formula: v = √(GM * (2/r - 1/a))
/// @param distance_meters Current distance from center (r) (m)
/// @param semi_major_axis_meters Semi-major axis (a) (m)
/// @param central_mass_kilograms Mass of central body (kg)
/// @param G_constant Gravitational constant
/// @return Velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_orbital_velocity_meters_per_second(
    float distance_meters,
    float semi_major_axis_meters,
    float central_mass_kilograms,
    float G_constant = GRAVITATIONAL_CONSTANT
) {
    if (distance_meters <= EPSILON || semi_major_axis_meters <= EPSILON) return 0.0f;
    float term = (2.0f / distance_meters) - (1.0f / semi_major_axis_meters);
    return std::sqrt(G_constant * central_mass_kilograms * term);
}

/// Calculate escape velocity
/// Formula: v_e = √(2GM/r)
/// @param distance_meters Distance from center of mass (m)
/// @param central_mass_kilograms Mass of central body (kg)
/// @param G_constant Gravitational constant
/// @return Escape velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_escape_velocity_meters_per_second(
    float distance_meters,
    float central_mass_kilograms,
    float G_constant = GRAVITATIONAL_CONSTANT
) {
    if (distance_meters <= EPSILON) return 0.0f;
    return std::sqrt((2.0f * G_constant * central_mass_kilograms) / distance_meters);
}

// ============================================================
// Stellar Physics
// ============================================================

/// Calculate stellar luminosity (Stefan-Boltzmann Law)
/// Formula: L = 4πR²σT⁴
/// @param radius_meters Star radius (m)
/// @param temperature_kelvin Surface temperature (K)
/// @param sigma_constant Stefan-Boltzmann constant
/// @return Luminosity in Watts (W)
HOST_DEVICE inline constexpr float calculate_stellar_luminosity_watts(
    float radius_meters,
    float temperature_kelvin,
    float sigma_constant = STEFAN_BOLTZMANN_CONSTANT
) {
    float area = 4.0f * PI * radius_meters * radius_meters;
    float temp_pow4 = temperature_kelvin * temperature_kelvin;
    temp_pow4 *= temp_pow4;
    return area * sigma_constant * temp_pow4;
}

/// Calculate Wien's Displacement Law (Peak Wavelength)
/// Formula: λ_max = b / T
/// @param temperature_kelvin Surface temperature (K)
/// @return Peak wavelength in meters (m)
HOST_DEVICE inline constexpr float calculate_wien_peak_wavelength_meters(
    float temperature_kelvin
) {
    const float b_wien_constant = 2.897771955e-3f; // m·K
    return (temperature_kelvin > EPSILON) ? (b_wien_constant / temperature_kelvin) : 0.0f;
}

// ============================================================
// Cosmology & Distance
// ============================================================

/// Calculate distance modulus
/// Formula: m - M = 5 * log10(d) - 5 (where d is in parsecs)
/// @param apparent_magnitude m
/// @param absolute_magnitude M
/// @return Distance in Parsecs (pc)
HOST_DEVICE inline float calculate_distance_from_modulus_parsecs(
    float apparent_magnitude,
    float absolute_magnitude
) {
    float modulus = apparent_magnitude - absolute_magnitude;
    return std::pow(10.0f, (modulus + 5.0f) / 5.0f);
}

/// Calculate Hubble Flow Velocity (Hubble's Law)
/// Formula: v = H₀ * d
/// @param distance_megaparsecs Distance in Megaparsecs (Mpc)
/// @param hubble_constant_km_s_mpc Hubble constant (km/s/Mpc, default ~70)
/// @return Recession velocity in km/s
HOST_DEVICE inline constexpr float calculate_hubble_recession_velocity_km_per_second(
    float distance_megaparsecs,
    float hubble_constant_km_s_mpc = 70.0f
) {
    return hubble_constant_km_s_mpc * distance_megaparsecs;
}

/// Calculate Schwarzschild Radius
/// Formula: R_s = 2GM/c²
/// @param mass_kilograms Mass (kg)
/// @param G_constant Gravitational Constant
/// @param c_speed_of_light Speed of light (m/s)
/// @return Schwarzschild radius in meters (m)
HOST_DEVICE inline constexpr float calculate_schwarzschild_radius_meters(
    float mass_kilograms,
    float G_constant = GRAVITATIONAL_CONSTANT,
    float c_speed_of_light = 299792458.0f
) {
    return (2.0f * G_constant * mass_kilograms) / (c_speed_of_light * c_speed_of_light);
}

} // namespace astronomy
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_ASTRONOMY_H

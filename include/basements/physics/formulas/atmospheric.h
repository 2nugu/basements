#ifndef BASEMENTS_PHYSICS_FORMULAS_ATMOSPHERIC_H
#define BASEMENTS_PHYSICS_FORMULAS_ATMOSPHERIC_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace atmospheric {

using namespace basements::math;

constexpr float GAS_CONSTANT_DRY_AIR = 287.058f;      // J/(kg·K)
constexpr float SPECIFIC_HEAT_CP = 1005.0f;           // J/(kg·K)
constexpr float STANDARD_PRESSURE_PASCALS = 101325.0f; // Pa
constexpr float EARTH_ROTATION_RATE_RAD_PER_SEC = 7.2921159e-5f; // rad/s
constexpr float GRAVITY_ACCEL = 9.80665f;             // m/s²

// ============================================================
// Ideal Gas Law (Atmosphere)
// ============================================================

/// Calculate pressure from density using ideal gas law
/// Formula: P = ρRT
/// @param density_kg_per_cubic_meter Air density (kg/m³)
/// @param temperature_kelvin Temperature (K)
/// @param R_gas_constant Gas constant (J/(kg·K))
/// @return Pressure in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_pressure_from_density_pascals(
    float density_kg_per_cubic_meter,
    float temperature_kelvin,
    float R_gas_constant = GAS_CONSTANT_DRY_AIR
) {
    return density_kg_per_cubic_meter * R_gas_constant * temperature_kelvin;
}

/// Calculate density from pressure using ideal gas law
/// Formula: ρ = P/(RT)
/// @param pressure_pascals Pressure (Pa)
/// @param temperature_kelvin Temperature (K)
/// @param R_gas_constant Gas constant (J/(kg·K))
/// @return Density in kg/m³
HOST_DEVICE inline constexpr float calculate_density_from_pressure_kg_per_cubic_meter(
    float pressure_pascals,
    float temperature_kelvin,
    float R_gas_constant = GAS_CONSTANT_DRY_AIR
) {
    float denominator = R_gas_constant * temperature_kelvin;
    return (constexpr_abs(denominator) > EPSILON) ? (pressure_pascals / denominator) : 0.0f;
}

// ============================================================
// Hydrostatic Equation & Barometric Formula
// ============================================================

/// Calculate pressure at altitude assuming isothermal atmosphere (Barometric formula)
/// Formula: P(z) = P₀e^(-g z / (R T))
/// @param surface_pressure_pascals Pressure at reference level (Pa)
/// @param altitude_meters Altitude relative to reference (m)
/// @param temperature_kelvin Constant temperature (K)
/// @param gravity_accel Gravity (m/s²)
/// @param R_gas_constant Gas constant (J/(kg·K))
/// @return Pressure at altitude (Pa)
HOST_DEVICE inline float calculate_pressure_at_altitude_pascals(
    float surface_pressure_pascals,
    float altitude_meters,
    float temperature_kelvin,
    float gravity_accel = GRAVITY_ACCEL,
    float R_gas_constant = GAS_CONSTANT_DRY_AIR
) {
    float exponent = -(gravity_accel * altitude_meters) / (R_gas_constant * temperature_kelvin);
    return surface_pressure_pascals * std::exp(exponent);
}

/// Calculate altitude from pressure
/// Formula: z = -(RT/g)ln(P/P₀)
/// @param pressure_pascals Pressure at altitude (Pa)
/// @param surface_pressure_pascals Reference pressure (Pa)
/// @param temperature_kelvin Constant temperature (K)
/// @param gravity_accel Gravity (m/s²)
/// @param R_gas_constant Gas constant (J/(kg·K))
/// @return Altitude in meters (m)
HOST_DEVICE inline float calculate_altitude_from_pressure_meters(
    float pressure_pascals,
    float surface_pressure_pascals,
    float temperature_kelvin,
    float gravity_accel = GRAVITY_ACCEL,
    float R_gas_constant = GAS_CONSTANT_DRY_AIR
) {
    if (pressure_pascals <= EPSILON || surface_pressure_pascals <= EPSILON) return 0.0f;
    float scale_height = (R_gas_constant * temperature_kelvin) / gravity_accel;
    return -scale_height * std::log(pressure_pascals / surface_pressure_pascals);
}

// ============================================================
// Adiabatic Processes
// ============================================================

/// Calculate dry adiabatic lapse rate
/// Formula: Γ_d = g/c_p
/// @param gravity_accel Gravity (m/s²)
/// @param specific_heat_cp Specific heat at constant pressure (J/(kg·K))
/// @return Lapse rate in K/m
HOST_DEVICE inline constexpr float calculate_dry_adiabatic_lapse_rate_kelvin_per_meter(
    float gravity_accel = GRAVITY_ACCEL,
    float specific_heat_cp = SPECIFIC_HEAT_CP
) {
    return (specific_heat_cp > EPSILON) ? (gravity_accel / specific_heat_cp) : 0.0f;
}

/// Calculate potential temperature
/// Formula: θ = T(P₀/P)^(R/c_p)
/// @param temperature_kelvin Actual temperature (K)
/// @param pressure_pascals Actual pressure (Pa)
/// @param reference_pressure_pascals Reference pressure (Pa)
/// @param R_gas_constant Gas constant
/// @param specific_heat_cp Specific heat
/// @return Potential temperature in Kelvin (K)
HOST_DEVICE inline float calculate_potential_temperature_kelvin(
    float temperature_kelvin,
    float pressure_pascals,
    float reference_pressure_pascals = STANDARD_PRESSURE_PASCALS,
    float R_gas_constant = GAS_CONSTANT_DRY_AIR,
    float specific_heat_cp = SPECIFIC_HEAT_CP
) {
    if (pressure_pascals <= EPSILON) return 0.0f;
    float kappa = R_gas_constant / specific_heat_cp;
    return temperature_kelvin * std::pow(reference_pressure_pascals / pressure_pascals, kappa);
}

// ============================================================
// Humidity
// ============================================================

/// Calculate saturation vapor pressure (Magnus formula approximation)
/// Formula: e_s ≈ 611.2 * exp(17.67T / (T + 243.5))  [T in Celsius]
/// @param temperature_celsius Temperature (°C)
/// @return Saturation vapor pressure in Pascals (Pa)
HOST_DEVICE inline float calculate_saturation_vapor_pressure_pascals(
    float temperature_celsius
) {
    return 611.2f * std::exp((17.67f * temperature_celsius) / (temperature_celsius + 243.5f));
}

/// Calculate relative humidity
/// Formula: RH = (e/e_s) * 100
/// @param vapor_pressure_pascals Actual vapor pressure (Pa)
/// @param saturation_pressure_pascals Saturation vapor pressure (Pa)
/// @return Relative humidity (%)
HOST_DEVICE inline constexpr float calculate_relative_humidity_percent(
    float vapor_pressure_pascals,
    float saturation_pressure_pascals
) {
    return (saturation_pressure_pascals > EPSILON) 
           ? (vapor_pressure_pascals / saturation_pressure_pascals) * 100.0f 
           : 0.0f;
}

/// Calculate dew point (Magnus formula approximation)
/// Formula: T_d = (c * gamma) / (b - gamma), where gamma = ln(RH/100) + (bT)/(c+T)
/// Coefficients b=17.67, c=243.5
/// @param temperature_celsius Current temperature (°C)
/// @param relative_humidity_percent Relative humidity (%)
/// @return Dew point temperature in Celsius (°C)
HOST_DEVICE inline float calculate_dew_point_celsius(
    float temperature_celsius,
    float relative_humidity_percent
) {
    if (relative_humidity_percent <= EPSILON) return -273.15f;
    float b = 17.67f;
    float c = 243.5f;
    float gamma = std::log(relative_humidity_percent / 100.0f) + (b * temperature_celsius) / (c + temperature_celsius);
    return (c * gamma) / (b - gamma);
}

// ============================================================
// Wind & Force
// ============================================================

/// Calculate pressure gradient force magnitude
/// Formula: F_pg = (1/ρ) * |∇P|
/// @param pressure_gradient_pascals_per_meter Pressure gradient magnitude (Pa/m)
/// @param density_kg_per_cubic_meter Air density (kg/m³)
/// @return Force per unit mass (m/s²)
HOST_DEVICE inline constexpr float calculate_pressure_gradient_force_magnitude(
    float pressure_gradient_pascals_per_meter,
    float density_kg_per_cubic_meter
) {
    return (density_kg_per_cubic_meter > EPSILON) 
           ? (pressure_gradient_pascals_per_meter / density_kg_per_cubic_meter) 
           : 0.0f;
}

/// Calculate Coriolis parameter
/// Formula: f = 2Ω sin(φ)
/// @param latitude_radians Latitude (rad)
/// @param omega_rad_per_sec Earth rotation rate (rad/s)
/// @return Coriolis parameter (1/s)
HOST_DEVICE inline float calculate_coriolis_parameter(
    float latitude_radians,
    float omega_rad_per_sec = EARTH_ROTATION_RATE_RAD_PER_SEC
) {
    return 2.0f * omega_rad_per_sec * std::sin(latitude_radians);
}

/// Calculate Geostrophic wind speed
/// Formula: v_g = (1 / (ρf)) * (∂P/∂n)
/// @param pressure_gradient_pascals_per_meter Pressure gradient (Pa/m)
/// @param density_kg_per_cubic_meter Air density (kg/m³)
/// @param coriolis_parameter Coriolis parameter f (1/s)
/// @return Geostrophic wind speed (m/s)
HOST_DEVICE inline constexpr float calculate_geostrophic_wind_speed_meters_per_second(
    float pressure_gradient_pascals_per_meter,
    float density_kg_per_cubic_meter,
    float coriolis_parameter
) {
    float denominator = density_kg_per_cubic_meter * coriolis_parameter;
    return (constexpr_abs(denominator) > EPSILON) 
           ? (pressure_gradient_pascals_per_meter / denominator) 
           : 0.0f;
}

} // namespace atmospheric
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_ATMOSPHERIC_H

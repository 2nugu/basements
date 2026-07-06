#ifndef BASEMENTS_PHYSICS_FORMULAS_THERMODYNAMICS_H
#define BASEMENTS_PHYSICS_FORMULAS_THERMODYNAMICS_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace thermodynamics {

using namespace basements::math;

// Physical constants
constexpr float BOLTZMANN_CONSTANT_JOULES_PER_KELVIN = 1.380649e-23f;  // J/K
constexpr float GAS_CONSTANT_JOULES_PER_MOLE_KELVIN = 8.314462618f;    // J/(mol·K)
constexpr float AVOGADRO_NUMBER_PER_MOLE = 6.02214076e23f;             // 1/mol
constexpr float STEFAN_BOLTZMANN_CONSTANT = 5.670374419e-8f;           // W/(m²·K⁴)

// ============================================================
// Temperature Conversions
// ============================================================

/// Convert temperature from Celsius to Kelvin
/// Formula: K = °C + 273.15
/// @param temperature_celsius Temperature in degrees Celsius (°C)
/// @return Temperature in Kelvin (K)
HOST_DEVICE inline constexpr float convert_celsius_to_kelvin(
    float temperature_celsius
) {
    return temperature_celsius + 273.15f;
}

/// Convert temperature from Kelvin to Celsius
/// Formula: °C = K - 273.15
/// @param temperature_kelvin Temperature in Kelvin (K)
/// @return Temperature in degrees Celsius (°C)
HOST_DEVICE inline constexpr float convert_kelvin_to_celsius(
    float temperature_kelvin
) {
    return temperature_kelvin - 273.15f;
}

/// Convert temperature from Fahrenheit to Celsius
/// Formula: °C = (°F - 32) × 5/9
/// @param temperature_fahrenheit Temperature in degrees Fahrenheit (°F)
/// @return Temperature in degrees Celsius (°C)
HOST_DEVICE inline constexpr float convert_fahrenheit_to_celsius(
    float temperature_fahrenheit
) {
    return (temperature_fahrenheit - 32.0f) * (5.0f / 9.0f);
}

/// Convert temperature from Celsius to Fahrenheit
/// Formula: °F = °C × 9/5 + 32
/// @param temperature_celsius Temperature in degrees Celsius (°C)
/// @return Temperature in degrees Fahrenheit (°F)
HOST_DEVICE inline constexpr float convert_celsius_to_fahrenheit(
    float temperature_celsius
) {
    return temperature_celsius * (9.0f / 5.0f) + 32.0f;
}

// ============================================================
// Ideal Gas Law (PV = nRT)
// ============================================================

/// Calculate pressure using ideal gas law
/// Formula: P = nRT/V
/// @param amount_substance_moles Amount of substance (mol)
/// @param temperature_kelvin Absolute temperature (K)
/// @param volume_cubic_meters Volume of gas (m³)
/// @param gas_constant_joules_per_mole_kelvin Universal gas constant (J/(mol·K))
/// @return Pressure in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_pressure_pascals_ideal_gas(
    float amount_substance_moles,
    float temperature_kelvin,
    float volume_cubic_meters,
    float gas_constant_joules_per_mole_kelvin = GAS_CONSTANT_JOULES_PER_MOLE_KELVIN
) {
    return (volume_cubic_meters > EPSILON) 
        ? (amount_substance_moles * gas_constant_joules_per_mole_kelvin 
           * temperature_kelvin / volume_cubic_meters) 
        : 0.0f;
}

/// Calculate volume using ideal gas law
/// Formula: V = nRT/P
/// @param amount_substance_moles Amount of substance (mol)
/// @param temperature_kelvin Absolute temperature (K)
/// @param pressure_pascals Pressure (Pa)
/// @param gas_constant_joules_per_mole_kelvin Universal gas constant (J/(mol·K))
/// @return Volume in cubic meters (m³)
HOST_DEVICE inline constexpr float calculate_volume_cubic_meters_ideal_gas(
    float amount_substance_moles,
    float temperature_kelvin,
    float pressure_pascals,
    float gas_constant_joules_per_mole_kelvin = GAS_CONSTANT_JOULES_PER_MOLE_KELVIN
) {
    return (pressure_pascals > EPSILON) 
        ? (amount_substance_moles * gas_constant_joules_per_mole_kelvin 
           * temperature_kelvin / pressure_pascals) 
        : 0.0f;
}

/// Calculate temperature using ideal gas law
/// Formula: T = PV/(nR)
/// @param pressure_pascals Pressure (Pa)
/// @param volume_cubic_meters Volume (m³)
/// @param amount_substance_moles Amount of substance (mol)
/// @param gas_constant_joules_per_mole_kelvin Universal gas constant (J/(mol·K))
/// @return Temperature in Kelvin (K)
HOST_DEVICE inline constexpr float calculate_temperature_kelvin_ideal_gas(
    float pressure_pascals,
    float volume_cubic_meters,
    float amount_substance_moles,
    float gas_constant_joules_per_mole_kelvin = GAS_CONSTANT_JOULES_PER_MOLE_KELVIN
) {
    float denominator = amount_substance_moles * gas_constant_joules_per_mole_kelvin;
    return (denominator > EPSILON) 
        ? (pressure_pascals * volume_cubic_meters / denominator) 
        : 0.0f;
}

/// Calculate amount of substance using ideal gas law
/// Formula: n = PV/(RT)
/// @param pressure_pascals Pressure (Pa)
/// @param volume_cubic_meters Volume (m³)
/// @param temperature_kelvin Temperature (K)
/// @param gas_constant_joules_per_mole_kelvin Universal gas constant (J/(mol·K))
/// @return Amount of substance in moles (mol)
HOST_DEVICE inline constexpr float calculate_amount_moles_ideal_gas(
    float pressure_pascals,
    float volume_cubic_meters,
    float temperature_kelvin,
    float gas_constant_joules_per_mole_kelvin = GAS_CONSTANT_JOULES_PER_MOLE_KELVIN
) {
    float denominator = gas_constant_joules_per_mole_kelvin * temperature_kelvin;
    return (denominator > EPSILON) 
        ? (pressure_pascals * volume_cubic_meters / denominator) 
        : 0.0f;
}

// ============================================================
// Heat Transfer
// ============================================================

/// Calculate heat energy transferred
/// Formula: Q = m·c·ΔT
/// @param mass_kilograms Mass of substance (kg)
/// @param specific_heat_capacity_joules_per_kilogram_kelvin Specific heat capacity (J/(kg·K))
/// @param temperature_change_kelvin Temperature change (K)
/// @return Heat energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_heat_energy_joules(
    float mass_kilograms,
    float specific_heat_capacity_joules_per_kilogram_kelvin,
    float temperature_change_kelvin
) {
    return mass_kilograms * specific_heat_capacity_joules_per_kilogram_kelvin 
           * temperature_change_kelvin;
}

/// Calculate temperature change from heat
/// Formula: ΔT = Q/(m·c)
/// @param heat_energy_joules Heat energy (J)
/// @param mass_kilograms Mass of substance (kg)
/// @param specific_heat_capacity_joules_per_kilogram_kelvin Specific heat capacity (J/(kg·K))
/// @return Temperature change in Kelvin (K)
HOST_DEVICE inline constexpr float calculate_temperature_change_kelvin_from_heat(
    float heat_energy_joules,
    float mass_kilograms,
    float specific_heat_capacity_joules_per_kilogram_kelvin
) {
    float denominator = mass_kilograms * specific_heat_capacity_joules_per_kilogram_kelvin;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (heat_energy_joules / denominator) 
        : 0.0f;
}

/// Calculate specific heat capacity from heat
/// Formula: c = Q/(m·ΔT)
/// @param heat_energy_joules Heat energy (J)
/// @param mass_kilograms Mass of substance (kg)
/// @param temperature_change_kelvin Temperature change (K)
/// @return Specific heat capacity in J/(kg·K)
HOST_DEVICE inline constexpr float calculate_specific_heat_capacity_joules_per_kilogram_kelvin(
    float heat_energy_joules,
    float mass_kilograms,
    float temperature_change_kelvin
) {
    float denominator = mass_kilograms * temperature_change_kelvin;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (heat_energy_joules / denominator) 
        : 0.0f;
}

// ============================================================
// Heat Conduction (Fourier's Law)
// ============================================================

/// Calculate heat conduction rate
/// Formula: Q/t = k·A·ΔT/d
/// @param thermal_conductivity_watts_per_meter_kelvin Thermal conductivity (W/(m·K))
/// @param cross_sectional_area_square_meters Cross-sectional area (m²)
/// @param temperature_difference_kelvin Temperature difference (K)
/// @param thickness_meters Material thickness (m)
/// @return Heat transfer rate in Watts (W)
HOST_DEVICE inline constexpr float calculate_heat_conduction_rate_watts(
    float thermal_conductivity_watts_per_meter_kelvin,
    float cross_sectional_area_square_meters,
    float temperature_difference_kelvin,
    float thickness_meters
) {
    return (thickness_meters > EPSILON) 
        ? (thermal_conductivity_watts_per_meter_kelvin 
           * cross_sectional_area_square_meters 
           * temperature_difference_kelvin / thickness_meters) 
        : 0.0f;
}

/// Calculate thermal resistance
/// Formula: R = d/(k·A)
/// @param thickness_meters Material thickness (m)
/// @param thermal_conductivity_watts_per_meter_kelvin Thermal conductivity (W/(m·K))
/// @param cross_sectional_area_square_meters Cross-sectional area (m²)
/// @return Thermal resistance in K/W
HOST_DEVICE inline constexpr float calculate_thermal_resistance_kelvin_per_watt(
    float thickness_meters,
    float thermal_conductivity_watts_per_meter_kelvin,
    float cross_sectional_area_square_meters
) {
    float denominator = thermal_conductivity_watts_per_meter_kelvin 
                       * cross_sectional_area_square_meters;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (thickness_meters / denominator) 
        : 0.0f;
}

// ============================================================
// Convection
// ============================================================

/// Calculate convective heat transfer rate
/// Formula: Q/t = h·A·ΔT
/// @param heat_transfer_coefficient_watts_per_square_meter_kelvin Heat transfer coefficient (W/(m²·K))
/// @param surface_area_square_meters Surface area (m²)
/// @param temperature_difference_kelvin Temperature difference (K)
/// @return Heat transfer rate in Watts (W)
HOST_DEVICE inline constexpr float calculate_convective_heat_transfer_rate_watts(
    float heat_transfer_coefficient_watts_per_square_meter_kelvin,
    float surface_area_square_meters,
    float temperature_difference_kelvin
) {
    return heat_transfer_coefficient_watts_per_square_meter_kelvin 
           * surface_area_square_meters 
           * temperature_difference_kelvin;
}

// ============================================================
// Radiation (Stefan-Boltzmann Law)
// ============================================================

/// Calculate radiated power from a surface
/// Formula: P = ε·σ·A·T⁴
/// @param emissivity Emissivity (0-1, dimensionless)
/// @param surface_area_square_meters Surface area (m²)
/// @param temperature_kelvin Absolute temperature (K)
/// @param stefan_boltzmann_constant Stefan-Boltzmann constant (W/(m²·K⁴))
/// @return Radiated power in Watts (W)
HOST_DEVICE inline constexpr float calculate_radiated_power_watts(
    float emissivity,
    float surface_area_square_meters,
    float temperature_kelvin,
    float stefan_boltzmann_constant = STEFAN_BOLTZMANN_CONSTANT
) {
    float T4 = temperature_kelvin * temperature_kelvin 
             * temperature_kelvin * temperature_kelvin;
    return emissivity * stefan_boltzmann_constant 
           * surface_area_square_meters * T4;
}

/// Calculate net radiation between two surfaces
/// Formula: P = ε·σ·A·(T₁⁴ - T₂⁴)
/// @param emissivity Emissivity (0-1, dimensionless)
/// @param surface_area_square_meters Surface area (m²)
/// @param temperature_hot_kelvin Hot surface temperature (K)
/// @param temperature_cold_kelvin Cold surface temperature (K)
/// @param stefan_boltzmann_constant Stefan-Boltzmann constant (W/(m²·K⁴))
/// @return Net radiated power in Watts (W)
HOST_DEVICE inline constexpr float calculate_net_radiation_watts(
    float emissivity,
    float surface_area_square_meters,
    float temperature_hot_kelvin,
    float temperature_cold_kelvin,
    float stefan_boltzmann_constant = STEFAN_BOLTZMANN_CONSTANT
) {
    float T1_4 = temperature_hot_kelvin * temperature_hot_kelvin 
               * temperature_hot_kelvin * temperature_hot_kelvin;
    float T2_4 = temperature_cold_kelvin * temperature_cold_kelvin 
               * temperature_cold_kelvin * temperature_cold_kelvin;
    return emissivity * stefan_boltzmann_constant 
           * surface_area_square_meters * (T1_4 - T2_4);
}

// ============================================================
// Thermodynamic Work
// ============================================================

/// Calculate work in isobaric (constant pressure) process
/// Formula: W = P·ΔV
/// @param pressure_pascals Constant pressure (Pa)
/// @param volume_change_cubic_meters Volume change (m³)
/// @return Work in Joules (J)
HOST_DEVICE inline constexpr float calculate_isobaric_work_joules(
    float pressure_pascals,
    float volume_change_cubic_meters
) {
    return pressure_pascals * volume_change_cubic_meters;
}

/// Calculate work in isothermal (constant temperature) process
/// Formula: W = nRT·ln(V₂/V₁)
/// @param amount_substance_moles Amount of substance (mol)
/// @param temperature_kelvin Constant temperature (K)
/// @param initial_volume_cubic_meters Initial volume (m³)
/// @param final_volume_cubic_meters Final volume (m³)
/// @param gas_constant_joules_per_mole_kelvin Universal gas constant (J/(mol·K))
/// @return Work in Joules (J)
HOST_DEVICE inline float calculate_isothermal_work_joules(
    float amount_substance_moles,
    float temperature_kelvin,
    float initial_volume_cubic_meters,
    float final_volume_cubic_meters,
    float gas_constant_joules_per_mole_kelvin = GAS_CONSTANT_JOULES_PER_MOLE_KELVIN
) {
    return (initial_volume_cubic_meters > EPSILON && final_volume_cubic_meters > EPSILON) 
        ? (amount_substance_moles * gas_constant_joules_per_mole_kelvin 
           * temperature_kelvin * std::log(final_volume_cubic_meters / initial_volume_cubic_meters)) 
        : 0.0f;
}

// ============================================================
// Entropy
// ============================================================

/// Calculate entropy change in reversible process
/// Formula: ΔS = Q/T
/// @param heat_energy_joules Heat energy transferred (J)
/// @param temperature_kelvin Absolute temperature (K)
/// @return Entropy change in J/K
HOST_DEVICE inline constexpr float calculate_entropy_change_joules_per_kelvin(
    float heat_energy_joules,
    float temperature_kelvin
) {
    return (temperature_kelvin > EPSILON) 
        ? (heat_energy_joules / temperature_kelvin) 
        : 0.0f;
}

/// Calculate entropy change for ideal gas
/// Formula: ΔS = nR·ln(V₂/V₁)
/// @param amount_substance_moles Amount of substance (mol)
/// @param initial_volume_cubic_meters Initial volume (m³)
/// @param final_volume_cubic_meters Final volume (m³)
/// @param gas_constant_joules_per_mole_kelvin Universal gas constant (J/(mol·K))
/// @return Entropy change in J/K
HOST_DEVICE inline float calculate_entropy_change_ideal_gas_joules_per_kelvin(
    float amount_substance_moles,
    float initial_volume_cubic_meters,
    float final_volume_cubic_meters,
    float gas_constant_joules_per_mole_kelvin = GAS_CONSTANT_JOULES_PER_MOLE_KELVIN
) {
    return (initial_volume_cubic_meters > EPSILON && final_volume_cubic_meters > EPSILON) 
        ? (amount_substance_moles * gas_constant_joules_per_mole_kelvin 
           * std::log(final_volume_cubic_meters / initial_volume_cubic_meters)) 
        : 0.0f;
}

// ============================================================
// Efficiency
// ============================================================

/// Calculate Carnot efficiency (maximum theoretical efficiency)
/// Formula: η = 1 - T_cold/T_hot
/// @param cold_reservoir_temperature_kelvin Cold reservoir temperature (K)
/// @param hot_reservoir_temperature_kelvin Hot reservoir temperature (K)
/// @return Efficiency (0-1, dimensionless)
HOST_DEVICE inline constexpr float calculate_carnot_efficiency(
    float cold_reservoir_temperature_kelvin,
    float hot_reservoir_temperature_kelvin
) {
    return (hot_reservoir_temperature_kelvin > EPSILON) 
        ? (1.0f - cold_reservoir_temperature_kelvin / hot_reservoir_temperature_kelvin) 
        : 0.0f;
}

/// Calculate heat engine efficiency
/// Formula: η = W/Q_in
/// @param work_output_joules Work output (J)
/// @param heat_input_joules Heat input (J)
/// @return Efficiency (0-1, dimensionless)
HOST_DEVICE inline constexpr float calculate_heat_engine_efficiency(
    float work_output_joules,
    float heat_input_joules
) {
    return (heat_input_joules > EPSILON) 
        ? (work_output_joules / heat_input_joules) 
        : 0.0f;
}

/// Calculate coefficient of performance for refrigerator
/// Formula: COP = Q_cold/W
/// @param heat_removed_joules Heat removed from cold reservoir (J)
/// @param work_input_joules Work input (J)
/// @return Coefficient of performance (dimensionless)
HOST_DEVICE inline constexpr float calculate_refrigerator_coefficient_of_performance(
    float heat_removed_joules,
    float work_input_joules
) {
    return (work_input_joules > EPSILON) 
        ? (heat_removed_joules / work_input_joules) 
        : 0.0f;
}

/// Calculate coefficient of performance for heat pump
/// Formula: COP = Q_hot/W
/// @param heat_delivered_joules Heat delivered to hot reservoir (J)
/// @param work_input_joules Work input (J)
/// @return Coefficient of performance (dimensionless)
HOST_DEVICE inline constexpr float calculate_heat_pump_coefficient_of_performance(
    float heat_delivered_joules,
    float work_input_joules
) {
    return (work_input_joules > EPSILON) 
        ? (heat_delivered_joules / work_input_joules) 
        : 0.0f;
}

// ============================================================
// Kinetic Theory
// ============================================================

/// Calculate average kinetic energy per molecule
/// Formula: E_avg = (3/2)·k·T
/// @param temperature_kelvin Absolute temperature (K)
/// @param boltzmann_constant_joules_per_kelvin Boltzmann constant (J/K)
/// @return Average kinetic energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_average_kinetic_energy_per_molecule_joules(
    float temperature_kelvin,
    float boltzmann_constant_joules_per_kelvin = BOLTZMANN_CONSTANT_JOULES_PER_KELVIN
) {
    return 1.5f * boltzmann_constant_joules_per_kelvin * temperature_kelvin;
}

/// Calculate root-mean-square speed of gas molecules
/// Formula: v_rms = √(3RT/M)
/// @param temperature_kelvin Absolute temperature (K)
/// @param molar_mass_kilograms_per_mole Molar mass (kg/mol)
/// @param gas_constant_joules_per_mole_kelvin Universal gas constant (J/(mol·K))
/// @return RMS speed in meters per second (m/s)
HOST_DEVICE inline float calculate_rms_speed_meters_per_second(
    float temperature_kelvin,
    float molar_mass_kilograms_per_mole,
    float gas_constant_joules_per_mole_kelvin = GAS_CONSTANT_JOULES_PER_MOLE_KELVIN
) {
    return (molar_mass_kilograms_per_mole > EPSILON) 
        ? std::sqrt(3.0f * gas_constant_joules_per_mole_kelvin 
                    * temperature_kelvin / molar_mass_kilograms_per_mole) 
        : 0.0f;
}

/// Calculate mean free path of gas molecules
/// Formula: λ = kT/(√2·π·d²·P)
/// @param temperature_kelvin Absolute temperature (K)
/// @param pressure_pascals Pressure (Pa)
/// @param molecular_diameter_meters Molecular diameter (m)
/// @param boltzmann_constant_joules_per_kelvin Boltzmann constant (J/K)
/// @return Mean free path in meters (m)
HOST_DEVICE inline constexpr float calculate_mean_free_path_meters(
    float temperature_kelvin,
    float pressure_pascals,
    float molecular_diameter_meters,
    float boltzmann_constant_joules_per_kelvin = BOLTZMANN_CONSTANT_JOULES_PER_KELVIN
) {
    float denominator = 1.414213562f * PI * molecular_diameter_meters 
                       * molecular_diameter_meters * pressure_pascals;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (boltzmann_constant_joules_per_kelvin * temperature_kelvin / denominator) 
        : 0.0f;
}

} // namespace thermodynamics
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_THERMODYNAMICS_H

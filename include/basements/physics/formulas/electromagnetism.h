#ifndef BASEMENTS_PHYSICS_FORMULAS_ELECTROMAGNETISM_H
#define BASEMENTS_PHYSICS_FORMULAS_ELECTROMAGNETISM_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace electromagnetism {

using namespace basements::math;

// Physical constants
constexpr float COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED = 8.9875517923e9f;  // N⋅m²/C² (k)
constexpr float VACUUM_PERMITTIVITY_FARADS_PER_METER = 8.8541878128e-12f;                       // F/m (ε₀)
constexpr float VACUUM_PERMEABILITY_HENRIES_PER_METER = 1.25663706212e-6f;                      // H/m (μ₀)
constexpr float ELEMENTARY_CHARGE_COULOMBS = 1.602176634e-19f;                                  // C
constexpr float SPEED_OF_LIGHT_METERS_PER_SECOND = 299792458.0f;                                // m/s
constexpr float PLANCK_CONSTANT_JOULE_SECONDS = 6.62607015e-34f;                                // J·s (h)

// ============================================================
// Coulomb's Law
// ============================================================

/// Calculate electric force magnitude using Coulomb's law
/// Formula: F = k·|q₁q₂|/r²
/// @param charge1_coulombs First charge (C)
/// @param charge2_coulombs Second charge (C)
/// @param distance_meters Distance between charges (m)
/// @param coulomb_constant Coulomb constant (N⋅m²/C²)
/// @return Force magnitude in Newtons (N)
HOST_DEVICE inline constexpr float calculate_coulomb_force_newtons(
    float charge1_coulombs,
    float charge2_coulombs,
    float distance_meters,
    float coulomb_constant = COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED
) {
    return (distance_meters > EPSILON) ? 
           (coulomb_constant * constexpr_abs(charge1_coulombs * charge2_coulombs) / (distance_meters * distance_meters)) : 0.0f;
}

/// Calculate electric force vector using Coulomb's law
/// Formula: F = k·q₁q₂/r² · r̂
/// @param charge1_coulombs First charge (C)
/// @param charge2_coulombs Second charge (C)
/// @param position_vector Vector from charge1 to charge2
/// @param coulomb_constant Coulomb constant (N⋅m²/C²)
/// @return Force vector on charge2 in Newtons (N)
HOST_DEVICE inline Vec3 calculate_coulomb_force_vector_newtons(
    float charge1_coulombs,
    float charge2_coulombs,
    const Vec3& position_vector,
    float coulomb_constant = COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED
) {
    float distance = position_vector.length();
    if (distance < EPSILON) return Vec3::zero();
    
    // Force is repulsive if charges have same sign, attractive if opposite
    // With r vector pointing from 1 to 2, positive result means force is along r (repulsive)
    float force_magnitude = coulomb_constant * charge1_coulombs * charge2_coulombs / (distance * distance);
    return position_vector.normalized() * force_magnitude;
}

// ============================================================
// Electric Field
// ============================================================

/// Calculate electric field magnitude from point charge
/// Formula: E = k·|q|/r²
/// @param charge_coulombs Source charge (C)
/// @param distance_meters Distance from charge (m)
/// @param coulomb_constant Coulomb constant (N⋅m²/C²)
/// @return Electric field magnitude in Newtons per Coulomb (N/C) or Volts/meter (V/m)
HOST_DEVICE inline constexpr float calculate_electric_field_magnitude_newtons_per_coulomb(
    float charge_coulombs,
    float distance_meters,
    float coulomb_constant = COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED
) {
    return (distance_meters > EPSILON) ? 
           (coulomb_constant * constexpr_abs(charge_coulombs) / (distance_meters * distance_meters)) : 0.0f;
}

/// Calculate electric field vector from point charge
/// Formula: E = k·q/r² · r̂
/// @param charge_coulombs Source charge (C)
/// @param position_vector Vector from charge to point
/// @param coulomb_constant Coulomb constant (N⋅m²/C²)
/// @return Electric field vector in Newtons per Coulomb (N/C)
HOST_DEVICE inline Vec3 calculate_electric_field_vector_newtons_per_coulomb(
    float charge_coulombs,
    const Vec3& position_vector,
    float coulomb_constant = COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED
) {
    float distance = position_vector.length();
    if (distance < EPSILON) return Vec3::zero();
    
    float field_magnitude = coulomb_constant * charge_coulombs / (distance * distance);
    return position_vector.normalized() * field_magnitude;
}

/// Calculate force on charge in electric field
/// Formula: F = q·E
/// @param charge_coulombs Charge (C)
/// @param electric_field_vector Electric field vector (N/C)
/// @return Force vector in Newtons (N)
HOST_DEVICE inline Vec3 calculate_force_in_electric_field_newtons(
    float charge_coulombs,
    const Vec3& electric_field_vector
) {
    return electric_field_vector * charge_coulombs;
}

// ============================================================
// Electric Potential
// ============================================================

/// Calculate electric potential from point charge
/// Formula: V = k·q/r
/// @param charge_coulombs Source charge (C)
/// @param distance_meters Distance from charge (m)
/// @param coulomb_constant Coulomb constant (N⋅m²/C²)
/// @return Electric potential in Volts (V)
HOST_DEVICE inline constexpr float calculate_electric_potential_volts(
    float charge_coulombs,
    float distance_meters,
    float coulomb_constant = COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED
) {
    return (distance_meters > EPSILON) 
        ? (coulomb_constant * charge_coulombs / distance_meters) 
        : 0.0f;
}

/// Calculate electric potential energy of two point charges
/// Formula: U = k·q₁q₂/r
/// @param charge1_coulombs First charge (C)
/// @param charge2_coulombs Second charge (C)
/// @param distance_meters Distance between charges (m)
/// @param coulomb_constant Coulomb constant (N⋅m²/C²)
/// @return Potential energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_electric_potential_energy_joules(
    float charge1_coulombs,
    float charge2_coulombs,
    float distance_meters,
    float coulomb_constant = COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED
) {
    return (distance_meters > EPSILON) 
        ? (coulomb_constant * charge1_coulombs * charge2_coulombs / distance_meters) 
        : 0.0f;
}

/// Calculate electric field magnitude from potential gradient
/// Formula: E = -dV/dr
/// @param potential_difference_volts Change in potential (V)
/// @param distance_meters Distance over which change occurs (m)
/// @return Electric field magnitude in Volts per meter (V/m)
HOST_DEVICE inline constexpr float calculate_electric_field_from_potential_gradient(
    float potential_difference_volts,
    float distance_meters
) {
    return (constexpr_abs(distance_meters) > EPSILON) 
        ? (-potential_difference_volts / distance_meters) 
        : 0.0f;
}

// ============================================================
// Capacitance
// ============================================================

/// Calculate capacitance
/// Formula: C = Q/V
/// @param charge_coulombs Charge stored (C)
/// @param voltage_volts Potential difference (V)
/// @return Capacitance in Farads (F)
HOST_DEVICE inline constexpr float calculate_capacitance_farads(
    float charge_coulombs,
    float voltage_volts
) {
    return (constexpr_abs(voltage_volts) > EPSILON) 
        ? (charge_coulombs / voltage_volts) 
        : 0.0f;
}

/// Calculate charge stored in capacitor
/// Formula: Q = C·V
/// @param capacitance_farads Capacitance (F)
/// @param voltage_volts Potential difference (V)
/// @return Charge in Coulombs (C)
HOST_DEVICE inline constexpr float calculate_charge_from_capacitance_coulombs(
    float capacitance_farads,
    float voltage_volts
) {
    return capacitance_farads * voltage_volts;
}

/// Calculate energy stored in capacitor
/// Formula: U = ½CV²
/// @param capacitance_farads Capacitance (F)
/// @param voltage_volts Potential difference (V)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_capacitor_energy_joules(
    float capacitance_farads,
    float voltage_volts
) {
    return 0.5f * capacitance_farads * voltage_volts * voltage_volts;
}

/// Calculate capacitance of parallel plate capacitor
/// Formula: C = ε₀·A/d
/// @param plate_area_square_meters Area of plates (m²)
/// @param separation_meters Separation distance (m)
/// @param permittivity Permittivity of dielectric (F/m)
/// @return Capacitance in Farads (F)
HOST_DEVICE inline constexpr float calculate_parallel_plate_capacitance_farads(
    float plate_area_square_meters,
    float separation_meters,
    float permittivity = VACUUM_PERMITTIVITY_FARADS_PER_METER
) {
    return (separation_meters > EPSILON) 
        ? (permittivity * plate_area_square_meters / separation_meters) 
        : 0.0f;
}

// ============================================================
// Current and Resistance
// ============================================================

/// Calculate voltage using Ohm's law
/// Formula: V = I·R
/// @param current_amperes Current (A)
/// @param resistance_ohms Resistance (Ω)
/// @return Voltage in Volts (V)
HOST_DEVICE inline constexpr float calculate_voltage_volts_ohms_law(
    float current_amperes,
    float resistance_ohms
) {
    return current_amperes * resistance_ohms;
}

/// Calculate current using Ohm's law
/// Formula: I = V/R
/// @param voltage_volts Voltage (V)
/// @param resistance_ohms Resistance (Ω)
/// @return Current in Amperes (A)
HOST_DEVICE inline constexpr float calculate_current_amperes_ohms_law(
    float voltage_volts,
    float resistance_ohms
) {
    return (resistance_ohms > EPSILON) ? (voltage_volts / resistance_ohms) : 0.0f;
}

/// Calculate resistance using Ohm's law
/// Formula: R = V/I
/// @param voltage_volts Voltage (V)
/// @param current_amperes Current (A)
/// @return Resistance in Ohms (Ω)
HOST_DEVICE inline constexpr float calculate_resistance_ohms_ohms_law(
    float voltage_volts,
    float current_amperes
) {
    return (constexpr_abs(current_amperes) > EPSILON) ? (voltage_volts / current_amperes) : 0.0f;
}

/// Calculate electrical power (P = VI)
/// @param voltage_volts Voltage (V)
/// @param current_amperes Current (A)
/// @return Power in Watts (W)
HOST_DEVICE inline constexpr float calculate_electrical_power_watts(
    float voltage_volts,
    float current_amperes
) {
    return voltage_volts * current_amperes;
}

/// Calculate electrical power (P = I²R)
/// @param current_amperes Current (A)
/// @param resistance_ohms Resistance (Ω)
/// @return Power in Watts (W)
HOST_DEVICE inline constexpr float calculate_electrical_power_watts_i2r(
    float current_amperes,
    float resistance_ohms
) {
    return current_amperes * current_amperes * resistance_ohms;
}

/// Calculate resistance from resistivity
/// Formula: R = ρ·L/A
/// @param resistivity_ohm_meters Resistivity (Ω·m)
/// @param length_meters Conductor length (m)
/// @param cross_sectional_area_square_meters Conductor area (m²)
/// @return Resistance in Ohms (Ω)
HOST_DEVICE inline constexpr float calculate_resistance_from_resistivity_ohms(
    float resistivity_ohm_meters,
    float length_meters,
    float cross_sectional_area_square_meters
) {
    return (cross_sectional_area_square_meters > EPSILON) 
        ? (resistivity_ohm_meters * length_meters / cross_sectional_area_square_meters) 
        : 0.0f;
}

// ============================================================
// Magnetic Field
// ============================================================

/// Calculate magnetic force on moving charge
/// Formula: F = q·v·B·sin(θ)
/// @param charge_coulombs Charge (C)
/// @param velocity_meters_per_second Velocity magnitude (m/s)
/// @param magnetic_field_tesla Magnetic field strength (T)
/// @param angle_radians Angle between velocity and magnetic field (rad)
/// @return Force magnitude in Newtons (N)
HOST_DEVICE inline float calculate_magnetic_force_on_charge_newtons(
    float charge_coulombs,
    float velocity_meters_per_second,
    float magnetic_field_tesla,
    float angle_radians = HALF_PI
) {
    return charge_coulombs * velocity_meters_per_second * magnetic_field_tesla * std::sin(angle_radians);
}

/// Calculate Lorentz force vector
/// Formula: F = q·(E + v × B)
/// @param charge_coulombs Charge (C)
/// @param electric_field_vector Electric field (N/C or V/m)
/// @param velocity_vector Velocity (m/s)
/// @param magnetic_field_vector Magnetic field (T)
/// @return Force vector in Newtons (N)
HOST_DEVICE inline Vec3 calculate_lorentz_force_vector_newtons(
    float charge_coulombs,
    const Vec3& electric_field_vector,
    const Vec3& velocity_vector,
    const Vec3& magnetic_field_vector
) {
    return (electric_field_vector + velocity_vector.cross(magnetic_field_vector)) * charge_coulombs;
}

/// Calculate magnetic force on current-carrying wire
/// Formula: F = I·L·B·sin(θ)
/// @param current_amperes Current (A)
/// @param length_meters Wire length (m)
/// @param magnetic_field_tesla Magnetic field strength (T)
/// @param angle_radians Angle between wire and magnetic field (rad)
/// @return Force magnitude in Newtons (N)
HOST_DEVICE inline float calculate_magnetic_force_on_wire_newtons(
    float current_amperes,
    float length_meters,
    float magnetic_field_tesla,
    float angle_radians = HALF_PI
) {
    return current_amperes * length_meters * magnetic_field_tesla * std::sin(angle_radians);
}

/// Calculate cyclotron radius
/// Formula: r = mv/(qB)
/// @param mass_kilograms Particle mass (kg)
/// @param velocity_meters_per_second velocity (m/s)
/// @param charge_coulombs Particle charge (C)
/// @param magnetic_field_tesla Magnetic field strength (T)
/// @return Radius in meters (m)
HOST_DEVICE inline constexpr float calculate_cyclotron_radius_meters(
    float mass_kilograms,
    float velocity_meters_per_second,
    float charge_coulombs,
    float magnetic_field_tesla
) {
    float denominator = charge_coulombs * magnetic_field_tesla;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (mass_kilograms * velocity_meters_per_second / denominator) 
        : 0.0f;
}

/// Calculate cyclotron frequency
/// Formula: f = qB/(2πm)
/// @param charge_coulombs Particle charge (C)
/// @param magnetic_field_tesla Magnetic field strength (T)
/// @param mass_kilograms Particle mass (kg)
/// @return Frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_cyclotron_frequency_hertz(
    float charge_coulombs,
    float magnetic_field_tesla,
    float mass_kilograms
) {
    return (mass_kilograms > EPSILON) 
        ? (charge_coulombs * magnetic_field_tesla / (TWO_PI * mass_kilograms)) 
        : 0.0f;
}

// ============================================================
// Electromagnetic Induction
// ============================================================

/// Calculate induced EMF using Faraday's law
/// Formula: ε = -dΦ/dt
/// @param delta_magnetic_flux_webers Change in magnetic flux (Wb)
/// @param delta_time_seconds Time interval (s)
/// @return Induced EMF in Volts (V)
HOST_DEVICE inline constexpr float calculate_induced_emf_volts(
    float delta_magnetic_flux_webers,
    float delta_time_seconds
) {
    return (delta_time_seconds > EPSILON) 
        ? (-delta_magnetic_flux_webers / delta_time_seconds) 
        : 0.0f;
}

/// Calculate magnetic flux
/// Formula: Φ = B·A·cos(θ)
/// @param magnetic_field_tesla Magnetic field strength (T)
/// @param area_square_meters Area (m²)
/// @param angle_radians Angle between B and area normal (rad)
/// @return Magnetic flux in Webers (Wb)
HOST_DEVICE inline float calculate_magnetic_flux_webers(
    float magnetic_field_tesla,
    float area_square_meters,
    float angle_radians = 0.0f
) {
    return magnetic_field_tesla * area_square_meters * std::cos(angle_radians);
}

/// Calculate inductance
/// Formula: L = Φ/I
/// @param magnetic_flux_webers Magnetic flux (Wb)
/// @param current_amperes Current (A)
/// @return Inductance in Henries (H)
HOST_DEVICE inline constexpr float calculate_inductance_henries(
    float magnetic_flux_webers,
    float current_amperes
) {
    return (constexpr_abs(current_amperes) > EPSILON) 
        ? (magnetic_flux_webers / current_amperes) 
        : 0.0f;
}

/// Calculate energy stored in inductor
/// Formula: U = ½LI²
/// @param inductance_henries Inductance (H)
/// @param current_amperes Current (A)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_inductor_energy_joules(
    float inductance_henries,
    float current_amperes
) {
    return 0.5f * inductance_henries * current_amperes * current_amperes;
}

// ============================================================
// Electromagnetic Waves
// ============================================================

/// Calculate speed of electromagnetic wave in medium
/// Formula: c = 1/√(εμ)
/// @param permittivity Permittivity (F/m)
/// @param permeability Permeability (H/m)
/// @return Speed in meters per second (m/s)
HOST_DEVICE inline float calculate_em_wave_speed_meters_per_second(
    float permittivity = VACUUM_PERMITTIVITY_FARADS_PER_METER,
    float permeability = VACUUM_PERMEABILITY_HENRIES_PER_METER
) {
    return 1.0f / std::sqrt(permittivity * permeability);
}

/// Calculate photon energy
/// Formula: E = hf
/// @param frequency_hertz Frequency (Hz)
/// @param planck_constant_joule_seconds Planck constant (J·s)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_photon_energy_joules(
    float frequency_hertz,
    float planck_constant_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    return planck_constant_joule_seconds * frequency_hertz;
}

/// Calculate photon momentum
/// Formula: p = h/λ
/// @param wavelength_meters Wavelength (m)
/// @param planck_constant_joule_seconds Planck constant (J·s)
/// @return Momentum in kg·m/s
HOST_DEVICE inline constexpr float calculate_photon_momentum(
    float wavelength_meters,
    float planck_constant_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (wavelength_meters > EPSILON) 
        ? (planck_constant_joule_seconds / wavelength_meters) 
        : 0.0f;
}


    // ========================================================================
    // Magnetic Field & Induction
    // ========================================================================

    /// Calculate Magnetic Field from Straight Wire (Ampere's Law)
    /// Formula: B = (μ0 * I) / (2π * r)
    /// @param current_amperes Current (A)
    /// @param distance_meters Distance from wire (m)
    /// @param permeability_vacuum Permeability of free space (T·m/A)
    HOST_DEVICE inline constexpr float calculate_magnetic_field_straight_wire_tesla(
        float current_amperes,
        float distance_meters,
        float permeability_vacuum = 1.25663706e-6f // 4π * 10^-7
    ) {
        if (distance_meters < EPSILON) return 0.0f;
        return (permeability_vacuum * current_amperes) / (TWO_PI * distance_meters);
    }

    /// Calculate Magnetic Field at Center of Loop (Biot-Savart)
    /// Formula: B = (μ0 * I) / (2 * R)
    HOST_DEVICE inline constexpr float calculate_magnetic_field_loop_center_tesla(
        float current_amperes,
        float radius_meters,
        float permeability_vacuum = 1.25663706e-6f
    ) {
        if (radius_meters < EPSILON) return 0.0f;
        return (permeability_vacuum * current_amperes) / (2.0f * radius_meters);
    }

    /// Calculate Solenoid Magnetic Field
    /// Formula: B = μ0 * (N/L) * I
    /// @param number_of_turns Total turns
    /// @param length_meters Length of solenoid (m)
    HOST_DEVICE inline constexpr float calculate_magnetic_field_solenoid_tesla(
        float current_amperes,
        float number_of_turns,
        float length_meters,
        float permeability_vacuum = 1.25663706e-6f
    ) {
        if (length_meters < EPSILON) return 0.0f;
        float turn_density = number_of_turns / length_meters;
        return permeability_vacuum * turn_density * current_amperes;
    }

    /// Calculate Induced EMF (Faraday's Law - Magnitude)
    /// Formula: ε = |dΦ/dt| ~ |ΔΦ/Δt|
    /// @param magnetic_flux_change_webers Change in magnetic flux (Wb)
    /// @param time_interval_seconds Time interval (s)
    /// @param number_of_turns Number of turns (N)
    HOST_DEVICE inline constexpr float calculate_induced_emf_volts(
        float magnetic_flux_change_webers,
        float time_interval_seconds,
        float number_of_turns = 1.0f
    ) {
        if (time_interval_seconds < EPSILON) return 0.0f;
        return number_of_turns * constexpr_abs(magnetic_flux_change_webers / time_interval_seconds);
    }

    /// Calculate Motional EMF
    /// Formula: ε = B * L * v
    HOST_DEVICE inline constexpr float calculate_motional_emf_volts(
        float magnetic_field_tesla,
        float length_meters,
        float velocity_meters_per_second
    ) {
        return magnetic_field_tesla * length_meters * velocity_meters_per_second;
    }

    /// Calculate Inductance of Solenoid
    /// Formula: L = (μ0 * N^2 * A) / l
    HOST_DEVICE inline constexpr float calculate_inductance_solenoid_henries(
        float number_of_turns,
        float area_square_meters,
        float length_meters,
        float permeability_vacuum = 1.25663706e-6f
    ) {
        if (length_meters < EPSILON) return 0.0f;
        return (permeability_vacuum * number_of_turns * number_of_turns * area_square_meters) / length_meters;
    }
    
    // ========================================================================
    // Field Equation Helpers (Concepts)
    // ========================================================================

    /// Calculate Divergence of Vector Field (Discrete)
    /// div(F) = ∂Fx/∂x + ∂Fy/∂y + ∂Fz/∂z
    /// Central difference approximation
    HOST_DEVICE inline float calculate_divergence_discrete(
        const Vec3& field_x_plus, const Vec3& field_x_minus,
        const Vec3& field_y_plus, const Vec3& field_y_minus,
        const Vec3& field_z_plus, const Vec3& field_z_minus,
        float delta_x, float delta_y, float delta_z
    ) {
        float dfx_dx = (field_x_plus.x - field_x_minus.x) / (2.0f * delta_x);
        float dfy_dy = (field_y_plus.y - field_y_minus.y) / (2.0f * delta_y);
        float dfz_dz = (field_z_plus.z - field_z_minus.z) / (2.0f * delta_z);
        return dfx_dx + dfy_dy + dfz_dz;
    }

    /// Calculate Curl of Vector Field (Discrete)
    /// curl(F) = (∂Fz/∂y - ∂Fy/∂z)i + (∂Fx/∂z - ∂Fz/∂x)j + (∂Fy/∂x - ∂Fx/∂y)k
    HOST_DEVICE inline Vec3 calculate_curl_discrete(
        const Vec3& field_x_plus, const Vec3& field_x_minus,
        const Vec3& field_y_plus, const Vec3& field_y_minus,
        const Vec3& field_z_plus, const Vec3& field_z_minus,
        float delta_x, float delta_y, float delta_z
    ) {
        // Partial derivatives
        float dFz_dy = (field_y_plus.z - field_y_minus.z) / (2.0f * delta_y);
        float dFy_dz = (field_z_plus.y - field_z_minus.y) / (2.0f * delta_z);
        
        float dFx_dz = (field_z_plus.x - field_z_minus.x) / (2.0f * delta_z);
        float dFz_dx = (field_x_plus.z - field_x_minus.z) / (2.0f * delta_x);
        
        float dFy_dx = (field_x_plus.y - field_x_minus.y) / (2.0f * delta_x);
        float dFx_dy = (field_y_plus.x - field_y_minus.x) / (2.0f * delta_y);
        
        return Vec3(dFz_dy - dFy_dz, dFx_dz - dFz_dx, dFy_dx - dFx_dy);
    }

} // namespace electromagnetism
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_ELECTROMAGNETISM_H

#ifndef BASEMENTS_PHYSICS_FORMULAS_QUANTUM_H
#define BASEMENTS_PHYSICS_FORMULAS_QUANTUM_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace quantum {

using namespace basements::math;

// Physical constants
constexpr float PLANCK_CONSTANT_JOULE_SECONDS = 6.62607015e-34f;        // J·s (h)
constexpr float REDUCED_PLANCK_CONSTANT_JOULE_SECONDS = 1.054571817e-34f; // J·s (ℏ = h/(2π))
constexpr float ELECTRON_MASS_KILOGRAMS = 9.1093837015e-31f;            // kg
constexpr float ELECTRON_CHARGE_COULOMBS = 1.602176634e-19f;            // C
constexpr float RYDBERG_ENERGY_ELECTRON_VOLTS = 13.6f;                  // eV
constexpr float SPEED_OF_LIGHT_METERS_PER_SECOND = 299792458.0f;        // m/s
constexpr float COULOMB_CONSTANT = 8.9875517923e9f;                     // N⋅m²/C²

// ============================================================
// De Broglie Wavelength
// ============================================================

/// Calculate De Broglie wavelength from momentum
/// Formula: λ = h/p
/// @param momentum_kg_meters_per_second Momentum (kg·m/s)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Wavelength in meters (m)
HOST_DEVICE inline constexpr float calculate_de_broglie_wavelength_meters(
    float momentum_kg_meters_per_second,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (momentum_kg_meters_per_second > EPSILON) ? (h_joule_seconds / momentum_kg_meters_per_second) : 0.0f;
}

/// Calculate De Broglie wavelength from mass and velocity
/// Formula: λ = h/(mv)
/// @param mass_kilograms Mass (kg)
/// @param velocity_meters_per_second Velocity (m/s)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Wavelength in meters (m)
HOST_DEVICE inline constexpr float calculate_de_broglie_wavelength_from_velocity_meters(
    float mass_kilograms,
    float velocity_meters_per_second,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    float momentum = mass_kilograms * velocity_meters_per_second;
    return (momentum > EPSILON) ? (h_joule_seconds / momentum) : 0.0f;
}

/// Calculate momentum from wavelength
/// Formula: p = h/λ
/// @param wavelength_meters Wavelength (m)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Momentum in kilogram meters per second (kg·m/s)
HOST_DEVICE inline constexpr float calculate_momentum_from_wavelength_kg_meters_per_second(
    float wavelength_meters,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (wavelength_meters > EPSILON) ? (h_joule_seconds / wavelength_meters) : 0.0f;
}

// ============================================================
// Energy Quantization
// ============================================================

/// Calculate photon energy from frequency
/// Formula: E = hf
/// @param frequency_hertz Frequency (Hz)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_photon_energy_joules(
    float frequency_hertz,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    return h_joule_seconds * frequency_hertz;
}

/// Calculate photon energy from wavelength
/// Formula: E = hc/λ
/// @param wavelength_meters Wavelength (m)
/// @param h_joule_seconds Planck constant (J·s)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_photon_energy_from_wavelength_joules(
    float wavelength_meters,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return (wavelength_meters > EPSILON) 
           ? (h_joule_seconds * c_meters_per_second / wavelength_meters) 
           : 0.0f;
}

/// Calculate frequency from energy
/// Formula: f = E/h
/// @param energy_joules Energy (J)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_frequency_from_energy_hertz(
    float energy_joules,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (h_joule_seconds > EPSILON) ? (energy_joules / h_joule_seconds) : 0.0f;
}

// ============================================================
// Photoelectric Effect
// ============================================================

/// Calculate maximum kinetic energy of photoelectron
/// Formula: K_max = hf - φ
/// @param frequency_hertz Light frequency (Hz)
/// @param work_function_joules Work function of the material (J)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Maximum kinetic energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_photoelectric_max_kinetic_energy_joules(
    float frequency_hertz,
    float work_function_joules,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    float photon_energy = h_joule_seconds * frequency_hertz;
    return (photon_energy > work_function_joules) ? (photon_energy - work_function_joules) : 0.0f;
}

/// Calculate threshold frequency
/// Formula: f₀ = φ/h
/// @param work_function_joules Work function (J)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Threshold frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_threshold_frequency_hertz(
    float work_function_joules,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (h_joule_seconds > EPSILON) ? (work_function_joules / h_joule_seconds) : 0.0f;
}

/// Calculate stopping potential
/// Formula: V_s = K_max/e = (hf - φ)/e
/// @param frequency_hertz Light frequency (Hz)
/// @param work_function_joules Work function (J)
/// @param h_joule_seconds Planck constant (J·s)
/// @param e_coulombs Elementary charge (C)
/// @return Stopping potential in Volts (V)
HOST_DEVICE inline constexpr float calculate_stopping_potential_volts(
    float frequency_hertz,
    float work_function_joules,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS,
    float e_coulombs = ELECTRON_CHARGE_COULOMBS
) {
    float k_max = calculate_photoelectric_max_kinetic_energy_joules(frequency_hertz, work_function_joules, h_joule_seconds);
    return (e_coulombs > EPSILON) ? (k_max / e_coulombs) : 0.0f;
}

// ============================================================
// Compton Scattering
// ============================================================

/// Calculate Compton wavelength shift
/// Formula: Δλ = (h/(m_e·c))(1 - cosθ)
/// @param scattering_angle_radians Scattering angle (rad)
/// @param h_joule_seconds Planck constant (J·s)
/// @param mass_kilograms Particle mass, typically electron mass (kg)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Wavelength shift in meters (m)
HOST_DEVICE inline float calculate_compton_shift_meters(
    float scattering_angle_radians,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS,
    float mass_kilograms = ELECTRON_MASS_KILOGRAMS,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float denominator = mass_kilograms * c_meters_per_second;
    if (denominator <= EPSILON) return 0.0f;
    
    float compton_lambda = h_joule_seconds / denominator;
    return compton_lambda * (1.0f - std::cos(scattering_angle_radians));
}

/// Calculate Compton wavelength
/// Formula: λ_C = h/(m_e·c)
/// @param h_joule_seconds Planck constant (J·s)
/// @param mass_kilograms Particle mass (kg)
/// @param c_meters_per_second Speed of light (m/s)
/// @return Compton wavelength in meters (m)
HOST_DEVICE inline constexpr float calculate_compton_wavelength_meters(
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS,
    float mass_kilograms = ELECTRON_MASS_KILOGRAMS,
    float c_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    float denominator = mass_kilograms * c_meters_per_second;
    return (denominator > EPSILON) ? (h_joule_seconds / denominator) : 0.0f;
}

// ============================================================
// Heisenberg Uncertainty Principle
// ============================================================

/// Calculate minimum momentum uncertainty
/// Formula: Δp ≥ ℏ/(2Δx)
/// @param position_uncertainty_meters Uncertainty in position (m)
/// @param hbar_joule_seconds Reduced Planck constant (J·s)
/// @return Minimum momentum uncertainty in kg·m/s
HOST_DEVICE inline constexpr float calculate_min_momentum_uncertainty_kg_meters_per_second(
    float position_uncertainty_meters,
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (position_uncertainty_meters > EPSILON) ? 
           (hbar_joule_seconds / (2.0f * position_uncertainty_meters)) : 0.0f;
}

/// Calculate minimum energy uncertainty
/// Formula: ΔE ≥ ℏ/(2Δt)
/// @param time_uncertainty_seconds Uncertainty in time (s)
/// @param hbar_joule_seconds Reduced Planck constant (J·s)
/// @return Minimum energy uncertainty in Joules (J)
HOST_DEVICE inline constexpr float calculate_min_energy_uncertainty_joules(
    float time_uncertainty_seconds,
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (time_uncertainty_seconds > EPSILON) ? 
           (hbar_joule_seconds / (2.0f * time_uncertainty_seconds)) : 0.0f;
}

// ============================================================
// Hydrogen Atom (Bohr Model)
// ============================================================

/// Calculate energy level of Hydrogen atom
/// Formula: E_n = -13.6eV/n²
/// @param n Principal quantum number (integer, > 0)
/// @param ground_state_energy_ev Ground state energy (eV)
/// @return Energy in Electron Volts (eV)
HOST_DEVICE inline constexpr float calculate_hydrogen_energy_level_ev(
    int n,
    float ground_state_energy_ev = RYDBERG_ENERGY_ELECTRON_VOLTS
) {
    return (n > 0) ? (-ground_state_energy_ev / static_cast<float>(n * n)) : 0.0f;
}

/// Calculate Bohr radius
/// Formula: a₀ = ℏ²/(m_e·e²·k)
/// @param hbar_joule_seconds Reduced Planck constant
/// @param mass_kilograms Electron mass
/// @param charge_coulombs Electron charge
/// @param coulomb_constant Coulomb constant
/// @return Bohr radius in meters (m)
HOST_DEVICE inline constexpr float calculate_bohr_radius_meters(
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS,
    float mass_kilograms = ELECTRON_MASS_KILOGRAMS,
    float charge_coulombs = ELECTRON_CHARGE_COULOMBS,
    float coulomb_constant = COULOMB_CONSTANT
) {
    float denominator = mass_kilograms * charge_coulombs * charge_coulombs * coulomb_constant;
    return (denominator > EPSILON) 
           ? ((hbar_joule_seconds * hbar_joule_seconds) / denominator) 
           : 0.0f;
}

/// Calculate orbital radius
/// Formula: r_n = n²a₀
/// @param n Principal quantum number
/// @param bohr_radius_meters Bohr radius (m)
/// @return Orbital radius in meters (m)
HOST_DEVICE inline constexpr float calculate_orbital_radius_meters(
    int n,
    float bohr_radius_meters = 5.29177210903e-11f
) {
    return static_cast<float>(n * n) * bohr_radius_meters;
}

/// Calculate transition energy
/// Formula: ΔE = E_0(1/n₁² - 1/n₂²)
/// @param n_initial Initial quantum number
/// @param n_final Final quantum number
/// @param ground_state_energy_ev Ground state energy (eV)
/// @return Energy difference in Electron Volts (eV). Positive if photon emitted (n_i > n_f), negative if absorbed.
HOST_DEVICE inline constexpr float calculate_transition_energy_ev(
    int n_initial,
    int n_final,
    float ground_state_energy_ev = RYDBERG_ENERGY_ELECTRON_VOLTS
) {
    if (n_initial <= 0 || n_final <= 0) return 0.0f;
    return ground_state_energy_ev * (1.0f / (n_final * n_final) - 1.0f / (n_initial * n_initial));
}

/// Calculate photon wavelength from transition
/// Formula: λ = hc/|ΔE|
/// @param n_initial Initial quantum number
/// @param n_final Final quantum number
/// @return Wavelength in meters (m). Returns 0 if n_initial == n_final or error.
HOST_DEVICE inline constexpr float calculate_transition_wavelength_meters(
    int n_initial,
    int n_final
) {
    float energy_ev = constexpr_abs(calculate_transition_energy_ev(n_initial, n_final));
    if (energy_ev < EPSILON) return 0.0f;
    
    // Convert eV to Joules: 1 eV = 1.602...e-19 J
    float energy_joules = energy_ev * ELECTRON_CHARGE_COULOMBS;
    return (PLANCK_CONSTANT_JOULE_SECONDS * SPEED_OF_LIGHT_METERS_PER_SECOND) / energy_joules;
}

// ============================================================
// Quantum Tunneling
// ============================================================

/// Calculate tunneling probability (rectangular barrier)
/// Formula: T ≈ e^(-2κL), where κ = √(2m(V-E))/ℏ
/// @param barrier_width_meters Width of barrier (m)
/// @param barrier_height_joules Potential energy of barrier (J)
/// @param particle_energy_joules Total energy of particle (J)
/// @param mass_kilograms Particle mass (kg)
/// @param hbar_joule_seconds Reduced Planck constant
/// @return Transmission probability (0 to 1)
HOST_DEVICE inline float calculate_tunneling_probability(
    float barrier_width_meters,
    float barrier_height_joules,
    float particle_energy_joules,
    float mass_kilograms,
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS
) {
    if (particle_energy_joules >= barrier_height_joules) return 1.0f;
    
    float kappa = std::sqrt(2.0f * mass_kilograms * (barrier_height_joules - particle_energy_joules)) / hbar_joule_seconds;
    return std::exp(-2.0f * kappa * barrier_width_meters);
}

// ============================================================
// Wave Function & Particle in a Box
// ============================================================

/// Calculate probability density
/// Formula: P = |ψ|²
/// @param psi_real Real part of wavefunction
/// @param psi_imag Imaginary part of wavefunction
/// @return Probability density
HOST_DEVICE inline constexpr float calculate_probability_density(float psi_real, float psi_imag) {
    return psi_real * psi_real + psi_imag * psi_imag;
}

/// Calculate normalization constant for 1D box
/// Formula: A = √(2/L)
/// @param length_meters Length of the box (m)
/// @return Normalization constant
HOST_DEVICE inline float calculate_normalization_constant_1d_box(float length_meters) {
    return (length_meters > EPSILON) ? std::sqrt(2.0f / length_meters) : 0.0f;
}

/// Calculate energy levels for particle in a 1D box
/// Formula: E_n = n²h²/(8mL²)
/// @param n Quantum number (integer, > 0)
/// @param mass_kilograms Mass (kg)
/// @param length_meters Length of box (m)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_particle_in_box_energy_joules(
    int n,
    float mass_kilograms,
    float length_meters,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    if (n <= 0) return 0.0f;
    float numerator = static_cast<float>(n * n) * h_joule_seconds * h_joule_seconds;
    float denominator = 8.0f * mass_kilograms * length_meters * length_meters;
    return (denominator > EPSILON) ? (numerator / denominator) : 0.0f;
}

/// Calculate energy levels for particle in a 3D box
/// Formula: E = (h²/8m)(n_x²/L_x² + n_y²/L_y² + n_z²/L_z²)
/// @param nx Quantum number x
/// @param ny Quantum number y
/// @param nz Quantum number z
/// @param mass_kilograms Mass (kg)
/// @param lx_meters Length x (m)
/// @param ly_meters Length y (m)
/// @param lz_meters Length z (m)
/// @param h_joule_seconds Planck constant (J·s)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_particle_in_3d_box_energy_joules(
    int nx, int ny, int nz,
    float mass_kilograms,
    float lx_meters, float ly_meters, float lz_meters,
    float h_joule_seconds = PLANCK_CONSTANT_JOULE_SECONDS
) {
    float term_x = (lx_meters > EPSILON) ? (static_cast<float>(nx * nx) / (lx_meters * lx_meters)) : 0.0f;
    float term_y = (ly_meters > EPSILON) ? (static_cast<float>(ny * ny) / (ly_meters * ly_meters)) : 0.0f;
    float term_z = (lz_meters > EPSILON) ? (static_cast<float>(nz * nz) / (lz_meters * lz_meters)) : 0.0f;
    
    return (h_joule_seconds * h_joule_seconds / (8.0f * mass_kilograms)) * (term_x + term_y + term_z);
}

// ============================================================
// Harmonic Oscillator (Quantum)
// ============================================================

/// Calculate energy levels of quantum harmonic oscillator
/// Formula: E_n = (n + ½)ℏω
/// @param n Quantum number (integer, >= 0)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param hbar_joule_seconds Reduced Planck constant (J·s)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_harmonic_oscillator_energy_joules(
    int n,
    float angular_frequency_radians_per_second,
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS
) {
    return (static_cast<float>(n) + 0.5f) * hbar_joule_seconds * angular_frequency_radians_per_second;
}

/// Calculate zero-point energy
/// Formula: E₀ = ½ℏω
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param hbar_joule_seconds Reduced Planck constant (J·s)
/// @return Zero-point energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_zero_point_energy_joules(
    float angular_frequency_radians_per_second,
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS
) {
    return 0.5f * hbar_joule_seconds * angular_frequency_radians_per_second;
}

// ============================================================
// Spin
// ============================================================

/// Calculate magnitude of spin angular momentum
/// Formula: S = √(s(s+1))ℏ
/// @param spin_quantum_number Spin quantum number s (e.g. 0.5 for electrons)
/// @param hbar_joule_seconds Reduced Planck constant (J·s)
/// @return Spin magnitude in J·s
HOST_DEVICE inline float calculate_spin_magnitude_joule_seconds(
    float spin_quantum_number,
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS
) {
    return std::sqrt(spin_quantum_number * (spin_quantum_number + 1.0f)) * hbar_joule_seconds;
}

/// Calculate z-component of spin angular momentum
/// Formula: S_z = m_s·ℏ
/// @param magnetic_spin_number Magnetic spin number m_s
/// @param hbar_joule_seconds Reduced Planck constant (J·s)
/// @return Z-component of spin in J·s
HOST_DEVICE inline constexpr float calculate_spin_z_component_joule_seconds(
    float magnetic_spin_number,
    float hbar_joule_seconds = REDUCED_PLANCK_CONSTANT_JOULE_SECONDS
) {
    return magnetic_spin_number * hbar_joule_seconds;
}

} // namespace quantum
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_QUANTUM_H

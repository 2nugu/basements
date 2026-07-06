#ifndef BASEMENTS_PHYSICS_FORMULAS_WAVES_H
#define BASEMENTS_PHYSICS_FORMULAS_WAVES_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace waves {

using namespace basements::math;

constexpr float SPEED_OF_SOUND_AIR_METERS_PER_SECOND = 343.0f;  // m/s at 20°C

// ============================================================
// Simple Harmonic Motion (SHM)
// ============================================================

/// Calculate displacement in simple harmonic motion
/// Formula: x(t) = A·cos(ωt + φ)
/// @param amplitude_meters Amplitude of oscillation (m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param time_seconds Time elapsed (s)
/// @param phase_radians Phase angle (rad)
/// @return Displacement in meters (m)
HOST_DEVICE inline float calculate_shm_displacement_meters(
    float amplitude_meters,
    float angular_frequency_radians_per_second,
    float time_seconds,
    float phase_radians = 0.0f
) {
    return amplitude_meters * std::cos(
        angular_frequency_radians_per_second * time_seconds + phase_radians
    );
}

/// Calculate velocity in simple harmonic motion
/// Formula: v(t) = -Aω·sin(ωt + φ)
/// @param amplitude_meters Amplitude of oscillation (m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param time_seconds Time elapsed (s)
/// @param phase_radians Phase angle (rad)
/// @return Velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_shm_velocity_meters_per_second(
    float amplitude_meters,
    float angular_frequency_radians_per_second,
    float time_seconds,
    float phase_radians = 0.0f
) {
    return -amplitude_meters * angular_frequency_radians_per_second * 
           std::sin(angular_frequency_radians_per_second * time_seconds + phase_radians);
}

/// Calculate acceleration in simple harmonic motion
/// Formula: a(t) = -Aω²·cos(ωt + φ)
/// @param amplitude_meters Amplitude of oscillation (m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param time_seconds Time elapsed (s)
/// @param phase_radians Phase angle (rad)
/// @return Acceleration in meters per second squared (m/s²)
HOST_DEVICE inline float calculate_shm_acceleration_meters_per_second_squared(
    float amplitude_meters,
    float angular_frequency_radians_per_second,
    float time_seconds,
    float phase_radians = 0.0f
) {
    return -amplitude_meters * angular_frequency_radians_per_second 
           * angular_frequency_radians_per_second * 
           std::cos(angular_frequency_radians_per_second * time_seconds + phase_radians);
}

/// Calculate maximum velocity in simple harmonic motion
/// Formula: v_max = Aω
/// @param amplitude_meters Amplitude of oscillation (m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @return Maximum velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_shm_max_velocity_meters_per_second(
    float amplitude_meters,
    float angular_frequency_radians_per_second
) {
    return amplitude_meters * angular_frequency_radians_per_second;
}

/// Calculate maximum acceleration in simple harmonic motion
/// Formula: a_max = Aω²
/// @param amplitude_meters Amplitude of oscillation (m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @return Maximum acceleration in meters per second squared (m/s²)
HOST_DEVICE inline float calculate_shm_max_acceleration_meters_per_second_squared(
    float amplitude_meters,
    float angular_frequency_radians_per_second
) {
    return amplitude_meters * angular_frequency_radians_per_second 
           * angular_frequency_radians_per_second;
}

/// Calculate angular frequency from period
/// Formula: ω = 2π/T
/// @param period_seconds Period of oscillation (s)
/// @return Angular frequency in radians per second (rad/s)
HOST_DEVICE inline constexpr float calculate_angular_frequency_from_period(
    float period_seconds
) {
    return (period_seconds > EPSILON) 
        ? (TWO_PI / period_seconds) 
        : 0.0f;
}

/// Calculate period from angular frequency
/// Formula: T = 2π/ω
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @return Period in seconds (s)
HOST_DEVICE inline constexpr float calculate_period_from_angular_frequency(
    float angular_frequency_radians_per_second
) {
    return (angular_frequency_radians_per_second > EPSILON) 
        ? (TWO_PI / angular_frequency_radians_per_second) 
        : 0.0f;
}

/// Calculate angular frequency from frequency
/// Formula: ω = 2πf
/// @param frequency_hertz Frequency (Hz)
/// @return Angular frequency in radians per second (rad/s)
HOST_DEVICE inline constexpr float calculate_angular_frequency_from_frequency(
    float frequency_hertz
) {
    return TWO_PI * frequency_hertz;
}

// ============================================================
// Damped Oscillation
// ============================================================

/// Calculate displacement in damped oscillation
/// Formula: x(t) = Ae^(-γt)·cos(ω't + φ)
/// where ω' = √(ω₀² - γ²)
/// @param amplitude_meters Initial amplitude (m)
/// @param damping_coefficient Damping coefficient γ (1/s)
/// @param natural_frequency_radians_per_second Natural angular frequency ω₀ (rad/s)
/// @param time_seconds Time elapsed (s)
/// @param phase_radians Phase angle (rad)
/// @return Displacement in meters (m)
HOST_DEVICE inline float calculate_damped_displacement_meters(
    float amplitude_meters,
    float damping_coefficient,
    float natural_frequency_radians_per_second,
    float time_seconds,
    float phase_radians = 0.0f
) {
    float omega_prime_sq = natural_frequency_radians_per_second * natural_frequency_radians_per_second - 
                           damping_coefficient * damping_coefficient;
    
    if (omega_prime_sq < 0.0f) return 0.0f;  // Overdamped case not handled by this simple formula
    
    float omega_prime = std::sqrt(omega_prime_sq);
    return amplitude_meters * std::exp(-damping_coefficient * time_seconds) * 
           std::cos(omega_prime * time_seconds + phase_radians);
}

/// Calculate damping ratio
/// Formula: ζ = γ/ω₀
/// @param damping_coefficient Damping coefficient γ (1/s)
/// @param natural_frequency_radians_per_second Natural angular frequency ω₀ (rad/s)
/// @return Damping ratio (dimensionless)
HOST_DEVICE inline constexpr float calculate_damping_ratio(
    float damping_coefficient,
    float natural_frequency_radians_per_second
) {
    return (natural_frequency_radians_per_second > EPSILON) 
        ? (damping_coefficient / natural_frequency_radians_per_second) 
        : 0.0f;
}

/// Calculate quality factor
/// Formula: Q = ω₀/(2γ)
/// @param natural_frequency_radians_per_second Natural angular frequency ω₀ (rad/s)
/// @param damping_coefficient Damping coefficient γ (1/s)
/// @return Quality factor (dimensionless)
HOST_DEVICE inline constexpr float calculate_quality_factor(
    float natural_frequency_radians_per_second,
    float damping_coefficient
) {
    return (damping_coefficient > EPSILON) 
        ? (natural_frequency_radians_per_second / (2.0f * damping_coefficient)) 
        : 0.0f;
}

// ============================================================
// Wave Properties
// ============================================================

/// Calculate wave speed
/// Formula: v = fλ
/// @param frequency_hertz Frequency (Hz)
/// @param wavelength_meters Wavelength (m)
/// @return Wave speed in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_wave_speed_meters_per_second(
    float frequency_hertz,
    float wavelength_meters
) {
    return frequency_hertz * wavelength_meters;
}

/// Calculate wavelength from frequency
/// Formula: λ = v/f
/// @param wave_speed_meters_per_second Wave speed (m/s)
/// @param frequency_hertz Frequency (Hz)
/// @return Wavelength in meters (m)
HOST_DEVICE inline constexpr float calculate_wavelength_meters_from_frequency(
    float wave_speed_meters_per_second,
    float frequency_hertz
) {
    return (frequency_hertz > EPSILON) 
        ? (wave_speed_meters_per_second / frequency_hertz) 
        : 0.0f;
}

/// Calculate frequency from wavelength
/// Formula: f = v/λ
/// @param wave_speed_meters_per_second Wave speed (m/s)
/// @param wavelength_meters Wavelength (m)
/// @return Frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_frequency_hertz_from_wavelength(
    float wave_speed_meters_per_second,
    float wavelength_meters
) {
    return (wavelength_meters > EPSILON) 
        ? (wave_speed_meters_per_second / wavelength_meters) 
        : 0.0f;
}

/// Calculate wave number
/// Formula: k = 2π/λ
/// @param wavelength_meters Wavelength (m)
/// @return Wave number in radians per meter (rad/m)
HOST_DEVICE inline constexpr float calculate_wave_number_radians_per_meter(
    float wavelength_meters
) {
    return (wavelength_meters > EPSILON) 
        ? (TWO_PI / wavelength_meters) 
        : 0.0f;
}

/// Calculate wavelength from wave number
/// Formula: λ = 2π/k
/// @param wave_number_radians_per_meter Wave number (rad/m)
/// @return Wavelength in meters (m)
HOST_DEVICE inline constexpr float calculate_wavelength_meters_from_wave_number(
    float wave_number_radians_per_meter
) {
    return (wave_number_radians_per_meter > EPSILON) 
        ? (TWO_PI / wave_number_radians_per_meter) 
        : 0.0f;
}

// ============================================================
// Traveling Wave
// ============================================================

/// Calculate displacement of traveling wave
/// Formula: y(x,t) = A·sin(kx - ωt + φ)
/// @param amplitude_meters Amplitude (m)
/// @param wave_number_radians_per_meter Wave number (rad/m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param position_meters Position (m)
/// @param time_seconds Time (s)
/// @param phase_radians Phase angle (rad)
/// @return Displacement in meters (m)
HOST_DEVICE inline float calculate_traveling_wave_displacement_meters(
    float amplitude_meters,
    float wave_number_radians_per_meter,
    float angular_frequency_radians_per_second,
    float position_meters,
    float time_seconds,
    float phase_radians = 0.0f
) {
    return amplitude_meters * std::sin(
        wave_number_radians_per_meter * position_meters - 
        angular_frequency_radians_per_second * time_seconds + phase_radians
    );
}

/// Calculate phase velocity
/// Formula: v_p = ω/k
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param wave_number_radians_per_meter Wave number (rad/m)
/// @return Phase velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_phase_velocity_meters_per_second(
    float angular_frequency_radians_per_second,
    float wave_number_radians_per_meter
) {
    return (wave_number_radians_per_meter > EPSILON) 
        ? (angular_frequency_radians_per_second / wave_number_radians_per_meter) 
        : 0.0f;
}

/// Calculate group velocity
/// Formula: v_g = dω/dk
/// @param delta_angular_frequency Change in angular frequency
/// @param delta_wave_number Change in wave number
/// @return Group velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_group_velocity_meters_per_second(
    float delta_angular_frequency,
    float delta_wave_number
) {
    return (constexpr_abs(delta_wave_number) > EPSILON) 
        ? (delta_angular_frequency / delta_wave_number) 
        : 0.0f;
}

// ============================================================
// Standing Waves
// ============================================================

/// Calculate displacement of standing wave
/// Formula: y(x,t) = 2A·sin(kx)·cos(ωt)
/// @param amplitude_meters Component wave amplitude (m)
/// @param wave_number_radians_per_meter Wave number (rad/m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param position_meters Position (m)
/// @param time_seconds Time (s)
/// @return Displacement in meters (m)
HOST_DEVICE inline float calculate_standing_wave_displacement_meters(
    float amplitude_meters,
    float wave_number_radians_per_meter,
    float angular_frequency_radians_per_second,
    float position_meters,
    float time_seconds
) {
    return 2.0f * amplitude_meters * 
           std::sin(wave_number_radians_per_meter * position_meters) * 
           std::cos(angular_frequency_radians_per_second * time_seconds);
}

/// Calculate node position
/// Formula: x_n = nλ/2
/// @param n Node index (integer, 0, 1, 2...)
/// @param wavelength_meters Wavelength (m)
/// @return Position of n-th node in meters (m)
HOST_DEVICE inline constexpr float calculate_node_position_meters(
    int n,
    float wavelength_meters
) {
    return static_cast<float>(n) * wavelength_meters * 0.5f;
}

/// Calculate antinode position
/// Formula: x_a = (2n+1)λ/4
/// @param n Antinode index (integer, 0, 1, 2...)
/// @param wavelength_meters Wavelength (m)
/// @return Position of n-th antinode in meters (m)
HOST_DEVICE inline constexpr float calculate_antinode_position_meters(
    int n,
    float wavelength_meters
) {
    return static_cast<float>(2 * n + 1) * wavelength_meters * 0.25f;
}

/// Calculate fundamental frequency of a string
/// Formula: f₁ = v/(2L)
/// @param wave_speed_meters_per_second Speed of wave on string (m/s)
/// @param length_meters Length of string (m)
/// @return Fundamental frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_fundamental_frequency_string_hertz(
    float wave_speed_meters_per_second,
    float length_meters
) {
    return (length_meters > EPSILON) 
        ? (wave_speed_meters_per_second / (2.0f * length_meters)) 
        : 0.0f;
}

/// Calculate harmonic frequency
/// Formula: f_n = nf₁
/// @param harmonic_number Harmonic number (n = 1, 2, 3...)
/// @param fundamental_frequency_hertz Fundamental frequency (Hz)
/// @return Harmonic frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_harmonic_frequency_hertz(
    int harmonic_number,
    float fundamental_frequency_hertz
) {
    return static_cast<float>(harmonic_number) * fundamental_frequency_hertz;
}

/// Calculate wave speed on a stretched string
/// Formula: v = √(T/μ)
/// @param tension_newtons Tension in string (N)
/// @param linear_density_kilograms_per_meter Linear mass density (kg/m)
/// @return Wave speed in meters per second (m/s)
HOST_DEVICE inline float calculate_string_wave_speed_meters_per_second(
    float tension_newtons,
    float linear_density_kilograms_per_meter
) {
    return (linear_density_kilograms_per_meter > EPSILON) 
        ? std::sqrt(tension_newtons / linear_density_kilograms_per_meter) 
        : 0.0f;
}

// ============================================================
// Sound Waves
// ============================================================

/// Calculate sound intensity
/// Formula: I = P/A
/// @param power_watts Sound power (W)
/// @param area_square_meters Area (m²)
/// @return Sound intensity in Watts per square meter (W/m²)
HOST_DEVICE inline constexpr float calculate_sound_intensity_watts_per_square_meter(
    float power_watts,
    float area_square_meters
) {
    return (area_square_meters > EPSILON) 
        ? (power_watts / area_square_meters) 
        : 0.0f;
}

/// Calculate sound intensity level in decibels
/// Formula: β = 10·log₁₀(I/I₀)
/// @param intensity_watts_per_square_meter Sound intensity (W/m²)
/// @param reference_intensity Reference intensity (default 1e-12 W/m²)
/// @return Sound level in decibels (dB)
HOST_DEVICE inline float calculate_sound_level_decibels(
    float intensity_watts_per_square_meter,
    float reference_intensity = 1e-12f
) {
    return (intensity_watts_per_square_meter > EPSILON && reference_intensity > EPSILON) 
        ? (10.0f * std::log10(intensity_watts_per_square_meter / reference_intensity)) 
        : 0.0f;
}

/// Calculate intensity from decibels
/// Formula: I = I₀·10^(β/10)
/// @param decibels Sound level (dB)
/// @param reference_intensity Reference intensity (default 1e-12 W/m²)
/// @return Sound intensity in Watts per square meter (W/m²)
HOST_DEVICE inline float calculate_intensity_from_decibels(
    float decibels,
    float reference_intensity = 1e-12f
) {
    return reference_intensity * std::pow(10.0f, decibels / 10.0f);
}

/// Calculate sound pressure level
/// Formula: SPL = 20·log₁₀(p/p₀)
/// @param pressure_pascals Sound pressure (Pa)
/// @param reference_pressure Reference pressure (default 2e-5 Pa)
/// @return Sound pressure level in decibels (dB)
HOST_DEVICE inline float calculate_sound_pressure_level_decibels(
    float pressure_pascals,
    float reference_pressure = 2e-5f
) {
    return (pressure_pascals > EPSILON && reference_pressure > EPSILON) 
        ? (20.0f * std::log10(pressure_pascals / reference_pressure)) 
        : 0.0f;
}

/// Calculate intensity from pressure
/// Formula: I = p²/(2ρc)
/// @param pressure_pascals Sound pressure (Pa)
/// @param density_kilograms_per_cubic_meter Medium density (kg/m³)
/// @param sound_speed_meters_per_second Speed of sound (m/s)
/// @return Sound intensity in Watts per square meter (W/m²)
HOST_DEVICE inline constexpr float calculate_intensity_from_pressure_watts_per_square_meter(
    float pressure_pascals,
    float density_kilograms_per_cubic_meter,
    float sound_speed_meters_per_second
) {
    float denominator = 2.0f * density_kilograms_per_cubic_meter * sound_speed_meters_per_second;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (pressure_pascals * pressure_pascals / denominator) 
        : 0.0f;
}

/// Calculate acoustic impedance
/// Formula: Z = ρc
/// @param density_kilograms_per_cubic_meter Medium density (kg/m³)
/// @param sound_speed_meters_per_second Speed of sound (m/s)
/// @return Acoustic impedance in rayls (kg/(m²·s))
HOST_DEVICE inline constexpr float calculate_acoustic_impedance_rayls(
    float density_kilograms_per_cubic_meter,
    float sound_speed_meters_per_second
) {
    return density_kilograms_per_cubic_meter * sound_speed_meters_per_second;
}

// ============================================================
// Doppler Effect
// ============================================================

/// Calculate Doppler shifted frequency (general case)
/// Formula: f' = f(v ± v_o)/(v ∓ v_s)
/// @param source_frequency_hertz Source frequency (Hz)
/// @param wave_speed_meters_per_second Speed of wave (m/s)
/// @param observer_velocity_meters_per_second Observer velocity (+ moving towards source)
/// @param source_velocity_meters_per_second Source velocity (+ moving towards observer)
/// @return Observed frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_doppler_frequency_hertz(
    float source_frequency_hertz,
    float wave_speed_meters_per_second,
    float observer_velocity_meters_per_second,
    float source_velocity_meters_per_second
) {
    float numerator = wave_speed_meters_per_second + observer_velocity_meters_per_second;
    float denominator = wave_speed_meters_per_second - source_velocity_meters_per_second;
    
    return (constexpr_abs(denominator) > EPSILON) 
        ? (source_frequency_hertz * numerator / denominator) 
        : source_frequency_hertz;
}

/// Calculate Doppler shift (observer moving)
/// Formula: f' = f(1 + v_o/v)
/// @param source_frequency_hertz Source frequency (Hz)
/// @param observer_velocity_meters_per_second Observer velocity (m/s)
/// @param wave_speed_meters_per_second Wave speed (m/s)
/// @return Observed frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_doppler_observer_moving_frequency_hertz(
    float source_frequency_hertz,
    float observer_velocity_meters_per_second,
    float wave_speed_meters_per_second
) {
    return (constexpr_abs(wave_speed_meters_per_second) > EPSILON)
        ? (source_frequency_hertz * (1.0f + observer_velocity_meters_per_second / wave_speed_meters_per_second))
        : source_frequency_hertz;
}

/// Calculate Doppler shift (source moving)
/// Formula: f' = f/(1 - v_s/v)
/// @param source_frequency_hertz Source frequency (Hz)
/// @param source_velocity_meters_per_second Source velocity (m/s)
/// @param wave_speed_meters_per_second Wave speed (m/s)
/// @return Observed frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_doppler_source_moving_frequency_hertz(
    float source_frequency_hertz,
    float source_velocity_meters_per_second,
    float wave_speed_meters_per_second
) {
    float denominator = 1.0f - source_velocity_meters_per_second / wave_speed_meters_per_second;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (source_frequency_hertz / denominator) 
        : source_frequency_hertz;
}

// ============================================================
// Beat Frequency
// ============================================================

/// Calculate beat frequency
/// Formula: f_beat = |f₁ - f₂|
/// @param frequency1_hertz First frequency (Hz)
/// @param frequency2_hertz Second frequency (Hz)
/// @return Beat frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_beat_frequency_hertz(
    float frequency1_hertz,
    float frequency2_hertz
) {
    return constexpr_abs(frequency1_hertz - frequency2_hertz);
}

// ============================================================
// Resonance
// ============================================================

/// Calculate resonance frequency of LC circuit
/// Formula: f₀ = 1/(2π√(LC))
/// @param inductance_henries Inductance (H)
/// @param capacitance_farads Capacitance (F)
/// @return Resonance frequency in Hertz (Hz)
HOST_DEVICE inline float calculate_lc_resonance_frequency_hertz(
    float inductance_henries,
    float capacitance_farads
) {
    float product = inductance_henries * capacitance_farads;
    return (product > EPSILON) 
        ? (1.0f / (TWO_PI * std::sqrt(product))) 
        : 0.0f;
}

/// Calculate resonance frequency of mass-spring system
/// Formula: f₀ = (1/2π)√(k/m)
/// @param spring_constant_newtons_per_meter Spring constant (N/m)
/// @param mass_kilograms Mass (kg)
/// @return Resonance frequency in Hertz (Hz)
HOST_DEVICE inline float calculate_spring_resonance_frequency_hertz(
    float spring_constant_newtons_per_meter,
    float mass_kilograms
) {
    return (mass_kilograms > EPSILON) 
        ? (1.0f / TWO_PI * std::sqrt(spring_constant_newtons_per_meter / mass_kilograms)) 
        : 0.0f;
}

/// Calculate resonance frequency of pendulum
/// Formula: f₀ = (1/2π)√(g/L)
/// @param gravity_acceleration_meters_per_second_squared Acceleration due to gravity (m/s²)
/// @param length_meters Length of pendulum (m)
/// @return Resonance frequency in Hertz (Hz)
HOST_DEVICE inline float calculate_pendulum_resonance_frequency_hertz(
    float gravity_acceleration_meters_per_second_squared,
    float length_meters
) {
    return (length_meters > EPSILON) 
        ? (1.0f / TWO_PI * std::sqrt(gravity_acceleration_meters_per_second_squared / length_meters)) 
        : 0.0f;
}

// ============================================================
// Energy in Waves
// ============================================================

/// Calculate energy in simple harmonic motion
/// Formula: E = ½kA²
/// @param spring_constant_newtons_per_meter Spring constant (N/m)
/// @param amplitude_meters Amplitude (m)
/// @return Energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_shm_energy_joules(
    float spring_constant_newtons_per_meter,
    float amplitude_meters
) {
    return 0.5f * spring_constant_newtons_per_meter * amplitude_meters * amplitude_meters;
}

/// Calculate average power in a mechanical wave
/// Formula: P_avg = ½μω²A²v
/// @param linear_density_kilograms_per_meter Linear mass density (kg/m)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param amplitude_meters Amplitude (m)
/// @param wave_speed_meters_per_second Wave speed (m/s)
/// @return Average power in Watts (W)
HOST_DEVICE inline constexpr float calculate_wave_average_power_watts(
    float linear_density_kilograms_per_meter,
    float angular_frequency_radians_per_second,
    float amplitude_meters,
    float wave_speed_meters_per_second
) {
    return 0.5f * linear_density_kilograms_per_meter 
           * angular_frequency_radians_per_second * angular_frequency_radians_per_second 
           * amplitude_meters * amplitude_meters * wave_speed_meters_per_second;
}

/// Calculate intensity of a mechanical wave
/// Formula: I = P/A = ½ρvω²A²
/// @param density_kilograms_per_cubic_meter Medium density (kg/m³)
/// @param wave_speed_meters_per_second Wave speed (m/s)
/// @param angular_frequency_radians_per_second Angular frequency (rad/s)
/// @param amplitude_meters Amplitude (m)
/// @return Intensity in Watts per square meter (W/m²)
HOST_DEVICE inline constexpr float calculate_wave_intensity_watts_per_square_meter(
    float density_kilograms_per_cubic_meter,
    float wave_speed_meters_per_second,
    float angular_frequency_radians_per_second,
    float amplitude_meters
) {
    return 0.5f * density_kilograms_per_cubic_meter * wave_speed_meters_per_second 
           * angular_frequency_radians_per_second * angular_frequency_radians_per_second 
           * amplitude_meters * amplitude_meters;
}

// ============================================================
// Seismic Waves
// ============================================================

/// Calculate P-wave speed (primary wave)
/// Formula: v_p = √((K + 4μ/3)/ρ)
/// @param bulk_modulus_pascals Bulk modulus (Pa)
/// @param shear_modulus_pascals Shear modulus (Pa)
/// @param density_kilograms_per_cubic_meter Density (kg/m³)
/// @return Wave speed in meters per second (m/s)
HOST_DEVICE inline float calculate_p_wave_speed_meters_per_second(
    float bulk_modulus_pascals,
    float shear_modulus_pascals,
    float density_kilograms_per_cubic_meter
) {
    float numerator = bulk_modulus_pascals + (4.0f / 3.0f) * shear_modulus_pascals;
    return (density_kilograms_per_cubic_meter > EPSILON) 
        ? std::sqrt(numerator / density_kilograms_per_cubic_meter) 
        : 0.0f;
}

/// Calculate S-wave speed (secondary wave)
/// Formula: v_s = √(μ/ρ)
/// @param shear_modulus_pascals Shear modulus (Pa)
/// @param density_kilograms_per_cubic_meter Density (kg/m³)
/// @return Wave speed in meters per second (m/s)
HOST_DEVICE inline float calculate_s_wave_speed_meters_per_second(
    float shear_modulus_pascals,
    float density_kilograms_per_cubic_meter
) {
    return (density_kilograms_per_cubic_meter > EPSILON) 
        ? std::sqrt(shear_modulus_pascals / density_kilograms_per_cubic_meter) 
        : 0.0f;
}

/// Calculate earthquake magnitude (Richter scale)
/// Formula: M_L = log₁₀(A) + f(Δ)
/// @param amplitude_millimeters Maximum amplitude (mm)
/// @param distance_correction Distance correction factor (dimensionless)
/// @return Magnitude (dimensionless)
HOST_DEVICE inline float calculate_richter_magnitude(
    float amplitude_millimeters,
    float distance_correction = 0.0f
) {
    return (amplitude_millimeters > EPSILON) 
        ? (std::log10(amplitude_millimeters) + distance_correction) 
        : 0.0f;
}

/// Calculate earthquake energy from magnitude
/// Formula: log₁₀(E) = 11.8 + 1.5M  =>  E = 10^(11.8 + 1.5M)
/// @param magnitude Earthquake magnitude
/// @return Energy in Joules (J)
HOST_DEVICE inline float calculate_earthquake_energy_joules(
    float magnitude
) {
    return std::pow(10.0f, 11.8f + 1.5f * magnitude);
}

    // ========================================================================
    // Sound Waves
    // ========================================================================

    /// Calculate speed of sound in ideal gas
    /// Formula: v = sqrt(γ * R * T / M)
    /// @param adiabatic_index Gamma (1.4 for air)
    /// @param gas_constant_joules_per_mole_kelvin Universal gas constant
    /// @param temperature_kelvin Temperature (K)
    /// @param molar_mass_kilograms_per_mole Molar mass (kg/mol)
    HOST_DEVICE inline float calculate_speed_of_sound_ideal_gas_meters_per_second(
        float adiabatic_index,
        float gas_constant_joules_per_mole_kelvin,
        float temperature_kelvin,
        float molar_mass_kilograms_per_mole
    ) {
        if (molar_mass_kilograms_per_mole < EPSILON) return 0.0f;
        return std::sqrt(adiabatic_index * gas_constant_joules_per_mole_kelvin * temperature_kelvin / molar_mass_kilograms_per_mole);
    }



    /// Calculate Constructive Interference Condition (Path Difference)
    /// Formula: Δx = n * λ
    HOST_DEVICE inline bool is_constructive_interference(
        float path_difference_meters,
        float wavelength_meters,
        float tolerance = 1e-3f
    ) {
        if (wavelength_meters < EPSILON) return false;
        float ratio = path_difference_meters / wavelength_meters;
        float nearest_int = std::round(ratio);
        return std::abs(ratio - nearest_int) < tolerance;
    }

} // namespace waves
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_WAVES_H

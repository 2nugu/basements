#ifndef BASEMENTS_PHYSICS_FORMULAS_OPTICS_H
#define BASEMENTS_PHYSICS_FORMULAS_OPTICS_H

#include "basements/core/math/vec3.h"
#include "basements/core/math/common.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace optics {

using Vec3 = basements::math::Vec3;
constexpr float EPSILON = 1e-6f;
constexpr float HALF_PI = 1.57079632679f;

constexpr float SPEED_OF_LIGHT_METERS_PER_SECOND = 299792458.0f;  // m/s

// ============================================================
// Reflection
// ============================================================

/// Calculate reflection vector (Law of Reflection)
/// Formula: r = i - 2(i·n)n
/// @param incident_vector Normalized incident vector
/// @param normal_vector Normalized surface normal vector
/// @return Reflected vector
HOST_DEVICE inline Vec3 calculate_reflection_vector(
    const Vec3& incident_vector,
    const Vec3& normal_vector
) {
    return incident_vector - normal_vector * (2.0f * incident_vector.dot(normal_vector));
}

/// Calculate reflection angle (equal to incident angle)
/// Formula: θ_r = θ_i
/// @param incident_angle_radians Angle of incidence (rad)
/// @return Angle of reflection in radians (rad)
HOST_DEVICE inline constexpr float calculate_reflection_angle_radians(
    float incident_angle_radians
) {
    return incident_angle_radians;
}

// ============================================================
// Refraction (Snell's Law)
// ============================================================

/// Calculate refraction angle using Snell's law
/// Formula: n₁·sin(θ₁) = n₂·sin(θ₂)  =>  θ₂ = arcsin((n₁/n₂)·sin(θ₁))
/// @param incident_angle_radians Angle of incidence (rad)
/// @param refractive_index_1 Refractive index of first medium
/// @param refractive_index_2 Refractive index of second medium
/// @return Angle of refraction in radians (rad), or -1.0f if total internal reflection occurs
HOST_DEVICE inline float calculate_refraction_angle_radians(
    float incident_angle_radians,
    float refractive_index_1,
    float refractive_index_2
) {
    if (refractive_index_2 < EPSILON) return 0.0f;
    
    float sin_theta2 = (refractive_index_1 / refractive_index_2) * std::sin(incident_angle_radians);
    
    // Total internal reflection check
    if (std::abs(sin_theta2) > 1.0f) return -1.0f;
    
    return std::asin(sin_theta2);
}

/// Calculate refraction vector (Snell's law in vector form)
/// @param incident_vector Normalized incident vector
/// @param normal_vector Normalized surface normal vector
/// @param refractive_index_1 Refractive index of first medium
/// @param refractive_index_2 Refractive index of second medium
/// @return Refracted vector (zero vector if total internal reflection)
HOST_DEVICE inline Vec3 calculate_refraction_vector(
    const Vec3& incident_vector,
    const Vec3& normal_vector,
    float refractive_index_1,
    float refractive_index_2
) {
    if (refractive_index_2 < EPSILON) return Vec3::zero();
    
    float eta = refractive_index_1 / refractive_index_2;
    float cos_i = -normal_vector.dot(incident_vector);
    float sin_t2 = eta * eta * (1.0f - cos_i * cos_i);
    
    // Total internal reflection
    if (sin_t2 > 1.0f) return Vec3::zero();
    
    float cos_t = std::sqrt(1.0f - sin_t2);
    return incident_vector * eta + normal_vector * (eta * cos_i - cos_t);
}

/// Calculate critical angle for total internal reflection
/// Formula: θ_c = arcsin(n₂/n₁)
/// @param refractive_index_1 Refractive index of denser medium
/// @param refractive_index_2 Refractive index of less dense medium
/// @return Critical angle in radians (rad)
HOST_DEVICE inline float calculate_critical_angle_radians(
    float refractive_index_1,
    float refractive_index_2
) {
    if (refractive_index_1 < EPSILON || refractive_index_2 >= refractive_index_1) return HALF_PI;
    return std::asin(refractive_index_2 / refractive_index_1);
}

/// Calculate refractive index from speed of light in medium
/// Formula: n = c/v
/// @param speed_in_medium_meters_per_second Speed of light in medium (m/s)
/// @param speed_of_light_vacuum_meters_per_second Speed of light in vacuum (m/s)
/// @return Refractive index (dimensionless)
HOST_DEVICE inline constexpr float calculate_refractive_index(
    float speed_in_medium_meters_per_second,
    float speed_of_light_vacuum_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return (speed_in_medium_meters_per_second > EPSILON) 
        ? (speed_of_light_vacuum_meters_per_second / speed_in_medium_meters_per_second) 
        : 1.0f;
}

/// Calculate speed of light in medium
/// Formula: v = c/n
/// @param refractive_index Refractive index of medium
/// @param speed_of_light_vacuum_meters_per_second Speed of light in vacuum (m/s)
/// @return Speed of light in medium in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_speed_in_medium_meters_per_second(
    float refractive_index,
    float speed_of_light_vacuum_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return (refractive_index > EPSILON) 
        ? (speed_of_light_vacuum_meters_per_second / refractive_index) 
        : speed_of_light_vacuum_meters_per_second;
}

// ============================================================
// Lenses (Thin Lens Equation)
// ============================================================

/// Calculate image distance using thin lens equation
/// Formula: 1/f = 1/d_o + 1/d_i  =>  d_i = (d_o·f) / (d_o - f)
/// @param object_distance_meters Distance to object (m)
/// @param focal_length_meters Focal length (m)
/// @return Image distance in meters (m)
HOST_DEVICE inline constexpr float calculate_image_distance_meters(
    float object_distance_meters,
    float focal_length_meters
) {
    float denominator = object_distance_meters - focal_length_meters;
    return (basements::math::constexpr_abs(denominator) > EPSILON) ? 
           (object_distance_meters * focal_length_meters / denominator) : 0.0f;
}

/// Calculate object distance using thin lens equation
/// Formula: d_o = (d_i·f) / (d_i - f)
/// @param image_distance_meters Distance to image (m)
/// @param focal_length_meters Focal length (m)
/// @return Object distance in meters (m)
HOST_DEVICE inline constexpr float calculate_object_distance_meters(
    float image_distance_meters,
    float focal_length_meters
) {
    float denominator = image_distance_meters - focal_length_meters;
    return (basements::math::constexpr_abs(denominator) > EPSILON) ? 
           (image_distance_meters * focal_length_meters / denominator) : 0.0f;
}

/// Calculate focal length from object and image distances
/// Formula: f = (d_o·d_i) / (d_o + d_i)
/// @param object_distance_meters Distance to object (m)
/// @param image_distance_meters Distance to image (m)
/// @return Focal length in meters (m)
HOST_DEVICE inline constexpr float calculate_focal_length_meters(
    float object_distance_meters,
    float image_distance_meters
) {
    float denominator = object_distance_meters + image_distance_meters;
    return (basements::math::constexpr_abs(denominator) > EPSILON) ? 
           (object_distance_meters * image_distance_meters / denominator) : 0.0f;
}

/// Calculate magnification
/// Formula: m = -d_i/d_o
/// @param image_distance_meters Distance to image (m)
/// @param object_distance_meters Distance to object (m)
/// @return Magnification (dimensionless)
HOST_DEVICE inline constexpr float calculate_magnification(
    float image_distance_meters,
    float object_distance_meters
) {
    return (basements::math::constexpr_abs(object_distance_meters) > EPSILON) ? 
           (-image_distance_meters / object_distance_meters) : 0.0f;
}

/// Calculate image height from magnification
/// Formula: h_i = m·h_o
/// @param magnification Magnification factor
/// @param object_height_meters Height of object (m)
/// @return Image height in meters (m)
HOST_DEVICE inline constexpr float calculate_image_height_meters(
    float magnification,
    float object_height_meters
) {
    return magnification * object_height_meters;
}

/// Calculate focal length using Lens Maker's Equation
/// Formula: 1/f = (n-1)·(1/R₁ - 1/R₂)
/// @param refractive_index Lens refractive index
/// @param radius_curvature_1_meters Radius of curvature of first surface (m)
/// @param radius_curvature_2_meters Radius of curvature of second surface (m)
/// @return Focal length in meters (m)
HOST_DEVICE inline constexpr float calculate_lens_makers_focal_length_meters(
    float refractive_index,
    float radius_curvature_1_meters,
    float radius_curvature_2_meters
) {
    float curvature_term = (1.0f / radius_curvature_1_meters) - (1.0f / radius_curvature_2_meters);
    return (basements::math::constexpr_abs(curvature_term) > EPSILON) ? 
           (1.0f / ((refractive_index - 1.0f) * curvature_term)) : 0.0f;
}

// ============================================================
// Mirrors
// ============================================================

/// Calculate image distance for spherical mirror
/// Formula: 1/f = 1/d_o + 1/d_i (Same as lens)
/// @param object_distance_meters Distance to object (m)
/// @param focal_length_meters Focal length (m)
/// @return Image distance in meters (m)
HOST_DEVICE inline constexpr float calculate_mirror_image_distance_meters(
    float object_distance_meters,
    float focal_length_meters
) {
    return calculate_image_distance_meters(object_distance_meters, focal_length_meters);
}

/// Calculate focal length of spherical mirror
/// Formula: f = R/2
/// @param radius_of_curvature_meters Radius of curvature (m)
/// @return Focal length in meters (m)
HOST_DEVICE inline constexpr float calculate_mirror_focal_length_meters(
    float radius_of_curvature_meters
) {
    return radius_of_curvature_meters * 0.5f;
}

// ============================================================
// Wave Optics
// ============================================================

/// Calculate wavelength from frequency (in vacuum)
/// Formula: λ = c/f
/// @param frequency_hertz Frequency (Hz)
/// @param speed_of_light_meters_per_second Speed of light (m/s)
/// @return Wavelength in meters (m)
HOST_DEVICE inline constexpr float calculate_vacuum_wavelength_meters(
    float frequency_hertz,
    float speed_of_light_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return (frequency_hertz > EPSILON) 
        ? (speed_of_light_meters_per_second / frequency_hertz) 
        : 0.0f;
}

/// Calculate frequency from wavelength (in vacuum)
/// Formula: f = c/λ
/// @param wavelength_meters Wavelength (m)
/// @param speed_of_light_meters_per_second Speed of light (m/s)
/// @return Frequency in Hertz (Hz)
HOST_DEVICE inline constexpr float calculate_frequency_hertz_from_vacuum_wavelength(
    float wavelength_meters,
    float speed_of_light_meters_per_second = SPEED_OF_LIGHT_METERS_PER_SECOND
) {
    return (wavelength_meters > EPSILON) 
        ? (speed_of_light_meters_per_second / wavelength_meters) 
        : 0.0f;
}

/// Calculate wavelength in medium
/// Formula: λ_medium = λ_vacuum/n
/// @param wavelength_vacuum_meters Wavelength in vacuum (m)
/// @param refractive_index Refractive index of medium
/// @return Wavelength in medium in meters (m)
HOST_DEVICE inline constexpr float calculate_wavelength_in_medium_meters(
    float wavelength_vacuum_meters,
    float refractive_index
) {
    return (refractive_index > EPSILON) ? 
           (wavelength_vacuum_meters / refractive_index) : wavelength_vacuum_meters;
}

// ============================================================
// Interference
// ============================================================

/// Calculate path difference for constructive interference
/// Formula: Δx = m·λ
/// @param order Interference order (integer)
/// @param wavelength_meters Wavelength (m)
/// @return Path difference in meters (m)
HOST_DEVICE inline constexpr float calculate_constructive_interference_path_meters(
    int order,
    float wavelength_meters
) {
    return static_cast<float>(order) * wavelength_meters;
}

/// Calculate path difference for destructive interference
/// Formula: Δx = (m + 0.5)·λ
/// @param order Interference order (integer)
/// @param wavelength_meters Wavelength (m)
/// @return Path difference in meters (m)
HOST_DEVICE inline constexpr float calculate_destructive_interference_path_meters(
    int order,
    float wavelength_meters
) {
    return (static_cast<float>(order) + 0.5f) * wavelength_meters;
}

/// Calculate angle for double slit maxima
/// Formula: sin(θ) = m·λ/d  =>  θ = arcsin(m·λ/d)
/// @param order Order of maximum (integer)
/// @param wavelength_meters Wavelength (m)
/// @param slit_separation_meters Distance between slits (m)
/// @return Angle in radians (rad)
HOST_DEVICE inline float calculate_double_slit_angle_radians(
    int order,
    float wavelength_meters,
    float slit_separation_meters
) {
    return (slit_separation_meters > EPSILON) ? 
           std::asin(static_cast<float>(order) * wavelength_meters / slit_separation_meters) : 0.0f;
}

/// Calculate fringe spacing in double slit experiment
/// Formula: y = m·λ·L/d (small angle approximation)
/// @param wavelength_meters Wavelength (m)
/// @param screen_distance_meters Distance to screen (m)
/// @param slit_separation_meters Separation between slits (m)
/// @return Fringe spacing in meters (m)
HOST_DEVICE inline constexpr float calculate_fringe_spacing_meters(
    float wavelength_meters,
    float screen_distance_meters,
    float slit_separation_meters
) {
    return (slit_separation_meters > EPSILON) ? 
           (wavelength_meters * screen_distance_meters / slit_separation_meters) : 0.0f;
}

// ============================================================
// Diffraction
// ============================================================

/// Calculate angle for single slit minima
/// Formula: sin(θ) = m·λ/a  =>  θ = arcsin(m·λ/a)
/// @param order Order of minimum (integer, non-zero)
/// @param wavelength_meters Wavelength (m)
/// @param slit_width_meters Width of slit (m)
/// @return Angle in radians (rad)
HOST_DEVICE inline float calculate_single_slit_minima_angle_radians(
    int order,
    float wavelength_meters,
    float slit_width_meters
) {
    return (slit_width_meters > EPSILON) ? 
           std::asin(static_cast<float>(order) * wavelength_meters / slit_width_meters) : 0.0f;
}

/// Calculate Rayleigh criterion angle (minimum angular resolution)
/// Formula: θ_min = 1.22·λ/D
/// @param wavelength_meters Wavelength (m)
/// @param aperture_diameter_meters Diameter of aperture (m)
/// @return Minimum resolvable angle in radians (rad)
HOST_DEVICE inline constexpr float calculate_rayleigh_criterion_radians(
    float wavelength_meters,
    float aperture_diameter_meters
) {
    return (aperture_diameter_meters > EPSILON) ? 
           (1.22f * wavelength_meters / aperture_diameter_meters) : 0.0f;
}

/// Calculate resolving power of aperture
/// Formula: R = D/(1.22·λ)
/// @param aperture_diameter_meters Diameter of aperture (m)
/// @param wavelength_meters Wavelength (m)
/// @return Resolving power (dimensionless/inverse radians)
HOST_DEVICE inline constexpr float calculate_resolving_power(
    float aperture_diameter_meters,
    float wavelength_meters
) {
    return (wavelength_meters > EPSILON) ? 
           (aperture_diameter_meters / (1.22f * wavelength_meters)) : 0.0f;
}

// ============================================================
// Polarization
// ============================================================

/// Calculate intensity after polarizer (Malus's Law)
/// Formula: I = I₀·cos²(θ)
/// @param initial_intensity_watts_per_square_meter Initial intensity (W/m²)
/// @param angle_radians Angle between polarization axis and light polarization (rad)
/// @return Transmitted intensity in Watts per square meter (W/m²)
HOST_DEVICE inline float calculate_intensity_after_polarizer_watts_per_square_meter(
    float initial_intensity_watts_per_square_meter,
    float angle_radians
) {
    float cos_theta = std::cos(angle_radians);
    return initial_intensity_watts_per_square_meter * cos_theta * cos_theta;
}

/// Calculate Brewster's angle
/// Formula: θ_B = arctan(n₂/n₁)
/// @param refractive_index_1 Refractive index of initial medium
/// @param refractive_index_2 Refractive index of reflecting medium
/// @return Brewster's angle in radians (rad)
HOST_DEVICE inline float calculate_brewster_angle_radians(
    float refractive_index_1,
    float refractive_index_2
) {
    return (refractive_index_1 > EPSILON) 
        ? std::atan(refractive_index_2 / refractive_index_1) 
        : 0.0f;
}

// ============================================================
// Photometry
// ============================================================

/// Calculate illuminance from luminous intensity
/// Formula: E = I/r²
/// @param luminous_intensity_candela Luminous intensity (cd)
/// @param distance_meters Distance from source (m)
/// @return Illuminance in Lux (lx)
HOST_DEVICE inline constexpr float calculate_illuminance_lux(
    float luminous_intensity_candela,
    float distance_meters
) {
    return (distance_meters > EPSILON) ? 
           (luminous_intensity_candela / (distance_meters * distance_meters)) : 0.0f;
}

/// Calculate intensity at new distance (Inverse Square Law)
/// Formula: I₂ = I₁·(r₁/r₂)²
/// @param initial_intensity Initial intensity
/// @param initial_distance_meters Initial distance (m)
/// @param final_distance_meters Final distance (m)
/// @return Intensity at final distance
HOST_DEVICE inline constexpr float calculate_intensity_at_distance(
    float initial_intensity,
    float initial_distance_meters,
    float final_distance_meters
) {
    return (final_distance_meters > EPSILON) ? 
           (initial_intensity * (initial_distance_meters / final_distance_meters) 
            * (initial_distance_meters / final_distance_meters)) : 0.0f;
}

} // namespace optics
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_OPTICS_H

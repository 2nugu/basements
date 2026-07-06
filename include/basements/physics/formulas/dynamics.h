#ifndef BASEMENTS_PHYSICS_FORMULAS_DYNAMICS_H
#define BASEMENTS_PHYSICS_FORMULAS_DYNAMICS_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace dynamics {

using namespace basements::math;

// Physical constants
constexpr float GRAVITATIONAL_CONSTANT_SI = 6.67430e-11f;  // G in N·m²/kg² (SI units)

// ============================================================
// Newton's Laws of Motion
// ============================================================

/// Calculate force using Newton's Second Law
/// Formula: F = m·a
/// @param object_mass_kilograms Mass of the object (kg)
/// @param acceleration_meters_per_second_squared Acceleration (m/s²)
/// @return Force in Newtons (N)
HOST_DEVICE inline constexpr float calculate_force_newtons_second_law(
    float object_mass_kilograms,
    float acceleration_meters_per_second_squared
) {
    return object_mass_kilograms * acceleration_meters_per_second_squared;
}

/// Calculate acceleration from force and mass
/// Formula: a = F / m
/// @param force_newtons Applied force (N)
/// @param object_mass_kilograms Mass of the object (kg)
/// @return Acceleration in meters per second squared
HOST_DEVICE inline constexpr float calculate_acceleration_from_force(
    float force_newtons,
    float object_mass_kilograms
) {
    return (object_mass_kilograms > EPSILON) ? 
           (force_newtons / object_mass_kilograms) : 0.0f;
}

/// Calculate mass from force and acceleration
/// Formula: m = F / a
/// @param force_newtons Applied force (N)
/// @param acceleration_meters_per_second_squared Resulting acceleration (m/s²)
/// @return Mass in kilograms
HOST_DEVICE inline constexpr float calculate_mass_from_force_and_acceleration(
    float force_newtons,
    float acceleration_meters_per_second_squared
) {
    return (constexpr_abs(acceleration_meters_per_second_squared) > EPSILON) ? 
           (force_newtons / acceleration_meters_per_second_squared) : 0.0f;
}

// Vector versions of Newton's Second Law
HOST_DEVICE inline Vec3 calculate_force_vector_newtons_second_law(
    float object_mass_kilograms,
    const Vec3& acceleration_meters_per_second_squared
) {
    return acceleration_meters_per_second_squared * object_mass_kilograms;
}

HOST_DEVICE inline Vec3 calculate_acceleration_vector_from_force(
    const Vec3& force_vector_newtons,
    float object_mass_kilograms
) {
    return (object_mass_kilograms > EPSILON) ? 
           (force_vector_newtons / object_mass_kilograms) : Vec3::zero();
}

// ============================================================
// Linear Momentum
// ============================================================

/// Calculate linear momentum
/// Formula: p = m·v
/// @param object_mass_kilograms Mass of the object (kg)
/// @param velocity_meters_per_second Velocity (m/s)
/// @return Momentum in kilogram-meters per second (kg·m/s)
HOST_DEVICE inline constexpr float calculate_linear_momentum(
    float object_mass_kilograms,
    float velocity_meters_per_second
) {
    return object_mass_kilograms * velocity_meters_per_second;
}

/// Calculate velocity from momentum and mass
/// Formula: v = p / m
/// @param momentum_kilogram_meters_per_second Momentum (kg·m/s)
/// @param object_mass_kilograms Mass (kg)
/// @return Velocity in meters per second
HOST_DEVICE inline constexpr float calculate_velocity_from_momentum(
    float momentum_kilogram_meters_per_second,
    float object_mass_kilograms
) {
    return (object_mass_kilograms > EPSILON) ? 
           (momentum_kilogram_meters_per_second / object_mass_kilograms) : 0.0f;
}

/// Calculate mass from momentum and velocity
/// Formula: m = p / v
/// @param momentum_kilogram_meters_per_second Momentum (kg·m/s)
/// @param velocity_meters_per_second Velocity (m/s)
/// @return Mass in kilograms
HOST_DEVICE inline constexpr float calculate_mass_from_momentum_and_velocity(
    float momentum_kilogram_meters_per_second,
    float velocity_meters_per_second
) {
    return (constexpr_abs(velocity_meters_per_second) > EPSILON) ? 
           (momentum_kilogram_meters_per_second / velocity_meters_per_second) : 0.0f;
}

// Vector versions of momentum
HOST_DEVICE inline Vec3 calculate_linear_momentum_vector(
    float object_mass_kilograms,
    const Vec3& velocity_meters_per_second
) {
    return velocity_meters_per_second * object_mass_kilograms;
}

HOST_DEVICE inline Vec3 calculate_velocity_vector_from_momentum(
    const Vec3& momentum_vector_kilogram_meters_per_second,
    float object_mass_kilograms
) {
    return (object_mass_kilograms > EPSILON) ? 
           (momentum_vector_kilogram_meters_per_second / object_mass_kilograms) : Vec3::zero();
}

// ============================================================
// Impulse (Change in Momentum)
// ============================================================

/// Calculate impulse from force and time
/// Formula: J = F·Δt
/// @param force_newtons Applied force (N)
/// @param time_interval_seconds Duration of force application (s)
/// @return Impulse in Newton-seconds (N·s)
HOST_DEVICE inline constexpr float calculate_impulse_from_force(
    float force_newtons,
    float time_interval_seconds
) {
    return force_newtons * time_interval_seconds;
}

/// Calculate average force from impulse
/// Formula: F = J / Δt
/// @param impulse_newton_seconds Impulse (N·s)
/// @param time_interval_seconds Duration (s)
/// @return Average force in Newtons
HOST_DEVICE inline constexpr float calculate_average_force_from_impulse(
    float impulse_newton_seconds,
    float time_interval_seconds
) {
    return (time_interval_seconds > EPSILON) ? 
           (impulse_newton_seconds / time_interval_seconds) : 0.0f;
}

/// Calculate time from impulse and force
/// Formula: Δt = J / F
/// @param impulse_newton_seconds Impulse (N·s)
/// @param force_newtons Applied force (N)
/// @return Time interval in seconds
HOST_DEVICE inline constexpr float calculate_time_from_impulse_and_force(
    float impulse_newton_seconds,
    float force_newtons
) {
    return (constexpr_abs(force_newtons) > EPSILON) ? 
           (impulse_newton_seconds / force_newtons) : 0.0f;
}

// Vector versions of impulse
HOST_DEVICE inline Vec3 calculate_impulse_vector_from_force(
    const Vec3& force_vector_newtons,
    float time_interval_seconds
) {
    return force_vector_newtons * time_interval_seconds;
}

HOST_DEVICE inline Vec3 calculate_average_force_vector_from_impulse(
    const Vec3& impulse_vector_newton_seconds,
    float time_interval_seconds
) {
    return (time_interval_seconds > EPSILON) ? 
           (impulse_vector_newton_seconds / time_interval_seconds) : Vec3::zero();
}

/// Calculate velocity change from impulse
/// Formula: Δv = J / m
/// @param impulse_vector_newton_seconds Impulse vector (N·s)
/// @param object_mass_kilograms Mass (kg)
/// @return Velocity change vector in meters per second
HOST_DEVICE inline Vec3 calculate_velocity_change_from_impulse(
    const Vec3& impulse_vector_newton_seconds,
    float object_mass_kilograms
) {
    return (object_mass_kilograms > EPSILON) ? 
           (impulse_vector_newton_seconds / object_mass_kilograms) : Vec3::zero();
}

/// Calculate impulse from momentum change
/// Formula: J = Δp = m·Δv
/// @param momentum_change_vector Momentum change vector (kg·m/s)
/// @return Impulse vector in Newton-seconds
HOST_DEVICE inline Vec3 calculate_impulse_from_momentum_change(
    const Vec3& momentum_change_vector
) {
    return momentum_change_vector;
}

// ============================================================
// Friction Forces
// ============================================================

/// Calculate friction force
/// Formula: F_friction = μ·N
/// @param coefficient_of_friction Friction coefficient (dimensionless, typically 0-1+)
/// @param normal_force_newtons Normal force perpendicular to surface (N)
/// @return Friction force in Newtons
HOST_DEVICE inline constexpr float calculate_friction_force_newtons(
    float coefficient_of_friction,
    float normal_force_newtons
) {
    return coefficient_of_friction * constexpr_abs(normal_force_newtons);
}

/// Calculate normal force from friction force
/// Formula: N = F_friction / μ
/// @param friction_force_newtons Friction force (N)
/// @param coefficient_of_friction Friction coefficient
/// @return Normal force in Newtons
HOST_DEVICE inline constexpr float calculate_normal_force_from_friction(
    float friction_force_newtons,
    float coefficient_of_friction
) {
    return (coefficient_of_friction > EPSILON) ? 
           (friction_force_newtons / coefficient_of_friction) : 0.0f;
}

/// Calculate friction coefficient from forces
/// Formula: μ = F_friction / N
/// @param friction_force_newtons Friction force (N)
/// @param normal_force_newtons Normal force (N)
/// @return Coefficient of friction (dimensionless)
HOST_DEVICE inline constexpr float calculate_friction_coefficient(
    float friction_force_newtons,
    float normal_force_newtons
) {
    return (constexpr_abs(normal_force_newtons) > EPSILON) ? 
           (friction_force_newtons / constexpr_abs(normal_force_newtons)) : 0.0f;
}

/// Calculate maximum static friction force
/// Formula: F_static_max = μ_s·N
/// @param static_friction_coefficient Static friction coefficient (dimensionless)
/// @param normal_force_newtons Normal force (N)
/// @return Maximum static friction in Newtons
HOST_DEVICE inline constexpr float calculate_maximum_static_friction_force(
    float static_friction_coefficient,
    float normal_force_newtons
) {
    return static_friction_coefficient * constexpr_abs(normal_force_newtons);
}

/// Calculate kinetic friction force
/// Formula: F_kinetic = μ_k·N
/// @param kinetic_friction_coefficient Kinetic friction coefficient (dimensionless)
/// @param normal_force_newtons Normal force (N)
/// @return Kinetic friction force in Newtons
HOST_DEVICE inline constexpr float calculate_kinetic_friction_force(
    float kinetic_friction_coefficient,
    float normal_force_newtons
) {
    return kinetic_friction_coefficient * constexpr_abs(normal_force_newtons);
}

// ============================================================
// Drag Forces (Air/Fluid Resistance)
// ============================================================

/// Calculate linear drag force (low speed)
/// Formula: F_drag = -b·v
/// @param linear_drag_coefficient Drag coefficient (N·s/m)
/// @param velocity_meters_per_second Velocity (m/s)
/// @return Drag force in Newtons (negative indicates opposition to motion)
HOST_DEVICE inline constexpr float calculate_linear_drag_force(
    float linear_drag_coefficient,
    float velocity_meters_per_second
) {
    return -linear_drag_coefficient * velocity_meters_per_second;
}

HOST_DEVICE inline Vec3 calculate_linear_drag_force_vector(
    float linear_drag_coefficient,
    const Vec3& velocity_meters_per_second
) {
    return velocity_meters_per_second * (-linear_drag_coefficient);
}

/// Calculate quadratic drag force (high speed)
/// Formula: F_drag = -½·ρ·v²·C_d·A
/// @param fluid_density_kg_per_cubic_meter Density of fluid (air: ~1.225 kg/m³, water: ~1000 kg/m³)
/// @param velocity_meters_per_second Object velocity relative to fluid (m/s)
/// @param drag_coefficient Dimensionless drag coefficient (sphere: ~0.47, cube: ~1.05)
/// @param cross_sectional_area_square_meters Frontal area perpendicular to motion (m²)
/// @return Drag force in Newtons (negative indicates opposition to motion)
HOST_DEVICE inline constexpr float calculate_quadratic_drag_force_newtons(
    float fluid_density_kg_per_cubic_meter,
    float velocity_meters_per_second,
    float drag_coefficient,
    float cross_sectional_area_square_meters
) {
    return -0.5f * fluid_density_kg_per_cubic_meter 
         * velocity_meters_per_second * constexpr_abs(velocity_meters_per_second) 
         * drag_coefficient * cross_sectional_area_square_meters;
}

HOST_DEVICE inline Vec3 calculate_quadratic_drag_force_vector_newtons(
    float fluid_density_kg_per_cubic_meter,
    const Vec3& velocity_meters_per_second,
    float drag_coefficient,
    float cross_sectional_area_square_meters
) {
    float speed = velocity_meters_per_second.length();
    if (speed < EPSILON) return Vec3::zero();
    
    float drag_magnitude = 0.5f * fluid_density_kg_per_cubic_meter 
                         * speed * speed * drag_coefficient * cross_sectional_area_square_meters;
    return velocity_meters_per_second.normalized() * (-drag_magnitude);
}

/// Calculate terminal velocity (when drag equals weight)
/// Formula: v_terminal = sqrt(2·m·g / (ρ·C_d·A))
/// @param object_mass_kilograms Mass of falling object (kg)
/// @param gravity_acceleration_meters_per_second_squared Gravity magnitude, typically 9.81 (m/s²)
/// @param fluid_density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param drag_coefficient Drag coefficient (dimensionless)
/// @param cross_sectional_area_square_meters Cross-sectional area (m²)
/// @return Terminal velocity in meters per second
HOST_DEVICE inline float calculate_terminal_velocity_meters_per_second(
    float object_mass_kilograms,
    float gravity_acceleration_meters_per_second_squared,
    float fluid_density_kg_per_cubic_meter,
    float drag_coefficient,
    float cross_sectional_area_square_meters
) {
    float denominator = fluid_density_kg_per_cubic_meter * drag_coefficient * cross_sectional_area_square_meters;
    return (denominator > EPSILON) ? 
           std::sqrt(2.0f * object_mass_kilograms * gravity_acceleration_meters_per_second_squared / denominator) : 0.0f;
}

// ============================================================
// Spring Forces (Hooke's Law)
// ============================================================

/// Calculate spring restoring force
/// Formula: F = -k·x
/// @param spring_constant_newtons_per_meter Spring stiffness (N/m)
/// @param displacement_from_equilibrium_meters Displacement from rest position (m)
/// @return Restoring force in Newtons (negative indicates toward equilibrium)
HOST_DEVICE inline constexpr float calculate_spring_restoring_force(
    float spring_constant_newtons_per_meter,
    float displacement_from_equilibrium_meters
) {
    return -spring_constant_newtons_per_meter * displacement_from_equilibrium_meters;
}

/// Calculate spring constant from force and displacement
/// Formula: k = -F / x
/// @param force_newtons Applied force (N)
/// @param displacement_from_equilibrium_meters Resulting displacement (m)
/// @return Spring constant in Newtons per meter
HOST_DEVICE inline constexpr float calculate_spring_constant(
    float force_newtons,
    float displacement_from_equilibrium_meters
) {
    return (constexpr_abs(displacement_from_equilibrium_meters) > EPSILON) ? 
           (-force_newtons / displacement_from_equilibrium_meters) : 0.0f;
}

/// Calculate displacement from spring force
/// Formula: x = -F / k
/// @param force_newtons Applied force (N)
/// @param spring_constant_newtons_per_meter Spring stiffness (N/m)
/// @return Displacement in meters
HOST_DEVICE inline constexpr float calculate_spring_displacement(
    float force_newtons,
    float spring_constant_newtons_per_meter
) {
    return (spring_constant_newtons_per_meter > EPSILON) ? 
           (-force_newtons / spring_constant_newtons_per_meter) : 0.0f;
}

// Vector version of spring force
HOST_DEVICE inline Vec3 calculate_spring_restoring_force_vector(
    float spring_constant_newtons_per_meter,
    const Vec3& displacement_from_equilibrium_meters
) {
    return displacement_from_equilibrium_meters * (-spring_constant_newtons_per_meter);
}

/// Calculate spring oscillation period
/// Formula: T = 2π·sqrt(m/k)
/// @param attached_mass_kilograms Mass attached to spring (kg)
/// @param spring_constant_newtons_per_meter Spring stiffness (N/m)
/// @return Period in seconds
HOST_DEVICE inline float calculate_spring_oscillation_period(
    float attached_mass_kilograms,
    float spring_constant_newtons_per_meter
) {
    return (spring_constant_newtons_per_meter > EPSILON) ? 
           (2.0f * PI * std::sqrt(attached_mass_kilograms / spring_constant_newtons_per_meter)) : 0.0f;
}

/// Calculate spring oscillation frequency
/// Formula: f = (1/2π)·sqrt(k/m)
/// @param spring_constant_newtons_per_meter Spring stiffness (N/m)
/// @param attached_mass_kilograms Mass attached to spring (kg)
/// @return Frequency in Hertz
HOST_DEVICE inline float calculate_spring_oscillation_frequency(
    float spring_constant_newtons_per_meter,
    float attached_mass_kilograms
) {
    return (attached_mass_kilograms > EPSILON) ? 
           (1.0f / (2.0f * PI) * std::sqrt(spring_constant_newtons_per_meter / attached_mass_kilograms)) : 0.0f;
}

// ============================================================
// Gravitational Force (Universal Gravitation)
// ============================================================

/// Calculate gravitational force between two masses
/// Formula: F = G·m₁·m₂ / r²
/// @param first_mass_kilograms Mass of first object (kg)
/// @param second_mass_kilograms Mass of second object (kg)
/// @param distance_between_centers_meters Distance between mass centers (m)
/// @return Gravitational attraction force in Newtons
HOST_DEVICE inline constexpr float calculate_gravitational_force_newtons(
    float first_mass_kilograms,
    float second_mass_kilograms,
    float distance_between_centers_meters
) {
    if (distance_between_centers_meters < EPSILON) {
        return 0.0f;  // Avoid division by zero
    }
    
    return GRAVITATIONAL_CONSTANT_SI * first_mass_kilograms * second_mass_kilograms 
         / (distance_between_centers_meters * distance_between_centers_meters);
}

/// Calculate gravitational field strength
/// Formula: g = G·M / r²
/// @param source_mass_kilograms Mass creating the field (kg)
/// @param distance_from_center_meters Distance from mass center (m)
/// @return Gravitational field strength in meters per second squared
HOST_DEVICE inline constexpr float calculate_gravitational_field_strength(
    float source_mass_kilograms,
    float distance_from_center_meters
) {
    if (distance_from_center_meters < EPSILON) {
        return 0.0f;
    }
    
    return GRAVITATIONAL_CONSTANT_SI * source_mass_kilograms 
         / (distance_from_center_meters * distance_from_center_meters);
}

/// Calculate escape velocity from gravitational body
/// Formula: v_escape = sqrt(2·G·M / r)
/// @param central_body_mass_kilograms Mass of planet/star (kg)
/// @param distance_from_center_meters Distance from mass center (m)
/// @return Escape velocity in meters per second
HOST_DEVICE inline float calculate_escape_velocity_meters_per_second(
    float central_body_mass_kilograms,
    float distance_from_center_meters
) {
    return (distance_from_center_meters > EPSILON) ? 
           std::sqrt(2.0f * GRAVITATIONAL_CONSTANT_SI * central_body_mass_kilograms / distance_from_center_meters) : 0.0f;
}

/// Calculate circular orbital velocity
/// Formula: v_orbital = sqrt(G·M / r)
/// @param central_body_mass_kilograms Mass of planet/star being orbited (kg)
/// @param orbital_radius_meters Distance from mass center (m)
/// @return Orbital velocity in meters per second
HOST_DEVICE inline float calculate_circular_orbital_velocity(
    float central_body_mass_kilograms,
    float orbital_radius_meters
) {
    return (orbital_radius_meters > EPSILON) ? 
           std::sqrt(GRAVITATIONAL_CONSTANT_SI * central_body_mass_kilograms / orbital_radius_meters) : 0.0f;
}

} // namespace dynamics
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_DYNAMICS_H

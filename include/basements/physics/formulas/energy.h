#ifndef BASEMENTS_PHYSICS_FORMULAS_ENERGY_H
#define BASEMENTS_PHYSICS_FORMULAS_ENERGY_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace energy {

using namespace basements::math;

// ============================================================
// Kinetic Energy (Energy of Motion)
// ============================================================

/// Calculate kinetic energy from mass and velocity
/// Formula: E_kinetic = ½·m·v²
/// @param object_mass_kilograms Mass of the object (kg)
/// @param velocity_meters_per_second Velocity magnitude (m/s)
/// @return Kinetic energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_kinetic_energy_joules(
    float object_mass_kilograms,
    float velocity_meters_per_second
) {
    return 0.5f * object_mass_kilograms * velocity_meters_per_second * velocity_meters_per_second;
}

/// Calculate velocity from kinetic energy and mass
/// Formula: v = sqrt(2·E_k / m)
/// @param kinetic_energy_joules Kinetic energy (J)
/// @param object_mass_kilograms Mass (kg)
/// @return Velocity in meters per second
HOST_DEVICE inline float calculate_velocity_from_kinetic_energy(
    float kinetic_energy_joules,
    float object_mass_kilograms
) {
    return (object_mass_kilograms > EPSILON) ? 
           std::sqrt(2.0f * kinetic_energy_joules / object_mass_kilograms) : 0.0f;
}

/// Calculate mass from kinetic energy and velocity
/// Formula: m = 2·E_k / v²
/// @param kinetic_energy_joules Kinetic energy (J)
/// @param velocity_meters_per_second Velocity (m/s)
/// @return Mass in kilograms
HOST_DEVICE inline constexpr float calculate_mass_from_kinetic_energy(
    float kinetic_energy_joules,
    float velocity_meters_per_second
) {
    return (constexpr_abs(velocity_meters_per_second) > EPSILON) ? 
           (2.0f * kinetic_energy_joules / (velocity_meters_per_second * velocity_meters_per_second)) : 0.0f;
}

// Vector version for 3D velocity
HOST_DEVICE inline float calculate_kinetic_energy_joules_from_velocity_vector(
    float object_mass_kilograms,
    const Vec3& velocity_vector_meters_per_second
) {
    return 0.5f * object_mass_kilograms * velocity_vector_meters_per_second.length_squared();
}

// ============================================================
// Gravitational Potential Energy
// ============================================================

/// Calculate gravitational potential energy
/// Formula: E_potential = m·g·h
/// @param object_mass_kilograms Mass of the object (kg)
/// @param gravity_acceleration_meters_per_second_squared Gravity magnitude, typically 9.81 (m/s²)
/// @param height_above_reference_meters Height above reference level (m)
/// @return Gravitational potential energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_gravitational_potential_energy_joules(
    float object_mass_kilograms,
    float gravity_acceleration_meters_per_second_squared,
    float height_above_reference_meters
) {
    return object_mass_kilograms * gravity_acceleration_meters_per_second_squared * height_above_reference_meters;
}

/// Calculate height from gravitational potential energy
/// Formula: h = E_p / (m·g)
/// @param potential_energy_joules Potential energy (J)
/// @param object_mass_kilograms Mass (kg)
/// @param gravity_acceleration_meters_per_second_squared Gravity magnitude (m/s²)
/// @return Height in meters
HOST_DEVICE inline constexpr float calculate_height_from_gravitational_potential_energy(
    float potential_energy_joules,
    float object_mass_kilograms,
    float gravity_acceleration_meters_per_second_squared
) {
    float denominator = object_mass_kilograms * gravity_acceleration_meters_per_second_squared;
    return (constexpr_abs(denominator) > EPSILON) ? 
           (potential_energy_joules / denominator) : 0.0f;
}

// ============================================================
// Elastic Potential Energy (Spring Energy)
// ============================================================

/// Calculate elastic potential energy stored in spring
/// Formula: E_elastic = ½·k·x²
/// @param spring_constant_newtons_per_meter Spring stiffness (N/m)
/// @param displacement_from_equilibrium_meters Compression/extension from rest (m)
/// @return Elastic potential energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_elastic_potential_energy_joules(
    float spring_constant_newtons_per_meter,
    float displacement_from_equilibrium_meters
) {
    return 0.5f * spring_constant_newtons_per_meter 
         * displacement_from_equilibrium_meters * displacement_from_equilibrium_meters;
}

/// Calculate displacement from elastic potential energy
/// Formula: x = sqrt(2·E / k)
/// @param elastic_energy_joules Elastic potential energy (J)
/// @param spring_constant_newtons_per_meter Spring stiffness (N/m)
/// @return Displacement in meters
HOST_DEVICE inline float calculate_spring_displacement_from_energy(
    float elastic_energy_joules,
    float spring_constant_newtons_per_meter
) {
    return (spring_constant_newtons_per_meter > EPSILON) ? 
           std::sqrt(2.0f * elastic_energy_joules / spring_constant_newtons_per_meter) : 0.0f;
}

/// Calculate spring constant from elastic energy and displacement
/// Formula: k = 2·E / x²
/// @param elastic_energy_joules Elastic potential energy (J)
/// @param displacement_from_equilibrium_meters Displacement (m)
/// @return Spring constant in Newtons per meter
HOST_DEVICE inline constexpr float calculate_spring_constant_from_elastic_energy(
    float elastic_energy_joules,
    float displacement_from_equilibrium_meters
) {
    return (constexpr_abs(displacement_from_equilibrium_meters) > EPSILON) ? 
           (2.0f * elastic_energy_joules / (displacement_from_equilibrium_meters * displacement_from_equilibrium_meters)) : 0.0f;
}

// ============================================================
// Work (Energy Transfer)
// ============================================================

/// Calculate work done by constant force
/// Formula: W = F·d·cos(θ)
/// @param force_newtons Applied force magnitude (N)
/// @param displacement_meters Distance moved (m)
/// @param angle_between_force_and_displacement_radians Angle between force and displacement (rad), 0 = same direction
/// @return Work done in Joules (J)
HOST_DEVICE inline float calculate_work_joules_from_force_and_displacement(
    float force_newtons,
    float displacement_meters,
    float angle_between_force_and_displacement_radians = 0.0f
) {
    return force_newtons * displacement_meters * std::cos(angle_between_force_and_displacement_radians);
}

/// Calculate work done by force vector (dot product)
/// Formula: W = F·d
/// @param force_vector_newtons Force vector (N)
/// @param displacement_vector_meters Displacement vector (m)
/// @return Work done in Joules (J)
HOST_DEVICE inline float calculate_work_joules_from_vectors(
    const Vec3& force_vector_newtons,
    const Vec3& displacement_vector_meters
) {
    return force_vector_newtons.dot(displacement_vector_meters);
}

/// Calculate force from work and displacement
/// Formula: F = W / d
/// @param work_joules Work done (J)
/// @param displacement_meters Distance moved (m)
/// @return Force magnitude in Newtons
HOST_DEVICE inline constexpr float calculate_force_from_work_and_displacement(
    float work_joules,
    float displacement_meters
) {
    return (constexpr_abs(displacement_meters) > EPSILON) ? 
           (work_joules / displacement_meters) : 0.0f;
}

/// Calculate displacement from work and force
/// Formula: d = W / F
/// @param work_joules Work done (J)
/// @param force_newtons Applied force (N)
/// @return Displacement in meters
HOST_DEVICE inline constexpr float calculate_displacement_from_work_and_force(
    float work_joules,
    float force_newtons
) {
    return (constexpr_abs(force_newtons) > EPSILON) ? 
           (work_joules / force_newtons) : 0.0f;
}

// ============================================================
// Power (Rate of Energy Transfer)
// ============================================================

/// Calculate power from work and time
/// Formula: P = W / t
/// @param work_joules Work done (J)
/// @param time_interval_seconds Time taken (s)
/// @return Power in Watts (W)
HOST_DEVICE inline constexpr float calculate_power_watts_from_work(
    float work_joules,
    float time_interval_seconds
) {
    return (time_interval_seconds > EPSILON) ? 
           (work_joules / time_interval_seconds) : 0.0f;
}

/// Calculate work from power and time
/// Formula: W = P·t
/// @param power_watts Power (W)
/// @param time_interval_seconds Time (s)
/// @return Work done in Joules
HOST_DEVICE inline constexpr float calculate_work_from_power_and_time(
    float power_watts,
    float time_interval_seconds
) {
    return power_watts * time_interval_seconds;
}

/// Calculate time from work and power
/// Formula: t = W / P
/// @param work_joules Work done (J)
/// @param power_watts Power (W)
/// @return Time in seconds
HOST_DEVICE inline constexpr float calculate_time_from_work_and_power(
    float work_joules,
    float power_watts
) {
    return (constexpr_abs(power_watts) > EPSILON) ? 
           (work_joules / power_watts) : 0.0f;
}

/// Calculate power from force and velocity
/// Formula: P = F·v
/// @param force_newtons Applied force (N)
/// @param velocity_meters_per_second Velocity (m/s)
/// @return Power in Watts
HOST_DEVICE inline constexpr float calculate_power_watts_from_force_and_velocity(
    float force_newtons,
    float velocity_meters_per_second
) {
    return force_newtons * velocity_meters_per_second;
}

/// Calculate power from force and velocity vectors (dot product)
/// Formula: P = F·v
/// @param force_vector_newtons Force vector (N)
/// @param velocity_vector_meters_per_second Velocity vector (m/s)
/// @return Power in Watts
HOST_DEVICE inline float calculate_power_watts_from_vectors(
    const Vec3& force_vector_newtons,
    const Vec3& velocity_vector_meters_per_second
) {
    return force_vector_newtons.dot(velocity_vector_meters_per_second);
}

/// Calculate force from power and velocity
/// Formula: F = P / v
/// @param power_watts Power (W)
/// @param velocity_meters_per_second Velocity (m/s)
/// @return Force in Newtons
HOST_DEVICE inline constexpr float calculate_force_from_power_and_velocity(
    float power_watts,
    float velocity_meters_per_second
) {
    return (constexpr_abs(velocity_meters_per_second) > EPSILON) ? 
           (power_watts / velocity_meters_per_second) : 0.0f;
}

/// Calculate velocity from power and force
/// Formula: v = P / F
/// @param power_watts Power (W)
/// @param force_newtons Force (N)
/// @return Velocity in meters per second
HOST_DEVICE inline constexpr float calculate_velocity_from_power_and_force(
    float power_watts,
    float force_newtons
) {
    return (constexpr_abs(force_newtons) > EPSILON) ? 
           (power_watts / force_newtons) : 0.0f;
}

// ============================================================
// Energy Conservation
// ============================================================

/// Complete energy state of an object
struct MechanicalEnergyState {
    float kinetic_energy_joules;              // Energy of motion (J)
    float potential_energy_joules;            // Energy of position (J)
    float total_mechanical_energy_joules;     // Sum of kinetic and potential (J)
    
    MechanicalEnergyState() 
        : kinetic_energy_joules(0.0f)
        , potential_energy_joules(0.0f)
        , total_mechanical_energy_joules(0.0f) {}
    
    constexpr MechanicalEnergyState(float kinetic_joules, float potential_joules) 
        : kinetic_energy_joules(kinetic_joules)
        , potential_energy_joules(potential_joules)
        , total_mechanical_energy_joules(kinetic_joules + potential_joules) {}
    
    constexpr float get_total_energy() const { return total_mechanical_energy_joules; }
};

/// Calculate complete energy state from motion parameters
/// @param object_mass_kilograms Mass (kg)
/// @param velocity_vector_meters_per_second Velocity vector (m/s)
/// @param height_above_reference_meters Height (m)
/// @param gravity_acceleration_meters_per_second_squared Gravity magnitude (m/s²)
/// @return Complete energy state
HOST_DEVICE inline MechanicalEnergyState calculate_mechanical_energy_state(
    float object_mass_kilograms,
    const Vec3& velocity_vector_meters_per_second,
    float height_above_reference_meters,
    float gravity_acceleration_meters_per_second_squared
) {
    float kinetic = calculate_kinetic_energy_joules_from_velocity_vector(
        object_mass_kilograms, velocity_vector_meters_per_second);
    float potential = calculate_gravitational_potential_energy_joules(
        object_mass_kilograms, gravity_acceleration_meters_per_second_squared, height_above_reference_meters);
    return MechanicalEnergyState(kinetic, potential);
}

/// Calculate velocity at different height using energy conservation
/// Formula: ½·m·v₁² + m·g·h₁ = ½·m·v₂² + m·g·h₂
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param initial_height_meters Starting height (m)
/// @param final_height_meters Ending height (m)
/// @param gravity_acceleration_meters_per_second_squared Gravity magnitude (m/s²)
/// @return Velocity at final height in meters per second
HOST_DEVICE inline float calculate_velocity_at_height_using_energy_conservation(
    float initial_velocity_meters_per_second,
    float initial_height_meters,
    float final_height_meters,
    float gravity_acceleration_meters_per_second_squared
) {
    float height_change_meters = final_height_meters - initial_height_meters;
    float velocity_squared = initial_velocity_meters_per_second * initial_velocity_meters_per_second 
                           - 2.0f * gravity_acceleration_meters_per_second_squared * height_change_meters;
    return (velocity_squared >= 0.0f) ? std::sqrt(velocity_squared) : 0.0f;
}

/// Calculate height at different velocity using energy conservation
/// Formula: h₂ = h₁ + (v₁² - v₂²) / (2·g)
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param initial_height_meters Starting height (m)
/// @param final_velocity_meters_per_second Ending velocity (m/s)
/// @param gravity_acceleration_meters_per_second_squared Gravity magnitude (m/s²)
/// @return Height at final velocity in meters
HOST_DEVICE inline constexpr float calculate_height_at_velocity_using_energy_conservation(
    float initial_velocity_meters_per_second,
    float initial_height_meters,
    float final_velocity_meters_per_second,
    float gravity_acceleration_meters_per_second_squared
) {
    float velocity_squared_difference = initial_velocity_meters_per_second * initial_velocity_meters_per_second 
                                      - final_velocity_meters_per_second * final_velocity_meters_per_second;
    return initial_height_meters + velocity_squared_difference / (2.0f * gravity_acceleration_meters_per_second_squared);
}

// ============================================================
// Efficiency and Mechanical Advantage
// ============================================================

/// Calculate efficiency (ratio of useful output to total input)
/// Formula: η = W_out / W_in
/// @param useful_work_output_joules Useful work produced (J)
/// @param total_work_input_joules Total work supplied (J)
/// @return Efficiency as decimal (0.0 to 1.0, where 1.0 = 100%)
HOST_DEVICE inline constexpr float calculate_efficiency_ratio(
    float useful_work_output_joules,
    float total_work_input_joules
) {
    return (total_work_input_joules > EPSILON) ? 
           (useful_work_output_joules / total_work_input_joules) : 0.0f;
}

/// Calculate output work from efficiency
/// Formula: W_out = η·W_in
/// @param efficiency_ratio Efficiency (0.0 to 1.0)
/// @param total_work_input_joules Total work supplied (J)
/// @return Useful work output in Joules
HOST_DEVICE inline constexpr float calculate_output_work_from_efficiency(
    float efficiency_ratio,
    float total_work_input_joules
) {
    return efficiency_ratio * total_work_input_joules;
}

/// Calculate input work from efficiency
/// Formula: W_in = W_out / η
/// @param useful_work_output_joules Useful work produced (J)
/// @param efficiency_ratio Efficiency (0.0 to 1.0)
/// @return Total work input in Joules
HOST_DEVICE inline constexpr float calculate_input_work_from_efficiency(
    float useful_work_output_joules,
    float efficiency_ratio
) {
    return (efficiency_ratio > EPSILON) ? 
           (useful_work_output_joules / efficiency_ratio) : 0.0f;
}

/// Calculate power efficiency
/// Formula: η = P_out / P_in
/// @param useful_power_output_watts Useful power produced (W)
/// @param total_power_input_watts Total power supplied (W)
/// @return Efficiency as decimal (0.0 to 1.0)
HOST_DEVICE inline constexpr float calculate_power_efficiency_ratio(
    float useful_power_output_watts,
    float total_power_input_watts
) {
    return (total_power_input_watts > EPSILON) ? 
           (useful_power_output_watts / total_power_input_watts) : 0.0f;
}

/// Calculate mechanical advantage (force multiplication)
/// Formula: MA = F_out / F_in
/// @param output_force_newtons Force produced by machine (N)
/// @param input_force_newtons Force applied to machine (N)
/// @return Mechanical advantage (dimensionless)
HOST_DEVICE inline constexpr float calculate_mechanical_advantage_ratio(
    float output_force_newtons,
    float input_force_newtons
) {
    return (constexpr_abs(input_force_newtons) > EPSILON) ? 
           (output_force_newtons / input_force_newtons) : 0.0f;
}

/// Calculate output force from mechanical advantage
/// Formula: F_out = MA·F_in
/// @param mechanical_advantage_ratio Mechanical advantage
/// @param input_force_newtons Input force (N)
/// @return Output force in Newtons
HOST_DEVICE inline constexpr float calculate_output_force_from_mechanical_advantage(
    float mechanical_advantage_ratio,
    float input_force_newtons
) {
    return mechanical_advantage_ratio * input_force_newtons;
}

/// Calculate input force from mechanical advantage
/// Formula: F_in = F_out / MA
/// @param output_force_newtons Output force (N)
/// @param mechanical_advantage_ratio Mechanical advantage
/// @return Input force in Newtons
HOST_DEVICE inline constexpr float calculate_input_force_from_mechanical_advantage(
    float output_force_newtons,
    float mechanical_advantage_ratio
) {
    return (constexpr_abs(mechanical_advantage_ratio) > EPSILON) ? 
           (output_force_newtons / mechanical_advantage_ratio) : 0.0f;
}

} // namespace energy
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_ENERGY_H

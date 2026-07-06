#ifndef BASEMENTS_PHYSICS_FORMULAS_ROTATION_H
#define BASEMENTS_PHYSICS_FORMULAS_ROTATION_H

#include "basements/core/math/vec3.h"
#include "basements/core/math/matrix3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace rotation {

using namespace basements::math;

// ============================================================
// Angular Kinematics (Rotational Motion)
// ============================================================

/// Calculate angular displacement from constant angular velocity
/// Formula: θ = ω·t
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Angular displacement in radians
HOST_DEVICE inline constexpr float calculate_angular_displacement_radians(
    float angular_velocity_radians_per_second,
    float elapsed_time_seconds
) {
    return angular_velocity_radians_per_second * elapsed_time_seconds;
}

/// Calculate angular velocity from angular displacement
/// Formula: ω = θ / t
/// @param angular_displacement_radians Angular displacement (rad)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Angular velocity in radians per second
HOST_DEVICE inline constexpr float calculate_angular_velocity_from_displacement(
    float angular_displacement_radians,
    float elapsed_time_seconds
) {
    return (elapsed_time_seconds > EPSILON) ? 
           (angular_displacement_radians / elapsed_time_seconds) : 0.0f;
}

/// Calculate angular velocity from angular acceleration
/// Formula: ω = ω₀ + α·t
/// @param initial_angular_velocity_radians_per_second Starting angular velocity (rad/s)
/// @param angular_acceleration_radians_per_second_squared Angular acceleration (rad/s²)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final angular velocity in radians per second
HOST_DEVICE inline constexpr float calculate_angular_velocity_from_angular_acceleration(
    float initial_angular_velocity_radians_per_second,
    float angular_acceleration_radians_per_second_squared,
    float elapsed_time_seconds
) {
    return initial_angular_velocity_radians_per_second 
         + angular_acceleration_radians_per_second_squared * elapsed_time_seconds;
}

/// Calculate angular acceleration from angular velocity change
/// Formula: α = Δω / t
/// @param final_angular_velocity_radians_per_second Ending angular velocity (rad/s)
/// @param initial_angular_velocity_radians_per_second Starting angular velocity (rad/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Angular acceleration in radians per second squared
HOST_DEVICE inline constexpr float calculate_angular_acceleration_from_velocity_change(
    float final_angular_velocity_radians_per_second,
    float initial_angular_velocity_radians_per_second,
    float elapsed_time_seconds
) {
    return (elapsed_time_seconds > EPSILON) ? 
           ((final_angular_velocity_radians_per_second - initial_angular_velocity_radians_per_second) / elapsed_time_seconds) : 0.0f;
}

/// Calculate angular displacement with constant angular acceleration
/// Formula: θ = θ₀ + ω₀·t + ½·α·t²
/// @param initial_angular_displacement_radians Starting angle (rad)
/// @param initial_angular_velocity_radians_per_second Starting angular velocity (rad/s)
/// @param angular_acceleration_radians_per_second_squared Angular acceleration (rad/s²)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final angular displacement in radians
HOST_DEVICE inline constexpr float calculate_angular_displacement_from_angular_acceleration(
    float initial_angular_displacement_radians,
    float initial_angular_velocity_radians_per_second,
    float angular_acceleration_radians_per_second_squared,
    float elapsed_time_seconds
) {
    return initial_angular_displacement_radians 
         + initial_angular_velocity_radians_per_second * elapsed_time_seconds 
         + 0.5f * angular_acceleration_radians_per_second_squared * elapsed_time_seconds * elapsed_time_seconds;
}

// ============================================================
// Relationship Between Linear and Angular Motion
// ============================================================

/// Calculate tangential velocity from angular velocity
/// Formula: v = ω·r
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @param radius_meters Distance from rotation axis (m)
/// @return Tangential velocity in meters per second
HOST_DEVICE inline constexpr float calculate_tangential_velocity_meters_per_second(
    float angular_velocity_radians_per_second,
    float radius_meters
) {
    return angular_velocity_radians_per_second * radius_meters;
}

/// Calculate angular velocity from tangential velocity
/// Formula: ω = v / r
/// @param tangential_velocity_meters_per_second Tangential velocity (m/s)
/// @param radius_meters Distance from rotation axis (m)
/// @return Angular velocity in radians per second
HOST_DEVICE inline constexpr float calculate_angular_velocity_from_tangential_velocity(
    float tangential_velocity_meters_per_second,
    float radius_meters
) {
    return (radius_meters > EPSILON) ? 
           (tangential_velocity_meters_per_second / radius_meters) : 0.0f;
}

/// Calculate tangential acceleration from angular acceleration
/// Formula: a_tangential = α·r
/// @param angular_acceleration_radians_per_second_squared Angular acceleration (rad/s²)
/// @param radius_meters Distance from rotation axis (m)
/// @return Tangential acceleration in meters per second squared
HOST_DEVICE inline constexpr float calculate_tangential_acceleration_meters_per_second_squared(
    float angular_acceleration_radians_per_second_squared,
    float radius_meters
) {
    return angular_acceleration_radians_per_second_squared * radius_meters;
}

/// Calculate centripetal acceleration from tangential velocity
/// Formula: a_centripetal = v² / r
/// @param tangential_velocity_meters_per_second Tangential velocity (m/s)
/// @param radius_meters Distance from rotation axis (m)
/// @return Centripetal acceleration in meters per second squared
HOST_DEVICE inline constexpr float calculate_centripetal_acceleration_from_velocity(
    float tangential_velocity_meters_per_second,
    float radius_meters
) {
    return (radius_meters > EPSILON) ? 
           (tangential_velocity_meters_per_second * tangential_velocity_meters_per_second / radius_meters) : 0.0f;
}

/// Calculate centripetal acceleration from angular velocity
/// Formula: a_centripetal = ω²·r
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @param radius_meters Distance from rotation axis (m)
/// @return Centripetal acceleration in meters per second squared
HOST_DEVICE inline constexpr float calculate_centripetal_acceleration_from_angular_velocity(
    float angular_velocity_radians_per_second,
    float radius_meters
) {
    return angular_velocity_radians_per_second * angular_velocity_radians_per_second * radius_meters;
}

/// Calculate centripetal force from tangential velocity
/// Formula: F_centripetal = m·v² / r
/// @param object_mass_kilograms Mass of rotating object (kg)
/// @param tangential_velocity_meters_per_second Tangential velocity (m/s)
/// @param radius_meters Distance from rotation axis (m)
/// @return Centripetal force in Newtons
HOST_DEVICE inline constexpr float calculate_centripetal_force_newtons_from_velocity(
    float object_mass_kilograms,
    float tangential_velocity_meters_per_second,
    float radius_meters
) {
    return (radius_meters > EPSILON) ? 
           (object_mass_kilograms * tangential_velocity_meters_per_second * tangential_velocity_meters_per_second / radius_meters) : 0.0f;
}

/// Calculate centripetal force from angular velocity
/// Formula: F_centripetal = m·ω²·r
/// @param object_mass_kilograms Mass of rotating object (kg)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @param radius_meters Distance from rotation axis (m)
/// @return Centripetal force in Newtons
HOST_DEVICE inline constexpr float calculate_centripetal_force_newtons_from_angular_velocity(
    float object_mass_kilograms,
    float angular_velocity_radians_per_second,
    float radius_meters
) {
    return object_mass_kilograms * angular_velocity_radians_per_second * angular_velocity_radians_per_second * radius_meters;
}

// ============================================================
// Torque (Rotational Force)
// ============================================================

/// Calculate torque magnitude from lever arm and force
/// Formula: τ = r·F·sin(θ)
/// @param lever_arm_length_meters Distance from rotation axis to force application point (m)
/// @param force_newtons Applied force magnitude (N)
/// @param angle_between_lever_and_force_radians Angle between lever arm and force (rad), π/2 = perpendicular
/// @return Torque magnitude in Newton-meters (N·m)
HOST_DEVICE inline float calculate_torque_newton_meters(
    float lever_arm_length_meters,
    float force_newtons,
    float angle_between_lever_and_force_radians = HALF_PI
) {
    return lever_arm_length_meters * force_newtons * std::sin(angle_between_lever_and_force_radians);
}

/// Calculate torque vector from lever arm and force vectors (cross product)
/// Formula: τ = r × F
/// @param lever_arm_vector_meters Position vector from rotation axis to force point (m)
/// @param force_vector_newtons Force vector (N)
/// @return Torque vector in Newton-meters (N·m)
HOST_DEVICE inline Vec3 calculate_torque_vector_newton_meters(
    const Vec3& lever_arm_vector_meters,
    const Vec3& force_vector_newtons
) {
    return lever_arm_vector_meters.cross(force_vector_newtons);
}

/// Calculate torque from moment of inertia and angular acceleration
/// Formula: τ = I·α
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @param angular_acceleration_radians_per_second_squared Angular acceleration (rad/s²)
/// @return Torque in Newton-meters
HOST_DEVICE inline constexpr float calculate_torque_from_inertia_and_angular_acceleration(
    float moment_of_inertia_kg_meter_squared,
    float angular_acceleration_radians_per_second_squared
) {
    return moment_of_inertia_kg_meter_squared * angular_acceleration_radians_per_second_squared;
}

/// Calculate angular acceleration from torque and moment of inertia
/// Formula: α = τ / I
/// @param torque_newton_meters Applied torque (N·m)
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @return Angular acceleration in radians per second squared
HOST_DEVICE inline constexpr float calculate_angular_acceleration_from_torque(
    float torque_newton_meters,
    float moment_of_inertia_kg_meter_squared
) {
    return (moment_of_inertia_kg_meter_squared > EPSILON) ? 
           (torque_newton_meters / moment_of_inertia_kg_meter_squared) : 0.0f;
}

/// Calculate moment of inertia from torque and angular acceleration
/// Formula: I = τ / α
/// @param torque_newton_meters Applied torque (N·m)
/// @param angular_acceleration_radians_per_second_squared Resulting angular acceleration (rad/s²)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_from_torque(
    float torque_newton_meters,
    float angular_acceleration_radians_per_second_squared
) {
    return (constexpr_abs(angular_acceleration_radians_per_second_squared) > EPSILON) ? 
           (torque_newton_meters / angular_acceleration_radians_per_second_squared) : 0.0f;
}

// Vector version for 3D rotation with inertia tensor
HOST_DEVICE inline Vec3 calculate_angular_acceleration_vector_from_torque(
    const Vec3& torque_vector_newton_meters,
    const Matrix3& inertia_tensor_kg_meter_squared
) {
    return inertia_tensor_kg_meter_squared.inversed() * torque_vector_newton_meters;
}

// ============================================================
// Angular Momentum (Rotational Momentum)
// ============================================================

/// Calculate angular momentum from moment of inertia and angular velocity
/// Formula: L = I·ω
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Angular momentum in kilogram-meter squared per second (kg·m²/s)
HOST_DEVICE inline constexpr float calculate_angular_momentum_kg_meter_squared_per_second(
    float moment_of_inertia_kg_meter_squared,
    float angular_velocity_radians_per_second
) {
    return moment_of_inertia_kg_meter_squared * angular_velocity_radians_per_second;
}

/// Calculate angular velocity from angular momentum
/// Formula: ω = L / I
/// @param angular_momentum_kg_meter_squared_per_second Angular momentum (kg·m²/s)
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @return Angular velocity in radians per second
HOST_DEVICE inline constexpr float calculate_angular_velocity_from_angular_momentum(
    float angular_momentum_kg_meter_squared_per_second,
    float moment_of_inertia_kg_meter_squared
) {
    return (moment_of_inertia_kg_meter_squared > EPSILON) ? 
           (angular_momentum_kg_meter_squared_per_second / moment_of_inertia_kg_meter_squared) : 0.0f;
}

/// Calculate moment of inertia from angular momentum
/// Formula: I = L / ω
/// @param angular_momentum_kg_meter_squared_per_second Angular momentum (kg·m²/s)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_from_angular_momentum(
    float angular_momentum_kg_meter_squared_per_second,
    float angular_velocity_radians_per_second
) {
    return (constexpr_abs(angular_velocity_radians_per_second) > EPSILON) ? 
           (angular_momentum_kg_meter_squared_per_second / angular_velocity_radians_per_second) : 0.0f;
}

// Vector version for 3D rotation
HOST_DEVICE inline Vec3 calculate_angular_momentum_vector_kg_meter_squared_per_second(
    const Matrix3& inertia_tensor_kg_meter_squared,
    const Vec3& angular_velocity_vector_radians_per_second
) {
    return inertia_tensor_kg_meter_squared * angular_velocity_vector_radians_per_second;
}

// ============================================================
// Rotational Kinetic Energy
// ============================================================

/// Calculate rotational kinetic energy
/// Formula: E_rotational = ½·I·ω²
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Rotational kinetic energy in Joules (J)
HOST_DEVICE inline constexpr float calculate_rotational_kinetic_energy_joules(
    float moment_of_inertia_kg_meter_squared,
    float angular_velocity_radians_per_second
) {
    return 0.5f * moment_of_inertia_kg_meter_squared 
         * angular_velocity_radians_per_second * angular_velocity_radians_per_second;
}

/// Calculate angular velocity from rotational kinetic energy
/// Formula: ω = sqrt(2·E / I)
/// @param rotational_energy_joules Rotational kinetic energy (J)
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @return Angular velocity in radians per second
HOST_DEVICE inline float calculate_angular_velocity_from_rotational_energy(
    float rotational_energy_joules,
    float moment_of_inertia_kg_meter_squared
) {
    return (moment_of_inertia_kg_meter_squared > EPSILON) ? 
           std::sqrt(2.0f * rotational_energy_joules / moment_of_inertia_kg_meter_squared) : 0.0f;
}

/// Calculate moment of inertia from rotational kinetic energy
/// Formula: I = 2·E / ω²
/// @param rotational_energy_joules Rotational kinetic energy (J)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_from_rotational_energy(
    float rotational_energy_joules,
    float angular_velocity_radians_per_second
) {
    return (constexpr_abs(angular_velocity_radians_per_second) > EPSILON) ? 
           (2.0f * rotational_energy_joules / (angular_velocity_radians_per_second * angular_velocity_radians_per_second)) : 0.0f;
}

// ============================================================
// Moment of Inertia for Common Shapes
// ============================================================

/// Calculate moment of inertia for point mass
/// Formula: I = m·r²
/// @param object_mass_kilograms Mass (kg)
/// @param distance_from_axis_meters Distance from rotation axis (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_point_mass(
    float object_mass_kilograms,
    float distance_from_axis_meters
) {
    return object_mass_kilograms * distance_from_axis_meters * distance_from_axis_meters;
}

/// Calculate moment of inertia for solid sphere about center
/// Formula: I = (2/5)·m·r²
/// @param sphere_mass_kilograms Mass of sphere (kg)
/// @param sphere_radius_meters Radius of sphere (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_solid_sphere(
    float sphere_mass_kilograms,
    float sphere_radius_meters
) {
    return 0.4f * sphere_mass_kilograms * sphere_radius_meters * sphere_radius_meters;
}

/// Calculate moment of inertia for hollow sphere about center
/// Formula: I = (2/3)·m·r²
/// @param shell_mass_kilograms Mass of spherical shell (kg)
/// @param shell_radius_meters Radius of shell (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_hollow_sphere(
    float shell_mass_kilograms,
    float shell_radius_meters
) {
    return (2.0f / 3.0f) * shell_mass_kilograms * shell_radius_meters * shell_radius_meters;
}

/// Calculate moment of inertia for solid cylinder about central axis
/// Formula: I = (1/2)·m·r²
/// @param cylinder_mass_kilograms Mass of cylinder (kg)
/// @param cylinder_radius_meters Radius of cylinder (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_solid_cylinder(
    float cylinder_mass_kilograms,
    float cylinder_radius_meters
) {
    return 0.5f * cylinder_mass_kilograms * cylinder_radius_meters * cylinder_radius_meters;
}

/// Calculate moment of inertia for hollow cylinder about central axis
/// Formula: I = m·r²
/// @param cylinder_mass_kilograms Mass of cylindrical shell (kg)
/// @param cylinder_radius_meters Radius of cylinder (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_hollow_cylinder(
    float cylinder_mass_kilograms,
    float cylinder_radius_meters
) {
    return cylinder_mass_kilograms * cylinder_radius_meters * cylinder_radius_meters;
}

/// Calculate moment of inertia for rod about center
/// Formula: I = (1/12)·m·L²
/// @param rod_mass_kilograms Mass of rod (kg)
/// @param rod_length_meters Length of rod (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_rod_about_center(
    float rod_mass_kilograms,
    float rod_length_meters
) {
    return (1.0f / 12.0f) * rod_mass_kilograms * rod_length_meters * rod_length_meters;
}

/// Calculate moment of inertia for rod about end
/// Formula: I = (1/3)·m·L²
/// @param rod_mass_kilograms Mass of rod (kg)
/// @param rod_length_meters Length of rod (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_rod_about_end(
    float rod_mass_kilograms,
    float rod_length_meters
) {
    return (1.0f / 3.0f) * rod_mass_kilograms * rod_length_meters * rod_length_meters;
}

/// Calculate moment of inertia for rectangular plate about center
/// Formula: I = (1/12)·m·(a² + b²)
/// @param plate_mass_kilograms Mass of plate (kg)
/// @param plate_width_meters Width dimension (m)
/// @param plate_height_meters Height dimension (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_rectangular_plate(
    float plate_mass_kilograms,
    float plate_width_meters,
    float plate_height_meters
) {
    return (1.0f / 12.0f) * plate_mass_kilograms 
         * (plate_width_meters * plate_width_meters + plate_height_meters * plate_height_meters);
}

/// Calculate moment of inertia for disk about center
/// Formula: I = (1/2)·m·r²
/// @param disk_mass_kilograms Mass of disk (kg)
/// @param disk_radius_meters Radius of disk (m)
/// @return Moment of inertia in kilogram-meter squared
HOST_DEVICE inline constexpr float calculate_moment_of_inertia_disk(
    float disk_mass_kilograms,
    float disk_radius_meters
) {
    return 0.5f * disk_mass_kilograms * disk_radius_meters * disk_radius_meters;
}

// ============================================================
// Period and Frequency
// ============================================================

/// Calculate period from angular velocity
/// Formula: T = 2π / ω
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Period in seconds (time for one complete rotation)
HOST_DEVICE inline constexpr float calculate_period_seconds_from_angular_velocity(
    float angular_velocity_radians_per_second
) {
    return (angular_velocity_radians_per_second > EPSILON) ? 
           (TWO_PI / angular_velocity_radians_per_second) : 0.0f;
}

/// Calculate angular velocity from period
/// Formula: ω = 2π / T
/// @param period_seconds Time for one complete rotation (s)
/// @return Angular velocity in radians per second
HOST_DEVICE inline constexpr float calculate_angular_velocity_from_period(
    float period_seconds
) {
    return (period_seconds > EPSILON) ? (TWO_PI / period_seconds) : 0.0f;
}

/// Calculate frequency from angular velocity
/// Formula: f = ω / (2π)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Frequency in Hertz (rotations per second)
HOST_DEVICE inline constexpr float calculate_frequency_hertz_from_angular_velocity(
    float angular_velocity_radians_per_second
) {
    return angular_velocity_radians_per_second / TWO_PI;
}

/// Calculate angular velocity from frequency
/// Formula: ω = 2π·f
/// @param frequency_hertz Rotations per second (Hz)
/// @return Angular velocity in radians per second
HOST_DEVICE inline constexpr float calculate_angular_velocity_from_frequency(
    float frequency_hertz
) {
    return TWO_PI * frequency_hertz;
}

/// Calculate period from frequency
/// Formula: T = 1 / f
/// @param frequency_hertz Frequency (Hz)
/// @return Period in seconds
HOST_DEVICE inline constexpr float calculate_period_from_frequency(
    float frequency_hertz
) {
    return (frequency_hertz > EPSILON) ? (1.0f / frequency_hertz) : 0.0f;
}

/// Calculate frequency from period
/// Formula: f = 1 / T
/// @param period_seconds Period (s)
/// @return Frequency in Hertz
HOST_DEVICE inline constexpr float calculate_frequency_from_period(
    float period_seconds
) {
    return (period_seconds > EPSILON) ? (1.0f / period_seconds) : 0.0f;
}

// ============================================================
// Rolling Motion (Without Slipping)
// ============================================================

/// Calculate linear velocity of rolling object
/// Formula: v = ω·r (no slipping condition)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @param object_radius_meters Radius of rolling object (m)
/// @return Linear velocity in meters per second
HOST_DEVICE inline constexpr float calculate_rolling_velocity_meters_per_second(
    float angular_velocity_radians_per_second,
    float object_radius_meters
) {
    return angular_velocity_radians_per_second * object_radius_meters;
}

/// Calculate total kinetic energy of rolling object
/// Formula: E_total = ½·m·v² + ½·I·ω²
/// @param object_mass_kilograms Mass (kg)
/// @param linear_velocity_meters_per_second Linear velocity of center of mass (m/s)
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Total kinetic energy in Joules
HOST_DEVICE inline constexpr float calculate_total_rolling_kinetic_energy_joules(
    float object_mass_kilograms,
    float linear_velocity_meters_per_second,
    float moment_of_inertia_kg_meter_squared,
    float angular_velocity_radians_per_second
) {
    return 0.5f * object_mass_kilograms * linear_velocity_meters_per_second * linear_velocity_meters_per_second 
         + 0.5f * moment_of_inertia_kg_meter_squared * angular_velocity_radians_per_second * angular_velocity_radians_per_second;
}

/// Calculate acceleration of object rolling down incline
/// Formula: a = g·sin(θ) / (1 + I/(m·r²))
/// @param gravity_acceleration_meters_per_second_squared Gravity magnitude, typically 9.81 (m/s²)
/// @param incline_angle_radians Angle of incline from horizontal (rad)
/// @param moment_of_inertia_kg_meter_squared Moment of inertia (kg·m²)
/// @param object_mass_kilograms Mass (kg)
/// @param object_radius_meters Radius (m)
/// @return Linear acceleration down incline in meters per second squared
HOST_DEVICE inline float calculate_rolling_acceleration_down_incline(
    float gravity_acceleration_meters_per_second_squared,
    float incline_angle_radians,
    float moment_of_inertia_kg_meter_squared,
    float object_mass_kilograms,
    float object_radius_meters
) {
    float denominator = 1.0f + moment_of_inertia_kg_meter_squared 
                      / (object_mass_kilograms * object_radius_meters * object_radius_meters);
    return (denominator > EPSILON) ? 
           (gravity_acceleration_meters_per_second_squared * std::sin(incline_angle_radians) / denominator) : 0.0f;
}

} // namespace rotation
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_ROTATION_H

#ifndef BASEMENTS_PHYSICS_FORMULAS_KINEMATICS_H
#define BASEMENTS_PHYSICS_FORMULAS_KINEMATICS_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace kinematics {

using namespace basements::math;

// ============================================================
// 1D Kinematics (Linear Motion)
// ============================================================

/// Calculate position from constant velocity motion
/// Formula: x = x₀ + v·t
/// @param initial_position_meters Starting position (m)
/// @param velocity_meters_per_second Constant velocity (m/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final position in meters
HOST_DEVICE inline constexpr float calculate_position_from_constant_velocity(
    float initial_position_meters,
    float velocity_meters_per_second,
    float elapsed_time_seconds
) {
    return initial_position_meters 
         + velocity_meters_per_second * elapsed_time_seconds;
}

/// Calculate position from constant acceleration motion
/// Formula: x = x₀ + v₀·t + ½·a·t²
/// @param initial_position_meters Starting position (m)
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param acceleration_meters_per_second_squared Constant acceleration (m/s²)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final position in meters
HOST_DEVICE inline constexpr float calculate_position_from_constant_acceleration(
    float initial_position_meters,
    float initial_velocity_meters_per_second,
    float acceleration_meters_per_second_squared,
    float elapsed_time_seconds
) {
    return initial_position_meters 
         + initial_velocity_meters_per_second * elapsed_time_seconds 
         + 0.5f * acceleration_meters_per_second_squared * elapsed_time_seconds * elapsed_time_seconds;
}

/// Calculate velocity from constant acceleration
/// Formula: v = v₀ + a·t
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param acceleration_meters_per_second_squared Constant acceleration (m/s²)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final velocity in meters per second
HOST_DEVICE inline constexpr float calculate_velocity_from_constant_acceleration(
    float initial_velocity_meters_per_second,
    float acceleration_meters_per_second_squared,
    float elapsed_time_seconds
) {
    return initial_velocity_meters_per_second 
         + acceleration_meters_per_second_squared * elapsed_time_seconds;
}

/// Calculate average velocity from displacement
/// Formula: v_avg = Δx / t
/// @param final_position_meters Ending position (m)
/// @param initial_position_meters Starting position (m)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Average velocity in meters per second
HOST_DEVICE inline constexpr float calculate_average_velocity_from_displacement(
    float final_position_meters,
    float initial_position_meters,
    float elapsed_time_seconds
) {
    return (elapsed_time_seconds > EPSILON) ? 
           ((final_position_meters - initial_position_meters) / elapsed_time_seconds) : 0.0f;
}

/// Calculate time required for velocity change
/// Formula: t = Δv / a
/// @param final_velocity_meters_per_second Ending velocity (m/s)
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param acceleration_meters_per_second_squared Constant acceleration (m/s²)
/// @return Time required in seconds
HOST_DEVICE inline constexpr float calculate_time_for_velocity_change(
    float final_velocity_meters_per_second,
    float initial_velocity_meters_per_second,
    float acceleration_meters_per_second_squared
) {
    return (constexpr_abs(acceleration_meters_per_second_squared) > EPSILON) ? 
           ((final_velocity_meters_per_second - initial_velocity_meters_per_second) / acceleration_meters_per_second_squared) : 0.0f;
}

/// Calculate final velocity from displacement and acceleration (time-independent)
/// Formula: v² = v₀² + 2·a·Δx
/// @param displacement_meters Distance traveled (m)
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param acceleration_meters_per_second_squared Constant acceleration (m/s²)
/// @return Final velocity in meters per second
HOST_DEVICE inline float calculate_velocity_from_displacement_and_acceleration(
    float displacement_meters,
    float initial_velocity_meters_per_second,
    float acceleration_meters_per_second_squared
) {
    float velocity_squared = initial_velocity_meters_per_second * initial_velocity_meters_per_second 
                           + 2.0f * acceleration_meters_per_second_squared * displacement_meters;
    return (velocity_squared >= 0.0f) ? std::sqrt(velocity_squared) : 0.0f;
}

/// Calculate acceleration from velocity change over time
/// Formula: a = Δv / t
/// @param final_velocity_meters_per_second Ending velocity (m/s)
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Acceleration in meters per second squared
HOST_DEVICE inline float calculate_acceleration_from_velocity_change(
    float final_velocity_meters_per_second,
    float initial_velocity_meters_per_second,
    float elapsed_time_seconds
) {
    return (elapsed_time_seconds > EPSILON) ? 
           ((final_velocity_meters_per_second - initial_velocity_meters_per_second) / elapsed_time_seconds) : 0.0f;
}

/// Calculate acceleration from displacement and time
/// Formula: a = 2·(x - x₀ - v₀·t) / t²
/// @param final_position_meters Ending position (m)
/// @param initial_position_meters Starting position (m)
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Acceleration in meters per second squared
HOST_DEVICE inline constexpr float calculate_acceleration_from_displacement(
    float final_position_meters,
    float initial_position_meters,
    float initial_velocity_meters_per_second,
    float elapsed_time_seconds
) {
    return (elapsed_time_seconds > EPSILON) ? 
           (2.0f * (final_position_meters - initial_position_meters - initial_velocity_meters_per_second * elapsed_time_seconds) 
            / (elapsed_time_seconds * elapsed_time_seconds)) : 0.0f;
}

/// Calculate average velocity from initial and final velocities
/// Formula: v_avg = (v₀ + v) / 2
/// @param initial_velocity_meters_per_second Starting velocity (m/s)
/// @param final_velocity_meters_per_second Ending velocity (m/s)
/// @return Average velocity in meters per second
HOST_DEVICE inline constexpr float calculate_average_velocity(
    float initial_velocity_meters_per_second,
    float final_velocity_meters_per_second
) {
    return (initial_velocity_meters_per_second + final_velocity_meters_per_second) * 0.5f;
}

/// Calculate displacement from average velocity
/// Formula: Δx = v_avg · t
/// @param average_velocity_meters_per_second Average velocity (m/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Displacement in meters
HOST_DEVICE inline constexpr float calculate_displacement_from_average_velocity(
    float average_velocity_meters_per_second,
    float elapsed_time_seconds
) {
    return average_velocity_meters_per_second * elapsed_time_seconds;
}

// ============================================================
// 3D Kinematics (Vector Motion)
// ============================================================

/// Calculate 3D position from constant velocity
/// Formula: r = r₀ + v·t
/// @param initial_position_meters Starting position vector (m)
/// @param velocity_meters_per_second Constant velocity vector (m/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final position vector in meters
HOST_DEVICE inline Vec3 calculate_position_vector_from_constant_velocity(
    const Vec3& initial_position_meters,
    const Vec3& velocity_meters_per_second,
    float elapsed_time_seconds
) {
    return initial_position_meters + velocity_meters_per_second * elapsed_time_seconds;
}

/// Calculate 3D position from constant acceleration
/// Formula: r = r₀ + v₀·t + ½·a·t²
/// @param initial_position_meters Starting position vector (m)
/// @param initial_velocity_meters_per_second Starting velocity vector (m/s)
/// @param acceleration_meters_per_second_squared Constant acceleration vector (m/s²)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final position vector in meters
HOST_DEVICE inline Vec3 calculate_position_vector_from_constant_acceleration(
    const Vec3& initial_position_meters,
    const Vec3& initial_velocity_meters_per_second,
    const Vec3& acceleration_meters_per_second_squared,
    float elapsed_time_seconds
) {
    return initial_position_meters 
         + initial_velocity_meters_per_second * elapsed_time_seconds 
         + acceleration_meters_per_second_squared * (0.5f * elapsed_time_seconds * elapsed_time_seconds);
}

/// Calculate 3D velocity from constant acceleration
/// Formula: v = v₀ + a·t
/// @param initial_velocity_meters_per_second Starting velocity vector (m/s)
/// @param acceleration_meters_per_second_squared Constant acceleration vector (m/s²)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Final velocity vector in meters per second
HOST_DEVICE inline Vec3 calculate_velocity_vector_from_constant_acceleration(
    const Vec3& initial_velocity_meters_per_second,
    const Vec3& acceleration_meters_per_second_squared,
    float elapsed_time_seconds
) {
    return initial_velocity_meters_per_second + acceleration_meters_per_second_squared * elapsed_time_seconds;
}

/// Calculate 3D velocity from displacement
/// Formula: v = Δr / t
/// @param displacement_vector_meters Displacement vector (m)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Velocity vector in meters per second
HOST_DEVICE inline Vec3 calculate_velocity_vector_from_displacement(
    const Vec3& displacement_vector_meters,
    float elapsed_time_seconds
) {
    return (elapsed_time_seconds > EPSILON) ? 
           (displacement_vector_meters / elapsed_time_seconds) : Vec3::zero();
}

/// Calculate 3D acceleration from velocity change
/// Formula: a = Δv / t
/// @param final_velocity_meters_per_second Ending velocity vector (m/s)
/// @param initial_velocity_meters_per_second Starting velocity vector (m/s)
/// @param elapsed_time_seconds Time elapsed (s)
/// @return Acceleration vector in meters per second squared
HOST_DEVICE inline Vec3 calculate_acceleration_vector_from_velocity_change(
    const Vec3& final_velocity_meters_per_second,
    const Vec3& initial_velocity_meters_per_second,
    float elapsed_time_seconds
) {
    return (elapsed_time_seconds > EPSILON) ? 
           ((final_velocity_meters_per_second - initial_velocity_meters_per_second) / elapsed_time_seconds) : Vec3::zero();
}

// ============================================================
// Projectile Motion
// ============================================================

/// Result of projectile motion calculation
struct ProjectileMotionResult {
    Vec3 position_meters;                    // Current position (m)
    Vec3 velocity_meters_per_second;         // Current velocity (m/s)
    float total_flight_time_seconds;         // Total time in air (s)
    float maximum_height_meters;             // Peak height reached (m)
    float horizontal_range_meters;           // Total horizontal distance (m)
    bool is_valid_trajectory;                // Whether calculation succeeded
};

/// Calculate projectile motion state at specific time
/// @param initial_position_meters Starting position (m)
/// @param initial_velocity_meters_per_second Launch velocity (m/s)
/// @param gravity_acceleration_meters_per_second_squared Gravity vector, typically (0, -9.81, 0) (m/s²)
/// @param elapsed_time_seconds Time since launch (s)
/// @return Complete projectile state
HOST_DEVICE inline ProjectileMotionResult calculate_projectile_motion_at_time(
    const Vec3& initial_position_meters,
    const Vec3& initial_velocity_meters_per_second,
    const Vec3& gravity_acceleration_meters_per_second_squared,
    float elapsed_time_seconds
) {
    ProjectileMotionResult result;
    
    // Position: r = r₀ + v₀·t + ½·g·t²
    result.position_meters = initial_position_meters 
                           + initial_velocity_meters_per_second * elapsed_time_seconds 
                           + gravity_acceleration_meters_per_second_squared * (0.5f * elapsed_time_seconds * elapsed_time_seconds);
    
    // Velocity: v = v₀ + g·t
    result.velocity_meters_per_second = initial_velocity_meters_per_second 
                                      + gravity_acceleration_meters_per_second_squared * elapsed_time_seconds;
    
    // Total flight time (when returns to initial height)
    // Solve: y₀ + vy·t + ½·gy·t² = y₀  =>  t = -2·vy/gy
    if (std::abs(gravity_acceleration_meters_per_second_squared.y) > EPSILON) {
        result.total_flight_time_seconds = -2.0f * initial_velocity_meters_per_second.y 
                                          / gravity_acceleration_meters_per_second_squared.y;
    } else {
        result.total_flight_time_seconds = 0.0f;
    }
    
    // Maximum height (when vertical velocity = 0)
    // vy² = vy₀² + 2·gy·Δy  =>  Δy = -vy₀²/(2·gy)
    if (std::abs(gravity_acceleration_meters_per_second_squared.y) > EPSILON) {
        float height_change_meters = -(initial_velocity_meters_per_second.y * initial_velocity_meters_per_second.y) 
                                    / (2.0f * gravity_acceleration_meters_per_second_squared.y);
        result.maximum_height_meters = initial_position_meters.y + height_change_meters;
    } else {
        result.maximum_height_meters = initial_position_meters.y;
    }
    
    // Horizontal range at landing
    result.horizontal_range_meters = initial_velocity_meters_per_second.x * result.total_flight_time_seconds;
    
    result.is_valid_trajectory = (result.total_flight_time_seconds >= 0.0f);
    
    return result;
}

/// Calculate launch angle needed for desired range (flat ground)
/// Formula: θ = ½·arcsin(R·g/v₀²)
/// @param desired_range_meters Target horizontal distance (m)
/// @param initial_speed_meters_per_second Launch speed (m/s)
/// @param gravity_magnitude_meters_per_second_squared Magnitude of gravity, typically 9.81 (m/s²)
/// @return Launch angle in radians (0 if impossible)
HOST_DEVICE inline float calculate_launch_angle_for_range(
    float desired_range_meters,
    float initial_speed_meters_per_second,
    float gravity_magnitude_meters_per_second_squared
) {
    if (initial_speed_meters_per_second < EPSILON || gravity_magnitude_meters_per_second_squared < EPSILON) {
        return 0.0f;
    }
    
    float sin_twice_angle = (desired_range_meters * gravity_magnitude_meters_per_second_squared) 
                          / (initial_speed_meters_per_second * initial_speed_meters_per_second);
    
    // Check if range is achievable
    if (sin_twice_angle > 1.0f) {
        return 0.0f;  // Range not achievable with given speed
    }
    
    return 0.5f * std::asin(sin_twice_angle);
}

/// Calculate maximum possible range for given launch speed
/// Formula: R_max = v₀² / g  (achieved at 45° angle)
/// @param initial_speed_meters_per_second Launch speed (m/s)
/// @param gravity_magnitude_meters_per_second_squared Magnitude of gravity, typically 9.81 (m/s²)
/// @return Maximum range in meters
HOST_DEVICE inline constexpr float calculate_maximum_projectile_range(
    float initial_speed_meters_per_second,
    float gravity_magnitude_meters_per_second_squared
) {
    return (gravity_magnitude_meters_per_second_squared > EPSILON) ? 
           (initial_speed_meters_per_second * initial_speed_meters_per_second / gravity_magnitude_meters_per_second_squared) : 0.0f;
}

// ============================================================
// Circular Motion
// ============================================================

/// Calculate tangential velocity from angular velocity
/// Formula: v = ω·r
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @param radius_meters Distance from rotation axis (m)
/// @return Tangential velocity in meters per second
HOST_DEVICE inline constexpr float calculate_tangential_velocity(
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

/// Calculate period from angular velocity
/// Formula: T = 2π / ω
/// @param angular_velocity_radians_per_second Angular velocity (rad/s)
/// @return Period in seconds (time for one complete rotation)
HOST_DEVICE inline constexpr float calculate_period_from_angular_velocity(
    float angular_velocity_radians_per_second
) {
    return (angular_velocity_radians_per_second > EPSILON) ? 
           (2.0f * PI / angular_velocity_radians_per_second) : 0.0f;
}

/// Calculate frequency from period
/// Formula: f = 1 / T
/// @param period_seconds Time for one complete rotation (s)
/// @return Frequency in Hertz (rotations per second)
HOST_DEVICE inline constexpr float calculate_frequency_from_period(
    float period_seconds
) {
    return (period_seconds > EPSILON) ? (1.0f / period_seconds) : 0.0f;
}

/// Calculate angular velocity from frequency
/// Formula: ω = 2π·f
/// @param frequency_hertz Rotations per second (Hz)
/// @return Angular velocity in radians per second
HOST_DEVICE inline constexpr float calculate_angular_velocity_from_frequency(
    float frequency_hertz
) {
    return 2.0f * PI * frequency_hertz;
}

} // namespace kinematics
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_KINEMATICS_H

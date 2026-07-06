#ifndef BASEMENTS_ENGINE_VALIDATION_H
#define BASEMENTS_ENGINE_VALIDATION_H

#include "basements/core/math/vec3.h"
#include "error_handling.h"
#include <cmath>
#include <limits>

namespace basements {
namespace engine {

using namespace basements::math;

/// Numerical validation utilities
class NumericalValidator {
public:
    // Time step limits
    static constexpr float MIN_TIME_STEP_SECONDS = 1e-6f;   // 1 microsecond
    static constexpr float MAX_TIME_STEP_SECONDS = 0.1f;    // 100 milliseconds
    static constexpr float RECOMMENDED_TIME_STEP_SECONDS = 0.016f;  // ~60 FPS

    // Physical limits
    static constexpr float MAX_VELOCITY_METERS_PER_SECOND = 1e6f;  // Speed of light / 300
    static constexpr float MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1e8f;
    static constexpr float MAX_FORCE_NEWTONS = 1e12f;
    static constexpr float MIN_MASS_KILOGRAMS = 1e-6f;
    static constexpr float MAX_MASS_KILOGRAMS = 1e12f;

    /// Validate time step
    static void validate_time_step(float delta_time_seconds) {
        if (delta_time_seconds < MIN_TIME_STEP_SECONDS) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Time step too small: " + std::to_string(delta_time_seconds) + 
                " (min: " + std::to_string(MIN_TIME_STEP_SECONDS) + ")"
            );
        }
        
        if (delta_time_seconds > MAX_TIME_STEP_SECONDS) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Time step too large: " + std::to_string(delta_time_seconds) + 
                " (max: " + std::to_string(MAX_TIME_STEP_SECONDS) + ")"
            );
        }
    }

    /// Check if value is finite
    static bool is_finite(float value) {
        return std::isfinite(value);
    }

    /// Check if Vec3 is finite
    static bool is_finite(const Vec3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    /// Validate velocity
    static void validate_velocity(const Vec3& velocity_meters_per_second) {
        if (!is_finite(velocity_meters_per_second)) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Velocity contains NaN or Inf"
            );
        }

        float speed = velocity_meters_per_second.length();
        if (speed > MAX_VELOCITY_METERS_PER_SECOND) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Velocity too large: " + std::to_string(speed) + 
                " m/s (max: " + std::to_string(MAX_VELOCITY_METERS_PER_SECOND) + ")"
            );
        }
    }

    /// Validate acceleration
    static void validate_acceleration(const Vec3& acceleration_meters_per_second_squared) {
        if (!is_finite(acceleration_meters_per_second_squared)) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Acceleration contains NaN or Inf"
            );
        }

        float magnitude = acceleration_meters_per_second_squared.length();
        if (magnitude > MAX_ACCELERATION_METERS_PER_SECOND_SQUARED) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Acceleration too large: " + std::to_string(magnitude)
            );
        }
    }

    /// Validate force
    static void validate_force(const Vec3& force_newtons) {
        if (!is_finite(force_newtons)) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Force contains NaN or Inf"
            );
        }

        float magnitude = force_newtons.length();
        if (magnitude > MAX_FORCE_NEWTONS) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Force too large: " + std::to_string(magnitude)
            );
        }
    }

    /// Validate mass
    static void validate_mass(float mass_kilograms) {
        if (!is_finite(mass_kilograms)) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Mass contains NaN or Inf"
            );
        }

        if (mass_kilograms < MIN_MASS_KILOGRAMS) {
            ErrorManager::throw_error(
                EngineError::INVALID_STATE,
                "Mass too small: " + std::to_string(mass_kilograms)
            );
        }

        if (mass_kilograms > MAX_MASS_KILOGRAMS) {
            ErrorManager::throw_error(
                EngineError::NUMERICAL_INSTABILITY,
                "Mass too large: " + std::to_string(mass_kilograms)
            );
        }
    }
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_VALIDATION_H

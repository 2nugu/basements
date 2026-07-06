#ifndef BASEMENTS_ENGINE_DEBUG_UTILS_H
#define BASEMENTS_ENGINE_DEBUG_UTILS_H

#include "basements/core/math/vec3.h"
#include <string>
#include <sstream>
#include <iomanip>

namespace basements {
namespace engine {

using namespace basements::math;

/// Debugging utilities for physics engine
class DebugUtils {
public:
    /// Convert Vec3 to string
    static std::string vec3_to_string(const Vec3& v, int precision = 3) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision);
        oss << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return oss.str();
    }

    /// Convert float to string with units
    static std::string float_to_string(float value, const std::string& unit = "", int precision = 3) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision);
        oss << value;
        if (!unit.empty()) {
            oss << " " << unit;
        }
        return oss.str();
    }

    /// Format position for debug output
    static std::string format_position(const Vec3& position_meters) {
        return "Position: " + vec3_to_string(position_meters) + " m";
    }

    /// Format velocity for debug output
    static std::string format_velocity(const Vec3& velocity_meters_per_second) {
        float speed = velocity_meters_per_second.length();
        return "Velocity: " + vec3_to_string(velocity_meters_per_second) + " m/s" +
               " (speed: " + float_to_string(speed, "m/s") + ")";
    }

    /// Format acceleration for debug output
    static std::string format_acceleration(const Vec3& acceleration_meters_per_second_squared) {
        float magnitude = acceleration_meters_per_second_squared.length();
        return "Acceleration: " + vec3_to_string(acceleration_meters_per_second_squared) + " m/s²" +
               " (magnitude: " + float_to_string(magnitude, "m/s²") + ")";
    }

    /// Format energy for debug output
    static std::string format_energy(float kinetic_joules, float potential_joules) {
        float total = kinetic_joules + potential_joules;
        std::ostringstream oss;
        oss << "Energy: E_k=" << float_to_string(kinetic_joules, "J")
            << ", E_p=" << float_to_string(potential_joules, "J")
            << ", Total=" << float_to_string(total, "J");
        return oss.str();
    }

    /// Format momentum for debug output
    static std::string format_momentum(const Vec3& momentum_kg_m_per_s) {
        float magnitude = momentum_kg_m_per_s.length();
        return "Momentum: " + vec3_to_string(momentum_kg_m_per_s) + " kg·m/s" +
               " (magnitude: " + float_to_string(magnitude, "kg·m/s") + ")";
    }

    /// Create object state summary
    static std::string object_state_summary(
        const Vec3& position_meters,
        const Vec3& velocity_meters_per_second,
        float kinetic_energy_joules,
        float potential_energy_joules
    ) {
        std::ostringstream oss;
        oss << "Object State:\n";
        oss << "  " << format_position(position_meters) << "\n";
        oss << "  " << format_velocity(velocity_meters_per_second) << "\n";
        oss << "  " << format_energy(kinetic_energy_joules, potential_energy_joules);
        return oss.str();
    }

    /// Check for NaN or Inf
    static bool has_invalid_values(const Vec3& v) {
        return !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z);
    }

    /// Get invalid value description
    static std::string describe_invalid_values(const Vec3& v) {
        std::ostringstream oss;
        oss << "Invalid values detected: ";
        if (!std::isfinite(v.x)) oss << "x=" << v.x << " ";
        if (!std::isfinite(v.y)) oss << "y=" << v.y << " ";
        if (!std::isfinite(v.z)) oss << "z=" << v.z << " ";
        return oss.str();
    }
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_DEBUG_UTILS_H

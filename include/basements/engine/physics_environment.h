#ifndef BASEMENTS_PHYSICS_ENVIRONMENT_H
#define BASEMENTS_PHYSICS_ENVIRONMENT_H

/**
 * @file physics_environment.h
 * @brief Defines the global and local physics environment settings.
 */

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include <map>
#include <string>
#include <vector>

namespace basements {
namespace engine {

/**
 * @brief Represents a set of physical laws and constants for a simulation environment.
 */
struct PhysicsEnvironment {
    // === Metadata ===
    std::string name = "Earth";
    std::string description = "Standard Earth gravity and atmosphere.";

    // === Mechanics ===
    math::Vec3 gravity = {0.0f, -9.81f, 0.0f}; // m/s²
    float air_density = 1.225f;               // kg/m³ (at sea level)
    float air_viscosity = 1.81e-5f;           // Pa·s
    
    // === Thermodynamics ===
    float ambient_temperature = 293.15f;      // Kelvin (20°C)
    float stefan_boltzmann = 5.67e-8f;        // W/m²K⁴
    float boltzmann_constant = 1.38e-23f;     // J/K
    
    // === Electromagnetism ===
    float vacuum_permittivity = 8.85e-12f;    // F/m
    float vacuum_permeability = 1.26e-6f;     // H/m
    float speed_of_light = 2.998e8f;          // m/s
    
    // === Universal Constants ===
    float gravitational_constant = 6.674e-11f;// m³/kg·s²
    float planck_constant = 6.626e-34f;       // J·s

    // === Feature Flags ===
    bool enable_gravity = true;
    bool enable_drag = true;
    bool enable_thermodynamics = false;
    bool enable_relativity = false; // If true, use relativistic formulas

    // === Presets ===
    static PhysicsEnvironment Earth() {
        PhysicsEnvironment env;
        env.name = "Earth";
        env.description = "Standard Earth environment. Sealevel pressure.";
        return env;
    }

    static PhysicsEnvironment Moon() {
        PhysicsEnvironment env;
        env.name = "Moon";
        env.description = "Vacuum environment with low gravity.";
        env.gravity = {0.0f, -1.62f, 0.0f};
        env.air_density = 0.0f;
        env.air_viscosity = 0.0f;
        env.ambient_temperature = 250.0f; // Varies wildly
        return env;
    }

    static PhysicsEnvironment Mars() {
        PhysicsEnvironment env;
        env.name = "Mars";
        env.description = "Thin atmosphere, lower gravity.";
        env.gravity = {0.0f, -3.71f, 0.0f};
        env.air_density = 0.020f;
        env.ambient_temperature = 210.0f;
        return env;
    }

    static PhysicsEnvironment Space() {
        PhysicsEnvironment env;
        env.name = "Space";
        env.description = "Interstellar space. Microgravity, vacuum.";
        env.gravity = {0.0f, 0.0f, 0.0f};
        env.air_density = 0.0f;
        env.ambient_temperature = 2.7f; // CMB
        return env;
    }
};

/**
 * @brief Manages the active environment and per-object overrides.
 */
class EnvironmentManager {
public:
    // Global environment
    PhysicsEnvironment global_env = PhysicsEnvironment::Earth();
    
    // Per-object overrides (NodeID -> Environment)
    // Note: In a real ECS, this would be a component.
    std::map<int, PhysicsEnvironment> overrides;

    void set_global(const PhysicsEnvironment& env) {
        global_env = env;
    }

    const PhysicsEnvironment& get(int node_id = -1) const {
        if (node_id != -1) {
            auto it = overrides.find(node_id);
            if (it != overrides.end()) {
                return it->second;
            }
        }
        return global_env;
    }

    void set_override(int node_id, const PhysicsEnvironment& env) {
        overrides[node_id] = env;
    }

    void remove_override(int node_id) {
        overrides.erase(node_id);
    }
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_PHYSICS_ENVIRONMENT_H

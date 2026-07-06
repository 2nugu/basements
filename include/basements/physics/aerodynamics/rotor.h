/**
 * @file rotor.h
 * @brief Rotor/Propeller Physics for Drones and Aircraft
 * 
 * Simplified Blade Element Momentum Theory (BEMT) implementation.
 * Generates Thrust and Reaction Torque based on angular velocity (RPM).
 */

#ifndef BASEMENTS_AERODYNAMICS_ROTOR_H
#define BASEMENTS_AERODYNAMICS_ROTOR_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/rigid_body.h"
#include <cmath>

namespace basements {
namespace aerodynamics {

using namespace basements::math;
using namespace basements::physics;

/**
 * @brief Rotor Configuration
 */
struct RotorConfig {
    float radius;           // Blade radius (m)
    float thrust_coeff;     // Thrust coefficient (Ct)
    float torque_coeff;     // Torque coefficient (Cq)
    float air_density;      // Air density (kg/m³)
    int direction;          // Spin direction: +1 = CCW, -1 = CW
    
    RotorConfig()
        : radius(0.1f)
        , thrust_coeff(0.1f)
        , torque_coeff(0.01f)
        , air_density(1.225f)
        , direction(1)
    {}
    
    // Preset for common drone propellers
    static RotorConfig Drone5Inch() {
        RotorConfig cfg;
        cfg.radius = 0.0635f;  // 5 inch = 127mm diameter
        cfg.thrust_coeff = 0.12f;
        cfg.torque_coeff = 0.012f;
        return cfg;
    }
    
    static RotorConfig Drone10Inch() {
        RotorConfig cfg;
        cfg.radius = 0.127f;  // 10 inch
        cfg.thrust_coeff = 0.15f;
        cfg.torque_coeff = 0.015f;
        return cfg;
    }
};

/**
 * @brief Rotor State
 */
struct RotorState {
    float omega;            // Angular velocity (rad/s)
    float target_omega;     // Commanded angular velocity
    float thrust;           // Current thrust output (N)
    float torque;           // Current reaction torque (N·m)
    
    RotorState() : omega(0), target_omega(0), thrust(0), torque(0) {}
};

/**
 * @brief Rotor Physics Calculator
 * 
 * Simplified model:
 *   Thrust = Ct * rho * omega^2 * R^4
 *   Torque = Cq * rho * omega^2 * R^5
 */
class Rotor {
public:
    RotorConfig config;
    RotorState state;
    
    // Attachment
    int body_index;         // Parent body index
    Vec3 local_position;    // Position on body (local)
    Vec3 local_axis;        // Thrust direction (local, typically +Y or +Z)
    
    Rotor() : body_index(-1), local_position(Vec3::zero()), local_axis(Vec3(0, 1, 0)) {}
    
    /**
     * @brief Set target RPM
     */
    void set_rpm(float rpm) {
        state.target_omega = rpm * (2.0f * PI / 60.0f);
    }
    
    /**
     * @brief Get current RPM
     */
    float get_rpm() const {
        return state.omega * (60.0f / (2.0f * PI));
    }
    
    /**
     * @brief Update rotor physics (call each frame)
     * @param dt Delta time (seconds)
     * @param motor_time_constant Motor response time constant
     */
    void update(float dt, float motor_time_constant = 0.05f) {
        // Motor dynamics: first-order lag
        float alpha = dt / (motor_time_constant + dt);
        state.omega += alpha * (state.target_omega - state.omega);
        
        // Clamp to prevent negative spin
        if (state.omega < 0) state.omega = 0;
        
        // Calculate thrust: T = Ct * rho * omega^2 * R^4
        float R = config.radius;
        float R4 = R * R * R * R;
        state.thrust = config.thrust_coeff * config.air_density * 
                       state.omega * state.omega * R4;
        
        // Calculate torque: Q = Cq * rho * omega^2 * R^5
        float R5 = R4 * R;
        state.torque = config.torque_coeff * config.air_density * 
                       state.omega * state.omega * R5 * config.direction;
    }
    
    /**
     * @brief Apply forces to parent body
     */
    void apply_to_body(RigidBody& body) const {
        if (body_index < 0) return;
        
        // Transform local axis to world
        Vec3 world_axis = body.orientation.rotate(local_axis);
        Vec3 world_pos = body.position + body.orientation.rotate(local_position);
        
        // Thrust force along axis
        Vec3 thrust_force = world_axis * state.thrust;
        body.apply_force(thrust_force);
        
        // Reaction torque (opposite to spin direction)
        Vec3 reaction_torque = world_axis * (-state.torque);
        body.apply_torque(reaction_torque);
        
        // Torque from thrust offset (if rotor not at CoM)
        Vec3 r = world_pos - body.position;
        Vec3 moment = r.cross(thrust_force);
        body.apply_torque(moment);
    }
};

/**
 * @brief Quadcopter Configuration (4 rotors)
 */
class QuadcopterController {
public:
    Rotor rotors[4];
    float arm_length;
    
    QuadcopterController() : arm_length(0.25f) {}
    
    /**
     * @brief Initialize standard X-configuration quadcopter
     */
    void init_x_config(int body_index, float arm = 0.25f) {
        arm_length = arm;
        
        // Front-Right (CW)
        rotors[0].body_index = body_index;
        rotors[0].local_position = Vec3(arm, 0, arm);
        rotors[0].local_axis = Vec3(0, 1, 0);
        rotors[0].config.direction = -1;
        
        // Front-Left (CCW)
        rotors[1].body_index = body_index;
        rotors[1].local_position = Vec3(-arm, 0, arm);
        rotors[1].local_axis = Vec3(0, 1, 0);
        rotors[1].config.direction = 1;
        
        // Rear-Left (CW)
        rotors[2].body_index = body_index;
        rotors[2].local_position = Vec3(-arm, 0, -arm);
        rotors[2].local_axis = Vec3(0, 1, 0);
        rotors[2].config.direction = -1;
        
        // Rear-Right (CCW)
        rotors[3].body_index = body_index;
        rotors[3].local_position = Vec3(arm, 0, -arm);
        rotors[3].local_axis = Vec3(0, 1, 0);
        rotors[3].config.direction = 1;
    }
    
    /**
     * @brief Set motor commands (RPM for each motor)
     */
    void set_motor_rpms(float m0, float m1, float m2, float m3) {
        rotors[0].set_rpm(m0);
        rotors[1].set_rpm(m1);
        rotors[2].set_rpm(m2);
        rotors[3].set_rpm(m3);
    }
    
    /**
     * @brief Mixer: Convert Throttle/Roll/Pitch/Yaw to motor commands
     * @param throttle 0.0 - 1.0
     * @param roll -1.0 to 1.0
     * @param pitch -1.0 to 1.0
     * @param yaw -1.0 to 1.0
     * @param max_rpm Maximum motor RPM
     */
    void mix_controls(float throttle, float roll, float pitch, float yaw, float max_rpm = 10000.0f) {
        float base = throttle * max_rpm;
        float r = roll * 0.3f * max_rpm;
        float p = pitch * 0.3f * max_rpm;
        float y = yaw * 0.2f * max_rpm;
        
        // X-config mixing
        float m0 = base - r - p - y;  // FR
        float m1 = base + r - p + y;  // FL
        float m2 = base + r + p - y;  // RL
        float m3 = base - r + p + y;  // RR
        
        // Clamp
        m0 = std::max(0.0f, std::min(max_rpm, m0));
        m1 = std::max(0.0f, std::min(max_rpm, m1));
        m2 = std::max(0.0f, std::min(max_rpm, m2));
        m3 = std::max(0.0f, std::min(max_rpm, m3));
        
        set_motor_rpms(m0, m1, m2, m3);
    }
    
    /**
     * @brief Update all rotors
     */
    void update(float dt) {
        for (int i = 0; i < 4; ++i) {
            rotors[i].update(dt);
        }
    }
    
    /**
     * @brief Apply all rotor forces to body
     */
    void apply_to_body(RigidBody& body) {
        for (int i = 0; i < 4; ++i) {
            rotors[i].apply_to_body(body);
        }
    }
    
    /**
     * @brief Get total thrust
     */
    float get_total_thrust() const {
        float total = 0;
        for (int i = 0; i < 4; ++i) {
            total += rotors[i].state.thrust;
        }
        return total;
    }
};

} // namespace aerodynamics
} // namespace basements

#endif // BASEMENTS_AERODYNAMICS_ROTOR_H

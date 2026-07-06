#ifndef BASEMENTS_ENGINE_SYSTEMS_OPTIMIZED_MECHANICAL_SYSTEM_H
#define BASEMENTS_ENGINE_SYSTEMS_OPTIMIZED_MECHANICAL_SYSTEM_H

#include "basements/engine/core/quantity_storage.h"
#include "basements/engine/core/quantity_handle.h"
#include "basements/engine/core/compute_graph.h"
#include "basements/core/math/vec3.h"

namespace basements {
namespace engine {

using namespace basements::math;

/// Optimized mechanical system using handles and compute graph
class OptimizedMechanicalSystem {
public:
    OptimizedMechanicalSystem()
        : time_(0.0f)
        , mass_(1.0f)
        , gravity_(0.0f, -9.81f, 0.0f)
    {
        setup_quantities();
        setup_compute_graph();
    }

    /// Initialize the mechanical system with initial conditions
    void initialize(const Vec3& initial_position, const Vec3& initial_velocity, float object_mass = 1.0f) {
        storage_.set(h_pos_x_, initial_position.x);
        storage_.set(h_pos_y_, initial_position.y);
        storage_.set(h_pos_z_, initial_position.z);
        
        storage_.set(h_vel_x_, initial_velocity.x);
        storage_.set(h_vel_y_, initial_velocity.y);
        storage_.set(h_vel_z_, initial_velocity.z);
        
        storage_.set(h_acc_x_, 0.0f);
        storage_.set(h_acc_y_, 0.0f);
        storage_.set(h_acc_z_, 0.0f);
        
        storage_.set(h_force_x_, 0.0f);
        storage_.set(h_force_y_, 0.0f);
        storage_.set(h_force_z_, 0.0f);
        
        storage_.set(h_mass_, object_mass);
        storage_.set(h_time_, 0.0f);
        
        current_mass_ = object_mass;
        current_simulation_time_ = 0.0f;
    }

    /// Update the system (optimized)
    void update(float dt) {
        // Get current state
        Vec3 force(
            storage_.get(h_force_x_),
            storage_.get(h_force_y_),
            storage_.get(h_force_z_)
        );
        
        // Apply gravity
        force = force + gravity_ * mass_;
        storage_.set(h_force_x_, force.x);
        storage_.set(h_force_y_, force.y);
        storage_.set(h_force_z_, force.z);
        
        // Calculate acceleration: a = F/m
        Vec3 accel = force / mass_;
        storage_.set(h_acc_x_, accel.x);
        storage_.set(h_acc_y_, accel.y);
        storage_.set(h_acc_z_, accel.z);
        
        // Semi-implicit Euler integration
        Vec3 vel(
            storage_.get(h_vel_x_),
            storage_.get(h_vel_y_),
            storage_.get(h_vel_z_)
        );
        
        Vec3 vel_new = vel + accel * dt;
        storage_.set(h_vel_x_, vel_new.x);
        storage_.set(h_vel_y_, vel_new.y);
        storage_.set(h_vel_z_, vel_new.z);
        
        Vec3 pos(
            storage_.get(h_pos_x_),
            storage_.get(h_pos_y_),
            storage_.get(h_pos_z_)
        );
        
        Vec3 pos_new = pos + vel_new * dt;
        storage_.set(h_pos_x_, pos_new.x);
        storage_.set(h_pos_y_, pos_new.y);
        storage_.set(h_pos_z_, pos_new.z);
        
        // Update time
        time_ += dt;
        storage_.set(h_time_, time_);
        
        // Execute compute graph for derived quantities
        compute_graph_.execute(storage_);
        
        // Reset force
        storage_.set(h_force_x_, 0.0f);
        storage_.set(h_force_y_, 0.0f);
        storage_.set(h_force_z_, 0.0f);
    }

    /// Apply force
    void apply_force(const Vec3& force) {
        Vec3 current(
            storage_.get(h_force_x_),
            storage_.get(h_force_y_),
            storage_.get(h_force_z_)
        );
        Vec3 new_force = current + force;
        storage_.set(h_force_x_, new_force.x);
        storage_.set(h_force_y_, new_force.y);
        storage_.set(h_force_z_, new_force.z);
    }

    /// Get position
    Vec3 get_position() const {
        return Vec3(
            storage_.get(h_pos_x_),
            storage_.get(h_pos_y_),
            storage_.get(h_pos_z_)
        );
    }

    /// Get velocity
    Vec3 get_velocity() const {
        return Vec3(
            storage_.get(h_vel_x_),
            storage_.get(h_vel_y_),
            storage_.get(h_vel_z_)
        );
    }

    /// Get kinetic energy
    float get_kinetic_energy() const {
        return storage_.get(h_kinetic_energy_);
    }

    /// Get potential energy
    float get_potential_energy() const {
        return storage_.get(h_potential_energy_);
    }

    /// Get total energy
    float get_total_energy() const {
        return storage_.get(h_total_energy_);
    }

    /// Set position
    void set_position(const Vec3& new_position) {
        storage_.set(h_pos_x_, new_position.x);
        storage_.set(h_pos_y_, new_position.y);
        storage_.set(h_pos_z_, new_position.z);
    }

    /// Set velocity
    void set_velocity(const Vec3& new_velocity) {
        storage_.set(h_vel_x_, new_velocity.x);
        storage_.set(h_vel_y_, new_velocity.y);
        storage_.set(h_vel_z_, new_velocity.z);
    }

    /// Get acceleration
    Vec3 get_acceleration() const {
        return Vec3(
            storage_.get(h_acc_x_),
            storage_.get(h_acc_y_),
            storage_.get(h_acc_z_)
        );
    }

    /// Get momentum (p = m*v)
    Vec3 get_momentum() const {
        Vec3 vel = get_velocity();
        return vel * mass_;
    }

    /// Apply impulse (J = Δp = m*Δv)
    void apply_impulse(const Vec3& impulse_newton_seconds) {
        if (mass_ > 1e-6f) {
            Vec3 velocity_change = impulse_newton_seconds / mass_;
            Vec3 current_velocity = get_velocity();
            set_velocity(current_velocity + velocity_change);
        }
    }

    /// Set gravity
    void set_gravity(const Vec3& new_gravity) {
        gravity_ = new_gravity;
    }

    /// Get gravity
    Vec3 get_gravity() const {
        return gravity_;
    }

    /// Get storage (for GPU transfer)
    const QuantityStorage& storage() const { return storage_; }
    QuantityStorage& storage() { return storage_; }

    /// Get compute graph (for GPU execution)
    const ComputeGraph& compute_graph() const { return compute_graph_; }

private:
    void setup_quantities() {
        // Position
        h_pos_x_ = storage_.add_quantity(0.0f);
        h_pos_y_ = storage_.add_quantity(0.0f);
        h_pos_z_ = storage_.add_quantity(0.0f);
        
        // Velocity
        h_vel_x_ = storage_.add_quantity(0.0f);
        h_vel_y_ = storage_.add_quantity(0.0f);
        h_vel_z_ = storage_.add_quantity(0.0f);
        
        // Acceleration
        h_acc_x_ = storage_.add_quantity(0.0f);
        h_acc_y_ = storage_.add_quantity(0.0f);
        h_acc_z_ = storage_.add_quantity(0.0f);
        
        // Force
        h_force_x_ = storage_.add_quantity(0.0f);
        h_force_y_ = storage_.add_quantity(0.0f);
        h_force_z_ = storage_.add_quantity(0.0f);
        
        // Scalar quantities
        h_mass_ = storage_.add_quantity(1.0f);
        h_time_ = storage_.add_quantity(0.0f);
        h_speed_ = storage_.add_quantity(0.0f);
        h_kinetic_energy_ = storage_.add_quantity(0.0f);
        h_potential_energy_ = storage_.add_quantity(0.0f);
        h_total_energy_ = storage_.add_quantity(0.0f);
    }

    void setup_compute_graph() {
        ComputeGraphBuilder builder(compute_graph_, storage_);
        
        // Build speed: sqrt(vx^2 + vy^2 + vz^2)
        builder.build_speed(h_speed_, h_vel_x_, h_vel_y_, h_vel_z_);
        
        // Build kinetic energy: 0.5 * m * v^2
        builder.build_kinetic_energy(h_kinetic_energy_, h_mass_, h_speed_);
        
        // Build potential energy: m * g * h
        QuantityHandle h_gravity = storage_.add_quantity(std::abs(gravity_.y));
        builder.build_potential_energy(
            h_potential_energy_, h_mass_, h_gravity, h_pos_y_);
        
        // Build total energy: E_k + E_p
        builder.build_total_energy(
            h_total_energy_, h_kinetic_energy_, h_potential_energy_);
    }

    QuantityStorage storage_;
    ComputeGraph compute_graph_;
    
    // Handles
    QuantityHandle h_pos_x_, h_pos_y_, h_pos_z_;
    QuantityHandle h_vel_x_, h_vel_y_, h_vel_z_;
    QuantityHandle h_acc_x_, h_acc_y_, h_acc_z_;
    QuantityHandle h_force_x_, h_force_y_, h_force_z_;
    QuantityHandle h_mass_, h_time_;
    QuantityHandle h_speed_;
    QuantityHandle h_kinetic_energy_;
    QuantityHandle h_potential_energy_;
    QuantityHandle h_total_energy_;
    
    // Cached values for performance
    float time_;
    float mass_;
    Vec3 gravity_;
    
    // Cached handles (aliased for compatibility) - REMOVED
    // QuantityHandle handle_position_x_, handle_position_y_, handle_position_z_;
    // QuantityHandle handle_velocity_x_, handle_velocity_y_, handle_velocity_z_;
    // QuantityHandle handle_acceleration_x_, handle_acceleration_y_, handle_acceleration_z_;
    // QuantityHandle handle_force_x_, handle_force_y_, handle_force_z_;
    // QuantityHandle handle_mass_, handle_simulation_time_;
    
    // Current state cache
    float current_mass_;
    float current_simulation_time_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_SYSTEMS_OPTIMIZED_MECHANICAL_SYSTEM_H

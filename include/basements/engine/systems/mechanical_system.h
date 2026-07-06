#ifndef BASEMENTS_ENGINE_SYSTEMS_MECHANICAL_SYSTEM_H
#define BASEMENTS_ENGINE_SYSTEMS_MECHANICAL_SYSTEM_H

#include "basements/engine/core/physics_state.h"
#include "basements/core/math/vec3.h"
#include <cmath>

namespace basements {
namespace engine {

using namespace basements::math;

/// Mechanical system integrating kinematics and dynamics
class MechanicalSystem {
public:
    MechanicalSystem(PhysicsState& state)
        : state_(state)
        , time_(0.0f)
        , mass_(1.0f)
        , gravity_(0.0f, -9.81f, 0.0f)
    {
        setup_quantities();
        setup_dependencies();
    }

    /// Initialize the system
    void initialize(const Vec3& position, const Vec3& velocity, float mass = 1.0f) {
        state_.set_vector("position", position);
        state_.set_vector("velocity", velocity);
        state_.set_vector("acceleration", Vec3::zero());
        state_.set_vector("force", Vec3::zero());
        state_.set_scalar("mass", mass);
        state_.set_scalar("time", 0.0f);
        
        mass_ = mass;
        time_ = 0.0f;
    }

    /// Update the system by one time step
    void update(float dt) {
        // Get current state
        Vec3 pos = state_.get_vector("position");
        Vec3 vel = state_.get_vector("velocity");
        Vec3 force = state_.get_vector("force");
        
        // Apply gravity
        force = force + gravity_ * mass_;
        state_.set_vector("force", force);
        
        // Calculate acceleration: a = F/m
        Vec3 accel = force / mass_;
        state_.set_vector("acceleration", accel);
        
        // Semi-implicit Euler integration
        // v_new = v + a*dt
        Vec3 vel_new = vel + accel * dt;
        state_.set_vector("velocity", vel_new);
        
        // x_new = x + v_new*dt
        Vec3 pos_new = pos + vel_new * dt;
        state_.set_vector("position", pos_new);
        
        // Update time
        time_ += dt;
        state_.set_scalar("time", time_);
        
        // Reset force for next frame
        state_.set_vector("force", Vec3::zero());
    }

    /// Apply an external force
    void apply_force(const Vec3& force) {
        Vec3 current_force = state_.get_vector("force");
        state_.set_vector("force", current_force + force);
    }

    /// Apply an impulse (instant velocity change)
    void apply_impulse(const Vec3& impulse) {
        Vec3 vel = state_.get_vector("velocity");
        state_.set_vector("velocity", vel + impulse / mass_);
    }

    /// Set gravity
    void set_gravity(const Vec3& gravity) {
        gravity_ = gravity;
    }

    /// Set mass
    void set_mass(float mass) {
        mass_ = mass;
        state_.set_scalar("mass", mass);
    }

    /// Get current kinetic energy
    float get_kinetic_energy() const {
        return state_.get_scalar("kinetic_energy");
    }

    /// Get current potential energy
    float get_potential_energy() const {
        return state_.get_scalar("potential_energy");
    }

    /// Get total energy
    float get_total_energy() const {
        return state_.get_scalar("total_energy");
    }

    /// Get momentum
    Vec3 get_momentum() const {
        return state_.get_vector("momentum");
    }

private:
    void setup_quantities() {
        // Define vector quantities
        state_.define_vector("position");
        state_.define_vector("velocity");
        state_.define_vector("acceleration");
        state_.define_vector("force");
        state_.define_vector("momentum");
        
        // Define scalar quantities
        state_.set_scalar("mass", mass_);
        state_.set_scalar("time", time_);
    }

    void setup_dependencies() {
        // Kinetic energy: E_k = 0.5 * m * v²
        state_.define_scalar("kinetic_energy", [this]() {
            Vec3 vel = state_.get_vector("velocity");
            float m = state_.get_scalar("mass");
            return 0.5f * m * vel.length_squared();
        }, {"velocity", "mass"});

        // Potential energy: E_p = m * g * h
        state_.define_scalar("potential_energy", [this]() {
            Vec3 pos = state_.get_vector("position");
            float m = state_.get_scalar("mass");
            float h = pos.y;  // Height
            float g = std::abs(gravity_.y);
            return m * g * h;
        }, {"position", "mass"});

        // Total energy: E_total = E_k + E_p
        state_.define_scalar("total_energy", [this]() {
            return state_.get_scalar("kinetic_energy") + 
                   state_.get_scalar("potential_energy");
        }, {"kinetic_energy", "potential_energy"});

        // Speed: |v|
        state_.define_scalar("speed", [this]() {
            return state_.get_vector("velocity").length();
        }, {"velocity"});

        // Set up momentum computation (p = mv)
        auto* mom_x = state_.get_vector_quantity("momentum");
        if (mom_x) {
            mom_x->x().set_compute_function([this]() {
                return state_.get_vector("velocity").x * state_.get_scalar("mass");
            });
            mom_x->y().set_compute_function([this]() {
                return state_.get_vector("velocity").y * state_.get_scalar("mass");
            });
            mom_x->z().set_compute_function([this]() {
                return state_.get_vector("velocity").z * state_.get_scalar("mass");
            });
            
            // Add dependencies
            auto* vel = state_.get_vector_quantity("velocity");
            auto* mass = state_.get_scalar_quantity("mass");
            if (vel && mass) {
                vel->x().add_dependent(&mom_x->x());
                vel->y().add_dependent(&mom_x->y());
                vel->z().add_dependent(&mom_x->z());
                mass->add_dependent(&mom_x->x());
                mass->add_dependent(&mom_x->y());
                mass->add_dependent(&mom_x->z());
            }
        }
    }

    PhysicsState& state_;
    float time_;
    float mass_;
    Vec3 gravity_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_SYSTEMS_MECHANICAL_SYSTEM_H

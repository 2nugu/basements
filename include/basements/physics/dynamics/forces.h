#ifndef BASEMENTS_DYNAMICS_FORCES_H
#define BASEMENTS_DYNAMICS_FORCES_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/rigid_body.h"
#include <cmath>

namespace basements {
namespace dynamics {

using namespace basements::math;
using namespace basements::physics;

/**
 * @brief Base interface for force generators
 * 
 * Force generators apply forces to rigid bodies each simulation step.
 * All parameters are public members for direct runtime adjustment.
 */
class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;
    
    // Apply forces (CPU)
    // Apply forces (CPU)
    virtual void apply(RigidBody* bodies, int num_bodies, float dt) = 0;

    // Helper for single body
    void apply(RigidBody& body, float dt = 0.0f) {
        apply(&body, 1, dt);
    }
};

// ============================================================================
// BODY FORCES (O(N) - Independent per body)
// ============================================================================

/**
 * @brief Uniform gravitational field (e.g., planetary surface)
 * 
 * Configurable for different celestial bodies:
 * - Earth: 9.81 m/s²
 * - Moon: 1.62 m/s²
 * - Mars: 3.71 m/s²
 * - Jupiter: 24.79 m/s²
 * - Sun: 274 m/s²
 */
class UniformGravity : public ForceGenerator {
public:
    Vec3 g;  // Gravitational acceleration (m/s²) - ADJUSTABLE AT RUNTIME
    
    // Presets for common celestial bodies
    static UniformGravity Earth()   { return UniformGravity(Vec3(0, -9.81f, 0)); }
    static UniformGravity Moon()    { return UniformGravity(Vec3(0, -1.62f, 0)); }
    static UniformGravity Mars()    { return UniformGravity(Vec3(0, -3.71f, 0)); }
    static UniformGravity Jupiter() { return UniformGravity(Vec3(0, -24.79f, 0)); }
    static UniformGravity Sun()     { return UniformGravity(Vec3(0, -274.0f, 0)); }
    static UniformGravity Custom(Vec3 gravity) { return UniformGravity(gravity); }
    
    explicit UniformGravity(Vec3 gravity = Vec3(0, -9.81f, 0)) : g(gravity) {}
    
    void apply(RigidBody* bodies, int num_bodies, float dt) override {
        (void)dt;  // Not time-dependent
        for (int i = 0; i < num_bodies; ++i) {
            if (bodies[i].type == BodyType::DYNAMIC) {
                bodies[i].force += bodies[i].mass * g;
            }
        }
    }
};

/**
 * @brief Aerodynamic drag force
 * 
 * F_drag = -0.5 * ρ * v² * C_d * A * v̂
 */
class AerodynamicDrag : public ForceGenerator {
public:
    float rho;   // Fluid density (kg/m³) - ADJUSTABLE
    float C_d;   // Drag coefficient - ADJUSTABLE
    float A;     // Cross-sectional area (m²) - ADJUSTABLE
    
    // Presets
    static AerodynamicDrag AirSphere(float radius) {
        return AerodynamicDrag(1.225f, 0.47f, 3.14159f * radius * radius);
    }
    static AerodynamicDrag WaterSphere(float radius) {
        return AerodynamicDrag(1000.0f, 0.47f, 3.14159f * radius * radius);
    }
    
    AerodynamicDrag(float density = 1.225f, float drag_coeff = 0.47f, float area = 1.0f)
        : rho(density), C_d(drag_coeff), A(area) {}
    
    void apply(RigidBody* bodies, int num_bodies, float dt) override {
        (void)dt;
        for (int i = 0; i < num_bodies; ++i) {
            if (bodies[i].type == BodyType::DYNAMIC) {
                Vec3 v = bodies[i].linear_velocity;
                float v_mag = v.length();
                if (v_mag > 0.01f) {
                    Vec3 drag = -0.5f * rho * v_mag * v_mag * C_d * A * v.normalized();
                    bodies[i].force += drag;
                }
            }
        }
    }
};

/**
 * @brief Linear damping (energy dissipation)
 */
class LinearDamping : public ForceGenerator {
public:
    float k;  // Damping coefficient - ADJUSTABLE
    
    explicit LinearDamping(float damping = 0.1f) : k(damping) {}
    
    void apply(RigidBody* bodies, int num_bodies, float dt) override {
        (void)dt;
        for (int i = 0; i < num_bodies; ++i) {
            if (bodies[i].type == BodyType::DYNAMIC) {
                bodies[i].force += -k * bodies[i].linear_velocity;
            }
        }
    }
};

// ============================================================================
// PAIR FORCES (O(N²) - Body-body interactions)
// ============================================================================

/**
 * @brief Universal gravitation (N-Body)
 * 
 * F = G * m₁ * m₂ / r²
 */
class NBodyGravity : public ForceGenerator {
public:
    float G;          // Gravitational constant (m³/kg/s²) - ADJUSTABLE
    float softening;  // Softening parameter - ADJUSTABLE
    
    // Presets
    static NBodyGravity RealPhysics() {
        return NBodyGravity(6.674e-11f, 0.0f);
    }
    static NBodyGravity Simulation(float scale = 1.0f) {
        return NBodyGravity(1.0f * scale, 0.1f);
    }
    
    NBodyGravity(float gravitational_constant = 6.674e-11f, float soft = 0.1f)
        : G(gravitational_constant), softening(soft) {}
    
    void apply(RigidBody* bodies, int num_bodies, float dt) override {
        (void)dt;
        for (int i = 0; i < num_bodies; ++i) {
            if (bodies[i].type == BodyType::STATIC) continue;
            
            for (int j = i + 1; j < num_bodies; ++j) {
                Vec3 r = bodies[j].position - bodies[i].position;
                float r_mag = r.length() + softening;
                float F_mag = G * bodies[i].mass * bodies[j].mass / (r_mag * r_mag);
                Vec3 F = F_mag * r.normalized();
                
                bodies[i].force += F;
                if (bodies[j].type == BodyType::DYNAMIC) {
                    bodies[j].force -= F;
                }
            }
        }
    }
};

/**
 * @brief Lennard-Jones potential (Molecular dynamics)
 */
class LennardJonesForce : public ForceGenerator {
public:
    float epsilon;  // Depth of potential well - ADJUSTABLE
    float sigma;    // Distance at zero potential - ADJUSTABLE
    float cutoff;   // Interaction cutoff radius - ADJUSTABLE
    
    LennardJonesForce(float eps = 1.0f, float sig = 1.0f)
        : epsilon(eps), sigma(sig), cutoff(2.5f * sig) {}
    
    void apply(RigidBody* bodies, int num_bodies, float dt) override {
        (void)dt;
        for (int i = 0; i < num_bodies; ++i) {
            for (int j = i + 1; j < num_bodies; ++j) {
                Vec3 r = bodies[j].position - bodies[i].position;
                float r_mag = r.length();
                
                if (r_mag < cutoff && r_mag > 0.01f) {
                    float sr = sigma / r_mag;
                    float sr6 = sr * sr * sr * sr * sr * sr;
                    float sr12 = sr6 * sr6;
                    float F_mag = 24.0f * epsilon * (2.0f * sr12 - sr6) / r_mag;
                    Vec3 F = F_mag * r.normalized();
                    
                    bodies[i].force += F;
                    bodies[j].force -= F;
                }
            }
        }
    }
};

// ============================================================================
// SPRING FORCES (Hooke's Law)
// ============================================================================

/**
 * @brief Spring force generator (Hooke's Law)
 * 
 * F = -k(x - x₀) - c·v
 * 
 * Simulates elastic springs connecting two bodies or a body to a fixed point.
 * Includes damping to prevent infinite oscillation.
 * 
 * Parameters (runtime adjustable):
 * - k: Spring stiffness (N/m)
 * - rest_length: Natural length (m)
 * - damping: Damping coefficient
 */
class SpringForce : public ForceGenerator {
public:
    float k;              // Spring constant (N/m)
    float rest_length;    // Rest length (m)
    float damping;        // Damping coefficient
    
    RigidBody* other;     // Other end of spring (nullptr = fixed point)
    Vec3 anchor_a;        // Attachment point on this body (local)
    Vec3 anchor_b;        // Attachment point on other body (local)
    Vec3 fixed_point;     // Fixed world position (if other == nullptr)
    
    SpringForce(RigidBody* other_body = nullptr, 
                float stiffness = 100.0f, 
                float length = 1.0f, 
                float damp = 0.1f)
        : k(stiffness)
        , rest_length(length)
        , damping(damp)
        , other(other_body)
        , anchor_a(Vec3::zero())
        , anchor_b(Vec3::zero())
        , fixed_point(Vec3::zero())
    {}
    
    void apply(RigidBody* bodies, int num_bodies, float dt) override {
        for (int i = 0; i < num_bodies; ++i) {
            apply(bodies[i]);
        }
    }
    
    void apply(RigidBody& body) {
        Vec3 pos_a = body.position + anchor_a;  // Simplified (no rotation)
        Vec3 pos_b;
        Vec3 vel_b = Vec3::zero();
        
        if (other) {
            pos_b = other->position + anchor_b;
            vel_b = other->linear_velocity;
        } else {
            pos_b = fixed_point;
        }
        
        Vec3 delta = pos_b - pos_a;
        float length = delta.length();
        
        if (length < EPSILON) return;
        
        Vec3 direction = delta / length;
        
        // Spring force: F = -k(x - x₀)
        float extension = length - rest_length;
        Vec3 spring_force = direction * (k * extension);
        
        // Damping force: F = -c·v_rel
        Vec3 rel_vel = vel_b - body.linear_velocity;
        float vel_along_spring = rel_vel.dot(direction);
        Vec3 damping_force = direction * (damping * vel_along_spring);
        
        Vec3 total_force = spring_force + damping_force;
        
        body.apply_force(total_force);
        
        // Newton's 3rd law
        if (other) {
            other->apply_force(-total_force);
        }
    }
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_FORCES_H

/**
 * @file sph.h
 * @brief Smoothed Particle Hydrodynamics (SPH) for Fluid Simulation
 * 
 * Basic SPH implementation for incompressible fluids.
 * Uses WCSPH (Weakly Compressible SPH) for stability.
 */

#ifndef BASEMENTS_FLUID_SPH_H
#define BASEMENTS_FLUID_SPH_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include <vector>
#include <cmath>

namespace basements {
namespace fluid {

using namespace basements::math;

/**
 * @brief SPH Particle
 */
struct SPHParticle {
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    float mass;
    float density;
    float pressure;
    
    SPHParticle()
        : position(Vec3::zero())
        , velocity(Vec3::zero())
        , acceleration(Vec3::zero())
        , mass(1.0f)
        , density(1000.0f)  // Water density
        , pressure(0.0f)
    {}
};

/**
 * @brief SPH Configuration
 */
struct SPHConfig {
    float rest_density;     // Reference density (kg/m³)
    float gas_constant;     // Stiffness for pressure
    float viscosity;        // Viscosity coefficient
    float smoothing_radius; // Kernel support radius (h)
    Vec3 gravity;           // Gravity vector
    float time_step;        // Simulation timestep
    
    // Boundary settings
    Vec3 bound_min;
    Vec3 bound_max;
    float bound_damping;    // Velocity damping at boundary
    
    SPHConfig()
        : rest_density(1000.0f)
        , gas_constant(2000.0f)
        , viscosity(0.1f)
        , smoothing_radius(0.1f)
        , gravity(Vec3(0, -9.81f, 0))
        , time_step(0.005f)
        , bound_min(Vec3(-1, -1, -1))
        , bound_max(Vec3(1, 1, 1))
        , bound_damping(0.5f)
    {}
    
    // Presets
    static SPHConfig Water() {
        SPHConfig cfg;
        cfg.rest_density = 1000.0f;
        cfg.gas_constant = 2000.0f;
        cfg.viscosity = 0.001f;
        return cfg;
    }
    
    static SPHConfig Oil() {
        SPHConfig cfg;
        cfg.rest_density = 920.0f;
        cfg.gas_constant = 1500.0f;
        cfg.viscosity = 0.1f;
        return cfg;
    }
    
    static SPHConfig Honey() {
        SPHConfig cfg;
        cfg.rest_density = 1400.0f;
        cfg.gas_constant = 3000.0f;
        cfg.viscosity = 10.0f;
        return cfg;
    }
};

/**
 * @brief SPH Kernels (Smoothing Functions)
 */
class SPHKernel {
public:
    // Poly6 kernel for density
    static float poly6(float r, float h) {
        if (r >= h) return 0.0f;
        float h2 = h * h;
        float h9 = h2 * h2 * h2 * h2 * h;
        float diff = h2 - r * r;
        return (315.0f / (64.0f * PI * h9)) * diff * diff * diff;
    }
    
    // Spiky kernel gradient for pressure
    static Vec3 spiky_grad(const Vec3& r_vec, float r, float h) {
        if (r <= 0.0f || r >= h) return Vec3::zero();
        float h6 = h * h * h * h * h * h;
        float coeff = -(45.0f / (PI * h6)) * (h - r) * (h - r) / r;
        return r_vec * coeff;
    }
    
    // Viscosity kernel Laplacian
    static float viscosity_laplacian(float r, float h) {
        if (r >= h) return 0.0f;
        float h6 = h * h * h * h * h * h;
        return (45.0f / (PI * h6)) * (h - r);
    }
};

/**
 * @brief SPH Solver
 */
class SPHSolver {
public:
    SPHConfig config;
    std::vector<SPHParticle> particles;
    
    SPHSolver() {}
    
    /**
     * @brief Initialize particles in a cube region
     */
    void init_cube(const Vec3& min, const Vec3& max, float spacing, float mass = 1.0f) {
        particles.clear();
        
        for (float x = min.x; x <= max.x; x += spacing) {
            for (float y = min.y; y <= max.y; y += spacing) {
                for (float z = min.z; z <= max.z; z += spacing) {
                    SPHParticle p;
                    p.position = Vec3(x, y, z);
                    p.mass = mass;
                    particles.push_back(p);
                }
            }
        }
    }
    
    /**
     * @brief Perform one simulation step
     */
    void step() {
        compute_density_pressure();
        compute_forces();
        integrate();
        handle_boundaries();
    }
    
    /**
     * @brief Get particle count
     */
    size_t particle_count() const { return particles.size(); }
    
private:
    void compute_density_pressure() {
        float h = config.smoothing_radius;
        
        for (auto& pi : particles) {
            pi.density = 0.0f;
            
            for (const auto& pj : particles) {
                Vec3 r_vec = pi.position - pj.position;
                float r = r_vec.length();
                pi.density += pj.mass * SPHKernel::poly6(r, h);
            }
            
            // Tait equation for pressure
            float ratio = pi.density / config.rest_density;
            pi.pressure = config.gas_constant * (ratio - 1.0f);
            if (pi.pressure < 0) pi.pressure = 0;  // No negative pressure
        }
    }
    
    void compute_forces() {
        float h = config.smoothing_radius;
        
        for (auto& pi : particles) {
            Vec3 f_pressure = Vec3::zero();
            Vec3 f_viscosity = Vec3::zero();
            
            for (const auto& pj : particles) {
                if (&pi == &pj) continue;
                
                Vec3 r_vec = pi.position - pj.position;
                float r = r_vec.length();
                
                if (r > h) continue;
                
                // Pressure force
                float p_avg = (pi.pressure + pj.pressure) / 2.0f;
                Vec3 grad = SPHKernel::spiky_grad(r_vec, r, h);
                f_pressure += -pj.mass * (p_avg / (pj.density + 0.001f)) * grad;
                
                // Viscosity force
                Vec3 vel_diff = pj.velocity - pi.velocity;
                float lap = SPHKernel::viscosity_laplacian(r, h);
                f_viscosity += config.viscosity * pj.mass * vel_diff * (lap / (pj.density + 0.001f));
            }
            
            // Gravity
            Vec3 f_gravity = config.gravity * pi.density;
            
            // Acceleration
            pi.acceleration = (f_pressure + f_viscosity + f_gravity) / (pi.density + 0.001f);
        }
    }
    
    void integrate() {
        float dt = config.time_step;
        
        for (auto& p : particles) {
            // Symplectic Euler
            p.velocity += p.acceleration * dt;
            p.position += p.velocity * dt;
        }
    }
    
    void handle_boundaries() {
        for (auto& p : particles) {
            // X bounds
            if (p.position.x < config.bound_min.x) {
                p.position.x = config.bound_min.x;
                p.velocity.x *= -config.bound_damping;
            }
            if (p.position.x > config.bound_max.x) {
                p.position.x = config.bound_max.x;
                p.velocity.x *= -config.bound_damping;
            }
            
            // Y bounds
            if (p.position.y < config.bound_min.y) {
                p.position.y = config.bound_min.y;
                p.velocity.y *= -config.bound_damping;
            }
            if (p.position.y > config.bound_max.y) {
                p.position.y = config.bound_max.y;
                p.velocity.y *= -config.bound_damping;
            }
            
            // Z bounds
            if (p.position.z < config.bound_min.z) {
                p.position.z = config.bound_min.z;
                p.velocity.z *= -config.bound_damping;
            }
            if (p.position.z > config.bound_max.z) {
                p.position.z = config.bound_max.z;
                p.velocity.z *= -config.bound_damping;
            }
        }
    }
};

} // namespace fluid
} // namespace basements

#endif // BASEMENTS_FLUID_SPH_H

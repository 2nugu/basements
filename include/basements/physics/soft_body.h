/**
 * @file soft_body.h
 * @brief Data structures for Soft Body Physics (PBD)
 */

#ifndef BASEMENTS_PHYSICS_SOFT_BODY_H
#define BASEMENTS_PHYSICS_SOFT_BODY_H

#include <vector>
#include "basements/core/math/vec3.h"

namespace basements {
namespace physics {

struct Particle {
    math::Vec3 position;
    math::Vec3 prev_position;
    math::Vec3 velocity;
    math::Vec3 force;
    float inv_mass; // 0.0 for fixed particles
    
    Particle() : inv_mass(1.0f) {}
};

struct DistanceConstraint {
    int p1_index;
    int p2_index;
    float rest_length;
    float stiffness; // 0.0 ~ 1.0 (1.0 = Rigid)
    
    DistanceConstraint(int p1, int p2, float len, float stiff = 0.5f) 
        : p1_index(p1), p2_index(p2), rest_length(len), stiffness(stiff) {}
};

struct SoftBody {
    std::vector<Particle> particles;
    std::vector<DistanceConstraint> constraints;
    
    float total_mass = 0.0f;
    
    void add_particle(const math::Vec3& pos, float mass = 1.0f) {
        Particle p;
        p.position = pos;
        p.prev_position = pos;
        p.velocity = math::Vec3(0,0,0);
        p.force = math::Vec3(0,0,0);
        
        if (mass <= 0.0f) {
            p.inv_mass = 0.0f; // Infinite mass
        } else {
            p.inv_mass = 1.0f / mass;
            total_mass += mass;
        }
        
        particles.push_back(p);
    }
    
    void add_constraint(int p1, int p2, float stiffness = 1.0f) {
        if (p1 < 0 || p1 >= particles.size()) return;
        if (p2 < 0 || p2 >= particles.size()) return;
        
        float dist = (particles[p1].position - particles[p2].position).length();
        constraints.emplace_back(p1, p2, dist, stiffness);
    }
    
    // Clear forces after step
    void clear_forces() {
        for (auto& p : particles) {
            p.force = math::Vec3(0,0,0);
        }
    }
};

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_SOFT_BODY_H

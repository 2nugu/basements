/**
 * @file pbd_solver.h
 * @brief Position Based Dynamics Solver
 */

#ifndef BASEMENTS_DYNAMICS_PBD_SOLVER_H
#define BASEMENTS_DYNAMICS_PBD_SOLVER_H

#include "basements/physics/soft_body.h"
#include "basements/core/math/common.h"

namespace basements {
namespace dynamics {

using namespace basements::physics;
using namespace basements::math;

class PBDSolver {
public:
    static void solve(SoftBody& sb, float dt, const Vec3& gravity, int iterations) {
        if (dt <= 0.0f) return;
        
        // 1. Prediction Step (Integration)
        for (auto& p : sb.particles) {
            if (p.inv_mass == 0.0f) continue;
            
            // Apply external forces (Gravity)
            p.velocity += (gravity + p.force * p.inv_mass) * dt;
            
            p.prev_position = p.position;
            p.position += p.velocity * dt;
            
            // Simple Floor Collision (y=0 plane)
            // Just for demonstration/test stability
            if (p.position.y < 0.0f) {
                p.position.y = 0.0f;
                // Friction could be added here
            }
        }
        
        // 2. Constraint Solving
        for (int iter = 0; iter < iterations; ++iter) {
            solve_distance_constraints(sb);
        }
        
        // 3. Velocity Update
        for (auto& p : sb.particles) {
            if (p.inv_mass == 0.0f) continue;
            
            p.velocity = (p.position - p.prev_position) / dt;
            
            // Damping (optional simple linear damping)
            p.velocity *= 0.99f;
        }
    }

private:
    static void solve_distance_constraints(SoftBody& sb) {
        for (const auto& C : sb.constraints) {
            Particle& p1 = sb.particles[C.p1_index];
            Particle& p2 = sb.particles[C.p2_index];
            
            float w1 = p1.inv_mass;
            float w2 = p2.inv_mass;
            float w_sum = w1 + w2;
            
            if (w_sum == 0.0f) continue;
            
            Vec3 delta = p1.position - p2.position;
            float dist = delta.length();
            
            // Avoid division by zero
            if (dist < 1e-6f) continue;
            
            Vec3 n = delta / dist; // Normalized direction
            float correction = (dist - C.rest_length) * C.stiffness; // PBD formula simplification
            
            // XPBD correction would be: C(x) / (w + alpha/dt^2), but simple PBD is fine here with stiffness < 1
            // Position correction magnitudes
            float lambda = correction / w_sum;
            
            Vec3 dx1 = n * (-lambda * w1);
            Vec3 dx2 = n * (lambda * w2);
            
            p1.position += dx1;
            p2.position += dx2;
        }
    }
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_PBD_SOLVER_H

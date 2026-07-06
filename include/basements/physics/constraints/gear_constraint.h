/**
 * @file gear_constraint.h
 * @brief Gear Constraint Implementation
 * 
 * Constrains the angular velocity of two bodies to maintain a specific ratio.
 * C = wA . nA + ratio * wB . nB = 0
 */

#ifndef BASEMENTS_CONSTRAINTS_GEAR_CONSTRAINT_H
#define BASEMENTS_CONSTRAINTS_GEAR_CONSTRAINT_H

#include "basements/physics/rigid_body.h"
#include "basements/core/math/common.h"

namespace basements {
namespace constraints {

using namespace basements::physics;

struct GearConstraint {
    int bodyA_index;
    int bodyB_index;
    
    Vec3 axisA; // Rotation axis of gear A (local or world? Usually body-local)
    Vec3 axisB; // Rotation axis of gear B (body-local)
    
    float ratio; // Gear Ratio. If 2.0, A rotates 1 rad => B rotates -2 rad? Or ratio = N_A / N_B?
                 // Standard: dThetaA * nA + ratio * dThetaB * nB = 0
    
    float accumulated_impulse;
    
    // Limits (optional, for rack and pinion limits etc)
    // For now, pure gear.
    
    HOST_DEVICE GearConstraint() 
        : bodyA_index(-1), bodyB_index(-1)
        , axisA(Vec3(0,1,0)), axisB(Vec3(0,1,0))
        , ratio(1.0f), accumulated_impulse(0.0f) {}
};

class GearSolver {
public:
    /**
     * @brief Solves gear constraint for a single iteration
     */
    HOST_DEVICE static void solve(
        GearConstraint& c,
        RigidBody& bA,
        RigidBody& bB,
        float dt
    ) {
        if (bA.inv_mass == 0.0f && bB.inv_mass == 0.0f) return;
        
        // World Space Axes
        // We assume axisA/axisB are given in LOCAL space.
        // Transform to world space.
        Vec3 nA = bA.orientation.rotate(c.axisA);
        Vec3 nB = bB.orientation.rotate(c.axisB);
        
        // Jacobian J = [0, nA, 0, ratio * nB]
        // Velocity constraint: Cdot = wA . nA + ratio * wB . nB
        
        float wA_dot_nA = bA.angular_velocity.dot(nA);
        float wB_dot_nB = bB.angular_velocity.dot(nB);
        
        float Cdot = wA_dot_nA + c.ratio * wB_dot_nB;
        
        // Effective Mass (Rotational only)
        // K = nA^T * Ia_inv * nA + ratio^2 * nB^T * Ib_inv * nB
        
        float termA = nA.dot(bA.get_inv_inertia_world() * nA);
        float termB = nB.dot(bB.get_inv_inertia_world() * nB);
        
        float effective_mass = termA + (c.ratio * c.ratio) * termB;
        
        if (effective_mass < EPSILON) return;
        
        // Positional Correction (Bias) - omitted for velocity-level gear for simplicity
        // But usually needed to prevent drift.
        // For now: pure velocity constraint.
        
        float lambda = -Cdot / effective_mass;
        
        // Accumulate (Warm starting could be added here)
        c.accumulated_impulse += lambda;
        
        // Apply impulses
        // P = J^T * lambda
        // TorqueA = nA * lambda
        // TorqueB = nB * ratio * lambda
        
        Vec3 torqueA = nA * lambda;
        Vec3 torqueB = nB * (c.ratio * lambda);
        
        // Apply to bodies
        if (bA.inv_mass > 0.0f) {
            bA.angular_velocity += bA.get_inv_inertia_world() * torqueA;
        }
        if (bB.inv_mass > 0.0f) {
            bB.angular_velocity += bB.get_inv_inertia_world() * torqueB;
        }
    }
};

} // namespace constraints
} // namespace basements

#endif // BASEMENTS_CONSTRAINTS_GEAR_CONSTRAINT_H

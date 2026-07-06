#ifndef BASEMENTS_DYNAMICS_CONSTRAINTS_H
#define BASEMENTS_DYNAMICS_CONSTRAINTS_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/rigid_body.h"

namespace basements {
namespace dynamics {

using namespace basements::math;
using namespace basements::physics;

// ============================================================================
// Constraint Types
// ============================================================================

enum class ConstraintType : int {
    CONTACT = 0,
    BALL_SOCKET = 1,
    HINGE = 2,
    DISTANCE = 3
};

// ============================================================================
// Constraint Base
// ============================================================================

/**
 * @brief Base structures for solver constraints
 * 
 * For a GPU-style solver, we often use a large array of generic constraints
 * or separate arrays for each type. 
 * We'll define plain structs (POD) for easy memory mapping.
 */

struct SolverBody {
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    float inv_mass;
    Matrix3 inv_inertia;
    // Pointers/indices if needed
    int body_index;
};

struct ContactConstraint {
    int bodyA_index;
    int bodyB_index;
    
    Vec3 normal;           // World space normal (B to A)
    Vec3 contact_point;    // World space contact point (or two points)
    float penetration;     // Depth
    
    // Cached tangent basis (PERFORMANCE FIX)
    Vec3 tangent1;         // First tangent direction
    Vec3 tangent2;         // Second tangent direction
    
    // Solver Cache (Warm starting)
    float accumulated_impulse_normal;
    float accumulated_impulse_tangent[2];
    
    // Jacobson/Effective Mass
    float effective_mass_normal;
    float effective_mass_tangent[2];
    
    // Friction
    float friction;
    float restitution;
    float bias;               // Restitution and position correction bias

    HOST_DEVICE ContactConstraint() 
        : bodyA_index(-1), bodyB_index(-1), 
          penetration(0.0f),
          accumulated_impulse_normal(0.0f), friction(0.5f), restitution(0.0f), bias(0.0f) {
        accumulated_impulse_tangent[0] = 0.0f;
        accumulated_impulse_tangent[1] = 0.0f;
    }
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_CONSTRAINTS_H

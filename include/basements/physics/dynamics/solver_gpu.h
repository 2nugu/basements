#ifndef BASEMENTS_DYNAMICS_SOLVER_GPU_H
#define BASEMENTS_DYNAMICS_SOLVER_GPU_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/matrix3.h"
#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/constraints.h"
#include <cuda_runtime.h>

namespace basements {
namespace dynamics {

using namespace basements::math;
using namespace basements::physics;

/**
 * @brief CUDA Kernel for solving contact constraints in parallel
 * 
 * Uses atomicAdd to apply impulses to rigid bodies, handling race conditions
 * when multiple contacts affect the same body.
 */
__global__ void solve_velocity_kernel(
    ContactConstraint* contacts, int num_contacts,
    RigidBody* bodies, int num_bodies
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_contacts) return;

    ContactConstraint& c = contacts[i];
    
    // Get Body Indices
    int idxA = c.bodyA_index;
    int idxB = c.bodyB_index;
    
    // Load Bodies (Read-Only Copy for Velocity Calculation)
    // We read current velocity state. Using atomics later handles the write.
    // Ideally, we might want to reload fresh velocity for better convergence in iterative solvers,
    // but standard parallel solvers often use the velocity from the start of the iteration 
    // or rely on atomics to effectively serialize updates.
    // Here we read directly from global memory.
    
    RigidBody& bA = bodies[idxA];
    RigidBody& bB = bodies[idxB];

    Vec3 rA = c.contact_point - bA.position;
    Vec3 rB = c.contact_point - bB.position;

    // Relative Velocity
    // v_rel = (vB + wB x rB) - (vA + wA x rA)
    Vec3 vA = bA.linear_velocity + bA.angular_velocity.cross(rA);
    Vec3 vB = bB.linear_velocity + bB.angular_velocity.cross(rB);
    Vec3 vRel = vB - vA;

    // Normal Velocity
    float vn = vRel.dot(c.normal);

    // Normal Impulse Lambda
    // lambda = -M_eff * (J*v + bias)
    // Bias is pre-calculated in pre_solve (restitution + baumgarte)
    float lambda = -c.effective_mass_normal * (vn + c.bias);

    // Clamp accumulated impulse
    float oldImpulse = c.accumulated_impulse_normal;
    float newImpulse = fmaxf(0.0f, oldImpulse + lambda);
    c.accumulated_impulse_normal = newImpulse;
    lambda = newImpulse - oldImpulse;

    // Computes Impulse Vectors
    Vec3 impulse = c.normal * lambda;

    // Apply Impulse Atomically
    // Linear Velocity: v += P * invM
    if (bA.type == BodyType::DYNAMIC) {
        Vec3 dv = impulse * (-bA.inv_mass);
        atomicAdd(&bodies[idxA].linear_velocity.x, dv.x);
        atomicAdd(&bodies[idxA].linear_velocity.y, dv.y);
        atomicAdd(&bodies[idxA].linear_velocity.z, dv.z);
    }
    if (bB.type == BodyType::DYNAMIC) {
        Vec3 dv = impulse * bB.inv_mass;
        atomicAdd(&bodies[idxB].linear_velocity.x, dv.x);
        atomicAdd(&bodies[idxB].linear_velocity.y, dv.y);
        atomicAdd(&bodies[idxB].linear_velocity.z, dv.z);
    }

    // Angular Velocity: w += I_inv * (r x P)
    // This is more complex because reading I_inv requires current orientation.
    // Orientation doesn't change during velocity solve iteration.
    // So we can read I_inv_world safely.
    if (bA.type == BodyType::DYNAMIC) {
        Vec3 torque_impulse = rA.cross(impulse * -1.0f);
        Vec3 dw = bA.get_inv_inertia_world() * torque_impulse;
        atomicAdd(&bodies[idxA].angular_velocity.x, dw.x);
        atomicAdd(&bodies[idxA].angular_velocity.y, dw.y);
        atomicAdd(&bodies[idxA].angular_velocity.z, dw.z);
    }
    if (bB.type == BodyType::DYNAMIC) {
        Vec3 torque_impulse = rB.cross(impulse);
        Vec3 dw = bB.get_inv_inertia_world() * torque_impulse;
        atomicAdd(&bodies[idxB].angular_velocity.x, dw.x);
        atomicAdd(&bodies[idxB].angular_velocity.y, dw.y);
        atomicAdd(&bodies[idxB].angular_velocity.z, dw.z);
    }
    
    // ========================================================================
    // FRICTION (Tangent) Impulse - CRITICAL FIX
    // ========================================================================
    // Use CACHED tangent basis from pre_solve
    for (int j = 0; j < 2; ++j) {
        Vec3 tangent = (j == 0) ? c.tangent1 : c.tangent2;
        
        // Relative velocity along tangent (must use UPDATED velocities)
        Vec3 vA_updated = bodies[idxA].linear_velocity + bodies[idxA].angular_velocity.cross(rA);
        Vec3 vB_updated = bodies[idxB].linear_velocity + bodies[idxB].angular_velocity.cross(rB);
        Vec3 dv = vB_updated - vA_updated;
        float vt = dv.dot(tangent);
        
        // Compute lambda for friction
        float lambda_t = -c.effective_mass_tangent[j] * vt;
        
        // Clamp based on Coulomb friction cone: |lambda_t| <= mu * lambda_n
        float max_friction = c.friction * c.accumulated_impulse_normal;
        float old_impulse = c.accumulated_impulse_tangent[j];
        float new_impulse = fminf(fmaxf(old_impulse + lambda_t, -max_friction), max_friction);
        
        c.accumulated_impulse_tangent[j] = new_impulse;
        lambda_t = new_impulse - old_impulse;
        
        // Apply Friction Impulse Atomically
        Vec3 impulse_t = tangent * lambda_t;
        
        if (bA.type == BodyType::DYNAMIC) {
            Vec3 dv_friction = impulse_t * (-bA.inv_mass);
            atomicAdd(&bodies[idxA].linear_velocity.x, dv_friction.x);
            atomicAdd(&bodies[idxA].linear_velocity.y, dv_friction.y);
            atomicAdd(&bodies[idxA].linear_velocity.z, dv_friction.z);
            
            Vec3 torque_friction = rA.cross(impulse_t * -1.0f);
            Vec3 dw_friction = bA.get_inv_inertia_world() * torque_friction;
            atomicAdd(&bodies[idxA].angular_velocity.x, dw_friction.x);
            atomicAdd(&bodies[idxA].angular_velocity.y, dw_friction.y);
            atomicAdd(&bodies[idxA].angular_velocity.z, dw_friction.z);
        }
        
        if (bB.type == BodyType::DYNAMIC) {
            Vec3 dv_friction = impulse_t * bB.inv_mass;
            atomicAdd(&bodies[idxB].linear_velocity.x, dv_friction.x);
            atomicAdd(&bodies[idxB].linear_velocity.y, dv_friction.y);
            atomicAdd(&bodies[idxB].linear_velocity.z, dv_friction.z);
            
            Vec3 torque_friction = rB.cross(impulse_t);
            Vec3 dw_friction = bB.get_inv_inertia_world() * torque_friction;
            atomicAdd(&bodies[idxB].angular_velocity.x, dw_friction.x);
            atomicAdd(&bodies[idxB].angular_velocity.y, dw_friction.y);
            atomicAdd(&bodies[idxB].angular_velocity.z, dw_friction.z);
        }
    }

    // Friction (Tangent) Impulse
    // ... Implement Friction similarly with atomics if needed ...
    // For brevity/stability in this first port, we focus on Normal Impulse.
    // Friction implementations follow the same pattern: Compute tangent, solve lambda_t, atomic apply.
}

/**
 * @brief CUDA Kernel for Integration
 * 
 * Updates Position and Orientation based on Velocity.
 * Independent per body (Embarrassingly Parallel).
 */
__global__ void integrate_kernel(RigidBody* bodies, int num_bodies, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_bodies) return;

    RigidBody& b = bodies[i];
    if (b.type == BodyType::STATIC) return;

    // Integrate Position
    b.position += b.linear_velocity * dt;

    // Integrate Orientation
    // dq/dt = 0.5 * w * q
    Quaternion q_vel(
        b.angular_velocity.x * dt,
        b.angular_velocity.y * dt,
        b.angular_velocity.z * dt,
        0.0f
    );
    Quaternion spin = q_vel * b.orientation;
    
    b.orientation.x += spin.x * 0.5f;
    b.orientation.y += spin.y * 0.5f;
    b.orientation.z += spin.z * 0.5f;
    b.orientation.w += spin.w * 0.5f;
    
    b.orientation.normalize();
}

/**
 * @brief Pre-solve kernel to calculate effective mass
 */
__global__ void pre_solve_kernel(
    ContactConstraint* contacts, int num_contacts,
    RigidBody* bodies, int num_bodies,
    float dt
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_contacts) return;

    ContactConstraint& c = contacts[i];
    int idxA = c.bodyA_index;
    int idxB = c.bodyB_index;
    
    RigidBody& bA = bodies[idxA];
    RigidBody& bB = bodies[idxB];
    
    Vec3 rA = c.contact_point - bA.position;
    Vec3 rB = c.contact_point - bB.position;
    
    // Compute and CACHE tangent basis (PERFORMANCE FIX)
    Vec3 t1, t2;
    if (fabsf(c.normal.x) >= 0.57735f) {
        t1 = Vec3(c.normal.y, -c.normal.x, 0.0f);
    } else {
        t1 = Vec3(0.0f, c.normal.z, -c.normal.y);
    }
    t1.normalize();
    t2 = c.normal.cross(t1);
    
    c.tangent1 = t1;
    c.tangent2 = t2;
    
    // Calculate Effective Mass for Normal
    Vec3 rnA = rA.cross(c.normal);
    Vec3 rnB = rB.cross(c.normal);
    // Note: get_inv_inertia_world() is HOST_DEVICE now.
    float angularA = rnA.dot(bA.get_inv_inertia_world() * rnA);
    float angularB = rnB.dot(bB.get_inv_inertia_world() * rnB);
    
    float kNormal = bA.inv_mass + bB.inv_mass + angularA + angularB;
    c.effective_mass_normal = (kNormal > EPSILON) ? 1.0f / kNormal : 0.0f;

    // Calculate Bias (Restitution)
    // Based on INITIAL relative velocity.
    Vec3 vA = bA.linear_velocity + bA.angular_velocity.cross(rA);
    Vec3 vB = bB.linear_velocity + bB.angular_velocity.cross(rB);
    Vec3 vRel = vB - vA;
    float vn = vRel.dot(c.normal);

    c.bias = 0.0f;
    if (vn < -0.1f) {
        c.bias = c.restitution * vn; // Negative bias to enforce bounce
    }
}

class SolverGPU {
public:
    void solve(
        ContactConstraint* d_contacts, int num_contacts,
        RigidBody* d_bodies, int num_bodies,
        float dt,
        int iterations = 10
    ) {
        int threadsPerBlock = 256;
        int blocksContacts = (num_contacts + threadsPerBlock - 1) / threadsPerBlock;
        int blocksBodies = (num_bodies + threadsPerBlock - 1) / threadsPerBlock;

        // 1. Pre-Solve
        pre_solve_kernel<<<blocksContacts, threadsPerBlock>>>(
            d_contacts, num_contacts, d_bodies, num_bodies, dt
        );
        cudaDeviceSynchronize();
        
        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            printf("CUDA Error after PreSolve: %s\n", cudaGetErrorString(err));
        }

        // 2. Sequential Impulse Iterations
        for (int i = 0; i < iterations; ++i) {
            solve_velocity_kernel<<<blocksContacts, threadsPerBlock>>>(
                d_contacts, num_contacts, d_bodies, num_bodies
            );
            // We ideally synchronize between iterations or use a color-graph approach
            // With pure atomics, we can run without sync but convergence might vary.
            // For stability in this implementation:
            cudaDeviceSynchronize(); 
        }

        // 3. Integrate
        integrate_kernel<<<blocksBodies, threadsPerBlock>>>(d_bodies, num_bodies, dt);
        cudaDeviceSynchronize();
    }
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_SOLVER_GPU_H

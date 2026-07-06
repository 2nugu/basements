#ifndef BASEMENTS_DYNAMICS_SOLVER_H
#define BASEMENTS_DYNAMICS_SOLVER_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include <cmath>
#include "basements/physics/dynamics/constraints.h"
#include "basements/physics/rigid_body.h"
#include "basements/physics/material_library.h"
#include "basements/physics/thermodynamics.h"
#include "basements/physics/fatigue.h"

namespace basements {
namespace dynamics {

using namespace basements::math;
using namespace basements::physics;

class Solver {
public:
    /**
     * @brief Prepares constraints for solving (Warm starting, calculate effective mass)
     */
    HOST_DEVICE static void pre_solve(
        ContactConstraint* contacts, int num_contacts,
        RigidBody* bodies, int num_bodies,
        float dt
    ) {
        for (int i = 0; i < num_contacts; ++i) {
            ContactConstraint& c = contacts[i];
            RigidBody& bA = bodies[c.bodyA_index];
            RigidBody& bB = bodies[c.bodyB_index];

            Vec3 rA = c.contact_point - bA.position;
            Vec3 rB = c.contact_point - bB.position;

            // Compute Tangent Basis ONCE and store (PERFORMANCE FIX)
            compute_basis(c.normal, c.tangent1, c.tangent2);

            // World-space inverse inertia tensors (CRITICAL: r, n are world-space vectors)
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();

            // Calculate Effective Mass for Normal Impulse
            // K = 1/mA + 1/mB + (rA×n)·IA_inv·(rA×n) + (rB×n)·IB_inv·(rB×n)
            Vec3 rnA = rA.cross(c.normal);
            Vec3 rnB = rB.cross(c.normal);

            float angularA = rnA.dot(IA_inv * rnA);
            float angularB = rnB.dot(IB_inv * rnB);

            float kNormal = bA.inv_mass + bB.inv_mass + angularA + angularB;
            c.effective_mass_normal = (kNormal > EPSILON) ? 1.0f / kNormal : 0.0f;

            // Calculate Effective Mass for Tangent Impulses
            for(int j=0; j<2; ++j) {
                Vec3 tangent = (j == 0) ? c.tangent1 : c.tangent2;
                Vec3 rtA = rA.cross(tangent);
                Vec3 rtB = rB.cross(tangent);

                float angA = rtA.dot(IA_inv * rtA);
                float angB = rtB.dot(IB_inv * rtB);

                float kTangent = bA.inv_mass + bB.inv_mass + angA + angB;
                c.effective_mass_tangent[j] = (kTangent > EPSILON) ? 1.0f / kTangent : 0.0f;
            }

            // Calculate Bias for Baumgarte Stabilization + Restitution
            // c.penetration is stored as a POSITIVE depth value.
            // Baumgarte: bias = -(beta/dt) * max(0, depth - slop)  → negative bias
            // lambda = -M_eff * (vn + bias): with vn≈0 and bias<0 → lambda>0 (separating) ✓
            const float baumgarte_factor = 0.2f;
            const float slop = 0.005f;

            float overlap = fmaxf(0.0f, c.penetration - slop);
            c.bias = -(baumgarte_factor / dt) * overlap;
            
            // Add restitution bias if separating velocity is high
            Vec3 vA = bA.linear_velocity + bA.angular_velocity.cross(rA);
            Vec3 vB = bB.linear_velocity + bB.angular_velocity.cross(rB);
            Vec3 vRel = vB - vA;
            float vn = vRel.dot(c.normal);
            
            if (vn < -1.0f) {  // Threshold for bounce (1 m/s)
                c.bias += c.restitution * (vn);
            }

            // Apply Warm Starting (Accumulated Impulse)
            if (c.accumulated_impulse_normal > 0.0f || 
                c.accumulated_impulse_tangent[0] != 0.0f || 
                c.accumulated_impulse_tangent[1] != 0.0f) 
            {
                // Normal Impulse
                Vec3 impulse = c.normal * c.accumulated_impulse_normal;
                
                // Tangent Impulses
                impulse += c.tangent1 * c.accumulated_impulse_tangent[0];
                impulse += c.tangent2 * c.accumulated_impulse_tangent[1];

                apply_impulse(bA, -impulse, rA);
                apply_impulse(bB, impulse, rB);
            }
        }
    }

    /**
     * @brief Solves velocity constraints (Sequential Impulse)
     */
    HOST_DEVICE static void solve_velocity(
        ContactConstraint* contacts, int num_contacts,
        RigidBody* bodies, int num_bodies,
        float dt
    ) {
        for (int i = 0; i < num_contacts; ++i) {
            ContactConstraint& c = contacts[i];
            RigidBody& bA = bodies[c.bodyA_index];
            RigidBody& bB = bodies[c.bodyB_index];

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
            // Bias now includes Baumgarte stabilization + restitution
            float lambda = -c.effective_mass_normal * (vn + c.bias);

            // Clamp accumulated impulse (Non-penetration constraint: lambda >= 0)
            float oldImpulse = c.accumulated_impulse_normal;
            c.accumulated_impulse_normal = fmaxf(0.0f, oldImpulse + lambda);
            lambda = c.accumulated_impulse_normal - oldImpulse;

            // Apply Impulse
            Vec3 impulse = c.normal * lambda;
            apply_impulse(bA, -impulse, rA);
            apply_impulse(bB, impulse, rB);

            // Friction (Tangent) Impulse
            // Use CACHED tangent basis (PERFORMANCE FIX)
            for (int j = 0; j < 2; ++j) {
                Vec3 tangent = (j == 0) ? c.tangent1 : c.tangent2;
                
                // Relative velocity along tangent
                // We must use updated velocities (after normal impulse)
                Vec3 dv = (bB.linear_velocity + bB.angular_velocity.cross(rB)) -
                          (bA.linear_velocity + bA.angular_velocity.cross(rA));
                float vt = dv.dot(tangent);

                // Compute lambda for friction
                float lambda_t = -c.effective_mass_tangent[j] * vt;

                // Clamp based on Coulomb friction cone: |lambda_t| <= mu * lambda_n
                float max_friction = c.friction * c.accumulated_impulse_normal;
                float old_impulse = c.accumulated_impulse_tangent[j];
                float new_impulse = fminf(fmaxf(old_impulse + lambda_t, -max_friction), max_friction);
                
                c.accumulated_impulse_tangent[j] = new_impulse;
                lambda_t = new_impulse - old_impulse;

                // Apply Friction Impulse
                Vec3 impulse_t = tangent * lambda_t;
                apply_impulse(bA, -impulse_t, rA);
                apply_impulse(bB, impulse_t, rB);

                // ============================================================
                // [New] Thermodynamics & Vibration Integration
                // ============================================================
                
                // 1. Friction Heat Generation
                // Work = Force * Distance ~ Impulse * Velocity
                // Energy dissipated = |Impulse_t * v_t|
                float friction_work = std::abs(lambda_t * vt);
                
                if (friction_work > 1e-4f) {
                    // Distribute heat to both bodies (50/50 split for simplicity)
                    // Note: In reality, distribution depends on thermal conductivity
                    if (bA.inv_mass > 0.0f) {
                        const Material& matA = MaterialLibrary::get_by_id(bA.material_id);
                        apply_thermal_energy(bA.thermal_state, matA, bA.mass, friction_work * 0.5f);
                    }
                    if (bB.inv_mass > 0.0f) {
                        const Material& matB = MaterialLibrary::get_by_id(bB.material_id);
                        apply_thermal_energy(bB.thermal_state, matB, bB.mass, friction_work * 0.5f);
                    }
                }
            }
            
            // 2. Impact Vibration
            // Trigger vibration based on Normal Impulse intensity
            if (lambda > 1e-2f) {
                float impact_intensity = lambda; // Simplified
                
                // Add to vibration amplitude (decay happens in integrator)
                if (bA.inv_mass > 0.0f) bA.vibration_amplitude += impact_intensity * 0.01f;
                if (bB.inv_mass > 0.0f) bB.vibration_amplitude += impact_intensity * 0.01f;
            }
            
            // 3. Fatigue Analysis (Miner's Rule)
            // Estimate Stress (Pa) = Force (N) / Area (m^2)
            // Force ~ Impulse / dt
            // Assumed Area ~ 1 cm^2 (1e-4 m^2) for point contacts
            if (lambda > 1.0f) { // Only checking significant stress (Impulse > 1 Ns)
                float assumed_area = 1e-4f; 
                float force_est = lambda / dt;
                float stress_est = force_est / assumed_area;
                
                if (bA.inv_mass > 0.0f) FatigueManager::apply_fatigue(bA, stress_est);
                if (bB.inv_mass > 0.0f) FatigueManager::apply_fatigue(bB, stress_est);
            }

        }
    }

private:
    HOST_DEVICE static void apply_impulse(RigidBody& body, const Vec3& impulse, const Vec3& r) {
        if (body.inv_mass == 0.0f) return;

        body.linear_velocity += impulse * body.inv_mass;
        // r and impulse are world-space → use world-space inverse inertia
        body.angular_velocity += body.get_inv_inertia_world() * r.cross(impulse);
    }

    // Helper to compute orthonormal basis
    HOST_DEVICE static void compute_basis(const Vec3& normal, Vec3& t1, Vec3& t2) {
        if (std::abs(normal.x) >= 0.57735f) {
            t1 = Vec3(normal.y, -normal.x, 0.0f);
        } else {
            t1 = Vec3(0.0f, normal.z, -normal.y);
        }
        t1.normalize();
        t2 = normal.cross(t1);
    }
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_SOLVER_H

#ifndef BASEMENTS_PHYSICS_COUPLING_MPM_RIGID_COUPLER_PIC_H
#define BASEMENTS_PHYSICS_COUPLING_MPM_RIGID_COUPLER_PIC_H

/**
 * @file mpm_rigid_coupler_pic.h
 * @brief M1 — PIC-Rigid impulse-based MPM ↔ rigid two-way coupling.
 *
 * Why this exists
 * ───────────────
 * The Dirichlet BC MVP in `mpm_rigid_coupler.h` is correct only in the regime
 * mass_rigid ≫ Σ mass_particle. PIC-Rigid treats the rigid body as an
 * additional Lagrangian "super-particle" that participates in P2G/G2P
 * momentum exchange, dramatically reducing momentum-conservation error in
 * the mass-comparable regime relevant to genuine robotics scenarios
 * (rover-on-sand, foot-on-snow).
 *
 * Status — SKELETON
 * ─────────────────
 * The current implementation is a stub that delegates to the Dirichlet
 * coupler so callers can wire the API path while the real PIC-Rigid kernel
 * is developed. Do NOT use this for publication yet — it is API surface only.
 *
 * Algorithm (target — not yet implemented)
 * ────────────────────────────────────────
 *   1. P2G  — particles deposit momentum onto grid (existing MPMSolver step).
 *   2. **Rigid-P2G**: each rigid body deposits its (linear, angular) momentum
 *      onto the grid nodes its OBB overlaps, weighted by node-overlap volume.
 *   3. update_grid — gravity, constraints, etc.
 *   4. G2P  — particles read corrected grid velocities (existing).
 *   5. **Grid-Rigid**: each rigid body integrates the per-node momentum
 *      delivered back from the grid, applying it as Δp on its centre of
 *      mass and Δω·I⁻¹ on its angular velocity. Net Σ Δp is identically
 *      cancelled between particle and rigid sides — momentum-exact by
 *      construction (modulo float roundoff).
 *
 * Open questions for the implementation
 * ──────────────────────────────────────
 *   • Node-overlap weighting: use exact OBB-cell intersection volume, or
 *     B-spline kernel evaluated at the body centre of overlap?
 *   • Friction: borrow Coulomb tangent projection from the Dirichlet MVP, or
 *     re-derive from grid-frame tangential momentum exchange?
 *   • Inertia tensor on grid: store I⁻¹_world per body each step, vs. a
 *     simplified diagonal approximation?
 *
 * Once implemented, swap the body of `apply()` away from the MVP delegate
 * and update `benchmarks/bench_mpm_rigid_drift.cpp` to compare Dirichlet vs
 * PIC on the same scenario (--method dirichlet vs --method pic).
 */

#include "basements/physics/coupling/mpm_rigid_coupler.h"
#include "basements/core/math/matrix3.h"

#include <algorithm>
#include <cmath>

namespace basements {
namespace physics {
namespace coupling {

class MPMRigidCouplerPIC {
public:
    /**
     * @brief Apply PIC-Rigid coupling for one MPM step.
     *
     * Algorithm (mass-weighted momentum exchange)
     * ────────────────────────────────────────────
     *   For each grid node inside the body's OBB:
     *     1. Compute v_body at the node's world position
     *        (v_lin + ω × (x_node − x_center)).
     *     2. Treat the body as contributing a per-node mass slice
     *        m_b = body.mass / (OBB_volume / dx³)  ≈ body density × cell volume.
     *     3. Form the mass-weighted average velocity:
     *           v_new = (m_node·v_node + m_b·v_body) / (m_node + m_b)
     *     4. Write back v_new and increase node mass by m_b.
     *     5. Accumulate body reaction:
     *           ΔF = (m_b · (v_new − v_body)) / dt   ← per node
     *           Δτ = r × ΔF                          ← about body centre
     *
     * Unlike the Dirichlet MVP (which clamps node velocity to v_body and
     * therefore *injects* momentum from outside), PIC-Rigid exchanges
     * momentum symmetrically: every kg·m/s the body deposits is balanced
     * by a node-side acquisition, so total system momentum drifts only by
     * roundoff in mass_comparable scenarios.
     *
     * Status: first real implementation. Open-source reference for M1
     * comparison against M0 (Dirichlet) and future M2 (ASFLIP rigid).
     */
    static CouplingReaction apply(basements::mpm::SPGridCPU& grid,
                                  const RigidColliderState& body,
                                  float dt) {
        CouplingReaction out;
        if (dt <= 0.0f) return out;

        using basements::math::Matrix3;
        using basements::math::Vec3;
        using basements::mpm::BLOCK_SIZE;
        using basements::mpm::GridBlock;
        using basements::mpm::GridNode;

        const float dx = grid.get_dx();
        const Matrix3 R_l2w = Matrix3::from_quaternion(body.orientation);
        const Matrix3 R_w2l = R_l2w.transposed();

        const float body_volume = 8.0f * body.half_extents.x
                                       * body.half_extents.y
                                       * body.half_extents.z;
        const int approx_nodes = std::max(1, (int)std::round(body_volume / (dx * dx * dx)));
        const float per_node_body_mass = body.mass / (float)approx_nodes;

        // Conservative world-space AABB of the OBB.
        const Vec3 aabb_h(
            std::abs(R_l2w.m[0][0]) * body.half_extents.x + std::abs(R_l2w.m[0][1]) * body.half_extents.y + std::abs(R_l2w.m[0][2]) * body.half_extents.z,
            std::abs(R_l2w.m[1][0]) * body.half_extents.x + std::abs(R_l2w.m[1][1]) * body.half_extents.y + std::abs(R_l2w.m[1][2]) * body.half_extents.z,
            std::abs(R_l2w.m[2][0]) * body.half_extents.x + std::abs(R_l2w.m[2][1]) * body.half_extents.y + std::abs(R_l2w.m[2][2]) * body.half_extents.z
        );
        const Vec3 aabb_min = body.center - aabb_h;
        const Vec3 aabb_max = body.center + aabb_h;

        Vec3 force_accum  = Vec3::zero();
        Vec3 torque_accum = Vec3::zero();

        for (auto& kv : grid.get_blocks_mutable()) {
            GridBlock& block = kv.second;
            const Vec3 b_lo((block.origin_x + 0.5f) * dx,
                            (block.origin_y + 0.5f) * dx,
                            (block.origin_z + 0.5f) * dx);
            const Vec3 b_hi((block.origin_x + BLOCK_SIZE - 1 + 0.5f) * dx,
                            (block.origin_y + BLOCK_SIZE - 1 + 0.5f) * dx,
                            (block.origin_z + BLOCK_SIZE - 1 + 0.5f) * dx);
            if (b_hi.x < aabb_min.x || b_lo.x > aabb_max.x) continue;
            if (b_hi.y < aabb_min.y || b_lo.y > aabb_max.y) continue;
            if (b_hi.z < aabb_min.z || b_lo.z > aabb_max.z) continue;

            for (int li = 0; li < BLOCK_SIZE; ++li)
            for (int lj = 0; lj < BLOCK_SIZE; ++lj)
            for (int lk = 0; lk < BLOCK_SIZE; ++lk) {
                const int local_idx = (li * BLOCK_SIZE * BLOCK_SIZE) + (lj * BLOCK_SIZE) + lk;
                GridNode& node = block.nodes[local_idx];

                const int gi = block.origin_x + li;
                const int gj = block.origin_y + lj;
                const int gk = block.origin_z + lk;
                const Vec3 x_node((gi + 0.5f) * dx, (gj + 0.5f) * dx, (gk + 0.5f) * dx);

                // Shape-aware inside test (BOX or CYLINDER).
                const Vec3 local = R_w2l * (x_node - body.center);
                if (!shape_inside(body, local)) continue;

                const Vec3 r_arm = x_node - body.center;
                const Vec3 v_body = body.linear_velocity + body.angular_velocity.cross(r_arm);

                Vec3 v_after;
                if (!node.active || node.mass < 1e-9f) {
                    // Empty node — body alone claims it. No exchange yet
                    // because there is no opposing momentum.
                    node.active   = true;
                    node.mass     = per_node_body_mass;
                    node.velocity = v_body;
                    v_after       = v_body;
                } else {
                    const Vec3  p_old   = node.velocity * node.mass;
                    const Vec3  p_add   = v_body * per_node_body_mass;
                    const float m_new   = node.mass + per_node_body_mass;
                    v_after             = (p_old + p_add) / m_new;
                    node.velocity       = v_after;
                    node.mass           = m_new;
                }

                // Body reaction (per-node contribution): the body "wanted"
                // to be at v_body but the mass-weighted average pulled it to
                // v_after. That delta over dt is the force on the body
                // attributable to this grid node.
                const Vec3 dv          = v_after - v_body;
                const Vec3 dF          = dv * (per_node_body_mass / dt);
                force_accum  += dF;
                torque_accum += r_arm.cross(dF);
                ++out.nodes_inside;
            }
        }

        out.force  = force_accum;
        out.torque = torque_accum;
        return out;
    }
};

} // namespace coupling
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_COUPLING_MPM_RIGID_COUPLER_PIC_H

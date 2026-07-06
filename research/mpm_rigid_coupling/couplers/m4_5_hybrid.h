#ifndef RESEARCH_MPM_RIGID_M4_5_H
#define RESEARCH_MPM_RIGID_M4_5_H

// M4.5 — Hybrid PIC-Rigid + ASFLIP-tangent + B-spline kernel.
// This is the paper's *novelty candidate*: each layer adds one physics
// assumption back into M1, so the bench can ablate them independently.
//
// Layers (see THEORY.md::M4.5 for derivations):
//   A — adaptive mass weighting (OBB-cell intersection volume)
//   B — Coulomb friction tangent projection (re-introduced from M0)
//   C — quadratic B-spline weight for mass+momentum addition (from M3)
//
// Each variant is its own type so bench_compare_all can list them as
// separate "methods" in the comparison table.

#include "basements/core/math/matrix3.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"

#include <algorithm>
#include <cmath>

namespace research { namespace mpm_rigid {

namespace m45_detail {

using basements::math::Matrix3;
using basements::math::Vec3;
using basements::mpm::BLOCK_SIZE;
using basements::mpm::GridBlock;
using basements::mpm::GridNode;
using basements::physics::coupling::CouplingReaction;
using basements::physics::coupling::RigidColliderState;
using basements::physics::coupling::kInactiveNodeMassEpsilon;

// Shape-aware variants now live in mpm_rigid_coupler.h
// (shape_outward_normal_local / shape_cell_overlap_fraction).
// The thin local wrappers below keep call-sites short.
inline Vec3 obb_outward_normal(const RigidColliderState& b, const Vec3& local) {
    return shape_outward_normal_local(b, local);
}

inline float cell_overlap_fraction(const RigidColliderState& b, const Vec3& local) {
    return shape_cell_overlap_fraction(b, local);
}

// 1D quadratic B-spline kernel; passes through 1 at integer offset 0, 0 at ±1.5.
inline float bspline1d(float xi) {
    float a = std::abs(xi);
    if (a < 0.5f) return 0.75f - a * a;
    if (a < 1.5f) { float t = 1.5f - a; return 0.5f * t * t; }
    return 0.0f;
}

inline float bspline_weight_3d(const Vec3& delta_over_dx) {
    return bspline1d(delta_over_dx.x)
         * bspline1d(delta_over_dx.y)
         * bspline1d(delta_over_dx.z);
}

// Inputs handed to the per-node velocity rule (the user-supplied lambda).
// All fields are pre-computed by sweep so each Layer's rule stays a pure
// function of these — no mutable side-effects allowed inside the lambda.
//
// Always pass by const reference: SweepInput is ~80 bytes and is constructed
// once per inside-OBB grid node (potentially thousands of times per call).
struct SweepInput {
    Vec3       local;          // node position in body-local frame
    Vec3       x_node;         // node position in world frame
    Vec3       r_arm;          // x_node − body.center (for torque computation)
    Vec3       v_body;         // rigid-body velocity at this node
    Matrix3    R_l2w;          // body-to-world rotation
    float      dt;             // time step
    bool       was_active;     // grid node had non-zero mass before this call
    float      old_mass;       // grid node mass before this call (0 if inactive)
    Vec3       old_velocity;   // grid node velocity before this call
    float      m_b;            // normalised body-mass slice (Σ m_b = body.mass)
    // Extension point — set by the caller of `sweep<>` if the Layer needs
    // additional per-call state (e.g. M2 ASFLIP F-bar history). Unowned.
    // Default nullptr keeps existing M0…M4.5 layers ABI-compatible.
    void*      user_data = nullptr;
};

// Per-node rule's return value: the post-coupling velocity for this node
// AND the *effective* body-mass slice actually used (Layers A/B return m_b
// unchanged; Layer C scales it by a kernel weight that intentionally breaks
// the normalised-mass invariant for the ablation).
struct SweepResult {
    Vec3  v_after;
    float m_b_effective;
};

// Two-pass sweep — Round 16 redesign:
//   PASS 1: enumerate every node inside the body's OBB, accumulate
//           the cell-overlap fraction sum S = Σ frac_i.
//   PASS 2: revisit the same nodes; per-node body mass is m_b,i = M·frac_i/S.
//           The sweep handles all node mutations centrally:
//             - node.active = true
//             - node.mass   = old_mass + m_b_effective
//             - node.velocity = v_after (returned by the rule)
//             - force/torque accumulation
//           The per-Layer rule (`compute_v_after`) is therefore a pure
//           function — no node mutation inside the lambda — which structurally
//           prevents the M4.5b/c "forgot to update node.mass" bug class.
template <class VelocityRule>
CouplingReaction sweep(
        basements::mpm::SPGridCPU& grid,
        const RigidColliderState& body,
        float dt,
        VelocityRule compute_v_after)
{
    CouplingReaction out;
    if (dt <= 0.0f) return out;

    const Matrix3 R_l2w = Matrix3::from_quaternion(body.orientation);
    const Matrix3 R_w2l = R_l2w.transposed();
    const float dx = grid.get_dx();

    const Vec3 aabb_h = shape_world_aabb_half(body, R_l2w);
    const Vec3 aabb_min = body.center - aabb_h;
    const Vec3 aabb_max = body.center + aabb_h;

    Vec3 force_accum  = Vec3::zero();
    Vec3 torque_accum = Vec3::zero();

    // ── PASS 1 — accumulate Σ frac_i over inside nodes ─────────────────
    float frac_sum = 0.0f;
    for (auto& kv : grid.get_blocks_mutable()) {
        GridBlock& blk = kv.second;
        Vec3 b_lo((blk.origin_x + 0.5f)*dx, (blk.origin_y + 0.5f)*dx, (blk.origin_z + 0.5f)*dx);
        Vec3 b_hi((blk.origin_x + BLOCK_SIZE-1 + 0.5f)*dx,
                  (blk.origin_y + BLOCK_SIZE-1 + 0.5f)*dx,
                  (blk.origin_z + BLOCK_SIZE-1 + 0.5f)*dx);
        if (b_hi.x < aabb_min.x || b_lo.x > aabb_max.x) continue;
        if (b_hi.y < aabb_min.y || b_lo.y > aabb_max.y) continue;
        if (b_hi.z < aabb_min.z || b_lo.z > aabb_max.z) continue;

        for (int li = 0; li < BLOCK_SIZE; ++li)
        for (int lj = 0; lj < BLOCK_SIZE; ++lj)
        for (int lk = 0; lk < BLOCK_SIZE; ++lk) {
            int gi = blk.origin_x + li;
            int gj = blk.origin_y + lj;
            int gk = blk.origin_z + lk;
            Vec3 x_node((gi + 0.5f)*dx, (gj + 0.5f)*dx, (gk + 0.5f)*dx);
            Vec3 local = R_w2l * (x_node - body.center);
            if (!shape_inside(body, local)) continue;
            frac_sum += cell_overlap_fraction(body, local);
        }
    }
    if (frac_sum < 1e-9f) return out;   // no overlap at all

    const float mass_per_unit_frac = body.mass / frac_sum;

    // ── PASS 2 — update with normalised m_b ───────────────────────────
    for (auto& kv : grid.get_blocks_mutable()) {
        GridBlock& blk = kv.second;
        Vec3 b_lo((blk.origin_x + 0.5f)*dx, (blk.origin_y + 0.5f)*dx, (blk.origin_z + 0.5f)*dx);
        Vec3 b_hi((blk.origin_x + BLOCK_SIZE-1 + 0.5f)*dx,
                  (blk.origin_y + BLOCK_SIZE-1 + 0.5f)*dx,
                  (blk.origin_z + BLOCK_SIZE-1 + 0.5f)*dx);
        if (b_hi.x < aabb_min.x || b_lo.x > aabb_max.x) continue;
        if (b_hi.y < aabb_min.y || b_lo.y > aabb_max.y) continue;
        if (b_hi.z < aabb_min.z || b_lo.z > aabb_max.z) continue;

        for (int li = 0; li < BLOCK_SIZE; ++li)
        for (int lj = 0; lj < BLOCK_SIZE; ++lj)
        for (int lk = 0; lk < BLOCK_SIZE; ++lk) {
            int idx = (li * BLOCK_SIZE * BLOCK_SIZE) + (lj * BLOCK_SIZE) + lk;
            GridNode& node = blk.nodes[idx];

            int gi = blk.origin_x + li;
            int gj = blk.origin_y + lj;
            int gk = blk.origin_z + lk;
            Vec3 x_node((gi + 0.5f)*dx, (gj + 0.5f)*dx, (gk + 0.5f)*dx);
            Vec3 local = R_w2l * (x_node - body.center);
            if (!shape_inside(body, local)) continue;

            float frac = cell_overlap_fraction(body, local);
            float m_b  = mass_per_unit_frac * frac;
            if (m_b <= 0.0f) continue;

            Vec3 r_arm = x_node - body.center;
            Vec3 v_body = body.linear_velocity + body.angular_velocity.cross(r_arm);

            // Snapshot old node state — the rule sees these as `was_active`,
            // `old_mass`, `old_velocity` and is forbidden from touching node.
            const bool  was_active = node.active &&
                                     node.mass >= kInactiveNodeMassEpsilon;
            const float old_mass   = was_active ? node.mass     : 0.0f;
            const Vec3  old_vel    = was_active ? node.velocity : Vec3::zero();

            SweepInput in{ local, x_node, r_arm, v_body,
                           R_l2w, dt, was_active, old_mass, old_vel, m_b };
            SweepResult r = compute_v_after(in, body);

            // Centralised commit — guarantees every Layer leaves the node in
            // a consistent state. Mass-effective m_b is what the Layer "spent"
            // (Layer C may reduce it via a kernel weight).
            node.active   = true;
            node.mass     = old_mass + r.m_b_effective;
            node.velocity = r.v_after;

            // Body reaction = (Δv across exchange) × (effective mass slice) / dt.
            Vec3 dF = (r.v_after - v_body) * (r.m_b_effective / dt);
            force_accum  += dF;
            torque_accum += r_arm.cross(dF);
            ++out.nodes_inside;
        }
    }
    out.force  = force_accum;
    out.torque = torque_accum;
    return out;
}

// Lambdas are pure functions of `SweepInput`. They MUST NOT touch any
// `GridNode` — sweep does that centrally, structurally preventing the
// "forgot node.mass = m_new" bug class.
inline Vec3 mass_weighted_avg(const SweepInput& in) {
    // Empty grid node: body alone defines the post-coupling velocity.
    if (!in.was_active) return in.v_body;
    Vec3  p_old = in.old_velocity * in.old_mass;
    Vec3  p_add = in.v_body * in.m_b;
    float m_new = in.old_mass + in.m_b;
    return (p_old + p_add) / m_new;
}

inline Vec3 apply_friction_tangent(const Vec3& v, const Vec3& v_body,
                                   const Vec3& n_world, float friction) {
    float vn = v.dot(n_world);
    Vec3  vt = v - n_world * vn;
    Vec3  vt_body = v_body - n_world * v_body.dot(n_world);
    Vec3  vt_blended = vt + (vt_body - vt) * std::min(1.0f, friction);
    return n_world * vn + vt_blended;
}

} // namespace m45_detail


// All three layers share the same sweep; only the per-node update logic
// differs. m_b is supplied PRE-NORMALISED by the sweep (Σ m_b,i = M_body).

// ─── Layer A: M1 + normalised adaptive (cell-overlap) mass weighting ──────
class M4_5a {
public:
    static basements::physics::coupling::CouplingReaction apply(
            basements::mpm::SPGridCPU& grid,
            const basements::physics::coupling::RigidColliderState& body,
            float dt)
    {
        using namespace m45_detail;
        return sweep(grid, body, dt,
            [](const SweepInput& in, const RigidColliderState&) -> SweepResult {
                return { mass_weighted_avg(in), in.m_b };
            });
    }
};


// ─── Layer B: A + Coulomb friction tangent projection ──────────────────────
class M4_5b {
public:
    static basements::physics::coupling::CouplingReaction apply(
            basements::mpm::SPGridCPU& grid,
            const basements::physics::coupling::RigidColliderState& body,
            float dt)
    {
        using namespace m45_detail;
        return sweep(grid, body, dt,
            [](const SweepInput& in, const RigidColliderState& b) -> SweepResult {
                Vec3 v_after = mass_weighted_avg(in);
                Vec3 n_world = (in.R_l2w * obb_outward_normal(b, in.local)).normalized();
                v_after = apply_friction_tangent(v_after, in.v_body, n_world, b.friction);
                return { v_after, in.m_b };
            });
    }
};


// ─── Layer C: B + quadratic B-spline kernel (full hybrid) ──────────────────
// Layer C scales the per-node mass slice by a quadratic B-spline kernel —
// intentionally breaks Σ m_b = M_body to quantify the resulting drift
// (Finding 4 of the paper ablation).
class M4_5c {
public:
    static basements::physics::coupling::CouplingReaction apply(
            basements::mpm::SPGridCPU& grid,
            const basements::physics::coupling::RigidColliderState& body,
            float dt)
    {
        using namespace m45_detail;
        const float dx = grid.get_dx();
        return sweep(grid, body, dt,
            [dx](const SweepInput& in, const RigidColliderState& b) -> SweepResult {
                Vec3 dnorm((in.x_node.x - b.center.x) / dx,
                           (in.x_node.y - b.center.y) / dx,
                           (in.x_node.z - b.center.z) / dx);
                float w = bspline_weight_3d(dnorm);
                if (w < 1e-6f) return { in.v_body, 0.0f };
                SweepInput in_k = in;
                in_k.m_b = in.m_b * w;
                Vec3 v_after = mass_weighted_avg(in_k);
                Vec3 n_world = (in.R_l2w * obb_outward_normal(b, in.local)).normalized();
                v_after = apply_friction_tangent(v_after, in.v_body, n_world, b.friction);
                return { v_after, in_k.m_b };
            });
    }
};

// Default M4.5 = full hybrid (Layer C).
using M4_5 = M4_5c;

} } // namespace

#endif

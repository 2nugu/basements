#ifndef BASEMENTS_PHYSICS_COUPLING_MPM_RIGID_COUPLER_H
#define BASEMENTS_PHYSICS_COUPLING_MPM_RIGID_COUPLER_H

/**
 * @file mpm_rigid_coupler.h
 * @brief Two-way MPM ↔ rigid-body coupling primitives.
 *
 * Why this exists
 * ───────────────
 * The Basements engine's headline claim is "From Earthworms to Galaxies" —
 * scale-spanning physics in a single platform. Until now MPM and RigidBody
 * each ran in isolation, which makes that claim hollow: a robot wheel can't
 * roll over a sand pile if particles and rigid bodies don't exchange forces.
 * This header defines the minimal coupling interface so a third-party caller
 * can drive an MPM grid from rigid bodies and read back the reaction force /
 * torque. It is deliberately data-oriented (no inheritance, no callbacks)
 * to keep both worlds independent and testable.
 *
 * Algorithm (MPM-side)
 * ────────────────────
 *   1. After p2g and grid integration (`update_grid`), the grid holds the
 *      hypothetical "next-step" node velocities.
 *   2. For each rigid collider, iterate every active grid node that lies
 *      inside the collider's OBB.
 *   3. Compute the rigid body's velocity at the node's world position:
 *          v_body = v_lin + ω × (x_node - x_center)
 *   4. **Rigid → Particle (Dirichlet BC)**: clamp the grid node's velocity
 *      component along the inward surface normal to v_body's normal component
 *      (no-penetration). Tangential component is partially damped by friction.
 *      The particle then inherits this corrected velocity in g2p.
 *   5. **Particle → Rigid (reaction)**: the per-node momentum change
 *          Δp = m_grid · (v_after - v_before)
 *      is accumulated, with negation, as an external impulse on the rigid
 *      body: F = -ΣΔp / dt, τ = -Σ(x_node - x_center) × Δp / dt.
 *
 * Conservation note
 * ─────────────────
 * This scheme is *not* exactly momentum-conserving — Dirichlet BCs can inject
 * momentum if the rigid body's mass is not part of the system. It is correct
 * in the regime mass_rigid ≫ Σ mass_particles (typical robotics use). For
 * mass-comparable two-way coupling, a Lagrange-multiplier or PIC-rigid
 * coupling scheme is required — out of scope for this MVP.
 */

#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/core/math/matrix3.h"
#include "basements/physics/mpm/spgrid_cpu.h"

#include <cmath>

namespace basements {
namespace physics {
namespace coupling {

// Coupling-wide constants. Named here so every coupler (engine + research)
// reads them from a single source of truth instead of using magic literals.
//
//   kInactiveNodeMassEpsilon — grid mass below this is "inactive": the
//                              sweep treats the node as empty even if its
//                              active flag is set.  1 ng range — far below
//                              any physical sand particle (10 mg).
inline constexpr float kInactiveNodeMassEpsilon = 1e-9f;

// Primitive shape supported by the coupler. BOX is the original / default.
// CYLINDER stores radius in half_extents.x (== half_extents.z), and half-length
// in half_extents.y. The cylinder axis is the body-local Y direction; rotate
// via `orientation` to point it elsewhere. Keeps the struct layout fixed.
enum class ShapeKind : int { BOX = 0, CYLINDER = 1 };

// Snapshot of a rigid body, expressed in MPM-friendly form.
struct RigidColliderState {
    basements::math::Vec3       center           = basements::math::Vec3::zero();
    basements::math::Quaternion orientation      = basements::math::Quaternion::identity();
    basements::math::Vec3       half_extents     {0.5f, 0.5f, 0.5f};
    basements::math::Vec3       linear_velocity  = basements::math::Vec3::zero();
    basements::math::Vec3       angular_velocity = basements::math::Vec3::zero();
    float                       friction         = 0.5f;
    // Body mass — only used by PIC-Rigid (M1) for mass-weighted exchange.
    // M0 Dirichlet ignores it. Defaulted so existing callers stay source-compatible.
    float                       mass             = 1.0f;
    // Shape primitive. Defaults to BOX so existing scenarios stay unchanged.
    ShapeKind                   shape            = ShapeKind::BOX;
};

// ─── Shape probe helpers (inline, shared by every coupler) ─────────────────
// `local` is the world-space query point transformed into body-local space:
//   local = R_world_to_local · (x_world − center)

inline bool shape_inside(const RigidColliderState& b,
                         const basements::math::Vec3& local)
{
    if (b.shape == ShapeKind::CYLINDER) {
        const float R = b.half_extents.x;       // radius
        const float L = b.half_extents.y;       // half-length along local Y
        const float r2 = local.x * local.x + local.z * local.z;
        return std::abs(local.y) <= L && r2 <= R * R;
    }
    // BOX
    return std::abs(local.x) <= b.half_extents.x
        && std::abs(local.y) <= b.half_extents.y
        && std::abs(local.z) <= b.half_extents.z;
}

// Outward normal at the closest face, expressed in body-local frame.
inline basements::math::Vec3 shape_outward_normal_local(
        const RigidColliderState& b,
        const basements::math::Vec3& local)
{
    using basements::math::Vec3;
    if (b.shape == ShapeKind::CYLINDER) {
        const float R = b.half_extents.x;
        const float L = b.half_extents.y;
        const float r = std::sqrt(local.x * local.x + local.z * local.z);
        const float axial_dist  = L - std::abs(local.y);
        const float radial_dist = R - r;
        if (axial_dist < radial_dist) {
            return Vec3(0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
        }
        if (r > 1e-9f) {
            return Vec3(local.x / r, 0.0f, local.z / r);
        }
        return Vec3(1.0f, 0.0f, 0.0f);
    }
    // BOX: closest-axis face (legacy logic).
    const float dx = b.half_extents.x - std::abs(local.x);
    const float dy = b.half_extents.y - std::abs(local.y);
    const float dz = b.half_extents.z - std::abs(local.z);
    if (dx <= dy && dx <= dz) return Vec3(local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
    if (dy <= dz)             return Vec3(0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
    return Vec3(0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f);
}

// Monotone, smooth fraction in [0, 1] that is 1 at the body centre and 0 at
// the surface. Used by the two-pass normalised sweep (M4.5x) to redistribute
// body mass over OBB-cell intersections. The two-pass sweep normalises
// Σ frac_i to M_body, so the exact functional form is *shape-aware monotone*
// rather than true cell-OBB intersection volume.
inline float shape_cell_overlap_fraction(
        const RigidColliderState& b,
        const basements::math::Vec3& local)
{
    auto clamp01 = [](float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); };

    if (b.shape == ShapeKind::CYLINDER) {
        const float R = b.half_extents.x;
        const float L = b.half_extents.y;
        const float r = std::sqrt(local.x * local.x + local.z * local.z);
        const float axial  = 1.0f - std::abs(local.y) / std::max(L, 1e-9f);
        const float radial = 1.0f - r / std::max(R, 1e-9f);
        return clamp01(axial) * clamp01(radial);
    }
    // BOX
    const float ux = clamp01(1.0f - std::abs(local.x) / std::max(b.half_extents.x, 1e-9f));
    const float uy = clamp01(1.0f - std::abs(local.y) / std::max(b.half_extents.y, 1e-9f));
    const float uz = clamp01(1.0f - std::abs(local.z) / std::max(b.half_extents.z, 1e-9f));
    return ux * uy * uz;
}

// World-space AABB half-extent.
//   BOX:      classic |R|·he expansion (each row sum of |R| times he).
//   CYLINDER: axis-aware tight bound — for each world axis i,
//             half_i = |a_i|·L  +  sqrt(1 − a_i²)·R,
//             where a = R_l2w·(0,1,0) is the cylinder axis direction in
//             world coords. This is the exact AABB of a swept disk-with-axis
//             (no corner over-extension that the box-bound would add).
inline basements::math::Vec3 shape_world_aabb_half(
        const RigidColliderState& b,
        const basements::math::Matrix3& R_l2w)
{
    using basements::math::Vec3;
    if (b.shape == ShapeKind::CYLINDER) {
        const float R = b.half_extents.x;
        const float L = b.half_extents.y;
        // axis = R_l2w * (0, 1, 0) = column 1 of R_l2w
        const float ax = R_l2w.m[0][1];
        const float ay = R_l2w.m[1][1];
        const float az = R_l2w.m[2][1];
        // Clamp 1 − a_i² ≥ 0 (float roundoff guard).
        auto perp = [&](float ai) {
            float s = 1.0f - ai * ai;
            return s > 0.0f ? std::sqrt(s) : 0.0f;
        };
        return Vec3(
            std::abs(ax) * L + perp(ax) * R,
            std::abs(ay) * L + perp(ay) * R,
            std::abs(az) * L + perp(az) * R
        );
    }
    // BOX
    const Vec3 he = b.half_extents;
    return Vec3(
        std::abs(R_l2w.m[0][0]) * he.x + std::abs(R_l2w.m[0][1]) * he.y + std::abs(R_l2w.m[0][2]) * he.z,
        std::abs(R_l2w.m[1][0]) * he.x + std::abs(R_l2w.m[1][1]) * he.y + std::abs(R_l2w.m[1][2]) * he.z,
        std::abs(R_l2w.m[2][0]) * he.x + std::abs(R_l2w.m[2][1]) * he.y + std::abs(R_l2w.m[2][2]) * he.z
    );
}

// Reaction load on the rigid body, accumulated from one MPM step's worth of
// grid-node BC corrections. dt-normalized so callers add it as a force/torque.
struct CouplingReaction {
    basements::math::Vec3 force  = basements::math::Vec3::zero();
    basements::math::Vec3 torque = basements::math::Vec3::zero();
    int   nodes_inside           = 0;   // diagnostic: # of grid nodes touched
};

class MPMRigidCoupler {
public:
    // Apply Dirichlet BC + accumulate reaction. Must be called between
    // `update_grid()` and `g2p`. The grid is mutated in place. `dt` must
    // match the MPM step size.
    static CouplingReaction apply(basements::mpm::SPGridCPU& grid,
                                  const RigidColliderState& body,
                                  float dt);

private:
    // World-space position of grid node (i, j, k). Cell-centered: node i lies
    // at (i + 0.5) * dx — matches the MPM p2g/g2p offset convention.
    static basements::math::Vec3 node_world(int i, int j, int k, float dx) {
        return basements::math::Vec3((i + 0.5f) * dx,
                                     (j + 0.5f) * dx,
                                     (k + 0.5f) * dx);
    }
};

} // namespace coupling
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_COUPLING_MPM_RIGID_COUPLER_H

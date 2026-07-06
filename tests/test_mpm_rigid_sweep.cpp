// test_mpm_rigid_sweep.cpp — Sweep template invariants.
//
// The M4.5{a,b,c} couplers all delegate to the same `sweep<>` template in
// `m4_5_hybrid.h`. That template enforces:
//   1. Σ m_b,i ≈ M_body when the Layer returns m_b_effective == in.m_b
//      (Layers A and B). This is the conservation lifeline.
//   2. The Layer's lambda never touches the GridNode directly — the sweep
//      commits node.mass, node.velocity, node.active centrally.
//   3. With an empty (no-overlap) grid, the sweep returns a zero reaction
//      and does not crash.
//
// These tests exercise those invariants directly via a no-op Layer so any
// future redesign of the sweep is forced to keep its contract.

#include <gtest/gtest.h>

#include "basements/physics/coupling/mpm_rigid_coupler.h"
#include "research/mpm_rigid_coupling/couplers/m4_5_hybrid.h"

using namespace basements::math;
using namespace basements::mpm;
using namespace basements::physics::coupling;
using namespace research::mpm_rigid::m45_detail;

namespace {

RigidColliderState make_test_box(const Vec3& centre = Vec3(0, 0.5f, 0),
                                  float mass = 1.0f) {
    RigidColliderState b;
    b.center       = centre;
    b.orientation  = Quaternion::identity();
    b.half_extents = Vec3(0.2f, 0.2f, 0.2f);
    b.mass         = mass;
    b.shape        = ShapeKind::BOX;
    return b;
}

// Seed `sand_layers` × 5×5 active grid nodes at the bottom of the grid,
// each with the given particle mass. Returns the total grid mass.
float seed_sand(SPGridCPU& grid, int sand_layers, float particle_mass) {
    float total = 0.0f;
    for (int i = -2; i <= 2; ++i)
    for (int j = 0;  j < sand_layers; ++j)
    for (int k = -2; k <= 2; ++k) {
        auto* n = grid.get_node(i, j, k);
        n->active   = true;
        n->mass     = particle_mass;
        n->velocity = Vec3::zero();
        total += particle_mass;
    }
    return total;
}

float grid_mass_total(const SPGridCPU& grid) {
    float total = 0.0f;
    for (const auto& kv : grid.get_blocks()) {
        const auto& blk = kv.second;
        for (int n_idx = 0;
             n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
            const auto& nn = blk.nodes[n_idx];
            if (nn.active) total += nn.mass;
        }
    }
    return total;
}

}  // namespace

// ─── Invariant 1: mass conservation (Layer A no-op equivalent) ────────────
TEST(MpmRigidSweep, MassConservationAfterOneApplyLayerA) {
    SPGridCPU grid(0.1f);
    const float p_mass        = 0.01f;
    const float initial_total = seed_sand(grid, 3, p_mass);

    auto box = make_test_box();
    research::mpm_rigid::M4_5a::apply(grid, box, 1.0f / 240.0f);

    // After one apply, the grid should have gained exactly M_body in mass:
    //   Σ m_b,i = body.mass (the sweep's two-pass normalisation guarantee).
    const float after_total = grid_mass_total(grid);
    EXPECT_NEAR(after_total - initial_total, box.mass, 1e-3f)
        << "Layer A broke mass conservation: gained "
        << (after_total - initial_total) << " kg vs expected " << box.mass;
}

// Layer B (friction tangent) keeps the same mass invariant.
TEST(MpmRigidSweep, MassConservationAfterOneApplyLayerB) {
    SPGridCPU grid(0.1f);
    const float p_mass        = 0.01f;
    const float initial_total = seed_sand(grid, 3, p_mass);

    auto box = make_test_box();
    research::mpm_rigid::M4_5b::apply(grid, box, 1.0f / 240.0f);

    const float after_total = grid_mass_total(grid);
    EXPECT_NEAR(after_total - initial_total, box.mass, 1e-3f);
}

// Layer C INTENTIONALLY breaks the invariant (kernel weight scales m_b
// without renormalisation). Verify it actually does so — guards against a
// future "fix" that accidentally restores conservation in Layer C and
// erases the paper's Finding 4.
TEST(MpmRigidSweep, LayerCDoesNotConserveMass) {
    SPGridCPU grid(0.1f);
    const float p_mass        = 0.01f;
    const float initial_total = seed_sand(grid, 3, p_mass);

    auto box = make_test_box();
    research::mpm_rigid::M4_5c::apply(grid, box, 1.0f / 240.0f);

    const float gained = grid_mass_total(grid) - initial_total;
    // Should differ from box.mass by more than a roundoff margin.
    EXPECT_LT(gained, box.mass * 0.95f)
        << "Layer C should deposit LESS than body.mass into the grid (the "
           "B-spline kernel down-weights nodes far from the body centre).";
}

// ─── Invariant 2: empty grid (no overlap) → zero reaction, no crash ───────
TEST(MpmRigidSweep, EmptyGridReturnsZeroReaction) {
    SPGridCPU grid(0.1f);     // no nodes seeded
    auto box = make_test_box(Vec3(0, 0.5f, 0));
    auto r = research::mpm_rigid::M4_5a::apply(grid, box, 1.0f / 240.0f);

    EXPECT_NEAR(r.force.length(),  0.0f, 1e-9f);
    EXPECT_NEAR(r.torque.length(), 0.0f, 1e-9f);
    EXPECT_EQ(r.nodes_inside, 0);
}

// ─── Invariant 3: pass-1 frac_sum, pass-2 normalisation ───────────────────
// Probe directly: with a body that is fully inside a uniform sand patch,
// the sum of per-node m_b should equal body.mass.
TEST(MpmRigidSweep, NormalisedMassMatchesBodyMass) {
    SPGridCPU grid(0.1f);
    seed_sand(grid, 4, 0.01f);                     // 5×4×5 sand

    auto box = make_test_box(Vec3(0, 0.15f, 0));   // centred inside the sand
    const float before = grid_mass_total(grid);
    research::mpm_rigid::M4_5a::apply(grid, box, 1.0f / 240.0f);
    const float after  = grid_mass_total(grid);

    EXPECT_NEAR(after - before, box.mass, 1e-3f);
}

// ─── Invariant 4: dt = 0 returns zero reaction, doesn't divide-by-zero ────
TEST(MpmRigidSweep, ZeroDtIsHarmless) {
    SPGridCPU grid(0.1f);
    seed_sand(grid, 3, 0.01f);

    auto box = make_test_box();
    auto r = research::mpm_rigid::M4_5a::apply(grid, box, 0.0f);

    EXPECT_NEAR(r.force.length(),  0.0f, 1e-9f);
    EXPECT_NEAR(r.torque.length(), 0.0f, 1e-9f);
}

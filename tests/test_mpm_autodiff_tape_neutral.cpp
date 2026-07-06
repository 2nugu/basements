// test_mpm_autodiff_tape_neutral.cpp — Pivot 6 in-place autodiff regression.
//
// Architectural commitment: `step(dt, tape)` with `tape == nullptr` must be
// *bit-exact equivalent* to `step(dt)`. Without this guarantee, the
// "forward-only consumers unaffected" claim of the in-place design breaks
// and external validation submissions (paper #2 §6 community section)
// could see drift from autodiff-side bugs.
//
// This test runs the same scenario twice — once with the forward-only
// overload, once with `step(dt, nullptr)` — and asserts every per-particle
// field is bit-identical at every step. CI must keep this green on every
// commit that touches MPMSolver or mpm_autodiff_tape.h.

#include <gtest/gtest.h>
#include <cstring>

#include "basements/physics/mpm/mpm_solver.h"

using namespace basements::math;
using namespace basements::mpm;

namespace {

// Tiny canonical scenario: 4 particles in a single cell, gravity for N steps.
// Small enough to make bit-exact comparison meaningful; not so trivial that
// stress force never engages.
void build_solver(MPMSolver& solver) {
    solver.initialize(0.1f);
    solver.set_material(1.0e5f, 0.3f, 1500.0f);
    solver.set_plastic_alpha(0.4f);
    solver.set_gravity(Vec3(0, -9.81f, 0));

    // 4 particles slightly offset to break perfect symmetry.
    solver.add_particle(Vec3(0.20f, 0.50f, 0.20f), Vec3::zero(), 0.01f);
    solver.add_particle(Vec3(0.30f, 0.55f, 0.20f), Vec3::zero(), 0.01f);
    solver.add_particle(Vec3(0.20f, 0.50f, 0.30f), Vec3::zero(), 0.01f);
    solver.add_particle(Vec3(0.30f, 0.55f, 0.30f), Vec3(0.01f, 0, 0), 0.01f);
}

// Field-by-field bit-exact equality. `memcmp` is unsafe here because
// Matrix3 has alignment padding (warning C4324) that contains
// uninitialised bytes; the *named* fields are what we care about.
bool bits_equal_f(float x, float y) {
    std::uint32_t bx, by;
    std::memcpy(&bx, &x, sizeof(float));
    std::memcpy(&by, &y, sizeof(float));
    return bx == by;
}

bool bits_equal_vec3(const Vec3& a, const Vec3& b) {
    return bits_equal_f(a.x, b.x)
        && bits_equal_f(a.y, b.y)
        && bits_equal_f(a.z, b.z);
}

bool bits_equal_mat3(const Matrix3& a, const Matrix3& b) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            if (!bits_equal_f(a.m[r][c], b.m[r][c])) return false;
    return true;
}

bool bit_exact_particles(const std::vector<Particle>& a,
                          const std::vector<Particle>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!bits_equal_vec3(a[i].position, b[i].position)) return false;
        if (!bits_equal_vec3(a[i].velocity, b[i].velocity)) return false;
        if (!bits_equal_mat3(a[i].F,        b[i].F))        return false;
        if (!bits_equal_mat3(a[i].C,        b[i].C))        return false;
        if (!bits_equal_f(a[i].mass,        b[i].mass))     return false;
        if (!bits_equal_f(a[i].volume,      b[i].volume))   return false;
        if (a[i].material_id != b[i].material_id)           return false;
    }
    return true;
}

TEST(MPMAutodiffTapeNeutral, ForwardOnlyAndNullTapeAreBitExact) {
    MPMSolver solver_a, solver_b;
    build_solver(solver_a);
    build_solver(solver_b);

    const float dt = 1.0f / 480.0f;
    const int   N  = 50;

    for (int i = 0; i < N; ++i) {
        solver_a.step(dt);               // forward-only overload
        solver_b.step(dt, nullptr);      // opt-in overload with null tape

        const auto& pa = solver_a.get_particles();
        const auto& pb = solver_b.get_particles();
        ASSERT_TRUE(bit_exact_particles(pa, pb))
            << "Bit-exact drift detected at step " << i
            << " — forward-only path is no longer equivalent to step(dt, nullptr)."
            << " Investigate any recent changes to MPMSolver::step or mpm_autodiff_tape.h.";
    }
}

// Negative-control: when a tape is provided, the simulation result must
// still match the forward-only path (the *forward computation* is identical
// — only the *recording side effect* differs). M2 work will extend this
// test to also check tape contents.
TEST(MPMAutodiffTapeNeutral, NonNullTapeDoesNotPerturbForwardResult) {
    MPMSolver solver_a, solver_b;
    build_solver(solver_a);
    build_solver(solver_b);

    AutodiffTape tape;

    const float dt = 1.0f / 480.0f;
    const int   N  = 50;

    for (int i = 0; i < N; ++i) {
        solver_a.step(dt);                 // no tape
        solver_b.step(dt, &tape);          // with tape (M1: not yet recording,
                                            //  but signature must not perturb forward)

        const auto& pa = solver_a.get_particles();
        const auto& pb = solver_b.get_particles();
        ASSERT_TRUE(bit_exact_particles(pa, pb))
            << "Tape-attached step diverged from forward-only at step " << i
            << " — the tape recording side-channel must not alter forward state.";
    }

    // M1: tape is empty because recording wires up in M2. Once M2 lands,
    // expect tape.size() == 4 * N * 3 (particles × steps × {p2g, plast, g2p}).
    EXPECT_EQ(tape.size(), static_cast<std::size_t>(0))
        << "M1 milestone: tape recording not yet wired. "
           "Update this expectation when M2 lands.";
}

} // namespace

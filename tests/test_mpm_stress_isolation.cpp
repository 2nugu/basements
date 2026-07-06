// test_mpm_stress_isolation.cpp — Isolates the constitutive stress loop.
//
// MLS-MPM transfers (mass, momentum, force) from particle to grid. Without
// the *force* term f_i = -(4/dx^2) V_p^0 τ_p (x_i - x_p) w_ip, plastic
// projection on F still happens but the projected stress never produces
// a reaction on neighbouring grid nodes, so sand has no internal yield
// resistance. This test catches that omission.
//
// Setup: two particles stacked vertically inside a single grid cell,
// both pre-strained to F = diag(1, 0.9, 1) (10% compressive Y strain),
// zero gravity. Expected: the compressed column expands — lower particle
// gains -v_y, upper particle gains +v_y. Magnitudes are stress-driven,
// so a stub solver returns ~0 for both.

#include <gtest/gtest.h>

#include "basements/physics/mpm/mpm_solver.h"

using namespace basements::math;
using namespace basements::mpm;

namespace {

TEST(MPMStressIsolation, CompressedColumnExpandsVertically) {
    MPMSolver solver;
    solver.initialize(0.1f);

    // Stiff enough that stress force is well above numerical noise; soft
    // enough that dt = 1/240, dx = 0.1, ρ = 1500 stays inside CFL.
    solver.set_material(/*E=*/1.0e5f, /*nu=*/0.3f, /*density=*/1500.0f);
    solver.set_gravity(Vec3(0, 0, 0));

    // Two particles stacked along Y, both inside the same cell footprint.
    solver.add_particle(Vec3(0.20f, 0.18f, 0.20f), Vec3::zero(), 0.01f);
    solver.add_particle(Vec3(0.20f, 0.22f, 0.20f), Vec3::zero(), 0.01f);

    // 10% compression along Y. F is set directly because we want to isolate
    // the stress→force path, not the path that *produces* F via grad_v.
    auto& particles = solver.get_particles_mutable();
    particles[0].F = Matrix3::scale(Vec3(1.0f, 0.9f, 1.0f));
    particles[1].F = Matrix3::scale(Vec3(1.0f, 0.9f, 1.0f));

    solver.step(1.0f / 240.0f);

    // Newton-3 reaction: compressive τ_yy pushes grid nodes away from the
    // particle along Y. g2p then transfers this back: lower particle ←
    // downward push, upper particle ← upward push.
    const float v_y_lower = particles[0].velocity.y;
    const float v_y_upper = particles[1].velocity.y;

    EXPECT_LT(v_y_lower, -1e-4f)
        << "Lower particle should be pushed downward by stress reaction. "
           "v_y_lower = " << v_y_lower
        << "  (≈0 ⇒ stress force loop is missing).";
    EXPECT_GT(v_y_upper, +1e-4f)
        << "Upper particle should be pushed upward by stress reaction. "
           "v_y_upper = " << v_y_upper
        << "  (≈0 ⇒ stress force loop is missing).";

    // Stress is uniaxial along Y, so X/Z velocities should remain ≈ 0.
    EXPECT_NEAR(particles[0].velocity.x, 0.0f, 1e-4f);
    EXPECT_NEAR(particles[0].velocity.z, 0.0f, 1e-4f);
    EXPECT_NEAR(particles[1].velocity.x, 0.0f, 1e-4f);
    EXPECT_NEAR(particles[1].velocity.z, 0.0f, 1e-4f);
}

// Hydrostatic check: F = (1-ε)·I produces ISOTROPIC outward push.
// All three axis-aligned neighbours should see force of equal magnitude.
TEST(MPMStressIsolation, HydrostaticCompressionIsIsotropic) {
    MPMSolver solver;
    solver.initialize(0.1f);
    solver.set_material(1.0e5f, 0.3f, 1500.0f);
    solver.set_gravity(Vec3(0, 0, 0));

    // Three orthogonal pairs of particles, each pair compressed along its
    // own axis, sharing the same centre cell. Hydrostatic state.
    const Vec3 centre(0.30f, 0.30f, 0.30f);
    const float offset = 0.02f;

    solver.add_particle(centre + Vec3(-offset, 0, 0), Vec3::zero(), 0.01f);
    solver.add_particle(centre + Vec3(+offset, 0, 0), Vec3::zero(), 0.01f);
    solver.add_particle(centre + Vec3(0, -offset, 0), Vec3::zero(), 0.01f);
    solver.add_particle(centre + Vec3(0, +offset, 0), Vec3::zero(), 0.01f);
    solver.add_particle(centre + Vec3(0, 0, -offset), Vec3::zero(), 0.01f);
    solver.add_particle(centre + Vec3(0, 0, +offset), Vec3::zero(), 0.01f);

    auto& particles = solver.get_particles_mutable();
    for (auto& p : particles) {
        p.F = Matrix3::scale(Vec3(0.95f, 0.95f, 0.95f)); // 5% isotropic
    }

    solver.step(1.0f / 240.0f);

    const float vx_minus = particles[0].velocity.x;
    const float vx_plus  = particles[1].velocity.x;
    const float vy_minus = particles[2].velocity.y;
    const float vy_plus  = particles[3].velocity.y;
    const float vz_minus = particles[4].velocity.z;
    const float vz_plus  = particles[5].velocity.z;

    EXPECT_LT(vx_minus, -1e-4f);
    EXPECT_GT(vx_plus,  +1e-4f);
    EXPECT_LT(vy_minus, -1e-4f);
    EXPECT_GT(vy_plus,  +1e-4f);
    EXPECT_LT(vz_minus, -1e-4f);
    EXPECT_GT(vz_plus,  +1e-4f);

    // Isotropy: magnitudes should match within 20% (some kernel asymmetry
    // tolerated because particle offset is finite).
    const float mag_x = 0.5f * (std::abs(vx_minus) + std::abs(vx_plus));
    const float mag_y = 0.5f * (std::abs(vy_minus) + std::abs(vy_plus));
    const float mag_z = 0.5f * (std::abs(vz_minus) + std::abs(vz_plus));
    const float mag_avg = (mag_x + mag_y + mag_z) / 3.0f;

    EXPECT_NEAR(mag_x, mag_avg, 0.20f * mag_avg);
    EXPECT_NEAR(mag_y, mag_avg, 0.20f * mag_avg);
    EXPECT_NEAR(mag_z, mag_avg, 0.20f * mag_avg);
}

} // namespace

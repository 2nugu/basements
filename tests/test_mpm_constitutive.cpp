// test_mpm_constitutive.cpp — Reviewer-grade self-validation of the
// MPM constitutive model. These tests are the CI gate for the paper's
// "calibrated against literature" claim: if any of these regresses, every
// scenario-level measurement downstream becomes suspect.
//
//   A. Zero-strain quiescence:  F = I  ⇒  Σ‖f_i‖ = 0
//   B. Linearity of small strain:  ‖f(2ε)‖ ≈ 2 · ‖f(ε)‖  in elastic regime
//   C. Drucker-Prager projection bound:  after step,  ‖ε̂‖ ≤ -α · tr(ε)

#include <gtest/gtest.h>
#include <cmath>

#include "basements/physics/mpm/mpm_solver.h"
#include "basements/core/math/svd.h"

using namespace basements::math;
using namespace basements::mpm;

namespace {

// Sum of |force| magnitudes across all active grid nodes.
float grid_force_sum(const SPGridCPU& g) {
    float total = 0.0f;
    for (const auto& pair : g.get_blocks()) {
        const auto& block = pair.second;
        for (int i = 0; i < BLOCK_SIZE * BLOCK_SIZE * BLOCK_SIZE; ++i) {
            if (block.nodes[i].active) {
                total += block.nodes[i].force.length();
            }
        }
    }
    return total;
}

// Measure grid force *after p2g but before update_grid*. We approximate by
// running one step at dt → 0 (so the velocity update doesn't matter) and
// summing the force field. Force lives on the grid only between p2g and
// the velocity-update step; after step() returns, force has been folded
// into velocity. So we have to run p2g manually — which is private.
// Workaround: run a step with very small dt and observe |v| change.
// Cleaner: read v_post-step directly and infer force magnitude from
//   v = (f / m) · dt + (...)
// We measure max grid speed proportional to force.
float max_grid_speed(const SPGridCPU& g) {
    float top = 0.0f;
    for (const auto& pair : g.get_blocks()) {
        const auto& block = pair.second;
        for (int i = 0; i < BLOCK_SIZE * BLOCK_SIZE * BLOCK_SIZE; ++i) {
            if (block.nodes[i].active) {
                top = std::max(top, block.nodes[i].velocity.length());
            }
        }
    }
    return top;
}

// ── A. Zero strain ⇒ zero grid force. Fundamental sanity. ─────────────────
TEST(MPMConstitutive, ZeroStrainProducesZeroGridForce) {
    MPMSolver solver;
    solver.initialize(0.1f);
    solver.set_material(1.0e5f, 0.3f, 1500.0f);
    solver.set_gravity(Vec3::zero());

    solver.add_particle(Vec3(0.20f, 0.20f, 0.20f), Vec3::zero(), 0.01f);
    // F default = identity.

    solver.step(1.0f / 240.0f);

    // After step with no strain and no gravity, grid speed should be 0 to
    // numerical precision.
    const float top_speed = max_grid_speed(solver.get_grid());
    EXPECT_LT(top_speed, 1e-6f)
        << "Identity F with no gravity should not produce grid motion, got "
        << top_speed;
}

// ── B. Linear regime: doubling strain doubles peak grid speed. ────────────
TEST(MPMConstitutive, SmallStrainProducesLinearResponse) {
    auto peak_speed_for_strain = [](float eps) -> float {
        MPMSolver solver;
        solver.initialize(0.1f);
        solver.set_material(1.0e5f, 0.3f, 1500.0f);
        solver.set_gravity(Vec3::zero());
        solver.add_particle(Vec3(0.20f, 0.20f, 0.20f), Vec3::zero(), 0.01f);

        auto& particles = solver.get_particles_mutable();
        particles[0].F = Matrix3::scale(Vec3(1.0f, 1.0f + eps, 1.0f));
        solver.step(1.0f / 240.0f);
        return max_grid_speed(solver.get_grid());
    };

    const float v_small = peak_speed_for_strain(+0.005f);
    const float v_large = peak_speed_for_strain(+0.010f);
    ASSERT_GT(v_small, 1e-8f) << "expected nonzero response in elastic regime";

    const float ratio = v_large / v_small;
    // In the linear regime ratio should be 2.0; allow 7% slack for finite
    // strain (ln(1+0.01)/ln(1+0.005) ≈ 1.995, plus PIC/APIC asymmetry).
    EXPECT_NEAR(ratio, 2.0f, 2.0f * 0.07f)
        << "Stress force not linear in small strain: ratio = " << ratio;
}

// ── C. Drucker-Prager projection bound after one step. ────────────────────
TEST(MPMConstitutive, DruckerPragerProjectionRespectsCone) {
    MPMSolver solver;
    solver.initialize(0.1f);
    solver.set_material(1.0e5f, 0.3f, 1500.0f);
    solver.set_plastic_alpha(0.6f);
    solver.set_gravity(Vec3::zero());
    solver.add_particle(Vec3(0.20f, 0.20f, 0.20f), Vec3::zero(), 0.01f);

    auto& particles = solver.get_particles_mutable();
    // Strongly deviatoric strain, well past yield: heavy shear-like pattern.
    particles[0].F = Matrix3::scale(Vec3(1.5f, 0.5f, 1.5f));

    solver.step(1.0f / 240.0f);

    // Decompose post-step F via SVD; check yield-cone inequality.
    //   ‖ε̂‖ ≤ -α · tr(ε)   (compression side, tr(ε) < 0)
    // or  tr(ε) ≥ 0  ⇒ projected to identity (tension cap)
    Matrix3 U_F, V_F;
    Vec3 sigma_F;
    SVD::compute(particles[0].F, U_F, sigma_F, V_F);
    sigma_F.x = std::max(sigma_F.x, 1e-6f);
    sigma_F.y = std::max(sigma_F.y, 1e-6f);
    sigma_F.z = std::max(sigma_F.z, 1e-6f);

    Vec3 eps(std::log(sigma_F.x), std::log(sigma_F.y), std::log(sigma_F.z));
    const float tr_eps = eps.x + eps.y + eps.z;
    Vec3 eps_hat = eps - Vec3(tr_eps / 3.0f);
    const float eps_hat_norm = eps_hat.length();

    const float alpha = 0.6f;
    if (tr_eps > 0.0f) {
        // Tension projection: should have collapsed to identity.
        EXPECT_LT(std::abs(eps.x), 1e-4f);
        EXPECT_LT(std::abs(eps.y), 1e-4f);
        EXPECT_LT(std::abs(eps.z), 1e-4f);
    } else {
        // Klar 2016 eq. 3.1 cone limit includes the bulk/shear kohesion
        // factor (3λ+2μ)/(2μ) — matches mpm_solver.h::apply_plasticity.
        const float E_param      = 1.0e5f;
        const float nu_param     = 0.3f;
        const float mu_param     = E_param / (2.0f * (1.0f + nu_param));
        const float lambda_param = E_param * nu_param
                                   / ((1.0f + nu_param) * (1.0f - 2.0f * nu_param));
        const float kohesion = (3.0f * lambda_param + 2.0f * mu_param)
                               / (2.0f * mu_param);
        const float cone_limit = -alpha * kohesion * tr_eps; // > 0

        // 5% slack: the projection is exact, but particle drift between
        // p2g/g2p adds a small residual via the affine grad_v feedback.
        EXPECT_LE(eps_hat_norm, cone_limit * 1.05f + 1e-4f)
            << "Drucker-Prager cone violated: ‖ε̂‖ = " << eps_hat_norm
            << " > " << cone_limit
            << "  (α=" << alpha << ", kohesion=" << kohesion
            << ", tr(ε)=" << tr_eps << ")";
    }
}

} // namespace

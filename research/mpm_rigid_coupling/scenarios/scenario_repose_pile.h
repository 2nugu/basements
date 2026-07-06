#ifndef RESEARCH_MPM_RIGID_SCENARIO_REPOSE_PILE_H
#define RESEARCH_MPM_RIGID_SCENARIO_REPOSE_PILE_H

// Angle-of-repose scenario — Week 1 (paper + library trust).
//
// Drops a column of MPM particles and lets them settle into a pile under
// gravity + the solver's Drucker-Prager plasticity. The resulting pile's
// surface slope is the "angle of repose"; for dry sand the literature
// value is 30–35° (Wong "Theory of Ground Vehicles"; Klar 2016).
//
// This is the *constitutive* test (no rigid body, no coupler) — it
// validates that our MPM sand parameters produce literature-consistent
// granular behaviour. If the measured angle is outside [25°, 40°], the
// constitutive model is mis-calibrated and every other scenario's
// absolute-number claim becomes suspect.
//
// Output: a CSV `outputs/csv/repose_pile.csv` with per-step pile metrics
// (height, footprint radius, derived angle) and a final
// `repose_pile_summary.csv` (settled angle in degrees, settled time).

#include "basements/physics/mpm/mpm_solver.h"
#include "basements/core/math/vec3.h"

#include <cmath>
#include <vector>

namespace research { namespace mpm_rigid {

struct ScenarioReposePile {
    // Particle column — initial state.
    // Lube et al. 2004 ("Axisymmetric collapses of granular columns")
    // showed runout dynamics depend on H/D ratio; for H/D ≤ 0.74 the inner
    // frustum survives and the outer slope reaches static angle of repose.
    // We use H/D ≈ 0.71 so the measured slope captures repose.
    // W2 refinement: 4× finer grid + matching dt for CFL.
    //   dx = 25 mm gives ~14 cells across the pile (vs 4 at dx=0.1) —
    //   adequate to resolve the slope without astronomical particle count.
    float dx              = 0.025f;
    int   column_nx       = 7;          // → diameter 7×0.0125 = 87.5 mm
    int   column_ny       = 5;          // → height   5×0.0125 = 62.5 mm, H/D = 0.71
    int   column_nz       = 7;
    float particle_spacing = 0.0125f;   // 4× finer than W1
    // Particle mass = ρ_bulk · spacing³.  ρ_bulk = 1500 kg/m³,
    // spacing = 0.0125 m  ⇒  m = 1500 · 1.953e-6 = 2.93e-3 kg/particle.
    float particle_mass    = 2.93e-3f;
    basements::math::Vec3 column_base{0.0f, 0.0125f, 0.0f};

    // Simulation — CFL at E=5e5 needs dt < dx/c_s ≈ dx/21.6 ≈ 1.16 ms.
    float dt              = 1.0f / 960.0f;  // 4× finer than W1
    float total_time      = 2.0f;
    float gravity_g       = 9.81f;
    float ground_y        = 0.0f;

    // Measurement
    float settle_time     = 1.5f;       // smaller scenario settles faster

    int num_steps() const { return (int)(total_time / dt + 0.5f); }
};

// Build the initial particle column. Returns the number of particles.
inline int seed_repose_column(basements::mpm::MPMSolver& solver,
                              const ScenarioReposePile& s) {
    using basements::math::Vec3;
    int n = 0;
    for (int i = 0; i < s.column_nx; ++i)
    for (int j = 0; j < s.column_ny; ++j)
    for (int k = 0; k < s.column_nz; ++k) {
        Vec3 pos = s.column_base + Vec3(
            (i - s.column_nx * 0.5f + 0.5f) * s.particle_spacing,
            j * s.particle_spacing,
            (k - s.column_nz * 0.5f + 0.5f) * s.particle_spacing
        );
        solver.add_particle(pos, Vec3::zero(), s.particle_mass);
        ++n;
    }
    return n;
}

// Compute pile metrics from the particle cloud.
struct ReposePileMetrics {
    float pile_height       = 0.0f;
    float pile_radius       = 0.0f;
    // Two angle estimates (both reported because each is biased differently):
    //   * angle_deg_geom: atan(h_peak / r_outer), overestimates for flat-top piles
    //   * angle_deg_slope: linear-fit to outer-surface profile, more honest
    //     but sensitive to bin-edge noise at low particle counts.
    float angle_deg         = 0.0f;    // alias = angle_deg_slope (existing code)
    float angle_deg_geom    = 0.0f;
    int   particle_count    = 0;
    int   particles_at_rest = 0;       // |v| < 0.01 m/s
};

inline ReposePileMetrics measure_pile(const basements::mpm::MPMSolver& solver,
                                      const ScenarioReposePile& s) {
    ReposePileMetrics m;
    const auto& particles = solver.get_particles();
    m.particle_count = (int)particles.size();
    if (particles.empty()) return m;

    // ── Outer-surface slope fit ────────────────────────────────────────────
    //  1. bin particles by radial distance from the pile axis (ring bins)
    //  2. for each bin take the *top* particle's height — that's the surface
    //  3. linear-fit  y = -tan(θ) · r + h₀  on the descending half
    //     and recover θ = repose angle.
    //
    //  This is more robust than atan(h_peak / r_base) for a column-collapse
    //  pile (which has a flat-topped frustum in the centre).

    const basements::math::Vec3 axis = s.column_base;

    // global max height + at-rest count
    float ymax = particles[0].position.y;
    for (const auto& p : particles) {
        if (p.position.y > ymax) ymax = p.position.y;
        if (p.velocity.length() < 0.01f) ++m.particles_at_rest;
    }
    m.pile_height = ymax - s.ground_y;

    // Bin by radial distance.
    const int   nbins    = 24;
    const float r_max_phys = 1.5f * (s.column_nx * s.particle_spacing);
    const float bin_dr   = r_max_phys / nbins;
    std::vector<float> surface_y(nbins, -1.0f);
    std::vector<float> surface_r(nbins, 0.0f);
    float r_outer = 0.0f;
    for (const auto& p : particles) {
        const float dxh = p.position.x - axis.x;
        const float dzh = p.position.z - axis.z;
        const float r   = std::sqrt(dxh*dxh + dzh*dzh);
        if (r > r_outer) r_outer = r;
        int b = (int)(r / bin_dr);
        if (b < 0 || b >= nbins) continue;
        if (p.position.y > surface_y[b]) {
            surface_y[b] = p.position.y;
            surface_r[b] = r;
        }
    }
    m.pile_radius = r_outer;

    // Find the *crest* bin (highest surface_y) and fit the descent from
    // crest → outer rim. We use linear least-squares  y = -m·r + b.
    int crest_bin = 0;
    for (int b = 1; b < nbins; ++b) {
        if (surface_y[b] > surface_y[crest_bin]) crest_bin = b;
    }
    int   n_fit = 0;
    float sum_r = 0, sum_y = 0, sum_rr = 0, sum_ry = 0;
    for (int b = crest_bin; b < nbins; ++b) {
        if (surface_y[b] < 0.0f) continue;          // empty bin
        if (surface_y[b] < s.ground_y + 0.005f) continue; // floor-only
        sum_r += surface_r[b];
        sum_y += surface_y[b];
        sum_rr += surface_r[b] * surface_r[b];
        sum_ry += surface_r[b] * surface_y[b];
        ++n_fit;
    }
    if (n_fit >= 2) {
        const float denom = (n_fit * sum_rr - sum_r * sum_r);
        if (std::abs(denom) > 1e-9f) {
            const float slope = (n_fit * sum_ry - sum_r * sum_y) / denom;
            m.angle_deg = std::atan(std::abs(slope)) * 180.0f / 3.14159265f;
        }
    }
    // Geometric companion: peak-to-rim ratio.
    if (r_outer > 1e-6f) {
        m.angle_deg_geom = std::atan(m.pile_height / r_outer)
                           * 180.0f / 3.14159265f;
    }
    return m;
}

} } // namespace research::mpm_rigid

#endif

#ifndef RESEARCH_MPM_RIGID_METRICS_H
#define RESEARCH_MPM_RIGID_METRICS_H

// Quantitative metrics tracked per (method, scenario) run.
// Definitions match THEORY.md "Quantitative metrics" section so the paper's
// Section 5 table reads directly from the per-step CSV columns and the
// post-run summary.

#include "basements/core/math/vec3.h"
#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"

#include <chrono>
#include <cmath>
#include <cstdint>

namespace research { namespace mpm_rigid {

// Dwell threshold = 30 % of the body's smallest half-extent. Body-relative
// so it carries meaning across scenarios of different scale (a 0.4 m box
// vs a 0.15 m wheel). Previous absolute 5 cm threshold collapsed dwell to
// 0 for small bodies — see Round 11 follow-up.
inline constexpr float kPenetrationDwellRelThreshold = 0.30f;

struct RunMetrics {
    // ── Momentum / energy conservation (running maxima)
    float max_abs_total_p_y      = 0.0f;    // M_max — kg·m/s
    float sum_abs_total_p_y      = 0.0f;    // for time-average
    float max_abs_energy_drift   = 0.0f;    // |ΔE/E₀|, dimensionless
    float energy_t0              = 0.0f;
    bool  energy_t0_set          = false;

    // ── Cost
    double step_time_us_sum      = 0.0;
    uint64_t step_count          = 0;

    // ── Penetration (sanity + time-resolved)
    float max_penetration_depth  = 0.0f;    // D_max — instantaneous worst (m)
    float sum_penetration_depth  = 0.0f;    // accumulator for mean
    uint64_t dwell_steps         = 0;       // # steps where max penetration > kPenetrationDwellThreshold

    // ── Cross-method comparison: filled in by bench AFTER all runs.
    float ratio_p_y_vs_baseline  = 1.0f;    // max_abs_total_p_y / max_abs_total_p_y(M0)

    // ── Convenience
    float step_time_avg_us() const {
        return step_count ? (float)(step_time_us_sum / (double)step_count) : 0.0f;
    }
    float avg_abs_total_p_y() const {
        return step_count ? (float)(sum_abs_total_p_y / (double)step_count) : 0.0f;
    }
    float avg_penetration_depth() const {
        return step_count ? (float)(sum_penetration_depth / (double)step_count) : 0.0f;
    }
    float dwell_fraction() const {
        return step_count ? (float)((double)dwell_steps / (double)step_count) : 0.0f;
    }

    void observe(float total_p_y, float E_total, float penetration_depth,
                 float min_half_extent, double step_time_us)
    {
        if (!energy_t0_set) {
            energy_t0     = E_total;
            energy_t0_set = true;
        }
        float drift = (std::abs(energy_t0) > 1e-9f)
                    ? std::abs(E_total - energy_t0) / std::abs(energy_t0)
                    : std::abs(E_total - energy_t0);

        float abs_p = std::abs(total_p_y);
        if (abs_p > max_abs_total_p_y)                     max_abs_total_p_y      = abs_p;
        sum_abs_total_p_y += abs_p;
        if (drift              > max_abs_energy_drift)     max_abs_energy_drift   = drift;
        if (penetration_depth  > max_penetration_depth)    max_penetration_depth  = penetration_depth;
        sum_penetration_depth += penetration_depth;
        const float dwell_cut = min_half_extent * kPenetrationDwellRelThreshold;
        if (penetration_depth > dwell_cut) ++dwell_steps;

        step_time_us_sum += step_time_us;
        ++step_count;
    }
};

// Compute the maximum penetration depth of any grid node into the body's OBB.
// Returns 0 if no node is inside.
inline float measure_penetration(
        const basements::mpm::SPGridCPU& grid,
        const basements::physics::coupling::RigidColliderState& body)
{
    using basements::math::Matrix3;
    using basements::math::Vec3;
    using basements::mpm::BLOCK_SIZE;

    Matrix3 R_w2l = Matrix3::from_quaternion(body.orientation).transposed();
    float max_depth = 0.0f;
    const float dx = grid.get_dx();
    for (const auto& kv : grid.get_blocks()) {
        const auto& blk = kv.second;
        for (int li = 0; li < BLOCK_SIZE; ++li)
        for (int lj = 0; lj < BLOCK_SIZE; ++lj)
        for (int lk = 0; lk < BLOCK_SIZE; ++lk) {
            int idx = (li * BLOCK_SIZE * BLOCK_SIZE) + (lj * BLOCK_SIZE) + lk;
            const auto& n = blk.nodes[idx];
            if (!n.active || n.mass <= 0.0f) continue;
            int gi = blk.origin_x + li;
            int gj = blk.origin_y + lj;
            int gk = blk.origin_z + lk;
            Vec3 x_node((gi + 0.5f) * dx, (gj + 0.5f) * dx, (gk + 0.5f) * dx);
            Vec3 local = R_w2l * (x_node - body.center);
            // Penetration = depth into OBB along closest face.
            float dx_pen = body.half_extents.x - std::abs(local.x);
            float dy_pen = body.half_extents.y - std::abs(local.y);
            float dz_pen = body.half_extents.z - std::abs(local.z);
            if (dx_pen < 0 || dy_pen < 0 || dz_pen < 0) continue;
            float pen = std::min({dx_pen, dy_pen, dz_pen});
            if (pen > max_depth) max_depth = pen;
        }
    }
    return max_depth;
}

class StepTimer {
public:
    StepTimer() : t0_(clock::now()) {}
    double elapsed_us() const {
        auto t1 = clock::now();
        return std::chrono::duration<double, std::micro>(t1 - t0_).count();
    }
private:
    using clock = std::chrono::high_resolution_clock;
    clock::time_point t0_;
};

} } // namespace

#endif

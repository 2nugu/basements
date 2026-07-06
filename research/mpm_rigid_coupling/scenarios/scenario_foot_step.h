#ifndef RESEARCH_MPM_RIGID_SCENARIO_FOOT_STEP_H
#define RESEARCH_MPM_RIGID_SCENARIO_FOOT_STEP_H

// Foot-step benchmark — third application axis for the paper.
//
// Geometry: thin flat plate (BOX), 0.20 × 0.04 × 0.20 m (half-extents
// 0.10 × 0.02 × 0.10), mass 1 kg. Released vertically above a sand bed.
// The metric of interest is the *rebound coefficient*
//   e = v_up_peak / |v_down_peak|
// computed from the time series of the body's vertical velocity:
//   • |v_down_peak|  =  maximum downward speed near the impact instant
//   • v_up_peak      =  maximum upward speed in the rebound window after
//                       impact (and before any subsequent re-impact)
//
// For dry sand a real foot-step rebound is small (e ≈ 0.05–0.15) — most
// of the impact energy goes into granular flow, not elastic rebound. Our
// MPM sand will overshoot this because the constitutive model is elastic-
// only (no rate-dependent plasticity), so the absolute value is more
// "qualitative" than calibrated. The per-method ranking, however, is
// what the paper measures.

#include "basements/physics/rigid_body.h"
#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"

namespace research { namespace mpm_rigid {

struct ScenarioFootStep {
    // Sand patch — must be wider than the foot to avoid edge effects.
    float dx              = 0.1f;
    int   grid_extent_i   = 3;        // 7 × 7 footprint
    int   grid_extent_k   = 3;
    int   grid_layers_y   = 3;
    float particle_mass   = 0.01f;

    // Foot — thin flat BOX. half_extents.y is the foot's *thickness/2*, so
    // y = 0.02 means a 4 cm thick plate. This is much thinner than the box
    // in box_drop, which is the whole point: surface-area-to-mass ratio
    // dominates the rebound dynamics differently from a cubical block.
    float foot_mass            = 1.0f;
    basements::math::Vec3 foot_half_extents{0.10f, 0.02f, 0.10f};
    basements::math::Vec3 foot_init_pos    {0.0f, 0.55f, 0.0f};

    // Simulation
    float dt              = 1.0f / 240.0f;
    float total_time      = 2.0f;     // shorter than rover_wheel — interested in first rebound
    float gravity_g       = 9.81f;
    int   frame_every_n   = 5;        // 48 fps
    float settle_time     = 0.05f;    // ignore velocity samples before this

    int num_steps() const { return (int)(total_time / dt + 0.5f); }
};

inline int seed_sand_for_foot(basements::mpm::SPGridCPU& grid,
                              const ScenarioFootStep& s) {
    int n = 0;
    for (int i = -s.grid_extent_i; i <= s.grid_extent_i; ++i)
    for (int j = 0; j < s.grid_layers_y; ++j)
    for (int k = -s.grid_extent_k; k <= s.grid_extent_k; ++k) {
        auto* node = grid.get_node(i, j, k);
        node->active   = true;
        node->mass     = s.particle_mass;
        node->velocity = basements::math::Vec3::zero();
        ++n;
    }
    return n;
}

inline basements::physics::RigidBody make_foot(const ScenarioFootStep& s) {
    using basements::math::Matrix3;
    using basements::math::Vec3;
    basements::physics::RigidBody f;
    f.make_dynamic(s.foot_mass);
    f.position         = s.foot_init_pos;
    f.linear_velocity  = Vec3::zero();
    f.linear_damping   = 0.0f;
    f.angular_damping  = 0.0f;
    const float hx = s.foot_half_extents.x;
    const float hy = s.foot_half_extents.y;
    const float hz = s.foot_half_extents.z;
    const float ax = (2.0f * hx) * (2.0f * hx);
    const float ay = (2.0f * hy) * (2.0f * hy);
    const float az = (2.0f * hz) * (2.0f * hz);
    const float fac = s.foot_mass / 12.0f;
    f.inertia_tensor = Matrix3(fac * (ay + az), 0, 0,
                               0, fac * (ax + az), 0,
                               0, 0, fac * (ax + ay));
    return f;
}

// Running tracker for the rebound coefficient.
// Algorithm:
//   1. Track minimum (most negative) v_y over the simulation → v_down_peak.
//   2. Once v_y first becomes >= 0 after that minimum, track the maximum
//      v_y until v_y becomes <= 0 again or the simulation ends → v_up_peak.
//   3. e = v_up_peak / |v_down_peak|.  Bounded to [0, 2] (cap protects
//      against numerical blow-up; physical e ≤ 1).
class ReboundTracker {
public:
    void observe(float t, float v_y, float settle_time) {
        if (t < settle_time) return;
        // Phase 1: still descending or at peak descent.
        if (state_ == Phase::DESCENT) {
            if (v_y < v_down_peak_) v_down_peak_ = v_y;
            if (v_y >= 0.0f && v_down_peak_ < 0.0f) {
                state_ = Phase::REBOUND;
            }
        }
        if (state_ == Phase::REBOUND) {
            if (v_y > v_up_peak_) v_up_peak_ = v_y;
            if (v_y <= 0.0f && v_up_peak_ > 0.0f) {
                state_ = Phase::DONE;
            }
        }
    }

    float coefficient() const {
        if (v_down_peak_ >= 0.0f) return 0.0f;
        float e = v_up_peak_ / (-v_down_peak_);
        if (e < 0.0f) return 0.0f;
        if (e > 2.0f) return 2.0f;
        return e;
    }
    float v_down_peak() const { return v_down_peak_; }
    float v_up_peak()   const { return v_up_peak_; }

private:
    enum class Phase { DESCENT, REBOUND, DONE };
    Phase state_         = Phase::DESCENT;
    float v_down_peak_   =  0.0f;     // most negative seen
    float v_up_peak_     =  0.0f;     // most positive after first sign change
};

} } // namespace

#endif

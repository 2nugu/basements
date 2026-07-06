#ifndef RESEARCH_MPM_RIGID_SCENARIO_TINE_DRAG_H
#define RESEARCH_MPM_RIGID_SCENARIO_TINE_DRAG_H

// Tine drag — Pivot 5 agricultural scenario.
//
// Models a cultivator tine being dragged horizontally through a packed
// soil bed at constant depth and velocity. Same MPM-rigid coupling
// pattern as `scenario_rover_wheel` (grid-direct sand seeding, no
// particles) so the coupler ablation extends cleanly to agricultural
// soil-machine interaction.
//
// Reported metrics:
//   * draft force F_x         — drag opposing motion (Newton)
//   * specific draught         — F_x / tine width (N/m)
//   * vertical force F_y       — uplift vs settling
// Connects to ASAE EP-291.2 standard draft equations (Wong 1989,
// Godwin 2007) without baking those in — we measure forces directly.

#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"
#include "basements/physics/rigid_body.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"

#include <cmath>

namespace research { namespace mpm_rigid {

struct ScenarioTineDrag {
    // Soil bed (grid extent in cells).
    float dx              = 0.05f;
    int   grid_extent_i_low  = -2;     // x in [-2, 10] × dx = [-0.10, 0.50] m
    int   grid_extent_i_high = 10;
    int   grid_extent_k      =  3;     // z in [-3, 3]  × dx = [-0.15, 0.15] m
    int   grid_layers_y      =  4;     // y in [0, 3]   × dx = [0, 0.20] m

    // Each active node is a "soil chunk" of ρ_bulk · dx³ = 1500·(0.05)³ = 0.1875 kg.
    float particle_mass    = 0.1875f;

    // Tine geometry — rectangular blade, body-Y is depth direction.
    basements::math::Vec3 tine_half_extents{0.005f, 0.05f, 0.04f}; // 10mm × 100mm × 80mm
    float tine_mass        = 0.5f;

    // Initial tine position: blade tip at operating depth, x at start.
    float operating_depth     = 0.06f;     // 6 cm below original surface
    basements::math::Vec3 tine_start_pos{-0.05f, 0.0f, 0.0f};

    // Tine kinematics: hold fixed (kinematic body) and drag along +X.
    // Real-world equivalent: rigid mount on tractor moving at v_x.
    float drag_velocity_x  = 0.20f;        // 0.2 m/s slow-rate quasi-static
    bool  hold_kinematic   = true;         // tine moves on prescribed trajectory

    // Simulation
    float dt              = 1.0f / 240.0f;
    float total_time      = 2.0f;
    float gravity_g       = 9.81f;

    // Measurement begins after this many seconds of warm-up (tine enters bed).
    float measurement_start_time = 0.5f;

    int num_steps() const { return (int)(total_time / dt + 0.5f); }

    // Initial tine y-position so tip reaches operating_depth.
    float tine_center_y_init() const {
        const float surface_y = (grid_layers_y - 1 + 0.5f) * dx;
        return surface_y - operating_depth + tine_half_extents.y;
    }
};

// Populate grid with packed soil bed (grid-direct, no particles).
inline int seed_soil_bed_grid(basements::mpm::SPGridCPU& grid,
                              const ScenarioTineDrag& s) {
    int n = 0;
    for (int i = s.grid_extent_i_low; i <= s.grid_extent_i_high; ++i)
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

// Initial rigid body for the tine (kinematic — constant linear velocity).
inline basements::physics::RigidBody make_tine(const ScenarioTineDrag& s) {
    using basements::math::Vec3;
    using basements::math::Matrix3;
    using basements::math::Quaternion;
    basements::physics::RigidBody tine;
    tine.make_dynamic(s.tine_mass);
    tine.position = basements::math::Vec3(
        s.tine_start_pos.x,
        s.tine_center_y_init(),
        s.tine_start_pos.z
    );
    tine.orientation = Quaternion::identity();
    tine.linear_velocity  = Vec3(s.drag_velocity_x, 0.0f, 0.0f);
    tine.angular_velocity = Vec3::zero();
    // Kinematic-by-protocol: bench driver overrides position each step
    // (we don't integrate the tine — its trajectory is prescribed).
    return tine;
}

// Collider state snapshot for the coupler (box shape).
inline basements::physics::coupling::RigidColliderState
make_tine_collider(const ScenarioTineDrag& s,
                   const basements::physics::RigidBody& tine) {
    basements::physics::coupling::RigidColliderState c;
    c.center           = tine.position;
    c.orientation      = tine.orientation;
    c.half_extents     = s.tine_half_extents;
    c.linear_velocity  = tine.linear_velocity;
    c.angular_velocity = tine.angular_velocity;
    c.friction         = 0.55f;                     // soil-on-steel typical
    c.mass             = s.tine_mass;
    c.shape            = basements::physics::coupling::ShapeKind::BOX;
    return c;
}

struct TineDragMetrics {
    float avg_draft_force_x_n      = 0.0f;
    float avg_vertical_force_y_n   = 0.0f;
    float specific_draught_n_per_m = 0.0f;
    int   samples                  = 0;
};

} } // namespace research::mpm_rigid

#endif

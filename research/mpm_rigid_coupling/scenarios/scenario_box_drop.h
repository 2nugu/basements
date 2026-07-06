#ifndef RESEARCH_MPM_RIGID_SCENARIO_BOX_DROP_H
#define RESEARCH_MPM_RIGID_SCENARIO_BOX_DROP_H

// Canonical box-on-sand drop scenario used across all M0..M4.5 comparisons.
// All numerical parameters live HERE — no magic numbers in the bench harness
// or coupler implementations.

#include "basements/physics/rigid_body.h"
#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"

namespace research { namespace mpm_rigid {

struct ScenarioBoxDrop {
    // Grid (Klar 2016 sand parameters — TODO: lock into material_presets.h)
    float dx              = 0.1f;
    int   grid_extent_i   = 3;        // i, k ∈ [-3..3]  → 7 × 7 footprint
    int   grid_extent_k   = 3;
    int   grid_layers_y   = 3;        // j ∈ [0..2]      → 3 layers
    float particle_mass   = 0.01f;    // per node, settled

    // Box
    float box_mass        = 1.0f;
    basements::math::Vec3 box_half_extents{0.2f, 0.2f, 0.2f};
    basements::math::Vec3 box_init_pos    {0.0f, 0.55f, 0.0f};

    // Simulation
    float dt              = 1.0f / 240.0f;
    float total_time      = 3.0f;
    float gravity_g       = 9.81f;

    // Frame export rate (every N steps → snapshot in frames CSV)
    int   frame_every_n   = 5;        // 240 / 5 = 48 fps — matches rover_wheel

    int num_steps() const {
        return (int)(total_time / dt + 0.5f);
    }
};

// Populate grid with the settled sand patch.
inline int seed_sand(basements::mpm::SPGridCPU& grid, const ScenarioBoxDrop& s) {
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

// Initial rigid body.
inline basements::physics::RigidBody make_box(const ScenarioBoxDrop& s) {
    using basements::math::Vec3;
    using basements::math::Matrix3;
    basements::physics::RigidBody box;
    box.make_dynamic(s.box_mass);
    box.position         = s.box_init_pos;
    box.linear_velocity  = Vec3::zero();
    box.linear_damping   = 0.0f;
    box.angular_damping  = 0.0f;
    float h = s.box_half_extents.y;
    float I = (1.0f / 12.0f) * s.box_mass * (2.0f * h * 2.0f * h);
    box.inertia_tensor = Matrix3(I, 0, 0, 0, I, 0, 0, 0, I);
    return box;
}

} } // namespace

#endif

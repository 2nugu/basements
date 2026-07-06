#ifndef RESEARCH_MPM_RIGID_SCENARIO_ROVER_WHEEL_H
#define RESEARCH_MPM_RIGID_SCENARIO_ROVER_WHEEL_H

// Rover-wheel benchmark — Phase 1.1b (validation scenario).
//
// Geometry: cylinder, axis along body-local Y, radius 0.15 m, half-length
//           0.05 m, mass 1 kg. Released vertically above a settled sand bed.
//           This is a *drop* test for now — true rolling contact (forward
//           torque + slip ratio + Bekker drawbar) is a follow-up extension.
//
// Why drop-first: the immediate goal is to validate the new cylinder
// primitive against the same per-method ablation pipeline used for box_drop.
// Rolling adds (a) angular dynamics + Coriolis terms, (b) tangential slip
// metric, (c) a contact-region-of-interest tracker. Layer that on once the
// cylinder probe is known good.

#include "basements/physics/rigid_body.h"
#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"

namespace research { namespace mpm_rigid {

enum class WheelMode { DROP, ROLL };

struct ScenarioRoverWheel {
    // Sand patch
    float dx              = 0.1f;
    int   grid_extent_i   = 6;        // wider for rolling
    int   grid_extent_k   = 4;
    int   grid_layers_y   = 3;
    float particle_mass   = 0.01f;

    // Wheel — cylinder. half_extents.x = radius, .y = half_length.
    float wheel_mass      = 1.0f;
    float wheel_radius    = 0.15f;
    float wheel_half_len  = 0.05f;
    basements::math::Vec3 wheel_init_pos{0.0f, 0.55f, 0.0f};

    // Mode
    WheelMode mode                       = WheelMode::ROLL;
    float     wheel_init_angular_vel     = 10.0f;   // rad/s about contact axis
    float     wheel_init_linear_vel_fwd  = 0.0f;    // m/s along +X (free rolling test ⇒ 0)
    float     wheel_drive_torque         = 0.0f;    // Nm — sustained forward torque

    // Simulation
    float dt              = 1.0f / 240.0f;
    float total_time      = 3.0f;
    float gravity_g       = 9.81f;
    int   frame_every_n   = 5;        // 240 / 5 = 48 fps — smooth animation

    // Bekker dry-sand parameters (Wong, "Theory of Ground Vehicles", 3rd ed., Table 2.3
    // for "dry sand"; used as a reference baseline NOT a calibrated match to our
    // MPM sand — the comparison is qualitative scaling).
    float bekker_n        = 1.1f;
    float bekker_kc       = 0.99e3f;      // N / m^(n+1)
    float bekker_kphi     = 1528e3f;      // N / m^(n+2)

    int num_steps() const { return (int)(total_time / dt + 0.5f); }

    // Bekker-style closed-form sinkage prediction for the wheel under static
    // load W = m·g. Uses the flat-plate approximation
    //   p = W / (b · L_contact_effective)
    //   p = (kc/b + kphi) z^n
    // with L_contact_effective ≈ 2·R (full wheel base width). Order-of-magnitude
    // reference only.
    float bekker_predicted_sinkage() const {
        const float W = wheel_mass * gravity_g;
        const float b = 2.0f * wheel_half_len;            // wheel width
        const float L = 2.0f * wheel_radius;              // contact length (upper bound)
        const float kk = bekker_kc / b + bekker_kphi;
        const float p_target = W / (b * L);
        return std::pow(p_target / kk, 1.0f / bekker_n);
    }
};

inline int seed_sand_for_wheel(basements::mpm::SPGridCPU& grid,
                               const ScenarioRoverWheel& s) {
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

inline basements::physics::RigidBody make_wheel(const ScenarioRoverWheel& s) {
    using basements::math::Matrix3;
    using basements::math::Quaternion;
    using basements::math::Vec3;
    basements::physics::RigidBody w;
    w.make_dynamic(s.wheel_mass);
    w.position         = s.wheel_init_pos;
    w.linear_damping   = 0.0f;
    w.angular_damping  = 0.0f;

    // ROLL mode: rotate the wheel so its body-local Y (cylinder axis) points
    // along world +Z. This makes the wheel's circular cross-section span
    // the world X-Y plane, ready to roll along world +X.
    //   Rotate 90° about world X axis  →  local Y → world Z.
    if (s.mode == WheelMode::ROLL) {
        w.orientation         = Quaternion::from_axis_angle(Vec3(1, 0, 0), 1.5707963f);
        w.linear_velocity     = Vec3(s.wheel_init_linear_vel_fwd, 0.0f, 0.0f);
        // ω = (0, 0, −ω₀) → bottom contact point moves in −X (correct rolling
        // direction). Right-hand rule check: with thumb along −Z, fingers
        // curl from +Y down to +X, so a point at world (0, −R, 0) (the
        // ground contact) has velocity = ω × r = (0,0,−ω₀)×(0,−R,0)
        //   = (−ω₀·(−R), 0, 0) − wait, that gives +X. Let me redo:
        //   (0,0,−ω₀) × (0,−R,0) = (0·0 − (−ω₀)·(−R), …) = (−ω₀R, …) → −X.
        // So bottom point moves in −X *relative to the wheel centre*. If
        // the wheel translates in +X, the ground point's world velocity is
        // v_x + (−ω₀R). Pure rolling ⇒ v_x − ω₀R = 0 ⇒ v_x = ω₀R > 0.
        // Hence ω = −ω₀ on world-Z produces +X forward rolling.
        w.angular_velocity    = Vec3(0.0f, 0.0f, -s.wheel_init_angular_vel);
    } else {
        w.orientation         = Quaternion::identity();
        w.linear_velocity     = Vec3::zero();
        w.angular_velocity    = Vec3::zero();
    }

    // Inertia of solid cylinder, axis = local Y:
    //   I_yy = 1/2 m r²,  I_xx = I_zz = 1/12 m (3r² + (2L)²)
    const float r2 = s.wheel_radius * s.wheel_radius;
    const float L  = 2.0f * s.wheel_half_len;
    const float Iyy = 0.5f * s.wheel_mass * r2;
    const float Ixx = (1.0f / 12.0f) * s.wheel_mass * (3.0f * r2 + L * L);
    w.inertia_tensor = Matrix3(Ixx, 0, 0,
                               0, Iyy, 0,
                               0, 0, Ixx);
    return w;
}

// Slip ratio s = 1 − v_forward / (ω·R). 0 = pure rolling, 1 = full slip,
// >1 = wheel translates faster than rolling would (over-driven), <0 =
// wheel translates backward despite forward spin.
// Sign convention: with ω = (0, 0, −ω₀) the forward rolling direction is
// +X (see make_wheel comment). We therefore use the magnitude |ω_z| · R
// as the "tip speed" denominator and the signed v_x as the numerator.
inline float slip_ratio(const basements::physics::RigidBody& w, float R) {
    const float v_fwd = w.linear_velocity.x;
    const float tip   = std::abs(w.angular_velocity.z) * R;
    if (tip < 1e-6f) return 0.0f;
    return 1.0f - v_fwd / tip;
}

// Helper to populate a RigidColliderState from the wheel's RigidBody for a
// CYLINDER coupling call. half_extents packs (radius, half_length, radius).
inline basements::physics::coupling::RigidColliderState
make_wheel_collider(const ScenarioRoverWheel& s,
                    const basements::physics::RigidBody& w) {
    basements::physics::coupling::RigidColliderState c;
    c.center           = w.position;
    c.orientation      = w.orientation;
    c.half_extents     = basements::math::Vec3(s.wheel_radius,
                                               s.wheel_half_len,
                                               s.wheel_radius);
    c.linear_velocity  = w.linear_velocity;
    c.angular_velocity = w.angular_velocity;
    c.friction         = 0.5f;
    c.mass             = w.mass;
    c.shape            = basements::physics::coupling::ShapeKind::CYLINDER;
    return c;
}

} } // namespace

#endif

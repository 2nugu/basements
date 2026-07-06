// bench_energy_drift.cpp — emits per-step energy/momentum time series as CSVs
// for offline drift analysis.
//
// Three integrator-only scenarios (RigidBody + TimeIntegrator):
//   • free fall                — gravitational accuracy
//   • isolated 2-body          — momentum conservation
//   • damped slider            — monotone dissipation
//
// Two PhysicsWorldGPU scenarios (full solver: GJK/EPA + Sequential Impulse +
// joints + contact caching):
//   • box stack drift          — Baumgarte / warm-start over 5 s
//   • hinge pendulum period    — joint constraint drift over 5 s
//
// Output dir is taken from BASEMENTS_FIGURES_DIR at compile time.
// CSVs are read by scripts/plot_energy_drift.py.

#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/integrator.h"
#include "basements/engine/physics_world_gpu.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

#ifndef BASEMENTS_FIGURES_DIR
#define BASEMENTS_FIGURES_DIR "outputs/figures"
#endif

using basements::math::Matrix3;
using basements::math::Quaternion;
using basements::math::Vec3;
using basements::physics::RigidBody;
using basements::physics::BodyType;
using basements::dynamics::TimeIntegrator;
using basements::engine::BodyHandle;
using basements::engine::ConstraintHandle;
using basements::engine::JointDescriptor;
using basements::engine::JointType;
using basements::engine::PhysicsWorldConfig;
using basements::engine::PhysicsWorldGPU;
using basements::engine::RigidBodyDesc;

namespace {

float kinetic_energy(const RigidBody& b) {
    if (b.type == BodyType::STATIC) return 0.0f;
    Vec3 L = b.inertia_tensor * b.angular_velocity;
    return 0.5f * b.mass * b.linear_velocity.length_squared()
         + 0.5f * b.angular_velocity.dot(L);
}

float potential_energy(const RigidBody& b, float g) {
    if (b.type == BodyType::STATIC) return 0.0f;
    return b.mass * g * b.position.y;
}

Vec3 linear_momentum(const RigidBody& b) {
    return b.linear_velocity * b.mass;
}

Vec3 angular_momentum_about_origin(const RigidBody& b) {
    Vec3 L_spin = b.inertia_tensor * b.angular_velocity;
    Vec3 L_orbit = b.position.cross(b.linear_velocity * b.mass);
    return L_spin + L_orbit;
}

std::string out_path(const char* name) {
    return std::string(BASEMENTS_FIGURES_DIR) + "/" + name;
}

// ─── scenario 1: free fall ──────────────────────────────────────────────────
void scenario_free_fall() {
    RigidBody b; b.make_dynamic(1.0f);
    b.position = Vec3(0.0f, 100.0f, 0.0f);
    b.linear_damping  = 0.0f;
    b.angular_damping = 0.0f;

    const float g  = 9.81f;
    const float dt = 1.0f / 240.0f;
    const int   N  = 240 * 4;   // 4 seconds

    std::string path = out_path("energy_free_fall.csv");
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) { std::fprintf(stderr, "[bench] cannot open %s\n", path.c_str()); return; }
    std::fprintf(f, "t,KE,PE,E_total,py,pz\n");

    for (int i = 0; i < N; ++i) {
        b.apply_force(Vec3(0.0f, -g * b.mass, 0.0f));
        TimeIntegrator::integrate(&b, 1, dt);
        float t  = (i + 1) * dt;
        float KE = kinetic_energy(b);
        float PE = potential_energy(b, g);
        Vec3  p  = linear_momentum(b);
        std::fprintf(f, "%.6f,%.8e,%.8e,%.8e,%.8e,%.8e\n",
                     t, KE, PE, KE + PE, p.y, p.z);
    }
    std::fclose(f);
    std::printf("[bench] wrote %s\n", path.c_str());
}

// ─── scenario 2: isolated 2-body (momentum invariant) ──────────────────────
void scenario_two_body_momentum() {
    std::vector<RigidBody> bodies(2);
    bodies[0].make_dynamic(2.0f);
    bodies[0].position = Vec3(-5.0f, 0.0f, 0.0f);
    bodies[0].linear_velocity = Vec3( 3.0f, 0.0f, 0.5f);
    bodies[0].angular_velocity = Vec3(0.0f, 0.7f, 0.0f);
    bodies[0].linear_damping = 0.0f; bodies[0].angular_damping = 0.0f;

    bodies[1].make_dynamic(1.0f);
    bodies[1].position = Vec3( 5.0f, 0.0f, 0.0f);
    bodies[1].linear_velocity = Vec3(-2.0f, 0.0f, -0.3f);
    bodies[1].angular_velocity = Vec3(0.0f, -1.1f, 0.0f);
    bodies[1].linear_damping = 0.0f; bodies[1].angular_damping = 0.0f;

    const float dt = 1.0f / 240.0f;
    const int   N  = 240 * 2;  // 2 seconds — bodies fly apart, no forces

    std::string path = out_path("momentum_isolated_pair.csv");
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "t,px,py,pz,Lx,Ly,Lz,KE_sys\n");

    for (int i = 0; i < N; ++i) {
        TimeIntegrator::integrate(bodies.data(), (int)bodies.size(), dt);
        float t = (i + 1) * dt;
        Vec3 p_sum   = linear_momentum(bodies[0]) + linear_momentum(bodies[1]);
        Vec3 L_sum   = angular_momentum_about_origin(bodies[0])
                     + angular_momentum_about_origin(bodies[1]);
        float KE_sys = kinetic_energy(bodies[0]) + kinetic_energy(bodies[1]);
        std::fprintf(f, "%.6f,%.8e,%.8e,%.8e,%.8e,%.8e,%.8e,%.8e\n",
                     t, p_sum.x, p_sum.y, p_sum.z,
                     L_sum.x, L_sum.y, L_sum.z, KE_sys);
    }
    std::fclose(f);
    std::printf("[bench] wrote %s\n", path.c_str());
}

// ─── scenario 3: linear damping (monotone dissipation) ──────────────────────
void scenario_damped_slider() {
    RigidBody b; b.make_dynamic(1.0f);
    b.position = Vec3::zero();
    b.linear_velocity = Vec3(10.0f, 0.0f, 0.0f);
    b.linear_damping = 0.5f;
    b.angular_damping = 0.0f;

    const float dt = 1.0f / 240.0f;
    const int   N  = 240 * 3;

    std::string path = out_path("energy_damped_slider.csv");
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "t,KE,vx\n");

    for (int i = 0; i < N; ++i) {
        TimeIntegrator::integrate(&b, 1, dt);
        float t = (i + 1) * dt;
        std::fprintf(f, "%.6f,%.8e,%.8e\n",
                     t, kinetic_energy(b), b.linear_velocity.x);
    }
    std::fclose(f);
    std::printf("[bench] wrote %s\n", path.c_str());
}

} // namespace

// ─── scenario 4: box stack drift via PhysicsWorldGPU ────────────────────────
// 3-box stack on a static ground. Records bottom/middle/top y positions to
// reveal Baumgarte penetration / warm-start residual drift the bare integrator
// scenarios cannot see (they have no contacts).
namespace {
BodyHandle make_pw_box(PhysicsWorldGPU& w, Vec3 pos, Vec3 half,
                       float mass, BodyType type) {
    RigidBodyDesc d;
    d.position    = pos;
    d.half_extents = half;
    d.type        = type;
    if (type == BodyType::STATIC) {
        d.mass = 0.0f;
    } else {
        d.mass = mass;
        float a2 = half.x * half.x * 4.0f;
        float b2 = half.y * half.y * 4.0f;
        float c2 = half.z * half.z * 4.0f;
        float f  = mass / 12.0f;
        d.inertia_tensor = Matrix3(f*(b2+c2), 0, 0,
                                   0, f*(a2+c2), 0,
                                   0, 0, f*(a2+b2));
    }
    d.restitution = 0.0f;
    d.friction    = 0.5f;
    return w.create_body(d);
}
} // namespace

void scenario_solver_stack(float duration_s) {
    PhysicsWorldConfig cfg;
    cfg.fixed_time_step   = 1.0f / 240.0f;
    cfg.solver_iterations = 20;
    cfg.gravity           = Vec3(0.0f, -9.81f, 0.0f);
    PhysicsWorldGPU world(cfg);

    const Vec3 h(0.5f);
    make_pw_box(world, Vec3(0, -0.5f, 0), Vec3(10.0f, 0.5f, 10.0f), 0.0f, BodyType::STATIC);
    BodyHandle b1 = make_pw_box(world, Vec3(0, 0.5f, 0), h, 1.0f, BodyType::DYNAMIC);
    BodyHandle b2 = make_pw_box(world, Vec3(0, 1.5f, 0), h, 1.0f, BodyType::DYNAMIC);
    BodyHandle b3 = make_pw_box(world, Vec3(0, 2.5f, 0), h, 1.0f, BodyType::DYNAMIC);

    const float dt = cfg.fixed_time_step;
    const int   N  = (int)std::round(duration_s / dt);

    std::string path = out_path("solver_stack_drift.csv");
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "t,y1,y2,y3,y1_drift,y2_drift,y3_drift\n");

    for (int i = 0; i < N; ++i) {
        world.step(dt);
        float t  = (i + 1) * dt;
        float y1 = world.get_body_position(b1).y;
        float y2 = world.get_body_position(b2).y;
        float y3 = world.get_body_position(b3).y;
        std::fprintf(f, "%.6f,%.8e,%.8e,%.8e,%.8e,%.8e,%.8e\n",
                     t, y1, y2, y3,
                     y1 - 0.5f, y2 - 1.5f, y3 - 2.5f);
    }
    std::fclose(f);
    std::printf("[bench] wrote %s\n", path.c_str());
}

// ─── scenario 5: hinge pendulum period via PhysicsWorldGPU ───────────────────
// A single hinge pendulum (rod attached to a fixed anchor) released from
// horizontal. Measures angle + system KE+PE to expose joint-constraint drift,
// which the integrator-only scenarios cannot reach.
void scenario_solver_pendulum(float duration_s) {
    PhysicsWorldConfig cfg;
    cfg.fixed_time_step   = 1.0f / 240.0f;
    cfg.solver_iterations = 20;
    cfg.gravity           = Vec3(0.0f, -9.81f, 0.0f);
    PhysicsWorldGPU world(cfg);

    // Static anchor at origin.
    BodyHandle anchor = make_pw_box(world, Vec3(0, 0, 0), Vec3(0.05f), 0.0f, BodyType::STATIC);
    // Rod CoM at (1, 0, 0) — rod extends from anchor (0,0,0) along +x.
    BodyHandle rod    = make_pw_box(world, Vec3(1, 0, 0), Vec3(1.0f, 0.05f, 0.05f),
                                    1.0f, BodyType::DYNAMIC);

    JointDescriptor jd;
    jd.type           = JointType::HINGE;
    jd.body_a         = anchor;
    jd.body_b         = rod;
    jd.local_anchor_a = Vec3::zero();
    jd.local_anchor_b = Vec3(-1.0f, 0.0f, 0.0f);     // pivot at rod's -x end
    jd.local_axis_a   = Vec3(0, 0, 1);
    jd.local_axis_b   = Vec3(0, 0, 1);
    world.create_joint(jd);

    const float dt = cfg.fixed_time_step;
    const int   N  = (int)std::round(duration_s / dt);

    std::string path = out_path("solver_pendulum.csv");
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "t,angle_deg,KE,PE,E_total\n");

    // PE reference = anchor height = 0; PE = m·g·y_com.
    const float g = 9.81f;
    for (int i = 0; i < N; ++i) {
        world.step(dt);
        float t = (i + 1) * dt;

        Vec3 pos = world.get_body_position(rod);
        // Angle of pendulum measured from +x (initial horizontal) toward -y.
        float ang_rad = std::atan2(pos.y, pos.x);
        float ang_deg = ang_rad * 57.29577951f;

        Vec3  lv  = world.get_body_linear_velocity(rod);
        Vec3  av  = world.get_body_angular_velocity(rod);
        float m   = world.get_body_mass(rod);

        // Approximate rotational inertia: rod-along-x, I_z = m/3 · L²  (L=1).
        float Izz = m / 3.0f;
        float KE  = 0.5f * m * lv.dot(lv) + 0.5f * Izz * av.z * av.z;
        float PE  = m * g * pos.y;

        std::fprintf(f, "%.6f,%.8e,%.8e,%.8e,%.8e\n",
                     t, ang_deg, KE, PE, KE + PE);
    }
    std::fclose(f);
    std::printf("[bench] wrote %s\n", path.c_str());
}

int main(int argc, char** argv) {
    bool long_form = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--long-form" || a == "-L") long_form = true;
        else if (a == "--help" || a == "-h") {
            std::printf("Usage: bench_energy_drift [--long-form]\n"
                        "  --long-form   solver scenarios run for 60 s (vs default 5 s)\n");
            return 0;
        }
    }

    const float solver_duration = long_form ? 60.0f : 5.0f;
    std::printf("[bench_energy_drift] dumping to %s  (solver duration: %.1f s)\n",
                BASEMENTS_FIGURES_DIR, solver_duration);

    scenario_free_fall();
    scenario_two_body_momentum();
    scenario_damped_slider();
    scenario_solver_stack(solver_duration);
    scenario_solver_pendulum(solver_duration);
    return 0;
}

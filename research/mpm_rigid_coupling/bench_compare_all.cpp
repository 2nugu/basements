// bench_compare_all.cpp — Runs every (method × scenario) pair, writes a
// uniquely-named CSV per run. Python scripts in scripts/ build the
// figures and videos.
//
// Usage:
//   bench_compare_all.exe --all
//   bench_compare_all.exe --method M1 --scenario box_drop
//   bench_compare_all.exe --list-methods --list-scenarios
//
// Output filenames: outputs/csv/{scenario}_{method}.csv (time series)
//                   outputs/csv/{scenario}_{method}_frames.csv (snapshots)

#include "couplers/m0_dirichlet.h"
#include "couplers/m1_pic_rigid.h"
#include "couplers/m2_asflip.h"
#include "couplers/m3_mls_mpm.h"
#include "couplers/m4_lagrange.h"
#include "couplers/m4_5_hybrid.h"
#include "scenarios/scenario_box_drop.h"
#include "scenarios/scenario_rover_wheel.h"
#include "scenarios/scenario_foot_step.h"
#include "scenarios/scenario_repose_pile.h"
#include "scenarios/scenario_tine_drag.h"
#include "frame_exporter.h"
#include "metrics.h"

#include "basements/physics/mpm/mpm_solver.h"

#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/integrator.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifndef RESEARCH_OUTPUT_DIR
#define RESEARCH_OUTPUT_DIR "research/mpm_rigid_coupling/outputs"
#endif

using basements::math::Vec3;
using basements::physics::RigidBody;
using basements::dynamics::TimeIntegrator;
using basements::physics::coupling::RigidColliderState;
using basements::physics::coupling::CouplingReaction;

namespace research_mr = research::mpm_rigid;

// All six methods exposed by name → function pointer dispatch.
using ApplyFn = CouplingReaction(*)(basements::mpm::SPGridCPU&,
                                    const RigidColliderState&, float);

template <class C>
CouplingReaction apply_adapter(basements::mpm::SPGridCPU& g,
                               const RigidColliderState& b, float dt) {
    return C::apply(g, b, dt);
}

struct MethodEntry { const char* name; ApplyFn fn; };
static const MethodEntry METHODS[] = {
    { "M0",    &apply_adapter<research_mr::M0>    },
    { "M1",    &apply_adapter<research_mr::M1>    },
    { "M2",    &apply_adapter<research_mr::M2>    },
    { "M3",    &apply_adapter<research_mr::M3>    },
    { "M4",    &apply_adapter<research_mr::M4>    },
    // M4.5 ablation ladder — each layer adds one assumption back.
    { "M4.5a", &apply_adapter<research_mr::M4_5a> },  // + adaptive mass
    { "M4.5b", &apply_adapter<research_mr::M4_5b> },  // + friction tangent
    { "M4.5",  &apply_adapter<research_mr::M4_5c> },  // + B-spline kernel
};

static void run_box_drop(const char* method_name, ApplyFn coupler) {
    research_mr::ScenarioBoxDrop S;
    basements::mpm::SPGridCPU grid(S.dx);
    int n_particles = research_mr::seed_sand(grid, S);
    RigidBody box = research_mr::make_box(S);

    char ts_path[256], fr_path[256];
    std::snprintf(ts_path, sizeof(ts_path),
                  "%s/csv/box_drop_%s.csv",        RESEARCH_OUTPUT_DIR, method_name);
    std::snprintf(fr_path, sizeof(fr_path),
                  "%s/csv/box_drop_%s_frames.csv", RESEARCH_OUTPUT_DIR, method_name);
    research_mr::FrameExporter exp(ts_path, fr_path);
    research_mr::RunMetrics metrics;

    const int N = S.num_steps();
    for (int i = 0; i < N; ++i) {
        research_mr::StepTimer step_timer;
        // Gravity on body.
        box.apply_force(Vec3(0.0f, -S.gravity_g * box.mass, 0.0f));

        // Grid momentum (pre).
        Vec3 p_grid_pre = Vec3::zero();
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) p_grid_pre += nn.velocity * nn.mass;
            }
        }

        // Coupling.
        RigidColliderState col;
        col.center           = box.position;
        col.orientation      = box.orientation;
        col.half_extents     = S.box_half_extents;
        col.linear_velocity  = box.linear_velocity;
        col.angular_velocity = box.angular_velocity;
        col.friction         = 0.5f;
        col.mass             = box.mass;
        CouplingReaction r = coupler(grid, col, S.dt);

        box.force  += r.force;
        box.torque += r.torque;
        TimeIntegrator::integrate(&box, 1, S.dt);

        // Apply gravity + floor BC to grid (keeps sand settled).
        for (auto& kv : grid.get_blocks_mutable()) {
            auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                auto& nn = blk.nodes[n_idx];
                if (!nn.active || nn.mass <= 0.0f) continue;
                nn.velocity.y += -S.gravity_g * S.dt;
                if (nn.velocity.y < 0.0f) nn.velocity.y = 0.0f;
            }
        }

        // Grid momentum (post) + scalar log.
        Vec3 p_grid_post = Vec3::zero();
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) p_grid_post += nn.velocity * nn.mass;
            }
        }

        float t        = (i + 1) * S.dt;
        Vec3  p_body   = box.linear_velocity * box.mass;
        float KE_box   = 0.5f * box.mass * box.linear_velocity.length_squared();
        col.center           = box.position;
        col.linear_velocity  = box.linear_velocity;
        col.angular_velocity = box.angular_velocity;
        exp.log_step(t, col, p_body, p_grid_post, KE_box, r.nodes_inside);

        if ((i % S.frame_every_n) == 0) exp.log_frame(t, col, grid);

        // Quantitative metrics — momentum, energy, penetration, step time.
        float E_total = 0.5f * box.mass * box.linear_velocity.length_squared()
                      + box.mass * S.gravity_g * box.position.y;
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active)
                    E_total += 0.5f * nn.mass * nn.velocity.length_squared();
            }
        }
        float pen = research_mr::measure_penetration(grid, col);
        const float min_half = std::min({col.half_extents.x,
                                         col.half_extents.y,
                                         col.half_extents.z});
        double step_us = step_timer.elapsed_us();
        metrics.observe((p_body + p_grid_post).y, E_total, pen, min_half, step_us);

        (void)p_grid_pre;
        (void)n_particles;
    }

    // Write metrics summary alongside the CSV (single line per run).
    char sum_path[256];
    std::snprintf(sum_path, sizeof(sum_path),
                  "%s/csv/box_drop_%s_metrics.csv", RESEARCH_OUTPUT_DIR, method_name);
    if (FILE* mf = std::fopen(sum_path, "w")) {
        std::fprintf(mf,
            "method,max_abs_total_p_y,avg_abs_total_p_y,max_abs_energy_drift,"
            "step_time_avg_us,max_penetration_m,avg_penetration_m,"
            "dwell_fraction,ratio_p_y_vs_M0\n"
            "%s,%.6e,%.6e,%.6e,%.3f,%.6e,%.6e,%.4f,%.6f\n",
            method_name,
            metrics.max_abs_total_p_y,
            metrics.avg_abs_total_p_y(),
            metrics.max_abs_energy_drift,
            metrics.step_time_avg_us(),
            metrics.max_penetration_depth,
            metrics.avg_penetration_depth(),
            metrics.dwell_fraction(),
            metrics.ratio_p_y_vs_baseline);
        std::fclose(mf);
    }
    std::printf("[bench] %-6s box_drop  |Σp|=%.3e (avg=%.3e)  |ΔE/E0|=%.3e  "
                "step=%.2fμs  pen_max=%.3e dwell=%.0f%% ← %s\n",
                method_name,
                metrics.max_abs_total_p_y,
                metrics.avg_abs_total_p_y(),
                metrics.max_abs_energy_drift,
                metrics.step_time_avg_us(),
                metrics.max_penetration_depth,
                metrics.dwell_fraction() * 100.0f,
                ts_path);

    // ratio_p_y_vs_M0 is left at default 1.0 in the per-method CSV; the
    // plot script computes it from M0's max for the summary table.
}

// ─── Repose-pile scenario: free MPM column → settled angle ────────────────
// ── Agricultural scenario — cultivator tine dragged through soil bed ───
// Pivot 5 agriculture-aligned scenario. Tine driven kinematically along +X
// at constant velocity; coupler injects per-step reaction onto the tine,
// integrated as time-averaged draft force.
static void run_tine_drag(const char* method_name, ApplyFn coupler) {
    research_mr::ScenarioTineDrag S;
    basements::mpm::SPGridCPU grid(S.dx);
    int n_active = research_mr::seed_soil_bed_grid(grid, S);
    (void)n_active;
    RigidBody tine = research_mr::make_tine(S);

    double sum_fx = 0.0, sum_fy = 0.0;
    int    samples = 0;

    char ts_path[256], sum_path[256];
    std::snprintf(ts_path, sizeof(ts_path),
                  "%s/csv/tine_drag_%s.csv",         RESEARCH_OUTPUT_DIR, method_name);
    std::snprintf(sum_path, sizeof(sum_path),
                  "%s/csv/tine_drag_%s_summary.csv", RESEARCH_OUTPUT_DIR, method_name);

    FILE* ts = std::fopen(ts_path, "w");
    if (ts) std::fprintf(ts, "t,tine_x,reaction_fx,reaction_fy,reaction_fz\n");

    const int N = S.num_steps();
    const float tine_width_z = 2.0f * S.tine_half_extents.z;
    for (int i = 0; i < N; ++i) {
        const float t = (i + 1) * S.dt;

        // Kinematic drive: integrate position by prescribed velocity (no
        // dynamic response — we ignore the coupler's reaction on the tine
        // motion, and instead *measure* the reaction as draft).
        tine.position.x += S.drag_velocity_x * S.dt;

        RigidColliderState col = research_mr::make_tine_collider(S, tine);
        CouplingReaction r = coupler(grid, col, S.dt);

        // Grid-side gravity + floor BC (matches box_drop/rover_wheel pattern).
        for (auto& kv : grid.get_blocks_mutable()) {
            auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                auto& nn = blk.nodes[n_idx];
                if (!nn.active || nn.mass <= 0.0f) continue;
                nn.velocity.y += -S.gravity_g * S.dt;
                if (nn.velocity.y < 0.0f) nn.velocity.y = 0.0f;
            }
        }

        if (t > S.measurement_start_time) {
            sum_fx += -r.force.x;        // reaction on tine is -F_tine→soil;
            sum_fy +=  r.force.y;        // draft is positive when opposing motion.
            ++samples;
        }
        if (ts && (i % 4 == 0)) {
            std::fprintf(ts, "%.4f,%.6e,%.6e,%.6e,%.6e\n",
                         t, tine.position.x, r.force.x, r.force.y, r.force.z);
        }
    }
    if (ts) std::fclose(ts);

    research_mr::TineDragMetrics M;
    if (samples > 0) {
        M.avg_draft_force_x_n    = (float)(sum_fx / samples);
        M.avg_vertical_force_y_n = (float)(sum_fy / samples);
        M.specific_draught_n_per_m = M.avg_draft_force_x_n / tine_width_z;
        M.samples = samples;
    }
    if (FILE* sf = std::fopen(sum_path, "w")) {
        std::fprintf(sf,
            "method,avg_draft_fx_n,avg_vertical_fy_n,specific_draught_n_per_m,samples\n"
            "%s,%.6e,%.6e,%.6e,%d\n",
            method_name, M.avg_draft_force_x_n, M.avg_vertical_force_y_n,
            M.specific_draught_n_per_m, M.samples);
        std::fclose(sf);
    }
    std::printf("[bench] %-5s tine_drag  Fx=%+.3eN  Fy=%+.3eN  draught=%+.3eN/m  n=%d ← %s\n",
                method_name, M.avg_draft_force_x_n, M.avg_vertical_force_y_n,
                M.specific_draught_n_per_m, M.samples, ts_path);
}

// Constitutive validation only (no rigid body, no coupler). Tests whether
// our MPM sand parameters reproduce the literature 30–35° dry-sand angle.
static void run_repose_pile() {
    research_mr::ScenarioReposePile S;
    // Lube 2004 H/D sweep — env REPOSE_HD overrides column_ny.
    // Keeps column_nx/nz fixed; H/D = column_ny / column_nx.
    if (const char* env_hd = std::getenv("REPOSE_HD")) {
        const float hd = (float)std::atof(env_hd);
        S.column_ny = std::max(2, (int)std::round(hd * (float)S.column_nx));
        // Lower the column base for tall columns so the top stays inside the grid.
        S.column_base.y = S.particle_spacing;
    }
    basements::mpm::MPMSolver solver;
    solver.initialize(S.dx);
    // Material: stiffer-than-default so column doesn't compact too much
    // under its own weight before friction develops. CFL margin at this E:
    //   c_s = √((λ+2μ)/ρ) ≈ 21.6 m/s, dt_CFL = dx/c_s ≈ 4.6 ms vs dt = 4.2 ms.
    solver.set_material(/*E=*/5.0e5f, /*nu=*/0.30f, /*ρ=*/1500.0f);
    // W2 calibrated to angle_geom ∈ [30°, 35°] @ dx=25mm, dt=1/960:
    //   α=0.52 → 30.4°,  α=0.55 → 35.1°,  α=0.58 → 40.2°.
    // The effective D-P→Mohr-Coulomb mapping with our kohesion-factor
    // projection is steeper than Klar's idealised formula; documenting
    // the *measured-vs-parameter* correspondence rather than relying on
    // the analytical conversion alone.
    float alpha_repose = 0.55f;
    if (const char* env = std::getenv("REPOSE_ALPHA")) {
        alpha_repose = (float)std::atof(env);
    }
    solver.set_plastic_alpha(alpha_repose);
    solver.set_floor_friction(0.60f);
    int n_particles = research_mr::seed_repose_column(solver, S);

    char ts_path[256], sum_path[256];
    std::snprintf(ts_path, sizeof(ts_path),
                  "%s/csv/repose_pile.csv",         RESEARCH_OUTPUT_DIR);
    std::snprintf(sum_path, sizeof(sum_path),
                  "%s/csv/repose_pile_summary.csv", RESEARCH_OUTPUT_DIR);

    FILE* ts = std::fopen(ts_path, "w");
    if (ts) std::fprintf(ts, "t,height,radius,angle_deg,particles_at_rest\n");

    research_mr::ReposePileMetrics final_metrics;
    const int N = S.num_steps();
    for (int i = 0; i < N; ++i) {
        solver.step(S.dt);
        const float t = (i + 1) * S.dt;
        if (t > S.settle_time) {
            final_metrics = research_mr::measure_pile(solver, S);
        }
        if (ts && (i % 24 == 0)) {
            auto m = research_mr::measure_pile(solver, S);
            std::fprintf(ts, "%.4f,%.6e,%.6e,%.4f,%d\n",
                         t, m.pile_height, m.pile_radius, m.angle_deg,
                         m.particles_at_rest);
        }
    }
    if (ts) std::fclose(ts);

    if (FILE* sf = std::fopen(sum_path, "w")) {
        std::fprintf(sf,
            "particle_count,settled_height_m,settled_radius_m,"
            "angle_slope_deg,angle_geom_deg,particles_at_rest,"
            "literature_target_deg\n"
            "%d,%.6e,%.6e,%.4f,%.4f,%d,30-35\n",
            final_metrics.particle_count,
            final_metrics.pile_height,
            final_metrics.pile_radius,
            final_metrics.angle_deg,
            final_metrics.angle_deg_geom,
            final_metrics.particles_at_rest);
        std::fclose(sf);
    }
    std::printf("[bench] repose_pile  angle_slope=%.2f°  angle_geom=%.2f°  "
                "h=%.3fm  r=%.3fm  rest=%d/%d (target 30–35°) ← %s\n",
                final_metrics.angle_deg,
                final_metrics.angle_deg_geom,
                final_metrics.pile_height,
                final_metrics.pile_radius,
                final_metrics.particles_at_rest,
                n_particles, ts_path);
}

// ─── Foot-step scenario: flat plate impact + rebound coefficient ───────────
static void run_foot_step(const char* method_name, ApplyFn coupler) {
    research_mr::ScenarioFootStep S;
    basements::mpm::SPGridCPU grid(S.dx);
    int n_particles = research_mr::seed_sand_for_foot(grid, S);
    RigidBody foot = research_mr::make_foot(S);
    research_mr::ReboundTracker rebound;

    char ts_path[256], fr_path[256];
    std::snprintf(ts_path, sizeof(ts_path),
                  "%s/csv/foot_step_%s.csv",        RESEARCH_OUTPUT_DIR, method_name);
    std::snprintf(fr_path, sizeof(fr_path),
                  "%s/csv/foot_step_%s_frames.csv", RESEARCH_OUTPUT_DIR, method_name);
    research_mr::FrameExporter exp(ts_path, fr_path);
    research_mr::RunMetrics metrics;

    const int N = S.num_steps();
    for (int i = 0; i < N; ++i) {
        research_mr::StepTimer step_timer;

        foot.apply_force(Vec3(0.0f, -S.gravity_g * foot.mass, 0.0f));

        RigidColliderState col;
        col.center           = foot.position;
        col.orientation      = foot.orientation;
        col.half_extents     = S.foot_half_extents;
        col.linear_velocity  = foot.linear_velocity;
        col.angular_velocity = foot.angular_velocity;
        col.friction         = 0.5f;
        col.mass             = foot.mass;
        col.shape            = basements::physics::coupling::ShapeKind::BOX;
        CouplingReaction r   = coupler(grid, col, S.dt);

        foot.force  += r.force;
        foot.torque += r.torque;
        TimeIntegrator::integrate(&foot, 1, S.dt);

        for (auto& kv : grid.get_blocks_mutable()) {
            auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                auto& nn = blk.nodes[n_idx];
                if (!nn.active || nn.mass <= 0.0f) continue;
                nn.velocity.y += -S.gravity_g * S.dt;
                if (nn.velocity.y < 0.0f) nn.velocity.y = 0.0f;
            }
        }

        Vec3 p_grid_post = Vec3::zero();
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) p_grid_post += nn.velocity * nn.mass;
            }
        }
        const float t = (i + 1) * S.dt;
        rebound.observe(t, foot.linear_velocity.y, S.settle_time);

        Vec3  p_body = foot.linear_velocity * foot.mass;
        float KE_box = 0.5f * foot.mass * foot.linear_velocity.length_squared();
        col.center           = foot.position;
        col.linear_velocity  = foot.linear_velocity;
        col.angular_velocity = foot.angular_velocity;
        exp.log_step(t, col, p_body, p_grid_post, KE_box, r.nodes_inside);
        if ((i % S.frame_every_n) == 0) exp.log_frame(t, col, grid);

        float E_total = KE_box + foot.mass * S.gravity_g * foot.position.y;
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) E_total += 0.5f * nn.mass * nn.velocity.length_squared();
            }
        }
        float pen = research_mr::measure_penetration(grid, col);
        const float min_half = std::min({col.half_extents.x,
                                         col.half_extents.y,
                                         col.half_extents.z});
        metrics.observe((p_body + p_grid_post).y, E_total, pen, min_half,
                        step_timer.elapsed_us());
        (void)n_particles;
    }

    // Standard metrics CSV (shared format).
    char sum_path[256];
    std::snprintf(sum_path, sizeof(sum_path),
                  "%s/csv/foot_step_%s_metrics.csv", RESEARCH_OUTPUT_DIR, method_name);
    if (FILE* mf = std::fopen(sum_path, "w")) {
        std::fprintf(mf,
            "method,max_abs_total_p_y,avg_abs_total_p_y,max_abs_energy_drift,"
            "step_time_avg_us,max_penetration_m,avg_penetration_m,"
            "dwell_fraction,ratio_p_y_vs_M0\n"
            "%s,%.6e,%.6e,%.6e,%.3f,%.6e,%.6e,%.4f,%.6f\n",
            method_name,
            metrics.max_abs_total_p_y,
            metrics.avg_abs_total_p_y(),
            metrics.max_abs_energy_drift,
            metrics.step_time_avg_us(),
            metrics.max_penetration_depth,
            metrics.avg_penetration_depth(),
            metrics.dwell_fraction(),
            metrics.ratio_p_y_vs_baseline);
        std::fclose(mf);
    }

    // Foot-step rebound summary.
    char reb_path[256];
    std::snprintf(reb_path, sizeof(reb_path),
                  "%s/csv/foot_step_%s_rebound.csv", RESEARCH_OUTPUT_DIR, method_name);
    if (FILE* rf = std::fopen(reb_path, "w")) {
        std::fprintf(rf,
            "method,v_down_peak,v_up_peak,rebound_coefficient_e\n"
            "%s,%.6e,%.6e,%.6f\n",
            method_name, rebound.v_down_peak(), rebound.v_up_peak(),
            rebound.coefficient());
        std::fclose(rf);
    }

    std::printf("[bench] %-6s foot_step  e=%.3f  v_down=%.2f m/s  v_up=%.2f m/s "
                " |Σp|=%.2e ← %s\n",
                method_name, rebound.coefficient(),
                rebound.v_down_peak(), rebound.v_up_peak(),
                metrics.max_abs_total_p_y, ts_path);
}

// ─── box_drop_dense: same geometry as box_drop, but body mass 9 kg so it has
//     the *same density* as the rover_wheel cylinder (≈ 141 kg/m³). Separates
//     the geometry effect from the mass-ratio effect that drives M0/M1's
//     regime sensitivity. Uses the box_drop entry path with one override.
static void run_box_drop_dense(const char* method_name, ApplyFn coupler) {
    research_mr::ScenarioBoxDrop S;
    S.box_mass = 9.0f;   // density-matched to wheel (V_box = 0.064 m³ → ρ = 141 kg/m³)
    basements::mpm::SPGridCPU grid(S.dx);
    int n_particles = research_mr::seed_sand(grid, S);
    RigidBody box = research_mr::make_box(S);

    char ts_path[256], fr_path[256];
    std::snprintf(ts_path, sizeof(ts_path),
                  "%s/csv/box_drop_dense_%s.csv",        RESEARCH_OUTPUT_DIR, method_name);
    std::snprintf(fr_path, sizeof(fr_path),
                  "%s/csv/box_drop_dense_%s_frames.csv", RESEARCH_OUTPUT_DIR, method_name);
    research_mr::FrameExporter exp(ts_path, fr_path);
    research_mr::RunMetrics metrics;

    const int N = S.num_steps();
    for (int i = 0; i < N; ++i) {
        research_mr::StepTimer step_timer;
        box.apply_force(Vec3(0.0f, -S.gravity_g * box.mass, 0.0f));

        RigidColliderState col;
        col.center           = box.position;
        col.orientation      = box.orientation;
        col.half_extents     = S.box_half_extents;
        col.linear_velocity  = box.linear_velocity;
        col.angular_velocity = box.angular_velocity;
        col.friction         = 0.5f;
        col.mass             = box.mass;
        CouplingReaction r   = coupler(grid, col, S.dt);

        box.force  += r.force;
        box.torque += r.torque;
        TimeIntegrator::integrate(&box, 1, S.dt);

        for (auto& kv : grid.get_blocks_mutable()) {
            auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                auto& nn = blk.nodes[n_idx];
                if (!nn.active || nn.mass <= 0.0f) continue;
                nn.velocity.y += -S.gravity_g * S.dt;
                if (nn.velocity.y < 0.0f) nn.velocity.y = 0.0f;
            }
        }

        Vec3 p_grid_post = Vec3::zero();
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) p_grid_post += nn.velocity * nn.mass;
            }
        }
        float t = (i + 1) * S.dt;
        Vec3  p_body = box.linear_velocity * box.mass;
        float KE_box = 0.5f * box.mass * box.linear_velocity.length_squared();
        col.center           = box.position;
        col.linear_velocity  = box.linear_velocity;
        col.angular_velocity = box.angular_velocity;
        exp.log_step(t, col, p_body, p_grid_post, KE_box, r.nodes_inside);
        if ((i % S.frame_every_n) == 0) exp.log_frame(t, col, grid);
        float E_total = KE_box + box.mass * S.gravity_g * box.position.y;
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) E_total += 0.5f * nn.mass * nn.velocity.length_squared();
            }
        }
        float pen = research_mr::measure_penetration(grid, col);
        const float min_half = std::min({col.half_extents.x,
                                         col.half_extents.y,
                                         col.half_extents.z});
        metrics.observe((p_body + p_grid_post).y, E_total, pen, min_half,
                        step_timer.elapsed_us());
        (void)n_particles;
    }

    char sum_path[256];
    std::snprintf(sum_path, sizeof(sum_path),
                  "%s/csv/box_drop_dense_%s_metrics.csv", RESEARCH_OUTPUT_DIR, method_name);
    if (FILE* mf = std::fopen(sum_path, "w")) {
        std::fprintf(mf,
            "method,max_abs_total_p_y,avg_abs_total_p_y,max_abs_energy_drift,"
            "step_time_avg_us,max_penetration_m,avg_penetration_m,"
            "dwell_fraction,ratio_p_y_vs_M0\n"
            "%s,%.6e,%.6e,%.6e,%.3f,%.6e,%.6e,%.4f,%.6f\n",
            method_name,
            metrics.max_abs_total_p_y,
            metrics.avg_abs_total_p_y(),
            metrics.max_abs_energy_drift,
            metrics.step_time_avg_us(),
            metrics.max_penetration_depth,
            metrics.avg_penetration_depth(),
            metrics.dwell_fraction(),
            metrics.ratio_p_y_vs_baseline);
        std::fclose(mf);
    }
    std::printf("[bench] %-6s box_drop_dense  |Σp|=%.3e dwell=%.0f%% step=%.2fμs (m=9kg) ← %s\n",
                method_name, metrics.max_abs_total_p_y,
                metrics.dwell_fraction() * 100.0f, metrics.step_time_avg_us(), ts_path);
}

// ─── Rover-wheel scenario (cylinder primitive, rolling motion) ─────────────
// Records, in addition to the standard `_metrics.csv`, a rolling-specific
// summary `_roll.csv` with time-averaged slip, sinkage, drawbar, and the
// ratio of measured sinkage to Bekker's analytical prediction.
static void run_rover_wheel(const char* method_name, ApplyFn coupler) {
    research_mr::ScenarioRoverWheel S;
    basements::mpm::SPGridCPU grid(S.dx);
    int n_particles = research_mr::seed_sand_for_wheel(grid, S);
    RigidBody wheel = research_mr::make_wheel(S);

    // Initial sand surface height (top of grid_layers_y nodes, cell-centered)
    const float sand_surface_y = (S.grid_layers_y - 1 + 0.5f) * S.dx;
    const float bekker_z       = S.bekker_predicted_sinkage();

    // Rolling running-averages
    double sum_slip      = 0.0;
    double sum_drawbar   = 0.0;
    double sum_sinkage   = 0.0;
    int    roll_samples  = 0;

    char ts_path[256], fr_path[256];
    std::snprintf(ts_path, sizeof(ts_path),
                  "%s/csv/rover_wheel_%s.csv",        RESEARCH_OUTPUT_DIR, method_name);
    std::snprintf(fr_path, sizeof(fr_path),
                  "%s/csv/rover_wheel_%s_frames.csv", RESEARCH_OUTPUT_DIR, method_name);
    research_mr::FrameExporter exp(ts_path, fr_path);
    research_mr::RunMetrics metrics;

    const int N = S.num_steps();
    for (int i = 0; i < N; ++i) {
        research_mr::StepTimer step_timer;

        wheel.apply_force(Vec3(0.0f, -S.gravity_g * wheel.mass, 0.0f));
        if (S.wheel_drive_torque != 0.0f) {
            // Drive torque about world Z (rolling axis after make_wheel rotation).
            wheel.apply_torque(Vec3(0.0f, 0.0f, S.wheel_drive_torque));
        }

        Vec3 p_grid_pre = Vec3::zero();
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) p_grid_pre += nn.velocity * nn.mass;
            }
        }

        RigidColliderState col = research_mr::make_wheel_collider(S, wheel);
        CouplingReaction r = coupler(grid, col, S.dt);

        wheel.force  += r.force;
        wheel.torque += r.torque;
        TimeIntegrator::integrate(&wheel, 1, S.dt);

        for (auto& kv : grid.get_blocks_mutable()) {
            auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                auto& nn = blk.nodes[n_idx];
                if (!nn.active || nn.mass <= 0.0f) continue;
                nn.velocity.y += -S.gravity_g * S.dt;
                if (nn.velocity.y < 0.0f) nn.velocity.y = 0.0f;
            }
        }

        Vec3 p_grid_post = Vec3::zero();
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active) p_grid_post += nn.velocity * nn.mass;
            }
        }

        float t        = (i + 1) * S.dt;
        Vec3  p_body   = wheel.linear_velocity * wheel.mass;
        float KE_box   = 0.5f * wheel.mass * wheel.linear_velocity.length_squared();
        col.center           = wheel.position;
        col.linear_velocity  = wheel.linear_velocity;
        col.angular_velocity = wheel.angular_velocity;
        exp.log_step(t, col, p_body, p_grid_post, KE_box, r.nodes_inside);
        if ((i % S.frame_every_n) == 0) exp.log_frame(t, col, grid);

        float E_total = KE_box + wheel.mass * S.gravity_g * wheel.position.y;
        for (const auto& kv : grid.get_blocks()) {
            const auto& blk = kv.second;
            for (int n_idx = 0;
                 n_idx < (int)(sizeof(blk.nodes) / sizeof(blk.nodes[0])); ++n_idx) {
                const auto& nn = blk.nodes[n_idx];
                if (nn.active)
                    E_total += 0.5f * nn.mass * nn.velocity.length_squared();
            }
        }
        float pen = research_mr::measure_penetration(grid, col);
        const float min_half_w = std::min({col.half_extents.x,
                                           col.half_extents.y,
                                           col.half_extents.z});
        metrics.observe((p_body + p_grid_post).y, E_total, pen, min_half_w,
                        step_timer.elapsed_us());

        // Rolling-specific samples in the final 1 s — the wheel that punches
        // through has long been below the sand by then, but its sinkage is
        // clamped at [0, 2R] so the average stays bounded and meaningful.
        if (t > S.total_time - 1.0f) {
            const float slip    = research_mr::slip_ratio(wheel, S.wheel_radius);
            const float drawbar = r.force.x;
            const float raw_sinkage = (sand_surface_y + S.wheel_radius) - wheel.position.y;
            const float clamped     = std::max(0.0f,
                                       std::min(raw_sinkage, 2.0f * S.wheel_radius));
            sum_slip    += slip;
            sum_drawbar += drawbar;
            sum_sinkage += clamped;
            ++roll_samples;
        }

        (void)p_grid_pre; (void)n_particles;
    }

    char sum_path[256];
    std::snprintf(sum_path, sizeof(sum_path),
                  "%s/csv/rover_wheel_%s_metrics.csv", RESEARCH_OUTPUT_DIR, method_name);
    if (FILE* mf = std::fopen(sum_path, "w")) {
        std::fprintf(mf,
            "method,max_abs_total_p_y,avg_abs_total_p_y,max_abs_energy_drift,"
            "step_time_avg_us,max_penetration_m,avg_penetration_m,"
            "dwell_fraction,ratio_p_y_vs_M0\n"
            "%s,%.6e,%.6e,%.6e,%.3f,%.6e,%.6e,%.4f,%.6f\n",
            method_name,
            metrics.max_abs_total_p_y,
            metrics.avg_abs_total_p_y(),
            metrics.max_abs_energy_drift,
            metrics.step_time_avg_us(),
            metrics.max_penetration_depth,
            metrics.avg_penetration_depth(),
            metrics.dwell_fraction(),
            metrics.ratio_p_y_vs_baseline);
        std::fclose(mf);
    }
    // Rolling-specific summary file (separate from the standard metrics CSV).
    char roll_path[256];
    std::snprintf(roll_path, sizeof(roll_path),
                  "%s/csv/rover_wheel_%s_roll.csv", RESEARCH_OUTPUT_DIR, method_name);
    if (FILE* rf = std::fopen(roll_path, "w")) {
        const float avg_slip    = roll_samples ? (float)(sum_slip    / roll_samples) : 0.0f;
        const float avg_drawbar = roll_samples ? (float)(sum_drawbar / roll_samples) : 0.0f;
        const float avg_sinkage = roll_samples ? (float)(sum_sinkage / roll_samples) : 0.0f;
        std::fprintf(rf,
            "method,avg_slip,avg_drawbar_N,avg_sinkage_m,bekker_sinkage_m,sinkage_ratio_meas_over_bekker\n"
            "%s,%.6f,%.6e,%.6e,%.6e,%.6e\n",
            method_name, avg_slip, avg_drawbar, avg_sinkage, bekker_z,
            (bekker_z > 1e-9f) ? (avg_sinkage / bekker_z) : -1.0f);
        std::fclose(rf);
    }

    std::printf("[bench] %-6s rover_wheel  |Σp|=%.3e  pen=%.3e dwell=%.0f%%  "
                "slip=%.2f  draw=%.2eN  sink=%.3em (Bekker %.3em, ratio %.1f) ← %s\n",
                method_name,
                metrics.max_abs_total_p_y,
                metrics.max_penetration_depth,
                metrics.dwell_fraction() * 100.0f,
                roll_samples ? (float)(sum_slip / roll_samples) : 0.0f,
                roll_samples ? (float)(sum_drawbar / roll_samples) : 0.0f,
                roll_samples ? (float)(sum_sinkage / roll_samples) : 0.0f,
                bekker_z,
                (bekker_z > 1e-9f && roll_samples)
                    ? ((float)(sum_sinkage / roll_samples) / bekker_z) : -1.0f,
                ts_path);
}

// ─── Case-study registry ──────────────────────────────────────────────────
// Adding a new case study now means: (1) write scenarios/scenario_X.h and a
// matching run_X() function, (2) add one row here. The bench's --scenario
// dispatch and --list-scenarios output are both derived from this table, so
// new entries are wired through automatically. method_aware_fn is set for
// rigid-coupling studies (iterated across every method in METHODS); the
// agnostic_fn is set for constitutive-only studies (single shot, no coupler).
using MethodAwareFn    = void (*)(const char* method_name, ApplyFn);
using MethodAgnosticFn = void (*)();

struct CaseStudyEntry {
    const char*       name;
    const char*       description;
    MethodAwareFn     method_aware_fn;     // nullable
    MethodAgnosticFn  agnostic_fn;          // nullable
};
static const CaseStudyEntry CASE_STUDIES[] = {
    { "box_drop",       "Box dropped on sand — momentum/energy/dwell",        &run_box_drop,       nullptr           },
    { "box_drop_dense", "Box drop with 9× body mass — mass-invariance",       &run_box_drop_dense, nullptr           },
    { "rover_wheel",    "Cylinder rolling on sand — sinkage/slip/Bekker",     &run_rover_wheel,    nullptr           },
    { "foot_step",      "Foot stamp — rebound coefficient",                   &run_foot_step,      nullptr           },
    { "tine_drag",      "Cultivator tine dragged through soil bed (Pivot 5 agricultural)", &run_tine_drag, nullptr },
    { "repose_pile",    "Angle of repose — constitutive validation",          nullptr,             &run_repose_pile  },
};

int main(int argc, char** argv) {
    const char* selected_method   = nullptr;
    const char* selected_scenario = nullptr;
    bool all = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--all") == 0) all = true;
        else if (std::strcmp(argv[i], "--method")   == 0 && i + 1 < argc) selected_method   = argv[++i];
        else if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) selected_scenario = argv[++i];
        else if (std::strcmp(argv[i], "--list-methods") == 0) {
            for (const auto& m : METHODS) std::printf("%s\n", m.name);
            return 0;
        }
        else if (std::strcmp(argv[i], "--list-scenarios") == 0) {
            for (const auto& cs : CASE_STUDIES) {
                std::printf("%-18s %s\n", cs.name, cs.description);
            }
            return 0;
        }
        else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            std::printf("Usage: bench_compare_all [--all | --method M? --scenario NAME]\n"
                        "       bench_compare_all --list-scenarios\n"
                        "       bench_compare_all --list-methods\n");
            return 0;
        }
    }

    std::vector<const MethodEntry*> run_methods;
    for (const auto& m : METHODS) {
        if (all || (selected_method && std::strcmp(selected_method, m.name) == 0)) {
            run_methods.push_back(&m);
        }
    }
    if (run_methods.empty()) {
        std::fprintf(stderr,
                     "no method selected. use --all or --method M0/M1/M2/M3/M4/M4.5\n");
        return 1;
    }

    // Dispatch: when no scenario is named, run the first method-aware study
    // (legacy behaviour) plus every method-agnostic study under --all.
    for (const auto& cs : CASE_STUDIES) {
        const bool matched = selected_scenario && std::strcmp(selected_scenario, cs.name) == 0;
        const bool default_pick = !selected_scenario && cs.method_aware_fn == &run_box_drop;
        const bool should_run = all || matched || default_pick;
        if (!should_run) continue;

        if (cs.method_aware_fn) {
            for (const auto* m : run_methods) cs.method_aware_fn(m->name, m->fn);
        } else if (cs.agnostic_fn) {
            cs.agnostic_fn();
        }
    }
    return 0;
}

// bench_mpm_rigid_drift.cpp — Baseline figure for the MPM ↔ rigid coupling MVP.
//
// Scenario:
//   A "sand patch" is seeded as a column of active grid nodes with mass and
//   small downward velocity (modelling settled granular state). A box drops
//   from above under gravity. Each frame we apply MPMRigidCoupler::apply()
//   to enforce no-penetration at the box's OBB and accumulate the reaction
//   force / torque on the box. The box is then integrated with TimeIntegrator.
//
// What this dump answers
// ──────────────────────
//   • Body y-position vs time: contact response of the Dirichlet BC.
//   • Body linear momentum   : impulse delivered per frame.
//   • Σ particle momentum    : grid-side reaction (negative of body impulse
//                              modulo numeric drift).
//   • |Σ_body p + Σ_grid p|  : momentum-conservation error — the key axis
//                              for comparing this MVP against future
//                              PIC-Rigid / ASFLIP implementations (M1, M2).
//
// CSV: outputs/figures/mpm_rigid_drift.csv

#include "basements/physics/rigid_body.h"
#include "basements/physics/dynamics/integrator.h"
#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"
#include "basements/physics/coupling/mpm_rigid_coupler_pic.h"

#include <cstdio>
#include <cstring>
#include <string>

#ifndef BASEMENTS_FIGURES_DIR
#define BASEMENTS_FIGURES_DIR "outputs/figures"
#endif

using basements::math::Vec3;
using basements::math::Matrix3;
using basements::physics::RigidBody;
using basements::physics::BodyType;
using basements::dynamics::TimeIntegrator;
using basements::mpm::GridNode;
using basements::mpm::SPGridCPU;
using basements::physics::coupling::CouplingReaction;
using basements::physics::coupling::MPMRigidCoupler;
using basements::physics::coupling::MPMRigidCouplerPIC;
using basements::physics::coupling::RigidColliderState;

int main(int argc, char** argv) {
    // ─── CLI: --method dirichlet (default) | pic ──────────────────────────
    const char* method = "dirichlet";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--method") == 0 && i + 1 < argc) {
            method = argv[++i];
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            std::printf("Usage: bench_mpm_rigid_drift [--method dirichlet|pic]\n");
            return 0;
        }
    }
    const bool use_pic = (std::strcmp(method, "pic") == 0);
    std::printf("[bench_mpm_rigid_drift] method = %s\n",
                use_pic ? "PIC-Rigid (M1)" : "Dirichlet (M0 MVP)");
    // ─── Grid: 5×3×5 settled sand patch, top layer at y = 0.25 m ──────────
    const float dx = 0.1f;
    SPGridCPU grid(dx);
    const float particle_mass = 0.01f;
    int n_particles = 0;
    Vec3 particle_init_v(0.0f, 0.0f, 0.0f);
    for (int i = -2; i <= 2; ++i)
    for (int j = 0; j <= 2; ++j)         // 3 vertical layers (y = 0.05, 0.15, 0.25)
    for (int k = -2; k <= 2; ++k) {
        GridNode* n = grid.get_node(i, j, k);
        n->active   = true;
        n->mass     = particle_mass;
        n->velocity = particle_init_v;
        ++n_particles;
    }
    const float grid_total_mass = particle_mass * (float)n_particles;

    // ─── Box: 1 kg dynamic body, released just above the sand top layer ───
    // Top of grid ≈ 0.30 m. Drop the box from 0.55 m so it makes contact
    // after a short fall (~0.2 s) — short enough that contact lifetime is
    // long compared to the 3 s window we record.
    RigidBody box; box.make_dynamic(1.0f);
    box.position         = Vec3(0.0f, 0.55f, 0.0f);
    box.linear_velocity  = Vec3::zero();
    box.linear_damping   = 0.0f;
    box.angular_damping  = 0.0f;
    // Box inertia: cube 0.4 m side → half = 0.2.
    {
        float h = 0.2f;
        float I = (1.0f / 12.0f) * 1.0f * (2.0f * h * 2.0f * h);  // m/12 * (h^2 + h^2)*4
        box.inertia_tensor = Matrix3(I, 0, 0, 0, I, 0, 0, 0, I);
    }
    const Vec3 box_half(0.2f, 0.2f, 0.2f);

    const float g  = 9.81f;
    const float dt = 1.0f / 240.0f;
    const int   N  = 240 * 3;  // 3 s — enough to land + bounce response

    // ─── CSV dump ──────────────────────────────────────────────────────────
    std::string fname = use_pic ? "mpm_rigid_drift_pic.csv"
                                 : "mpm_rigid_drift.csv";
    std::string path = std::string(BASEMENTS_FIGURES_DIR) + "/" + fname;
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) { std::fprintf(stderr, "[bench] cannot open %s\n", path.c_str()); return 1; }
    std::fprintf(f,
        "t,box_y,box_vy,p_body_y,p_grid_y,p_sum_y,KE_box,nodes_inside\n");

    for (int i = 0; i < N; ++i) {
        // 1) Body external forces — gravity.
        box.apply_force(Vec3(0.0f, -g * box.mass, 0.0f));

        // 2) Sample current grid momentum (pre-coupling).
        Vec3 p_grid_pre = Vec3::zero();
        for (auto& kv : grid.get_blocks()) {
            const auto& block = kv.second;
            for (int n_idx = 0; n_idx < (int)(sizeof(block.nodes)/sizeof(block.nodes[0])); ++n_idx) {
                const GridNode& nn = block.nodes[n_idx];
                if (nn.active) p_grid_pre += nn.velocity * nn.mass;
            }
        }

        // 3) Build collider state from current box and apply coupling.
        RigidColliderState col;
        col.center           = box.position;
        col.orientation      = box.orientation;
        col.half_extents     = box_half;
        col.linear_velocity  = box.linear_velocity;
        col.angular_velocity = box.angular_velocity;
        col.friction         = 0.5f;
        col.mass             = box.mass;  // PIC-Rigid: required for mass-weighted exchange
        CouplingReaction reaction = use_pic
            ? MPMRigidCouplerPIC::apply(grid, col, dt)
            : MPMRigidCoupler   ::apply(grid, col, dt);

        // 4) Apply reaction back onto the rigid body and integrate.
        box.force  += reaction.force;
        box.torque += reaction.torque;
        TimeIntegrator::integrate(&box, 1, dt);

        // 5) Apply gravity to grid (no real MPM step here — sand stays seeded).
        for (auto& kv : grid.get_blocks_mutable()) {
            auto& block = kv.second;
            for (int n_idx = 0; n_idx < (int)(sizeof(block.nodes)/sizeof(block.nodes[0])); ++n_idx) {
                GridNode& nn = block.nodes[n_idx];
                if (!nn.active || nn.mass <= 0.0f) continue;
                nn.velocity.y += -g * dt;
                // Pin to y >= 0 (ground): zero downward velocity at boundary.
                if (nn.velocity.y < 0.0f) nn.velocity.y = 0.0f;
            }
        }

        // 6) Measure grid momentum post-coupling.
        Vec3 p_grid_post = Vec3::zero();
        for (const auto& kv : grid.get_blocks()) {
            const auto& block = kv.second;
            for (int n_idx = 0; n_idx < (int)(sizeof(block.nodes)/sizeof(block.nodes[0])); ++n_idx) {
                const GridNode& nn = block.nodes[n_idx];
                if (nn.active) p_grid_post += nn.velocity * nn.mass;
            }
        }

        float t           = (i + 1) * dt;
        float p_body_y    = box.linear_velocity.y * box.mass;
        float p_grid_y    = p_grid_post.y;
        float p_sum_y     = p_body_y + p_grid_y;
        float KE_box      = 0.5f * box.mass * box.linear_velocity.length_squared();

        std::fprintf(f, "%.6f,%.8e,%.8e,%.8e,%.8e,%.8e,%.8e,%d\n",
                     t, box.position.y, box.linear_velocity.y,
                     p_body_y, p_grid_y, p_sum_y, KE_box, reaction.nodes_inside);

        (void)p_grid_pre;  // recorded for future per-step delta plot
    }
    std::fclose(f);
    std::printf("[bench] wrote %s (grid mass %.3f kg, particles %d)\n",
                path.c_str(), grid_total_mass, n_particles);
    return 0;
}

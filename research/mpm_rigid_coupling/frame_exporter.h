#ifndef RESEARCH_MPM_RIGID_FRAME_EXPORTER_H
#define RESEARCH_MPM_RIGID_FRAME_EXPORTER_H

// Dumps per-frame box pose + grid active-node state to CSV. The Python
// visualisation scripts read these CSVs and produce PNG + MP4 outputs;
// the C++ benchmark is otherwise headless.

#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/coupling/mpm_rigid_coupler.h"

#include <cstdio>
#include <string>

namespace research { namespace mpm_rigid {

class FrameExporter {
public:
    FrameExporter(const std::string& timeseries_csv,
                  const std::string& frames_csv)
        : ts_f_(std::fopen(timeseries_csv.c_str(), "w"))
        , fr_f_(std::fopen(frames_csv.c_str(),     "w"))
    {
        if (ts_f_)
            std::fprintf(ts_f_,
                "t,box_x,box_y,box_z,box_vy,p_body_y,p_grid_y,p_sum_y,KE,nodes_inside\n");
        if (fr_f_)
            std::fprintf(fr_f_,
                "t,kind,id,x,y,z,vx,vy,vz,mass\n");
            // kind: "B"=box centre, "N"=active grid node
    }

    ~FrameExporter() {
        if (ts_f_) std::fclose(ts_f_);
        if (fr_f_) std::fclose(fr_f_);
    }

    // Scalar time series — one row per simulation step.
    void log_step(float t,
                  const basements::physics::coupling::RigidColliderState& box,
                  basements::math::Vec3 p_body,
                  basements::math::Vec3 p_grid,
                  float KE,
                  int   nodes_inside) const {
        if (!ts_f_) return;
        std::fprintf(ts_f_,
                     "%.6f,%.6e,%.6e,%.6e,%.6e,%.8e,%.8e,%.8e,%.6e,%d\n",
                     t, box.center.x, box.center.y, box.center.z,
                     box.linear_velocity.y,
                     p_body.y, p_grid.y, (p_body + p_grid).y,
                     KE, nodes_inside);
    }

    // Per-frame snapshot of full scene state (used for animation rendering).
    // Call sparsely (e.g. every 24th step at 240Hz → 10 fps).
    void log_frame(float t,
                   const basements::physics::coupling::RigidColliderState& box,
                   const basements::mpm::SPGridCPU& grid) const {
        if (!fr_f_) return;
        std::fprintf(fr_f_,
                     "%.6f,B,0,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e\n",
                     t,
                     box.center.x, box.center.y, box.center.z,
                     box.linear_velocity.x, box.linear_velocity.y, box.linear_velocity.z,
                     0.0);
        const float dx = grid.get_dx();
        int node_serial = 0;
        for (const auto& kv : grid.get_blocks()) {
            const auto& block = kv.second;
            for (int li = 0; li < basements::mpm::BLOCK_SIZE; ++li)
            for (int lj = 0; lj < basements::mpm::BLOCK_SIZE; ++lj)
            for (int lk = 0; lk < basements::mpm::BLOCK_SIZE; ++lk) {
                int local_idx = (li * basements::mpm::BLOCK_SIZE * basements::mpm::BLOCK_SIZE)
                              + (lj * basements::mpm::BLOCK_SIZE) + lk;
                const auto& n = block.nodes[local_idx];
                if (!n.active || n.mass <= 0.0f) continue;
                int gi = block.origin_x + li;
                int gj = block.origin_y + lj;
                int gk = block.origin_z + lk;
                std::fprintf(fr_f_,
                             "%.6f,N,%d,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e\n",
                             t, node_serial,
                             (gi + 0.5f) * dx, (gj + 0.5f) * dx, (gk + 0.5f) * dx,
                             n.velocity.x, n.velocity.y, n.velocity.z,
                             n.mass);
                ++node_serial;
            }
        }
    }

private:
    FILE* ts_f_ = nullptr;
    FILE* fr_f_ = nullptr;
};

} }

#endif

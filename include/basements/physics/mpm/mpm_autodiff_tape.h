#pragma once

// mpm_autodiff_tape.h — Recording infrastructure for differentiable MPM.
//
// Pivot 6 in-place autodiff design: MPMSolver gains an opt-in
// `step(dt, AutodiffTape* tape)` overload. When `tape == nullptr` the
// forward-only hot path is unchanged. When `tape != nullptr` each P2G /
// update_grid / G2P call records intermediate state in the tape, allowing
// a downstream backward pass (paper #2 M2-M4) to reconstruct gradients
// without re-running forward.
//
// This header is M1 scaffolding. Forward recording wires up in M2;
// backward consumers wire up in M3 (plastic adjoint) and M4 (M4.5a
// non-local coupling gradient).
//
// Design constraints:
//   * `nullptr` tape costs at most one branch per recorded op (~ns).
//   * Tape memory grows linearly with (particles × ops_per_step × steps).
//     Caller responsible for clearing between rollouts.
//   * No virtual dispatch; entries are tagged unions for cache locality.
//   * Tape replay is the *gradient* path, not a forward re-execution
//     (we never replay forward from the tape; only the backward pass
//     reads recorded values).

#include "basements/core/math/vec3.h"
#include "basements/core/math/matrix3.h"

#include <cstdint>
#include <vector>

namespace basements {
namespace mpm {

// Operations recorded per step. Ordered by data flow: P2G → grid update
// → G2P. Each entry stores the *inputs* the backward pass needs so the
// chain rule can be applied without re-running forward kernels.
enum class TapeOpKind : std::uint8_t {
    P2G_PARTICLE     = 0,  // single particle → grid contribution
    GRID_UPDATE_NODE = 1,  // per-active-node velocity update
    G2P_PARTICLE     = 2,  // single particle ← grid interpolation
    PLASTICITY       = 3,  // per-particle Drucker-Prager projection
};

// One entry per recorded op. Tagged-union layout for cache-friendly
// linear traversal during backward pass.
struct TapeEntry {
    TapeOpKind op;
    // Particle / node index. Particle index for P2G/G2P/PLASTICITY,
    // (block_key, local_idx) packed for GRID_UPDATE_NODE.
    std::uint32_t entity_index;

    // Recorded forward inputs. Not all fields used by every op; the
    // op tag determines which fields to read on replay.
    basements::math::Vec3    position;            // particle pos at P2G time
    basements::math::Vec3    velocity;            // particle vel before G2P, grid vel before update, etc.
    basements::math::Matrix3 F;                   // deformation gradient pre-plasticity
    basements::math::Matrix3 C;                   // affine momentum matrix
    float                    mass;
    float                    volume;
    float                    dt;
};

// Linear tape; one instance per differentiable rollout. Caller owns the
// lifetime — MPMSolver only borrows.
class AutodiffTape {
public:
    AutodiffTape() = default;

    // Pre-allocate to bound peak memory; recommended at construction.
    explicit AutodiffTape(std::size_t reserve_capacity) {
        entries_.reserve(reserve_capacity);
    }

    void push(const TapeEntry& e) { entries_.push_back(e); }

    void clear() { entries_.clear(); }

    std::size_t size() const { return entries_.size(); }
    bool        empty() const { return entries_.empty(); }

    const std::vector<TapeEntry>& entries() const { return entries_; }
    std::vector<TapeEntry>&       entries_mutable() { return entries_; }

    // Memory accounting (paper #2 M6 RL training: per-rollout tape size
    // must fit GPU/CPU memory budget).
    std::size_t bytes() const {
        return entries_.capacity() * sizeof(TapeEntry);
    }

private:
    std::vector<TapeEntry> entries_;
};

} // namespace mpm
} // namespace basements

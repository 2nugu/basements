#ifndef RESEARCH_MPM_RIGID_M4_H
#define RESEARCH_MPM_RIGID_M4_H

// M4 — Lagrange-multiplier saddle-point coupling (Jiang 2020 SCA, technique
// adapted from implicit SPH for snow).
// Body and grid momenta solved as a single saddle-point system. Conservation
// is guaranteed within solver tolerance.
//
// Status: SKELETON. Most expensive to implement (preconditioned CG or
//         direct Schur-complement solve per step).
// TODO:
//   1. Assemble Jacobian of constraint (body anchor velocity = grid velocity
//      at OBB-cell intersection point).
//   2. Schur complement on the rigid 6-DoF.
//   3. Iterative solver (PCG) — re-use existing math.

#include "basements/physics/coupling/mpm_rigid_coupler.h"
#include "basements/physics/coupling/mpm_rigid_coupler_pic.h"

namespace research { namespace mpm_rigid {

class M4 {
public:
    static basements::physics::coupling::CouplingReaction apply(
            basements::mpm::SPGridCPU& grid,
            const basements::physics::coupling::RigidColliderState& body,
            float dt) {
        // TODO(M4): real Lagrange-multiplier saddle-point solve.
        return basements::physics::coupling::MPMRigidCouplerPIC::apply(grid, body, dt);
    }
};

} }

#endif

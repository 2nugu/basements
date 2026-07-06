#ifndef RESEARCH_MPM_RIGID_M2_H
#define RESEARCH_MPM_RIGID_M2_H

// M2 — ASFLIP rigid extension (Fei 2021 SIGGRAPH).
// F-bar split: F = F̄ · F_vol where F̄ is trace-preserving.
// Rigid-grid boundary exchange uses F̄ for tangential / F_vol for normal,
// giving momentum-exact normal coupling under float roundoff.
//
// Status: SKELETON. Delegates to M1 until the F-bar pipeline is wired.
// TODO(Phase 2):
//   1. Per-particle history of F̄ (extend mpm::Particle, or shadow buffer here).
//   2. Tangent / normal projection at OBB face.
//   3. Friction integrated over the F̄ component.

#include "basements/physics/coupling/mpm_rigid_coupler.h"
#include "basements/physics/coupling/mpm_rigid_coupler_pic.h"

namespace research { namespace mpm_rigid {

class M2 {
public:
    static basements::physics::coupling::CouplingReaction apply(
            basements::mpm::SPGridCPU& grid,
            const basements::physics::coupling::RigidColliderState& body,
            float dt) {
        // TODO(M2): real ASFLIP. For now match M1 so the comparison harness
        //           runs end-to-end; the M2 column in figures will overlay M1
        //           until the kernel lands.
        return basements::physics::coupling::MPMRigidCouplerPIC::apply(grid, body, dt);
    }
};

} }

#endif

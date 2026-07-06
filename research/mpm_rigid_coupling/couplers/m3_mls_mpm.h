#ifndef RESEARCH_MPM_RIGID_M3_H
#define RESEARCH_MPM_RIGID_M3_H

// M3 — MLS-MPM rigid coupling (Hu 2018 SIGGRAPH).
// Moving Least Squares kernel replaces our basic mass-weighted average.
// Per-particle B-spline weight at the rigid body's local contact point gives
// continuous derivatives → smoother force exchange + better long-horizon
// stability than M1.
//
// Status: SKELETON.
// TODO:
//   1. MLS kernel evaluation at OBB-cell intersection points.
//   2. APIC affine momentum tensor C contribution from rigid body.
//   3. Mass redistribution weighted by kernel, not uniform.

#include "basements/physics/coupling/mpm_rigid_coupler.h"
#include "basements/physics/coupling/mpm_rigid_coupler_pic.h"

namespace research { namespace mpm_rigid {

class M3 {
public:
    static basements::physics::coupling::CouplingReaction apply(
            basements::mpm::SPGridCPU& grid,
            const basements::physics::coupling::RigidColliderState& body,
            float dt) {
        // TODO(M3): real MLS-MPM rigid coupling.
        return basements::physics::coupling::MPMRigidCouplerPIC::apply(grid, body, dt);
    }
};

} }

#endif

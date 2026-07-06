#ifndef RESEARCH_MPM_RIGID_M1_H
#define RESEARCH_MPM_RIGID_M1_H

// M1 — PIC-Rigid (basic mass-weighted).
// Already implemented. Re-export.

#include "basements/physics/coupling/mpm_rigid_coupler_pic.h"

namespace research { namespace mpm_rigid {
    using M1 = basements::physics::coupling::MPMRigidCouplerPIC;
} }

#endif

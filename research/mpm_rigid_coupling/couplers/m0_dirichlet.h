#ifndef RESEARCH_MPM_RIGID_M0_H
#define RESEARCH_MPM_RIGID_M0_H

// M0 — Dirichlet velocity BC (MVP).
// Already implemented in the main engine. This header re-exports it under
// the M0 name used by the comparison benchmark.

#include "basements/physics/coupling/mpm_rigid_coupler.h"

namespace research { namespace mpm_rigid {
    using M0 = basements::physics::coupling::MPMRigidCoupler;
} }

#endif

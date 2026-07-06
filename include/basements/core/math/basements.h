#ifndef BASEMENTS_H
#define BASEMENTS_H

// ============================================================
// Basements Library - Unified Header
// ============================================================

// Core & Math (Foundation)
#include "basements/core/math/common.h"
#include "basements/core/math/vec2.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/vec4.h"
#include "basements/core/math/quaternion.h"
#include "basements/core/math/matrix3.h"
#include "basements/core/math/matrix4.h"
#include "basements/core/math/dual_quaternion.h"
#include "basements/core/math/transform.h"
#include "basements/core/math/advanced_numerics.h"
#include "basements/core/math/extended_metrics.h"

// Physics Engine (Simulation)
#include "basements/physics/rigid_body.h"
#include "basements/physics/rigid_body_soa.h"

// Collision Detection (Geometry)
#include "basements/physics/collision/shape.h"
#include "basements/physics/collision/primitives.h"
#include "basements/physics/collision/aabb.h"
#include "basements/physics/collision/spatial_hash.h"
#include "basements/physics/collision/gjk.h"
#include "basements/physics/collision/epa.h"

// Dynamics (Solver & Integrator)
#include "basements/physics/dynamics/constraints.h"
#include "basements/physics/dynamics/solver.h"
#include "basements/physics/dynamics/integrator.h"

// Physics Formulas (Knowledge Base)
#include "basements/physics/formulas/kinematics.h"
#include "basements/physics/formulas/dynamics.h"
#include "basements/physics/formulas/energy.h"
#include "basements/physics/formulas/rotation.h"
#include "basements/physics/formulas/thermodynamics.h"
#include "basements/physics/formulas/electromagnetism.h"
#include "basements/physics/formulas/fluid_dynamics.h"
#include "basements/physics/formulas/waves.h"
#include "basements/physics/formulas/optics.h"
#include "basements/physics/formulas/relativity.h"
#include "basements/physics/formulas/quantum.h"
#include "basements/physics/formulas/geophysics.h"
#include "basements/physics/formulas/atmospheric.h"
#include "basements/physics/formulas/oceanography.h"
#include "basements/physics/formulas/astronomy.h"
#include "basements/physics/formulas/geometry.h"

namespace basements {
    // Namespace alias for convenience
    namespace bm = basements::math;
    namespace bp = basements::physics;
    namespace bd = basements::dynamics;
    namespace bf = basements::physics::formulas;
}

#endif // BASEMENTS_H

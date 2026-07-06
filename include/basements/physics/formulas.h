#ifndef BASEMENTS_PHYSICS_FORMULAS_H
#define BASEMENTS_PHYSICS_FORMULAS_H

/**
 * @file formulas.h
 * @brief Unified header for all physics formulas
 * 
 * This header provides access to comprehensive physics and engineering
 * formulas organized by category. All functions are GPU-compatible
 * (marked with HOST_DEVICE) and support various input/output combinations.
 * 
 * Categories:
 * - Kinematics: Motion, velocity, acceleration
 * - Dynamics: Forces, momentum, impulse
 * - Energy: Kinetic, potential, work, power
 * - Rotation: Angular motion, torque, inertia
 * - Thermodynamics: Heat, temperature, entropy
 * - Electromagnetism: Electric/magnetic fields, circuits
 * - Optics: Reflection, refraction, lenses, interference
 * - Geometry: Distances, projections, intersections
 * 
 * @example
 * #include <basements/physics/formulas.h>
 * 
 * using namespace basements::physics::formulas;
 * 
 * // Calculate velocity from kinetic energy
 * float v = energy::velocity_from_kinetic_energy(100.0f, 2.0f);
 * 
 * // Calculate torque from lever arm and force
 * Vec3 tau = rotation::torque_from_lever_arm_force(r, F);
 * 
 * // Refract light through medium
 * Vec3 refracted = optics::refract_vector(incident, normal, n1, n2);
 */

// Include all formula categories
#include "formulas/kinematics.h"
#include "formulas/dynamics.h"
#include "formulas/energy.h"
#include "formulas/rotation.h"
#include "formulas/thermodynamics.h"
#include "formulas/electromagnetism.h"
#include "formulas/optics.h"
#include "formulas/geometry.h"
#include "formulas/fluid_dynamics.h"
#include "formulas/waves.h"
#include "formulas/relativity.h"
#include "formulas/quantum.h"
#include "formulas/geophysics.h"
#include "formulas/atmospheric.h"
#include "formulas/oceanography.h"
#include "formulas/astronomy.h"
#include "formulas/phase_field.h"

namespace basements {
namespace physics {
namespace formulas {

// Convenience: Bring all formula namespaces into formulas::
using namespace kinematics;
using namespace dynamics;
using namespace energy;
using namespace rotation;
using namespace thermodynamics;
using namespace electromagnetism;
using namespace optics;
using namespace geometry;
using namespace fluid_dynamics;
using namespace waves;
using namespace relativity;
using namespace quantum;
using namespace geophysics;
using namespace atmospheric;
using namespace oceanography;
using namespace astronomy;
using namespace phase_field;

} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_H

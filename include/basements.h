#ifndef BASEMENTS_H
#define BASEMENTS_H

/**
 * @file basements.h
 * @brief Unified header for Basements Physics Engine
 * 
 * Include this single header to access all mathematical primitives:
 * - Vec3: 3D vectors with SIMD optimization
 * - Quaternion: Rotation representation
 * - Matrix3: 3x3 matrices for rotations and inertia tensors
 * - Matrix4: 4x4 transformation matrices
 * - Dual Quaternion: Combined rotation and translation
 * - Common utilities and constants
 * 
 * @example
 * #include <basements/core/math/basements.h>
 * 
 * using namespace basements::math;
 * 
 * Vec3 v(1, 2, 3);
 * Quaternion q = Quaternion::from_axis_angle(Vec3::unit_z(), PI / 4.0f);
 * Vec3 rotated = q.rotate(v);
 */

// Core mathematical primitives
#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/core/math/matrix3.h"
#include "basements/core/math/matrix4.h"
#include "basements/core/math/dual_quaternion.h"

// Namespace alias for convenience
namespace bm = basements::math;

#endif // BASEMENTS_H

#ifndef BASEMENTS_IO_URDF_PHYSICS_BRIDGE_H
#define BASEMENTS_IO_URDF_PHYSICS_BRIDGE_H

/**
 * @file urdf_physics_bridge.h
 * @brief Spawn a parsed URDFRobot into a PhysicsWorldGPU as bodies + joints.
 *
 * Mapping policy:
 *   URDF type     → PhysicsWorld joint    Notes
 *   ─────────────────────────────────────────────────────────────────────
 *   REVOLUTE      → HINGE                 limits enabled, axis preserved
 *   CONTINUOUS    → HINGE                 limits disabled
 *   PRISMATIC     → SLIDER                limits enabled, axis preserved
 *   FIXED         → FIXED                 relative pose locked
 *   FLOATING      → (no joint)            child link is free 6-DOF
 *   PLANAR        → (no joint, warning)   no native PLANAR joint — falls back
 *                                         to FLOATING; caller must constrain.
 *
 * Multi-DOF / floating-base handling:
 *   A URDF "floating base" is just a link with no parent joint (typical) OR
 *   a FLOATING joint. In either case the bridge creates a DYNAMIC body with
 *   no constraint and the caller can drive it freely.
 *
 * Forward kinematics:
 *   Each link's world pose is computed by composing parent joint origins down
 *   from the root link (zero joint coordinates — rest pose). For closed loops
 *   the bridge picks the first parent encountered (URDF is a tree by spec).
 */

#include "basements/io/urdf_types.h"
#include "basements/engine/physics_world_gpu.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace basements {
namespace io {

// Per-joint coordinate to seed forward kinematics:
//   REVOLUTE / CONTINUOUS — angle in radians about joint axis
//   PRISMATIC             — translation in meters along joint axis
//   FIXED / FLOATING /
//     PLANAR              — silently ignored (no DOF or unsupported)
using URDFJointPositions = std::unordered_map<std::string, float>;

struct URDFBridgeOptions {
    // World-space pose of the root link.
    basements::math::Vec3       base_position    = basements::math::Vec3::zero();
    basements::math::Quaternion base_orientation = basements::math::Quaternion::identity();

    // If a URDF link has zero/missing inertial, fall back to this mass so the
    // body is still a valid DYNAMIC entity.
    float default_link_mass     = 1.0f;

    // Override: treat the root link as STATIC (typical for fixed-base manipulators).
    // If false, the root is DYNAMIC and any FLOATING joint above it adds no constraint.
    bool  fix_base              = false;

    // Default collision half-extents when no <collision> element is present.
    basements::math::Vec3 default_half_extents{0.05f, 0.05f, 0.05f};

    // Seed each revolute/prismatic joint at a non-zero coordinate. Useful for
    // dropping a manipulator into its MoveIt initial pose, or for sweeping
    // configurations in a unit test. Missing joints default to 0.
    URDFJointPositions initial_joint_positions;
};

struct URDFBridgeResult {
    // Per-link body handle (parallel to URDFRobot::links order).
    std::vector<basements::engine::BodyHandle>       link_bodies;
    // Per-joint constraint handle (parallel to URDFRobot::joints order).
    // Entries are invalid handles for FLOATING / PLANAR / unsupported.
    std::vector<basements::engine::ConstraintHandle> joint_constraints;

    std::unordered_map<std::string, basements::engine::BodyHandle>       link_by_name;
    std::unordered_map<std::string, basements::engine::ConstraintHandle> joint_by_name;

    std::vector<std::string> warnings;

    bool ok() const { return !link_bodies.empty(); }
};

class URDFPhysicsBridge {
public:
    // Spawn the robot. Returns an empty result with warnings if the URDF is
    // structurally invalid (cyclic parent chain, missing link reference).
    static URDFBridgeResult spawn(const URDFRobot& robot,
                                  basements::engine::PhysicsWorldGPU& world,
                                  const URDFBridgeOptions& opts = {});
};

} // namespace io
} // namespace basements

#endif // BASEMENTS_IO_URDF_PHYSICS_BRIDGE_H

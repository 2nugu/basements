#ifndef BASEMENTS_DYNAMICS_JOINTS_H
#define BASEMENTS_DYNAMICS_JOINTS_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/core/math/matrix3.h"

namespace basements {
namespace dynamics {

using namespace basements::math;

// ============================================================================
// Joint Type Enumeration
// ============================================================================

enum class JointType : int {
    BALL_SOCKET = 0,  ///< 3 positional DOF locked (free rotation)
    HINGE = 1,        ///< 5 DOF locked (rotation about single axis)
    SLIDER = 2,       ///< 5 DOF locked (translation along single axis)
    FIXED = 3         ///< All 6 DOF locked
};

// ============================================================================
// Ball-Socket Joint (Spherical Joint)
// ============================================================================

/**
 * @brief Ball-Socket (Spherical) Joint
 * 
 * Constrains two bodies to share a common anchor point while allowing
 * free rotation about that point. Locks 3 translational DOF.
 * 
 * Constraint: p_A + r_A = p_B + r_B  (world space anchor points coincide)
 * 
 * Where:
 *   p_A, p_B = body positions
 *   r_A = R_A * local_anchor_a  (rotated local anchor)
 *   r_B = R_B * local_anchor_b
 */
struct BallSocketJoint {
    // Body indices
    int body_a_index;
    int body_b_index;
    
    // Local anchor points (in body-local coordinates)
    Vec3 local_anchor_a;
    Vec3 local_anchor_b;
    
    // Solver cache
    Vec3 r_a;         ///< World-space anchor offset from body A
    Vec3 r_b;         ///< World-space anchor offset from body B
    Matrix3 K_inv;    ///< Inverse effective mass matrix (3x3)
    Vec3 bias;        ///< Position correction bias
    Vec3 accumulated_impulse;  ///< Warm starting
    
    HOST_DEVICE BallSocketJoint()
        : body_a_index(-1)
        , body_b_index(-1)
        , local_anchor_a(Vec3::zero())
        , local_anchor_b(Vec3::zero())
        , r_a(Vec3::zero())
        , r_b(Vec3::zero())
        , bias(Vec3::zero())
        , accumulated_impulse(Vec3::zero())
    {}
    
    HOST_DEVICE BallSocketJoint(int a, int b, const Vec3& anchor_a, const Vec3& anchor_b)
        : body_a_index(a)
        , body_b_index(b)
        , local_anchor_a(anchor_a)
        , local_anchor_b(anchor_b)
        , r_a(Vec3::zero())
        , r_b(Vec3::zero())
        , bias(Vec3::zero())
        , accumulated_impulse(Vec3::zero())
    {}
};

// ============================================================================
// Hinge Joint (Revolute Joint)
// ============================================================================

/**
 * @brief Hinge (Revolute) Joint
 * 
 * Constrains two bodies to rotate about a single axis. Combines a ball-socket
 * constraint with an angular constraint that locks 2 rotational DOF.
 * 
 * Constraints:
 *   1. Position: p_A + r_A = p_B + r_B (same as ball-socket)
 *   2. Angular:  axis_A x axis_B = 0 (axes aligned, 2 scalar constraints)
 * 
 * Optional features:
 *   - Angle limits (lower/upper bounds)
 *   - Motor (target velocity with max torque)
 */
struct HingeJoint {
    // Body indices
    int body_a_index;
    int body_b_index;
    
    // Local anchor points
    Vec3 local_anchor_a;
    Vec3 local_anchor_b;
    
    // Hinge axis (in body-local coordinates)
    Vec3 local_axis_a;
    Vec3 local_axis_b;
    
    // Angle limits (radians)
    bool enable_limits;
    float angle_lower_limit;
    float angle_upper_limit;
    
    // Motor
    bool enable_motor;
    float motor_target_velocity;  ///< Target angular velocity (rad/s)
    float motor_max_torque;       ///< Maximum motor torque (N·m)
    
    // Solver cache - positional (same as ball-socket)
    Vec3 r_a;
    Vec3 r_b;
    Matrix3 K_inv_pos;
    Vec3 bias_pos;
    Vec3 accumulated_impulse_pos;
    
    // Solver cache - angular
    Vec3 axis_a_world;   ///< World-space hinge axis from body A
    Vec3 axis_b_world;   ///< World-space hinge axis from body B
    Vec3 tangent1;       ///< First perpendicular axis
    Vec3 tangent2;       ///< Second perpendicular axis
    float effective_mass_angular[2];  ///< For the 2 angular constraints
    float accumulated_impulse_angular[2];
    
    // Motor cache
    float current_angle;
    float effective_mass_motor;
    float accumulated_impulse_motor;
    
    // Limit cache
    bool limit_active;
    float limit_direction;       ///< +1.0 or -1.0
    float limit_effective_mass;
    float limit_bias;
    float accumulated_impulse_limit;
    
    HOST_DEVICE HingeJoint()
        : body_a_index(-1)
        , body_b_index(-1)
        , local_anchor_a(Vec3::zero())
        , local_anchor_b(Vec3::zero())
        , local_axis_a(Vec3(1, 0, 0))
        , local_axis_b(Vec3(1, 0, 0))
        , enable_limits(false)
        , angle_lower_limit(-PI)
        , angle_upper_limit(PI)
        , enable_motor(false)
        , motor_target_velocity(0.0f)
        , motor_max_torque(0.0f)
        , r_a(Vec3::zero())
        , r_b(Vec3::zero())
        , bias_pos(Vec3::zero())
        , accumulated_impulse_pos(Vec3::zero())
        , current_angle(0.0f)
        , effective_mass_motor(0.0f)
        , accumulated_impulse_motor(0.0f)
        , limit_active(false)
        , limit_direction(0.0f)
        , limit_effective_mass(0.0f)
        , limit_bias(0.0f)
        , accumulated_impulse_limit(0.0f)
    {
        effective_mass_angular[0] = 0.0f;
        effective_mass_angular[1] = 0.0f;
        accumulated_impulse_angular[0] = 0.0f;
        accumulated_impulse_angular[1] = 0.0f;
    }
};

// ============================================================================
// Slider Joint (Prismatic Joint)
// ============================================================================

/**
 * @brief Slider (Prismatic) Joint
 * 
 * Constrains two bodies to translate along a single axis with no relative
 * rotation. Locks 5 DOF (2 translations + 3 rotations).
 */
struct SliderJoint {
    int body_a_index;
    int body_b_index;
    
    Vec3 local_anchor_a;
    Vec3 local_anchor_b;
    Vec3 local_axis_a;  ///< Sliding axis in body A's local space
    
    // Position limits along axis
    bool enable_limits;
    float position_lower_limit;
    float position_upper_limit;
    
    // Motor
    bool enable_motor;
    float motor_target_velocity;
    float motor_max_force;
    
    // Solver cache
    Vec3 r_a;
    Vec3 r_b;
    Vec3 axis_world;
    Vec3 perp1_world;
    Vec3 perp2_world;
    
    float accumulated_impulse_perp[2];  ///< Perpendicular translation
    float accumulated_impulse_rot[3];   ///< Rotation lock
    float accumulated_impulse_limit;
    float accumulated_impulse_motor;
    
    // Limit cache  
    bool limit_active;
    float limit_direction;
    float limit_effective_mass;
    float limit_bias;
    float current_position;
    
    HOST_DEVICE SliderJoint()
        : body_a_index(-1)
        , body_b_index(-1)
        , local_anchor_a(Vec3::zero())
        , local_anchor_b(Vec3::zero())
        , local_axis_a(Vec3(1, 0, 0))
        , enable_limits(false)
        , position_lower_limit(-1e10f)
        , position_upper_limit(1e10f)
        , enable_motor(false)
        , motor_target_velocity(0.0f)
        , motor_max_force(0.0f)
        , r_a(Vec3::zero())
        , r_b(Vec3::zero())
        , accumulated_impulse_limit(0.0f)
        , accumulated_impulse_motor(0.0f)
        , limit_active(false)
        , limit_direction(0.0f)
        , limit_effective_mass(0.0f)
        , limit_bias(0.0f)
        , current_position(0.0f)
    {
        accumulated_impulse_perp[0] = 0.0f;
        accumulated_impulse_perp[1] = 0.0f;
        accumulated_impulse_rot[0] = 0.0f;
        accumulated_impulse_rot[1] = 0.0f;
        accumulated_impulse_rot[2] = 0.0f;
    }
};

// ============================================================================
// Fixed Joint (Weld Joint)
// ============================================================================

/**
 * @brief Fixed (Weld) Joint
 * 
 * Rigidly connects two bodies with no relative motion. Locks all 6 DOF.
 * 
 * Constraints:
 *   1. Position: p_A + r_A = p_B + r_B (3 constraints)
 *   2. Orientation: q_A * q_ref = q_B (3 constraints via angular velocity)
 */
struct FixedJoint {
    int body_a_index;
    int body_b_index;
    
    Vec3 local_anchor_a;
    Vec3 local_anchor_b;
    
    // Reference relative orientation (q_B_local = q_A^-1 * q_B at creation)
    Quaternion relative_orientation;
    
    // Solver cache - positional
    Vec3 r_a;
    Vec3 r_b;
    Matrix3 K_inv_pos;
    Vec3 bias_pos;
    Vec3 accumulated_impulse_pos;
    
    // Solver cache - rotational
    Matrix3 K_inv_rot;
    Vec3 bias_rot;
    Vec3 accumulated_impulse_rot;
    
    HOST_DEVICE FixedJoint()
        : body_a_index(-1)
        , body_b_index(-1)
        , local_anchor_a(Vec3::zero())
        , local_anchor_b(Vec3::zero())
        , relative_orientation(Quaternion::identity())
        , r_a(Vec3::zero())
        , r_b(Vec3::zero())
        , bias_pos(Vec3::zero())
        , accumulated_impulse_pos(Vec3::zero())
        , bias_rot(Vec3::zero())
        , accumulated_impulse_rot(Vec3::zero())
    {}
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_JOINTS_H

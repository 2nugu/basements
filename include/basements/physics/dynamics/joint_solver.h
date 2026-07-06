#ifndef BASEMENTS_DYNAMICS_JOINT_SOLVER_H
#define BASEMENTS_DYNAMICS_JOINT_SOLVER_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/matrix3.h"
#include "basements/physics/dynamics/joints.h"
#include "basements/physics/rigid_body.h"
#include <iostream>

namespace basements {
namespace dynamics {

using namespace basements::math;
using namespace basements::physics;

/**
 * @brief Solver for joint constraints using Sequential Impulse method
 * 
 * Similar to the contact constraint Solver, this class provides static
 * methods for pre-solving and velocity solving of various joint types.
 */
class JointSolver {
public:
    // ========================================================================
    // Ball-Socket Joint Solver
    // ========================================================================
    
    /**
     * @brief Pre-solve ball-socket joints (compute effective mass, apply warm start)
     */
    HOST_DEVICE static void pre_solve_ball_socket(
        BallSocketJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies,
        float dt
    ) {
        // Baumgarte stabilization: typically 0.1-0.3
        const float beta = 0.2f;
        const float bias_factor = beta / dt;
        
        for (int i = 0; i < num_joints; ++i) {
            BallSocketJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            // Compute world-space anchor offsets
            j.r_a = bA.orientation.rotate(j.local_anchor_a);
            j.r_b = bB.orientation.rotate(j.local_anchor_b);
            
            // Compute effective mass matrix K
            // K = (1/mA + 1/mB) * I - [rA]x * IA_inv * [rA]x - [rB]x * IB_inv * [rB]x
            Matrix3 K = Matrix3::identity() * (bA.inv_mass + bB.inv_mass);
            
            // Skew symmetric matrix [r]x such that [r]x * v = r.cross(v)
            Matrix3 rA_skew = skew_symmetric(j.r_a);
            Matrix3 rB_skew = skew_symmetric(j.r_b);
            
            // Use world-space inertia tensors
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();
            
            K = K + rA_skew * IA_inv * rA_skew.transposed();
            K = K + rB_skew * IB_inv * rB_skew.transposed();
            
            // Invert K for effective mass
            j.K_inv = K.inversed();
            
            // Compute position error (constraint violation)
            Vec3 pA = bA.position + j.r_a;
            Vec3 pB = bB.position + j.r_b;
            Vec3 error = pB - pA;
            
            // Bias for Baumgarte stabilization
            j.bias = error * (bias_factor);
            
            // Warm starting
            if (j.accumulated_impulse.length_squared() > EPSILON) {
                apply_impulse_pair(bA, bB, j.accumulated_impulse, j.r_a, j.r_b,
                                   bA.get_inv_inertia_world(), bB.get_inv_inertia_world());
            }
        }
    }
    
    HOST_DEVICE static void solve_velocity_ball_socket(
        BallSocketJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies
    ) {
        for (int i = 0; i < num_joints; ++i) {
            BallSocketJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            // Relative velocity at anchor points
            Vec3 vA = bA.linear_velocity + bA.angular_velocity.cross(j.r_a);
            Vec3 vB = bB.linear_velocity + bB.angular_velocity.cross(j.r_b);
            Vec3 Cdot = vB - vA;
            
            // Compute impulse: lambda = -K_inv * (Cdot + bias)
            Vec3 impulse = j.K_inv * (-(Cdot + j.bias));

            // Accumulate impulse
            j.accumulated_impulse = j.accumulated_impulse + impulse;
            
            // Apply impulse using world-space inertia
            apply_impulse_pair(bA, bB, impulse, j.r_a, j.r_b,
                               bA.get_inv_inertia_world(), bB.get_inv_inertia_world());
        }
    }

    // ========================================================================
    // Hinge Joint Solver
    // ========================================================================
    
    HOST_DEVICE static void pre_solve_hinge(
        HingeJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies,
        float dt
    ) {
        const float beta = 0.2f;
        const float bias_factor = beta / dt;
        
        for (int i = 0; i < num_joints; ++i) {
            HingeJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            // === Positional constraint (same as ball-socket) ===
            j.r_a = bA.orientation.rotate(j.local_anchor_a);
            j.r_b = bB.orientation.rotate(j.local_anchor_b);
            
            Matrix3 K = Matrix3::identity() * (bA.inv_mass + bB.inv_mass);
            Matrix3 rA_skew = skew_symmetric(j.r_a);
            Matrix3 rB_skew = skew_symmetric(j.r_b);
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();
            K = K + rA_skew * IA_inv * rA_skew.transposed();
            K = K + rB_skew * IB_inv * rB_skew.transposed();
            j.K_inv_pos = K.inversed();
            
            Vec3 pA = bA.position + j.r_a;
            Vec3 pB = bB.position + j.r_b;
            j.bias_pos = (pB - pA) * (bias_factor);
            
            // === Angular constraint ===
            j.axis_a_world = bA.orientation.rotate(j.local_axis_a);
            j.axis_b_world = bB.orientation.rotate(j.local_axis_b);
            
            // Compute tangent basis perpendicular to axis
            compute_perpendicular_basis(j.axis_a_world, j.tangent1, j.tangent2);
            
            // Effective mass for angular constraints
            // For each tangent t: effective_mass = 1 / (t^T * (IA_inv + IB_inv) * t)
            Matrix3 I_sum = IA_inv + IB_inv;
            j.effective_mass_angular[0] = 1.0f / j.tangent1.dot(I_sum * j.tangent1);
            j.effective_mass_angular[1] = 1.0f / j.tangent2.dot(I_sum * j.tangent2);
            
            // Motor effective mass (about hinge axis)
            j.effective_mass_motor = 1.0f / j.axis_a_world.dot(I_sum * j.axis_a_world);
            
            // Warm starting
            if (j.accumulated_impulse_pos.length_squared() > EPSILON) {
                apply_impulse_pair(bA, bB, j.accumulated_impulse_pos, j.r_a, j.r_b, IA_inv, IB_inv);
            }
            
            
            Vec3 angular_impulse = j.tangent1 * j.accumulated_impulse_angular[0]
                                 + j.tangent2 * j.accumulated_impulse_angular[1]
                                 + j.axis_a_world * j.accumulated_impulse_motor;
            
            bA.angular_velocity = bA.angular_velocity - IA_inv * angular_impulse;
            bB.angular_velocity = bB.angular_velocity + IB_inv * angular_impulse;
            
            // === Angle limits ===
            if (j.enable_limits) {
                float current_angle = measure_hinge_angle(bA.orientation, bB.orientation, j.axis_a_world);
                j.current_angle = current_angle;
                
                float violation, direction;
                if (is_violating_limit(current_angle, j.angle_lower_limit, j.angle_upper_limit, violation, direction)) {
                    j.limit_active = true;
                    j.limit_direction = direction;
                    Matrix3 I_sum = IA_inv + IB_inv;
                    j.limit_effective_mass = 1.0f / j.axis_a_world.dot(I_sum * j.axis_a_world);
                    j.limit_bias = 0.0f;  // No Baumgarte for limits: velocity constraint is sufficient
                    
                    Vec3 limit_impulse = j.axis_a_world * (direction * j.accumulated_impulse_limit);
                    bA.angular_velocity = bA.angular_velocity - IA_inv * limit_impulse;
                    bB.angular_velocity = bB.angular_velocity + IB_inv * limit_impulse;
                } else {
                    j.limit_active = false;
                    j.accumulated_impulse_limit = 0.0f;
                }
            } else {
                j.limit_active = false;
            }
        }
    }
    
    HOST_DEVICE static void solve_velocity_hinge(
        HingeJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies
    ) {
        for (int i = 0; i < num_joints; ++i) {
            HingeJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();
            
            // === Positional constraint ===
            Vec3 vA = bA.linear_velocity + bA.angular_velocity.cross(j.r_a);
            Vec3 vB = bB.linear_velocity + bB.angular_velocity.cross(j.r_b);
            Vec3 Cdot_pos = vB - vA;
            
            Vec3 impulse_pos = j.K_inv_pos * (-(Cdot_pos + j.bias_pos));
            j.accumulated_impulse_pos = j.accumulated_impulse_pos + impulse_pos;
            apply_impulse_pair(bA, bB, impulse_pos, j.r_a, j.r_b, IA_inv, IB_inv);
            
            // === Angular constraint (lock rotation about tangent axes) ===
            Vec3 w_rel = bB.angular_velocity - bA.angular_velocity;
            
            for (int k = 0; k < 2; ++k) {
                Vec3 tangent = (k == 0) ? j.tangent1 : j.tangent2;
                float Cdot_angular = w_rel.dot(tangent);
                float lambda = -j.effective_mass_angular[k] * Cdot_angular;
                
                j.accumulated_impulse_angular[k] += lambda;
                
                Vec3 impulse = tangent * lambda;
                bA.angular_velocity = bA.angular_velocity - IA_inv * impulse;
                bB.angular_velocity = bB.angular_velocity + IB_inv * impulse;
            }
            
            // === Motor (optional) ===
            if (j.enable_motor) {
                Vec3 w_rel_updated = bB.angular_velocity - bA.angular_velocity;
                float w_axis = w_rel_updated.dot(j.axis_a_world);
                float Cdot_motor = w_axis - j.motor_target_velocity;
                float lambda_motor = -j.effective_mass_motor * Cdot_motor;
                
                // Clamp motor impulse
                float old_impulse = j.accumulated_impulse_motor;
                float max_impulse = j.motor_max_torque; // Already per-iteration
                j.accumulated_impulse_motor = fmaxf(-max_impulse, 
                                                     fminf(max_impulse, old_impulse + lambda_motor));
                lambda_motor = j.accumulated_impulse_motor - old_impulse;
                
                Vec3 motor_impulse = j.axis_a_world * lambda_motor;
                bA.angular_velocity = bA.angular_velocity - IA_inv * motor_impulse;
                bB.angular_velocity = bB.angular_velocity + IB_inv * motor_impulse;
            }
            
            // === Angle limits (unilateral) ===
            if (j.limit_active) {
                Vec3 w_rel = bB.angular_velocity - bA.angular_velocity;
                float Cdot_limit = w_rel.dot(j.axis_a_world) * j.limit_direction;
                
                float lambda_limit = -j.limit_effective_mass * (Cdot_limit + j.limit_bias);
                
                // Unilateral: clamp to [0, ∞)
                float old_limit = j.accumulated_impulse_limit;
                j.accumulated_impulse_limit = fmaxf(0.0f, old_limit + lambda_limit);
                lambda_limit = j.accumulated_impulse_limit - old_limit;
                
                Vec3 limit_impulse = j.axis_a_world * (j.limit_direction * lambda_limit);
                bA.angular_velocity = bA.angular_velocity - IA_inv * limit_impulse;
                bB.angular_velocity = bB.angular_velocity + IB_inv * limit_impulse;
            }
        }
    }

    // ========================================================================
    // Fixed Joint Solver
    // ========================================================================
    
    HOST_DEVICE static void pre_solve_fixed(
        FixedJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies,
        float dt
    ) {
        const float beta = 0.2f;
        const float bias_factor = beta / dt;
        
        for (int i = 0; i < num_joints; ++i) {
            FixedJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();
            
            // === Positional constraint ===
            j.r_a = bA.orientation.rotate(j.local_anchor_a);
            j.r_b = bB.orientation.rotate(j.local_anchor_b);
            
            Matrix3 K_pos = Matrix3::identity() * (bA.inv_mass + bB.inv_mass);
            Matrix3 rA_skew = skew_symmetric(j.r_a);
            Matrix3 rB_skew = skew_symmetric(j.r_b);
            K_pos = K_pos + rA_skew * IA_inv * rA_skew.transposed();
            K_pos = K_pos + rB_skew * IB_inv * rB_skew.transposed();
            j.K_inv_pos = K_pos.inversed();
            
            Vec3 pA = bA.position + j.r_a;
            Vec3 pB = bB.position + j.r_b;
            j.bias_pos = (pB - pA) * (bias_factor);
            
            // === Rotational constraint ===
            // Error = q_B * q_ref^-1 * q_A^-1 should be identity
            // We extract the vector part as the angular error
            Quaternion q_error = bB.orientation * j.relative_orientation.conjugate() * bA.orientation.conjugate();
            Vec3 rot_error(q_error.x, q_error.y, q_error.z);
            rot_error = rot_error * 2.0f;  // Small angle approximation
            
            j.K_inv_rot = (IA_inv + IB_inv).inversed();
            j.bias_rot = rot_error * (bias_factor);
            
            // Warm starting
            apply_impulse_pair(bA, bB, j.accumulated_impulse_pos, j.r_a, j.r_b, IA_inv, IB_inv);
            
            bA.angular_velocity = bA.angular_velocity - IA_inv * j.accumulated_impulse_rot;
            bB.angular_velocity = bB.angular_velocity + IB_inv * j.accumulated_impulse_rot;
        }
    }
    
    HOST_DEVICE static void solve_velocity_fixed(
        FixedJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies
    ) {
        for (int i = 0; i < num_joints; ++i) {
            FixedJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();
            
            // === Positional constraint ===
            Vec3 vA = bA.linear_velocity + bA.angular_velocity.cross(j.r_a);
            Vec3 vB = bB.linear_velocity + bB.angular_velocity.cross(j.r_b);
            Vec3 Cdot_pos = vB - vA;
            
            Vec3 impulse_pos = j.K_inv_pos * (-(Cdot_pos + j.bias_pos));
            j.accumulated_impulse_pos = j.accumulated_impulse_pos + impulse_pos;
            apply_impulse_pair(bA, bB, impulse_pos, j.r_a, j.r_b, IA_inv, IB_inv);
            
            // === Rotational constraint ===
            Vec3 w_rel = bB.angular_velocity - bA.angular_velocity;
            Vec3 impulse_rot = j.K_inv_rot * (-(w_rel + j.bias_rot));
            j.accumulated_impulse_rot = j.accumulated_impulse_rot + impulse_rot;
            
            bA.angular_velocity = bA.angular_velocity - IA_inv * impulse_rot;
            bB.angular_velocity = bB.angular_velocity + IB_inv * impulse_rot;
        }
    }

private:
    // ========================================================================
    // Helper Functions
    // ========================================================================
    
    HOST_DEVICE static void apply_impulse_pair(
        RigidBody& bA, RigidBody& bB,
        const Vec3& impulse, const Vec3& rA, const Vec3& rB,
        const Matrix3& IA_inv, const Matrix3& IB_inv
    ) {
        // Apply to body A (negative direction)
        if (bA.inv_mass > 0.0f) {
            bA.linear_velocity = bA.linear_velocity - impulse * bA.inv_mass;
            bA.angular_velocity = bA.angular_velocity - IA_inv * rA.cross(impulse);
        }
        
        // Apply to body B (positive direction)
        if (bB.inv_mass > 0.0f) {
            bB.linear_velocity = bB.linear_velocity + impulse * bB.inv_mass;
            bB.angular_velocity = bB.angular_velocity + IB_inv * rB.cross(impulse);
        }
    }
    
    // ========================================================================
    // Solver Diagnostics (for debugging and validation)
    // ========================================================================
    
    struct SolverDiagnostics {
        float max_position_error;   ///< Maximum constraint violation (m or rad)
        float total_impulse;         ///< Total impulse magnitude applied
        int active_constraints;      ///< Number of active constraints
        bool converged;              ///< Whether solver converged
        
        HOST_DEVICE SolverDiagnostics()
            : max_position_error(0.0f)
            , total_impulse(0.0f)
            , active_constraints(0)
            , converged(true)
        {}
    };
    
public:
    // ========================================================================
    // Slider Joint Solver
    // ========================================================================
    
    HOST_DEVICE static void pre_solve_slider(
        SliderJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies,
        float dt
    ) {
        const float beta = 0.2f;
        const float bias_factor = beta / dt;
        
        for (int i = 0; i < num_joints; ++i) {
            SliderJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            // Compute world-space anchors and axis
            j.r_a = bA.orientation.rotate(j.local_anchor_a);
            j.r_b = bB.orientation.rotate(j.local_anchor_b);
            j.axis_world = bA.orientation.rotate(j.local_axis_a);
            
            // Compute perpendicular basis
            compute_perpendicular_basis(j.axis_world, j.perp1_world, j.perp2_world);
            
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();
            
            // Warm start perpendicular translation
            for (int k = 0; k < 2; ++k) {
                Vec3 perp = (k == 0) ? j.perp1_world : j.perp2_world;
                Vec3 impulse = perp * j.accumulated_impulse_perp[k];
                apply_impulse_pair(bA, bB, impulse, j.r_a, j.r_b, IA_inv, IB_inv);
            }
            
            // Warm start rotation lock
            Vec3 axes[3] = {Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1)};
            for (int k = 0; k < 3; ++k) {
                Vec3 impulse_rot = axes[k] * j.accumulated_impulse_rot[k];
                bA.angular_velocity = bA.angular_velocity - IA_inv * impulse_rot;
                bB.angular_velocity = bB.angular_velocity + IB_inv * impulse_rot;
            }
            
            // === Position Limits ===
            if (j.enable_limits) {
                Vec3 anchor_world_a = bA.position + j.r_a;
                Vec3 anchor_world_b = bB.position + j.r_b;
                float current_slide_position = measure_slider_position(anchor_world_a, anchor_world_b, j.axis_world);
                j.current_position = current_slide_position;
                
                float limit_violation_amount, limit_push_direction;
                if (is_violating_limit(current_slide_position, j.position_lower_limit, j.position_upper_limit,
                                       limit_violation_amount, limit_push_direction)) {
                    j.limit_active = true;
                    j.limit_direction = limit_push_direction;

                    // Effective mass for translational limit along axis
                    float inv_mass_total = bA.inv_mass + bB.inv_mass;
                    Vec3 ra_cross_axis = j.r_a.cross(j.axis_world);
                    Vec3 rb_cross_axis = j.r_b.cross(j.axis_world);
                    j.limit_effective_mass = 1.0f / (inv_mass_total
                        + ra_cross_axis.dot(IA_inv * ra_cross_axis)
                        + rb_cross_axis.dot(IB_inv * rb_cross_axis));
                    j.limit_bias = 0.0f;  // No Baumgarte for limits: velocity constraint is sufficient
                    
                    // Warm start limit impulse
                    Vec3 warm_limit_impulse = j.axis_world * (limit_push_direction * j.accumulated_impulse_limit);
                    apply_impulse_pair(bA, bB, warm_limit_impulse, j.r_a, j.r_b, IA_inv, IB_inv);
                } else {
                    j.limit_active = false;
                    j.accumulated_impulse_limit = 0.0f;
                }
            } else {
                j.limit_active = false;
            }
        }
    }
    
    HOST_DEVICE static void solve_velocity_slider(
        SliderJoint* joints, int num_joints,
        RigidBody* bodies, int num_bodies
    ) {
        for (int i = 0; i < num_joints; ++i) {
            SliderJoint& j = joints[i];
            if (j.body_a_index < 0 || j.body_b_index < 0) continue;
            
            RigidBody& bA = bodies[j.body_a_index];
            RigidBody& bB = bodies[j.body_b_index];
            
            Matrix3 IA_inv = bA.get_inv_inertia_world();
            Matrix3 IB_inv = bB.get_inv_inertia_world();
            
            // === Perpendicular translation (2 DOF) ===
            for (int k = 0; k < 2; ++k) {
                Vec3 perp = (k == 0) ? j.perp1_world : j.perp2_world;
                
                Vec3 vA = bA.linear_velocity + bA.angular_velocity.cross(j.r_a);
                Vec3 vB = bB.linear_velocity + bB.angular_velocity.cross(j.r_b);
                float Cdot = (vB - vA).dot(perp);
                
                float inv_mass_sum = bA.inv_mass + bB.inv_mass;
                Vec3 ra_cross_perp = j.r_a.cross(perp);
                Vec3 rb_cross_perp = j.r_b.cross(perp);
                
                float K = inv_mass_sum 
                        + ra_cross_perp.dot(IA_inv * ra_cross_perp)
                        + rb_cross_perp.dot(IB_inv * rb_cross_perp);
                
                float lambda = -Cdot / K;
                j.accumulated_impulse_perp[k] += lambda;
                
                Vec3 impulse = perp * lambda;
                apply_impulse_pair(bA, bB, impulse, j.r_a, j.r_b, IA_inv, IB_inv);
            }
            
            // === Rotation lock (3 DOF) ===
            Vec3 w_rel = bB.angular_velocity - bA.angular_velocity;
            Vec3 axes[3] = {Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1)};
            
            for (int k = 0; k < 3; ++k) {
                float Cdot_rot = w_rel.dot(axes[k]);
                
                Matrix3 I_sum = IA_inv + IB_inv;
                float K_rot = axes[k].dot(I_sum * axes[k]);
                
                float lambda_rot = -Cdot_rot / K_rot;
                j.accumulated_impulse_rot[k] += lambda_rot;
                
                Vec3 impulse_rot = axes[k] * lambda_rot;
                bA.angular_velocity = bA.angular_velocity - IA_inv * impulse_rot;
                bB.angular_velocity = bB.angular_velocity + IB_inv * impulse_rot;
            }
            
            // === Position Limits (Unilateral Constraint) ===
            if (j.limit_active) {
                // Compute relative velocity along sliding axis
                Vec3 velocity_a = bA.linear_velocity + bA.angular_velocity.cross(j.r_a);
                Vec3 velocity_b = bB.linear_velocity + bB.angular_velocity.cross(j.r_b);
                float relative_slide_velocity = (velocity_b - velocity_a).dot(j.axis_world);
                
                // Constraint velocity (signed by limit direction)
                float constraint_velocity = relative_slide_velocity * j.limit_direction;
                float lambda_limit = -j.limit_effective_mass * (constraint_velocity + j.limit_bias);
                
                // Unilateral: only push, never pull (clamp to [0, ∞))
                float previous_accumulated = j.accumulated_impulse_limit;
                j.accumulated_impulse_limit = fmaxf(0.0f, previous_accumulated + lambda_limit);
                float applied_lambda = j.accumulated_impulse_limit - previous_accumulated;
                
                // Apply limit impulse along axis
                Vec3 limit_impulse = j.axis_world * (j.limit_direction * applied_lambda);
                apply_impulse_pair(bA, bB, limit_impulse, j.r_a, j.r_b, IA_inv, IB_inv);
            }
        }
    }
    
    // ========================================================================
    // Limit Helper Functions
    // ========================================================================
    
    /**
     * @brief Normalize angle to [-π, +π] range for wrap-around handling
     */
    HOST_DEVICE static float normalize_angle(float angle) {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }
    
    /**
     * @brief Measure current hinge angle (rotation about hinge axis)
     */
    HOST_DEVICE static float measure_hinge_angle(
        const Quaternion& q_A, const Quaternion& q_B,
        const Vec3& axis_world_A
    ) {
        // Relative rotation: q_rel = q_A^-1 * q_B
        Quaternion q_rel = q_A.conjugate() * q_B;
        
        Vec3 rot_axis = q_rel.axis();
        float rot_angle = q_rel.angle();
        
        // Project onto hinge axis to get signed angle
        float sign = rot_axis.dot(axis_world_A) >= 0.0f ? 1.0f : -1.0f;
        return normalize_angle(rot_angle * sign);
    }
    
    /**
     * @brief Measure slider position along axis
     */
    HOST_DEVICE static float measure_slider_position(
        const Vec3& p_A, const Vec3& p_B, const Vec3& axis_world
    ) {
        return (p_B - p_A).dot(axis_world);
    }
    
    /**
     * @brief Check if value violates limit constraint
     * 
     * Supports:
     * - Normal limits: lower < upper (e.g., 0° to 150°)
     * - Reverse limits: lower > upper (e.g., 0° to -150°, actual range is [-150°, 0°])
     * 
     * @param value Current angle or position
     * @param lower Lower limit
     * @param upper Upper limit
     * @param[out] violation Amount of violation (positive value)
     * @param[out] direction Direction to push (+1 toward upper, -1 toward lower)
     * @return true if limit is violated
     */
    HOST_DEVICE static bool is_violating_limit(
        float value, float lower, float upper,
        float& violation, float& direction
    ) {
        if (lower <= upper) {
            // Normal case: valid range is [lower, upper]
            if (value < lower) {
                violation = lower - value;
                direction = +1.0f;  // Push toward upper (increase value)
                return true;
            }
            if (value > upper) {
                violation = value - upper;
                direction = -1.0f;  // Push toward lower (decrease value)
                return true;
            }
        } else {
            // Reverse case: lower > upper
            // Valid range is OUTSIDE (upper, lower)
            // Forbidden zone is (upper, lower)
            if (value > upper && value < lower) {
                // Inside forbidden zone - push to nearest boundary
                float dist_to_upper = value - upper;
                float dist_to_lower = lower - value;
                
                if (dist_to_upper < dist_to_lower) {
                    // Closer to upper, push down
                    violation = dist_to_upper;
                    direction = -1.0f;
                } else {
                    // Closer to lower, push up
                    violation = dist_to_lower;
                    direction = +1.0f;
                }
                return true;
            }
        }
        
        return false;
    }
    
    // ========================================================================
    // Helper Functions
    // ========================================================================
    
    HOST_DEVICE static Matrix3 skew_symmetric(const Vec3& v) {
        return Matrix3(
            0.0f, -v.z, v.y,
            v.z, 0.0f, -v.x,
            -v.y, v.x, 0.0f
        );
    }
    
    HOST_DEVICE static void compute_perpendicular_basis(const Vec3& axis, Vec3& t1, Vec3& t2) {
        if (std::abs(axis.x) >= 0.57735f) {
            t1 = Vec3(axis.y, -axis.x, 0.0f);
        } else {
            t1 = Vec3(0.0f, axis.z, -axis.y);
        }
        t1 = t1.normalized();
        t2 = axis.cross(t1);
    }
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_JOINT_SOLVER_H

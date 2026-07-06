#ifndef BASEMENTS_ENGINE_PHYSICS_WORLD_GPU_H
#define BASEMENTS_ENGINE_PHYSICS_WORLD_GPU_H

/**
 * @file physics_world_gpu.h
 * @brief GPU-accelerated physics world - unified simulation orchestrator
 * 
 * This class manages the complete GPU-side physics simulation pipeline:
 *   1. Broadphase collision detection (Spatial Hash)
 *   2. Narrowphase collision detection (GJK/EPA)
 *   3. Contact constraint solving
 *   4. Joint constraint solving
 *   5. Integration (position/orientation update)
 * 
 * Design principles:
 *   - Structure-of-Arrays (SoA) layout for coalesced GPU memory access
 *   - Handle-based API for safe object references
 *   - Extensible pipeline with configurable stages
 *   - CPU fallback for non-CUDA environments
 */

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/core/math/matrix3.h"

#include "basements/core/types.h"
#include "basements/physics/rigid_body.h"
#include "basements/physics/rigid_body_soa.h"
#include "basements/physics/collision/aabb.h"
#include "basements/physics/collision/spatial_hash.h"
#include "basements/physics/collision/gjk.h"
#include "basements/physics/collision/epa.h"
#include "basements/physics/dynamics/constraints.h"
#include "basements/physics/dynamics/joints.h"
#include "basements/physics/dynamics/joint_solver.h"
#include "basements/physics/dynamics/integrator.h"
#include "basements/physics/dynamics/solver.h"

#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>

namespace basements {
namespace engine {

using namespace basements::math;
using namespace basements::physics;
using namespace basements::collision;
using namespace basements::dynamics;

// ============================================================================
// Forward Declarations
// ============================================================================

class PhysicsWorldGPU;

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for PhysicsWorldGPU initialization
 * 
 * Sensible defaults are provided. Override as needed.
 */
struct PhysicsWorldConfig {
    // Simulation parameters
    float fixed_time_step           = config::DEFAULT_TIME_STEP;  // 1/60 sec
    int solver_iterations           = config::DEFAULT_SOLVER_ITERATIONS;  // 10
    float baumgarte_factor          = config::DEFAULT_BAUMGARTE_FACTOR;  // 0.2
    
    // World bounds
    Vec3 gravity                    = Vec3(0.0f, config::DEFAULT_GRAVITY, 0.0f);
    
    // Memory allocation hints
    size_t initial_body_capacity    = config::DEFAULT_BODY_CAPACITY;  // 10000
    size_t initial_contact_capacity = config::DEFAULT_CONTACT_CAPACITY;  // 50000
    size_t initial_joint_capacity   = 5000;
    
    // Broadphase
    float spatial_hash_cell_size    = config::DEFAULT_CELL_SIZE;  // 2.0m
    int spatial_hash_table_size     = 8192;
    
    // Sleep/wake system
    bool enable_sleeping            = true;
    float sleep_linear_threshold    = config::SLEEP_LINEAR_THRESHOLD;
    float sleep_angular_threshold   = config::SLEEP_ANGULAR_THRESHOLD;
    float sleep_time_threshold      = config::SLEEP_TIME_THRESHOLD;
    
    // GPU settings
    bool prefer_gpu                 = true;  // Use GPU if available
    int cuda_device_id              = 0;
};

// ============================================================================
// Simulation Statistics
// ============================================================================

/**
 * @brief Performance and state statistics for the physics world
 */
struct SimulationStatistics {
    // Counts
    size_t active_body_count        = 0;
    size_t sleeping_body_count      = 0;
    size_t contact_count            = 0;
    size_t joint_count              = 0;
    
    // Timing (milliseconds)
    float broadphase_time_ms        = 0.0f;
    float narrowphase_time_ms       = 0.0f;
    float solver_time_ms            = 0.0f;
    float integration_time_ms       = 0.0f;
    float total_step_time_ms        = 0.0f;
    
    // Cumulative
    uint64_t total_steps            = 0;
    double total_simulation_time    = 0.0;  // seconds
};

// ============================================================================
// Collision Pair (Broadphase Output)
// ============================================================================

/**
 * @brief A potential collision pair from broadphase
 */
struct CollisionPair {
    BodyHandle body_a;
    BodyHandle body_b;
    
    HOST_DEVICE CollisionPair() = default;
    HOST_DEVICE CollisionPair(BodyHandle a, BodyHandle b) : body_a(a), body_b(b) {}
    
    HOST_DEVICE bool operator==(const CollisionPair& other) const {
        return (body_a == other.body_a && body_b == other.body_b) ||
               (body_a == other.body_b && body_b == other.body_a);
    }
};

// ============================================================================
// Joint Descriptor (for creating joints)
// ============================================================================

/**
 * @brief Descriptor for creating various joint types
 */
struct JointDescriptor {
    JointType type                  = JointType::BALL_SOCKET;
    
    BodyHandle body_a;
    BodyHandle body_b;
    
    // Anchor points (in body-local coordinates)
    Vec3 local_anchor_a             = Vec3::zero();
    Vec3 local_anchor_b             = Vec3::zero();
    
    // Axis (for hinge/slider joints)
    Vec3 local_axis_a               = Vec3(1.0f, 0.0f, 0.0f);
    Vec3 local_axis_b               = Vec3(1.0f, 0.0f, 0.0f);
    
    // Limits (angle for hinge, position for slider)
    bool enable_limits              = false;
    float lower_limit               = 0.0f;
    float upper_limit               = 0.0f;
    
    // Motor
    bool enable_motor               = false;
    float motor_target_velocity     = 0.0f;
    float motor_max_force_or_torque = 0.0f;
};

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief Callback for body-body contact events
 */
using ContactCallback = std::function<void(BodyHandle body_a, BodyHandle body_b, const Vec3& contact_point, const Vec3& normal)>;

/**
 * @brief Callback for body queries (raycast, overlap, etc.)
 */
using QueryCallback = std::function<bool(BodyHandle body)>;

// ============================================================================
// PhysicsWorldGPU - Main Class
// ============================================================================

/**
 * @brief GPU-accelerated physics world manager
 * 
 * Usage example:
 * @code
 *   PhysicsWorldConfig config;
 *   config.gravity = Vec3(0, -10, 0);
 *   
 *   PhysicsWorldGPU world(config);
 *   
 *   BodyHandle box = world.create_body(bodyDesc);
 *   JointHandle hinge = world.create_joint(jointDesc);
 *   
 *   while (running) {
 *       world.step(1.0f / 60.0f);
 *       Vec3 pos = world.get_body_position(box);
 *   }
 * @endcode
 */
class PhysicsWorldGPU {
public:
    // ========================================================================
    // Construction / Destruction
    // ========================================================================
    
    /**
     * @brief Construct a physics world with the given configuration
     */
    explicit PhysicsWorldGPU(const PhysicsWorldConfig& config = PhysicsWorldConfig());
    
    /**
     * @brief Destructor - releases GPU resources
     */
    ~PhysicsWorldGPU();
    
    // Disable copy (GPU resources are not trivially copyable)
    PhysicsWorldGPU(const PhysicsWorldGPU&) = delete;
    PhysicsWorldGPU& operator=(const PhysicsWorldGPU&) = delete;
    
    // Enable move
    PhysicsWorldGPU(PhysicsWorldGPU&& other) noexcept;
    PhysicsWorldGPU& operator=(PhysicsWorldGPU&& other) noexcept;
    
    // ========================================================================
    // Simulation Control
    // ========================================================================
    
    /**
     * @brief Advance simulation by delta_time
     * 
     * Internally uses fixed timestep. If delta_time > fixed_time_step,
     * multiple sub-steps are performed.
     * 
     * @param delta_time Time to advance (seconds)
     */
    void step(float delta_time);
    
    /**
     * @brief Perform a single fixed-timestep simulation step
     */
    void step_fixed();
    
    /**
     * @brief Reset the world to initial state
     */
    void reset();
    
    /**
     * @brief Clear all bodies, joints, and contacts
     */
    void clear();
    
    // ========================================================================
    // Body Management
    // ========================================================================
    
    /**
     * @brief Create a new rigid body from descriptor
     * @return Handle to the created body (check is_valid())
     */
    BodyHandle create_body(const RigidBodyDesc& description);
    
    /**
     * @brief Create a new rigid body directly from RigidBody struct
     */
    BodyHandle create_body(const RigidBody& body);
    
    /**
     * @brief Remove a body from the simulation
     * @note Removes all joints connected to this body
     */
    void destroy_body(BodyHandle handle);
    
    /**
     * @brief Check if a body handle is valid
     */
    bool is_body_valid(BodyHandle handle) const;
    
    /**
     * @brief Get the number of active bodies
     */
    size_t get_body_count() const;
    
    // ========================================================================
    // Body State Queries (Read)
    // ========================================================================
    
    Vec3 get_body_position(BodyHandle handle) const;
    Quaternion get_body_orientation(BodyHandle handle) const;
    Vec3 get_body_linear_velocity(BodyHandle handle) const;
    Vec3 get_body_angular_velocity(BodyHandle handle) const;
    float get_body_mass(BodyHandle handle) const;
    BodyType get_body_type(BodyHandle handle) const;
    bool is_body_sleeping(BodyHandle handle) const;
    
    /**
     * @brief Get a copy of the full rigid body data
     */
    RigidBody get_body(BodyHandle handle) const;
    
    // ========================================================================
    // Body State Modification (Write)
    // ========================================================================
    
    void set_body_position(BodyHandle handle, const Vec3& position);
    void set_body_orientation(BodyHandle handle, const Quaternion& orientation);
    void set_body_linear_velocity(BodyHandle handle, const Vec3& velocity);
    void set_body_angular_velocity(BodyHandle handle, const Vec3& velocity);
    void set_body_type(BodyHandle handle, BodyType type);
    
    /**
     * @brief Apply a force at the center of mass (accumulated until next step)
     */
    void apply_force(BodyHandle handle, const Vec3& force);
    
    /**
     * @brief Apply a force at a world-space point
     */
    void apply_force_at_point(BodyHandle handle, const Vec3& force, const Vec3& point);
    
    /**
     * @brief Apply a torque (accumulated until next step)
     */
    void apply_torque(BodyHandle handle, const Vec3& torque);
    
    /**
     * @brief Apply an instantaneous impulse at the center of mass
     */
    void apply_impulse(BodyHandle handle, const Vec3& impulse);
    
    /**
     * @brief Apply an instantaneous impulse at a world-space point
     */
    void apply_impulse_at_point(BodyHandle handle, const Vec3& impulse, const Vec3& point);
    
    /**
     * @brief Wake up a sleeping body
     */
    void wake_body(BodyHandle handle);
    
    // ========================================================================
    // Joint Management
    // ========================================================================
    
    /**
     * @brief Create a joint from descriptor
     * @return Handle to the created joint
     */
    ConstraintHandle create_joint(const JointDescriptor& descriptor);
    
    /**
     * @brief Remove a joint
     */
    void destroy_joint(ConstraintHandle handle);
    
    /**
     * @brief Check if a joint handle is valid
     */
    bool is_joint_valid(ConstraintHandle handle) const;
    
    /**
     * @brief Get the number of active joints
     */
    size_t get_joint_count() const;

    /**
     * @brief Inputs to @ref debug_feature_key — exposes contact cache key
     * computation for direct regression testing.
     */
    struct FeatureKeyInputs {
        Vec3 normal;
        Vec3 local_a;     // contact point in body A's local frame
        Vec3 half_a;      // body A half-extents
        Vec3 local_b;
        Vec3 half_b;
    };

    /**
     * @brief Compute the contact-cache feature key the engine would use for the
     *        given inputs. @p prev_key is the cached key from the previous
     *        frame on the same axis (used to apply octant hysteresis); pass
     *        any negative value for a fresh contact with no anchor.
     *
     * Used by tests to directly assert that micro-vibration in contact-point
     * coordinates does NOT change the returned feature key.
     */
    static uint32_t debug_feature_key(const FeatureKeyInputs& in, int prev_key = -1);

    /**
     * @brief Diagnostic — current size of the persistent contact cache.
     *
     * Exposed for regression tests of warm-start stability under micro-vibration.
     * With octant hysteresis active, the cache size stays bounded by the count
     * of *distinct* stable contact features even when local contact points
     * wobble near a face boundary.
     */
    size_t get_contact_cache_size() const;
    
    /**
     * @brief Set motor target velocity for a joint
     */
    void set_joint_motor_velocity(ConstraintHandle handle, float velocity);
    
    /**
     * @brief Set motor max force/torque for a joint
     */
    void set_joint_motor_max_force(ConstraintHandle handle, float max_force);
    
    /**
     * @brief Enable/disable joint motor
     */
    void set_joint_motor_enabled(ConstraintHandle handle, bool enabled);

    /**
     * @brief Kinematically force the joint's scalar coordinate to @p q.
     *
     * For HINGE   : @p q is the angle (radians) about the joint axis.
     * For SLIDER  : @p q is the translation (meters) along the joint axis.
     * For BALL_SOCKET / FIXED : ignored (no scalar coordinate).
     *
     * Implementation warps body B (the child) so the joint anchor coincides with
     * body A's anchor and the relative pose realises the requested coordinate.
     * Linear and angular velocities of body B are zeroed (kinematic playback).
     * The constraint solver will resolve any residual drift on the next step.
     *
     * Use this for ROS 2 joint-state playback, IK pose tests, or scrubbing.
     */
    void set_joint_position(ConstraintHandle handle, float q);

    /**
     * @brief Apply joint-level acceleration @p qddot as accumulated force/torque.
     *
     * For HINGE   : applies torque  τ = I_axis · qddot along the world joint axis,
     *               where I_axis = 1 / (axis · I⁻¹_world · axis) approximates the
     *               body's rotational inertia about the joint axis.
     * For SLIDER  : applies force   F = m · qddot along the world joint axis.
     * For BALL_SOCKET / FIXED : ignored.
     *
     * This is a single-step inverse-dynamics approximation (Coriolis /
     * centripetal terms and the constraint Jacobian are not included). Suitable
     * for ROS 2 trajectory generators that already include feedforward
     * acceleration. The applied force is consumed by the next step's integrator.
     */
    void set_joint_acceleration(ConstraintHandle handle, float qddot);

    /**
     * @brief Set the joint's scalar *velocity* to @p qdot without warping pose.
     *
     * For HINGE   : @p qdot is the angular speed (rad/s) about the joint axis.
     *               Body B's angular_velocity is set to A's angular_velocity +
     *               qdot · axis_world, and its linear velocity is recomputed so
     *               the anchor coincidence stays kinematically consistent.
     * For SLIDER  : @p qdot is the linear speed (m/s) along the joint axis.
     *               Body B's linear_velocity is set so the anchor point
     *               translates at qdot along the axis.
     * For BALL_SOCKET / FIXED : ignored.
     *
     * Pair with @ref set_joint_position to feed smooth ROS 2 joint command
     * streams (position + velocity tuple) without the zig-zag motion that
     * pure position warping produces at 100 Hz.
     */
    void set_joint_velocity(ConstraintHandle handle, float qdot);
    
    // ========================================================================
    // World Configuration
    // ========================================================================
    
    void set_gravity(const Vec3& gravity);
    Vec3 get_gravity() const;
    
    void set_time_step(float time_step);
    float get_time_step() const;
    
    void set_solver_iterations(int iterations);
    int get_solver_iterations() const;
    
    // ========================================================================
    // Collision Queries (not simulation-related)
    // ========================================================================
    
    /**
     * @brief Cast a ray and find the first body hit
     * @param origin Ray start point
     * @param direction Ray direction (normalized)
     * @param max_distance Maximum ray distance
     * @param[out] hit_point World position where ray hit
     * @param[out] hit_normal Surface normal at hit point
     * @return Handle to hit body, or invalid handle if no hit
     */
    BodyHandle raycast(const Vec3& origin, const Vec3& direction, float max_distance,
                       Vec3* hit_point = nullptr, Vec3* hit_normal = nullptr) const;
    
    /**
     * @brief Find all bodies overlapping with an AABB
     * @param aabb The query box
     * @param callback Called for each overlapping body
     */
    void query_aabb(const AABB& aabb, QueryCallback callback) const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    /**
     * @brief Set callback for contact events
     */
    void set_contact_callback(ContactCallback callback);
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    /**
     * @brief Get simulation statistics
     */
    SimulationStatistics get_statistics() const;
    
    /**
     * @brief Reset timing statistics
     */
    void reset_statistics();
    
    // ========================================================================
    // Advanced: Direct Buffer Access (for custom GPU kernels)
    // ========================================================================
    
    /**
     * @brief Get read-only access to internal body SoA
     * @warning Do not hold references across step() calls
     */
    const RigidBodySoA& get_body_buffer() const;
    
    /**
     * @brief Synchronize CPU-side buffers with GPU
     * Call after step() if you need immediate CPU access
     */
    void synchronize();
    
private:
    // ========================================================================
    // Internal Implementation
    // ========================================================================
    
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    // Pipeline stages (called internally by step_fixed)
    void apply_external_forces();
    void broadphase();
    void narrowphase();
    void solve_constraints();
    void integrate();
    void update_sleep_states();
    void update_contact_cache(float dt);   // persist accumulated impulses for warm-starting (age in seconds)
    void invoke_callbacks();
    
    // Utility
    size_t handle_to_index(BodyHandle handle) const;
    BodyHandle index_to_handle(size_t index) const;
};

// ============================================================================
// Convenience Factory Functions
// ============================================================================

/**
 * @brief Create a box rigid body descriptor
 */
inline RigidBodyDesc make_box_body(const Vec3& position, const Vec3& half_extents, float mass) {
    RigidBodyDesc desc;
    desc.position = position;
    desc.mass = mass;
    
    // Box inertia tensor: I = (m/12) * diag(h² + d², w² + d², w² + h²)
    float w = half_extents.x * 2.0f;
    float h = half_extents.y * 2.0f;
    float d = half_extents.z * 2.0f;
    float factor = mass / 12.0f;
    desc.inertia_tensor = Matrix3(
        factor * (h*h + d*d), 0.0f, 0.0f,
        0.0f, factor * (w*w + d*d), 0.0f,
        0.0f, 0.0f, factor * (w*w + h*h)
    );
    
    return desc;
}

/**
 * @brief Create a sphere rigid body descriptor
 */
inline RigidBodyDesc make_sphere_body(const Vec3& position, float radius, float mass) {
    RigidBodyDesc desc;
    desc.position = position;
    desc.mass = mass;
    
    // Sphere inertia tensor: I = (2/5) * m * r²
    float I = (2.0f / 5.0f) * mass * radius * radius;
    desc.inertia_tensor = Matrix3::identity() * I;
    
    return desc;
}

/**
 * @brief Create a hinge joint descriptor
 */
inline JointDescriptor make_hinge_joint(
    BodyHandle body_a, const Vec3& anchor_a, const Vec3& axis_a,
    BodyHandle body_b, const Vec3& anchor_b, const Vec3& axis_b
) {
    JointDescriptor desc;
    desc.type = JointType::HINGE;
    desc.body_a = body_a;
    desc.body_b = body_b;
    desc.local_anchor_a = anchor_a;
    desc.local_anchor_b = anchor_b;
    desc.local_axis_a = axis_a;
    desc.local_axis_b = axis_b;
    return desc;
}

/**
 * @brief Create a ball-socket joint descriptor
 */
inline JointDescriptor make_ball_socket_joint(
    BodyHandle body_a, const Vec3& anchor_a,
    BodyHandle body_b, const Vec3& anchor_b
) {
    JointDescriptor desc;
    desc.type = JointType::BALL_SOCKET;
    desc.body_a = body_a;
    desc.body_b = body_b;
    desc.local_anchor_a = anchor_a;
    desc.local_anchor_b = anchor_b;
    return desc;
}

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_PHYSICS_WORLD_GPU_H

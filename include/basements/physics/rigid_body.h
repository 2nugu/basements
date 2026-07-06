#ifndef BASEMENTS_PHYSICS_RIGID_BODY_H
#define BASEMENTS_PHYSICS_RIGID_BODY_H

#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/core/math/matrix3.h"
#include "basements/core/types.h"

#include "basements/physics/thermodynamics.h"

namespace basements {
namespace physics {

using namespace basements::math;

/**
 * @brief Rigid body representation for physics simulation
 * 
 * This class represents a single rigid body with position, orientation,
 * velocity, and mass properties. It's designed for CPU-side manipulation
 * with GPU-friendly memory layout considerations.
 */
struct RigidBody {
    // ============================================================
    // Transform State
    // ============================================================
    
    Vec3 position;              ///< Center of mass position (world space)
    Quaternion orientation;     ///< Rotation (world space)
    
    // ============================================================
    // Motion State
    // ============================================================
    
    Vec3 linear_velocity;       ///< Linear velocity (m/s)
    Vec3 angular_velocity;      ///< Angular velocity (rad/s)
    
    // ============================================================
    // Mass Properties
    // ============================================================
    
    float mass;                 ///< Mass (kg)
    float inv_mass;             ///< Inverse mass (1/mass), 0 for static bodies
    
    Matrix3 inertia_tensor;     ///< Inertia tensor (body space)
    Matrix3 inv_inertia_tensor; ///< Inverse inertia tensor (body space)
    
    // Damping properties
    float linear_damping;       ///< Linear damping coefficient (0-1)
    float angular_damping;      ///< Angular damping coefficient (0-1)
    
    // Electromagnetic properties
    float charge;               ///< Electric charge (Coulombs)
    
    // ============================================================
    // Force Accumulators
    // ============================================================
    
    Vec3 force;                 ///< Accumulated force (N)
    Vec3 torque;                ///< Accumulated torque (N·m)
    
    // ============================================================
    // Collision Shape
    // ============================================================
    
    ShapeHandle shape;          ///< Reference to collision shape
    
    // ============================================================
    // Material Reference
    // ============================================================
    
    uint32_t material_id;       ///< Material ID (see material_library.h)
    
    // ============================================================
    // Flags and State
    // ============================================================
    
    BodyType type;              ///< Static, Dynamic, or Kinematic
    bool is_sleeping;           ///< Sleep state for optimization
    float sleep_time;           ///< Time spent below sleep threshold
    
    // ============================================================
    // Thermal State
    // ============================================================
    
    ThermalState thermal_state; ///< Temperature and phase info
    
    // ============================================================
    // Vibration State
    // ============================================================
    
    float vibration_amplitude;  ///< Current vibration magnitude (0.0 - 1.0+)
    
    // ============================================================
    // Fatigue State
    // ============================================================
    
    struct FatigueState {
        float accumulated_damage = 0.0f; // 0.0 (New) to 1.0 (Failed)
        float last_stress_peak = 0.0f;   // For peak detection
        uint32_t cycle_count = 0;
    } fatigue_state;
    
    // ============================================================
    // User Data
    // ============================================================
    
    void* user_data;            ///< User-defined data pointer
    
    // ============================================================
    // Constructors
    // ============================================================
    
    HOST_DEVICE RigidBody()
        : position(Vec3::zero())
        , orientation(Quaternion::identity())
        , linear_velocity(Vec3::zero())
        , angular_velocity(Vec3::zero())
        , mass(1.0f)
        , inv_mass(1.0f)
        , inertia_tensor(Matrix3::identity())
        , inv_inertia_tensor(Matrix3::identity())
        , linear_damping(0.01f)
        , angular_damping(0.01f)
        , charge(0.0f)
        , force(Vec3::zero())
        , torque(Vec3::zero())
        , shape()
        , type(BodyType::DYNAMIC)
        , is_sleeping(false)
        , sleep_time(0.0f)
        , user_data(nullptr)
    {}
    
    // ============================================================
    // Methods
    // ============================================================
    
    /// Set mass and automatically compute inverse mass
    /// Set mass and automatically compute inverse mass
    HOST_DEVICE void set_mass(float m) {
        mass = m;
        inv_mass = (m > 0.0f) ? (1.0f / m) : 0.0f;
    }
    
    /// Set inertia tensor and compute inverse
    HOST_DEVICE void set_inertia_tensor(const Matrix3& I) {
        inertia_tensor = I;
        inv_inertia_tensor = I.inversed();
    }
    
    /// Make this body static (infinite mass)
    HOST_DEVICE void make_static() {
        type = BodyType::STATIC;
        mass = 0.0f;
        inv_mass = 0.0f;
        linear_velocity = Vec3::zero();
        angular_velocity = Vec3::zero();
    }
    
    /// Make this body dynamic (finite mass)
    HOST_DEVICE void make_dynamic(float m) {
        type = BodyType::DYNAMIC;
        set_mass(m);
    }
    
    /// Apply force at center of mass
    HOST_DEVICE void apply_force(const Vec3& f) {
        force += f;
    }
    
    /// Apply force at a point in world space
    HOST_DEVICE void apply_force_at_point(const Vec3& f, const Vec3& point) {
        force += f;
        Vec3 r = point - position;  // Lever arm
        torque += r.cross(f);
    }
    
    /// Apply torque
    HOST_DEVICE void apply_torque(const Vec3& t) {
        torque += t;
    }
    
    /// Apply impulse at center of mass
    /// Apply impulse at center of mass
    HOST_DEVICE void apply_impulse(const Vec3& impulse) {
        if (type != BodyType::DYNAMIC) return;
        linear_velocity += impulse * inv_mass;
    }
    
    /// Apply impulse at a point in world space
    HOST_DEVICE void apply_impulse_at_point(const Vec3& impulse, const Vec3& point) {
        if (type != BodyType::DYNAMIC) return;
        
        linear_velocity += impulse * inv_mass;
        
        Vec3 r = point - position;
        Vec3 angular_impulse = r.cross(impulse);
        
        // Transform to world space inertia
        Matrix3 rotation = Matrix3::from_quaternion(orientation);
        Matrix3 inv_inertia_world = rotation * inv_inertia_tensor * rotation.transposed();
        
        angular_velocity += inv_inertia_world * angular_impulse;
    }
    
    /// Clear force accumulators
    HOST_DEVICE void clear_forces() {
        force = Vec3::zero();
        torque = Vec3::zero();
    }
    
    /// Get world-space inverse inertia tensor
    HOST_DEVICE Matrix3 get_inv_inertia_world() const {
        Matrix3 rotation = Matrix3::from_quaternion(orientation);
        return rotation * inv_inertia_tensor * rotation.transposed();
    }
    
    /// Update world-space inertia tensor after orientation change
    /// CRITICAL: Must be called after orientation updates in integrator
    HOST_DEVICE void update_inertia_tensor() {
        // I_world = R * I_body * R^T
        // We store inv_inertia in body space, so we don't need to update it
        // The get_inv_inertia_world() method computes it on-demand
        // This method is here for future optimizations where we might cache I_world
        
        // For now, this is a no-op since we compute on-demand
        // In the future, we could cache:
        // Matrix3 rotation = Matrix3::from_quaternion(orientation);
        // cached_inv_inertia_world = rotation * inv_inertia_tensor * rotation.transposed();
    }
    
    /// Transform a point from local to world space
    HOST_DEVICE Vec3 local_to_world(const Vec3& local_point) const {
        return position + orientation.rotate(local_point);
    }
    
    /// Transform a point from world to local space
    HOST_DEVICE Vec3 world_to_local(const Vec3& world_point) const {
        return orientation.inverse().rotate(world_point - position);
    }
    
    /// Check if body should sleep
    HOST_DEVICE bool should_sleep() const {
        if (type != BodyType::DYNAMIC) return false;
        
        float lin_speed_sq = linear_velocity.length_squared();
        float ang_speed_sq = angular_velocity.length_squared();
        
        return (lin_speed_sq < config::SLEEP_LINEAR_THRESHOLD * config::SLEEP_LINEAR_THRESHOLD) &&
               (ang_speed_sq < config::SLEEP_ANGULAR_THRESHOLD * config::SLEEP_ANGULAR_THRESHOLD);
    }
    
    /// Wake up the body
    HOST_DEVICE void wake_up() {
        is_sleeping = false;
        sleep_time = 0.0f;
    }
};

/**
 * @brief Descriptor for creating a rigid body
 */
struct RigidBodyDesc {
    Vec3 position = Vec3::zero();
    Quaternion orientation = Quaternion::identity();
    Vec3 linear_velocity = Vec3::zero();
    Vec3 angular_velocity = Vec3::zero();

    float mass = 1.0f;
    Matrix3 inertia_tensor = Matrix3::identity();

    BodyType type = BodyType::DYNAMIC;
    ShapeHandle shape;

    Vec3 half_extents = Vec3(0.5f, 0.5f, 0.5f);  ///< Collision half-extents (box AABB)
    float restitution = 0.3f;
    float friction = 0.5f;

    void* user_data = nullptr;
};

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_RIGID_BODY_H

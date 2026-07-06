#ifndef BASEMENTS_PHYSICS_RIGID_BODY_SOA_H
#define BASEMENTS_PHYSICS_RIGID_BODY_SOA_H

#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/core/math/matrix3.h"
#include "basements/core/types.h"
#include <vector>
#include <memory>

namespace basements {
namespace physics {

using namespace basements::math;

/**
 * @brief Structure of Arrays layout for rigid bodies (GPU-friendly)
 * 
 * This layout separates each property into its own array, enabling:
 * - Coalesced memory access on GPU
 * - Better cache utilization
 * - SIMD vectorization opportunities
 */
class RigidBodySoA {
public:
    // ============================================================
    // Data Arrays
    // ============================================================
    
    std::vector<Vec3> positions;
    std::vector<Quaternion> orientations;
    
    std::vector<Vec3> linear_velocities;
    std::vector<Vec3> angular_velocities;
    
    std::vector<float> masses;
    std::vector<float> inv_masses;
    
    std::vector<Matrix3> inertia_tensors;
    std::vector<Matrix3> inv_inertia_tensors;
    
    std::vector<Vec3> forces;
    std::vector<Vec3> torques;
    
    std::vector<ShapeHandle> shapes;
    std::vector<BodyType> types;
    std::vector<bool> is_sleeping_flags;
    std::vector<float> sleep_times;
    
    std::vector<void*> user_data;
    
    // ============================================================
    // Capacity Management
    // ============================================================
    
    size_t count;
    size_t capacity;
    
    // ============================================================
    // Constructor
    // ============================================================
    
    RigidBodySoA(size_t initial_capacity = config::DEFAULT_BODY_CAPACITY)
        : count(0)
        , capacity(initial_capacity)
    {
        reserve(initial_capacity);
    }
    
    // ============================================================
    // Capacity Operations
    // ============================================================
    
    void reserve(size_t new_capacity) {
        capacity = new_capacity;
        
        positions.reserve(new_capacity);
        orientations.reserve(new_capacity);
        linear_velocities.reserve(new_capacity);
        angular_velocities.reserve(new_capacity);
        masses.reserve(new_capacity);
        inv_masses.reserve(new_capacity);
        inertia_tensors.reserve(new_capacity);
        inv_inertia_tensors.reserve(new_capacity);
        forces.reserve(new_capacity);
        torques.reserve(new_capacity);
        shapes.reserve(new_capacity);
        types.reserve(new_capacity);
        is_sleeping_flags.reserve(new_capacity);
        sleep_times.reserve(new_capacity);
        user_data.reserve(new_capacity);
    }
    
    void resize(size_t new_size) {
        count = new_size;
        
        positions.resize(new_size);
        orientations.resize(new_size);
        linear_velocities.resize(new_size);
        angular_velocities.resize(new_size);
        masses.resize(new_size);
        inv_masses.resize(new_size);
        inertia_tensors.resize(new_size);
        inv_inertia_tensors.resize(new_size);
        forces.resize(new_size);
        torques.resize(new_size);
        shapes.resize(new_size);
        types.resize(new_size);
        is_sleeping_flags.resize(new_size);
        sleep_times.resize(new_size);
        user_data.resize(new_size);
    }
    
    // ============================================================
    // Element Access
    // ============================================================
    
    /// Add a new body and return its index
    size_t add_body(const RigidBody& body) {
        if (count >= capacity) {
            reserve(capacity * 2);
        }
        
        size_t index = count++;
        
        positions.push_back(body.position);
        orientations.push_back(body.orientation);
        linear_velocities.push_back(body.linear_velocity);
        angular_velocities.push_back(body.angular_velocity);
        masses.push_back(body.mass);
        inv_masses.push_back(body.inv_mass);
        inertia_tensors.push_back(body.inertia_tensor);
        inv_inertia_tensors.push_back(body.inv_inertia_tensor);
        forces.push_back(body.force);
        torques.push_back(body.torque);
        shapes.push_back(body.shape);
        types.push_back(body.type);
        is_sleeping_flags.push_back(body.is_sleeping);
        sleep_times.push_back(body.sleep_time);
        user_data.push_back(body.user_data);
        
        return index;
    }
    
    /// Get a body at index (reconstructs AoS from SoA)
    RigidBody get_body(size_t index) const {
        RigidBody body;
        
        body.position = positions[index];
        body.orientation = orientations[index];
        body.linear_velocity = linear_velocities[index];
        body.angular_velocity = angular_velocities[index];
        body.mass = masses[index];
        body.inv_mass = inv_masses[index];
        body.inertia_tensor = inertia_tensors[index];
        body.inv_inertia_tensor = inv_inertia_tensors[index];
        body.force = forces[index];
        body.torque = torques[index];
        body.shape = shapes[index];
        body.type = types[index];
        body.is_sleeping = is_sleeping_flags[index];
        body.sleep_time = sleep_times[index];
        body.user_data = user_data[index];
        
        return body;
    }
    
    /// Set a body at index (decomposes AoS into SoA)
    void set_body(size_t index, const RigidBody& body) {
        positions[index] = body.position;
        orientations[index] = body.orientation;
        linear_velocities[index] = body.linear_velocity;
        angular_velocities[index] = body.angular_velocity;
        masses[index] = body.mass;
        inv_masses[index] = body.inv_mass;
        inertia_tensors[index] = body.inertia_tensor;
        inv_inertia_tensors[index] = body.inv_inertia_tensor;
        forces[index] = body.force;
        torques[index] = body.torque;
        shapes[index] = body.shape;
        types[index] = body.type;
        is_sleeping_flags[index] = body.is_sleeping;
        sleep_times[index] = body.sleep_time;
        user_data[index] = body.user_data;
    }
    
    /// Clear all bodies
    void clear() {
        count = 0;
        positions.clear();
        orientations.clear();
        linear_velocities.clear();
        angular_velocities.clear();
        masses.clear();
        inv_masses.clear();
        inertia_tensors.clear();
        inv_inertia_tensors.clear();
        forces.clear();
        torques.clear();
        shapes.clear();
        types.clear();
        is_sleeping_flags.clear();
        sleep_times.clear();
        user_data.clear();
    }
    
    // ============================================================
    // GPU Data Access (Raw Pointers)
    // ============================================================
    
    /// Get raw pointer to positions (for GPU upload)
    Vec3* get_positions_ptr() { return positions.data(); }
    const Vec3* get_positions_ptr() const { return positions.data(); }
    
    /// Get raw pointer to velocities
    Vec3* get_linear_velocities_ptr() { return linear_velocities.data(); }
    const Vec3* get_linear_velocities_ptr() const { return linear_velocities.data(); }
    
    /// Get raw pointer to forces
    Vec3* get_forces_ptr() { return forces.data(); }
    const Vec3* get_forces_ptr() const { return forces.data(); }
    
    /// Get raw pointer to inverse masses
    float* get_inv_masses_ptr() { return inv_masses.data(); }
    const float* get_inv_masses_ptr() const { return inv_masses.data(); }
    
    // ============================================================
    // Utility
    // ============================================================
    
    size_t size() const { return count; }
    bool empty() const { return count == 0; }
};

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_RIGID_BODY_SOA_H

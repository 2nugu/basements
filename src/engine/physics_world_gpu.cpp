#include "basements/engine/physics_world_gpu.h"

#include <algorithm>
#include <chrono>
#include <iostream>

// ============================================================================
// Platform-specific includes
// ============================================================================

#ifdef __CUDACC__
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "basements/physics/collision/spatial_hash_gpu.h"
#include "basements/physics/dynamics/solver_gpu.h"
#define USE_GPU 1
#else
#define USE_GPU 0
#endif

namespace basements {
namespace engine {

using namespace basements::math;
using namespace basements::physics;
using namespace basements::collision;
using namespace basements::dynamics;

// ============================================================================
// Internal Implementation Structure
// ============================================================================

struct PhysicsWorldGPU::Impl {
    // Configuration
    PhysicsWorldConfig config;
    
    // State
    bool is_initialized             = false;
    bool use_gpu                    = false;
    
    // Body storage (CPU-side, synchronized with GPU)
    RigidBodySoA body_buffer;
    std::vector<bool> body_active_flags;  // Slot active/deleted tracking
    std::vector<BodyID> handle_to_index;  // Handle ID -> buffer index
    std::vector<BodyHandle> index_to_handle;  // Buffer index -> Handle
    std::vector<Vec3> body_half_extents;  // Per-body collision half-extents
    BodyID next_body_id             = 0;
    size_t active_body_count        = 0;
    
    // Joint storage
    std::vector<BallSocketJoint> ball_socket_joints;
    std::vector<HingeJoint> hinge_joints;
    std::vector<SliderJoint> slider_joints;
    std::vector<FixedJoint> fixed_joints;
    std::unordered_map<ConstraintID, std::pair<JointType, size_t>> joint_map;
    ConstraintID next_joint_id      = 0;
    
    // Collision detection
    SpatialHash broadphase_hash;
    std::vector<CollisionPair> potential_pairs;
    std::vector<ContactConstraint> contacts;
    
    // Statistics
    SimulationStatistics statistics;
    
    // Callbacks
    ContactCallback contact_callback;
    
    // Timing
    float accumulated_time          = 0.0f;
    
    // GPU resources (when available)
#if USE_GPU
    RigidBody* d_bodies             = nullptr;
    ContactConstraint* d_contacts   = nullptr;
    AABB* d_aabbs                   = nullptr;
    bool gpu_buffers_allocated      = false;
    size_t gpu_body_capacity        = 0;
    size_t gpu_contact_capacity     = 0;
#endif
    
    // ========================================================================
    // Constructor / Destructor
    // ========================================================================
    
    explicit Impl(const PhysicsWorldConfig& cfg) 
        : config(cfg)
        , body_buffer(cfg.initial_body_capacity)
        , broadphase_hash()
    {
        body_active_flags.reserve(cfg.initial_body_capacity);
        handle_to_index.reserve(cfg.initial_body_capacity);
        index_to_handle.reserve(cfg.initial_body_capacity);
        potential_pairs.reserve(cfg.initial_body_capacity * 2);
        contacts.reserve(cfg.initial_contact_capacity);
        
        ball_socket_joints.reserve(cfg.initial_joint_capacity / 4);
        hinge_joints.reserve(cfg.initial_joint_capacity / 4);
        slider_joints.reserve(cfg.initial_joint_capacity / 4);
        fixed_joints.reserve(cfg.initial_joint_capacity / 4);
        
#if USE_GPU
        if (cfg.prefer_gpu) {
            int device_count = 0;
            cudaError_t err = cudaGetDeviceCount(&device_count);
            
            if (err == cudaSuccess && device_count > 0) {
                cudaSetDevice(cfg.cuda_device_id);
                use_gpu = true;
                allocate_gpu_buffers();
            }
        }
#endif
        
        is_initialized = true;
    }
    
    ~Impl() {
#if USE_GPU
        free_gpu_buffers();
#endif
    }
    
    // ========================================================================
    // GPU Buffer Management
    // ========================================================================
    
#if USE_GPU
    void allocate_gpu_buffers() {
        if (gpu_buffers_allocated) return;
        
        gpu_body_capacity = config.initial_body_capacity;
        gpu_contact_capacity = config.initial_contact_capacity;
        
        cudaMalloc(&d_bodies, gpu_body_capacity * sizeof(RigidBody));
        cudaMalloc(&d_contacts, gpu_contact_capacity * sizeof(ContactConstraint));
        cudaMalloc(&d_aabbs, gpu_body_capacity * sizeof(AABB));
        
        gpu_buffers_allocated = true;
    }
    
    void free_gpu_buffers() {
        if (!gpu_buffers_allocated) return;
        
        if (d_bodies) { cudaFree(d_bodies); d_bodies = nullptr; }
        if (d_contacts) { cudaFree(d_contacts); d_contacts = nullptr; }
        if (d_aabbs) { cudaFree(d_aabbs); d_aabbs = nullptr; }
        
        gpu_buffers_allocated = false;
    }
    
    void resize_gpu_buffers_if_needed() {
        if (body_buffer.count > gpu_body_capacity) {
            size_t new_capacity = gpu_body_capacity * 2;
            
            RigidBody* new_d_bodies;
            AABB* new_d_aabbs;
            
            cudaMalloc(&new_d_bodies, new_capacity * sizeof(RigidBody));
            cudaMalloc(&new_d_aabbs, new_capacity * sizeof(AABB));
            
            cudaMemcpy(new_d_bodies, d_bodies, gpu_body_capacity * sizeof(RigidBody), cudaMemcpyDeviceToDevice);
            
            cudaFree(d_bodies);
            cudaFree(d_aabbs);
            
            d_bodies = new_d_bodies;
            d_aabbs = new_d_aabbs;
            gpu_body_capacity = new_capacity;
        }
        
        if (contacts.size() > gpu_contact_capacity) {
            size_t new_capacity = gpu_contact_capacity * 2;
            
            ContactConstraint* new_d_contacts;
            cudaMalloc(&new_d_contacts, new_capacity * sizeof(ContactConstraint));
            
            cudaFree(d_contacts);
            d_contacts = new_d_contacts;
            gpu_contact_capacity = new_capacity;
        }
    }
    
    void upload_to_gpu() {
        resize_gpu_buffers_if_needed();
        
        // Convert SoA to AoS for GPU upload (temporary for this implementation)
        std::vector<RigidBody> bodies_aos(body_buffer.count);
        for (size_t i = 0; i < body_buffer.count; ++i) {
            bodies_aos[i] = body_buffer.get_body(i);
        }
        
        cudaMemcpy(d_bodies, bodies_aos.data(), body_buffer.count * sizeof(RigidBody), cudaMemcpyHostToDevice);
        
        if (!contacts.empty()) {
            cudaMemcpy(d_contacts, contacts.data(), contacts.size() * sizeof(ContactConstraint), cudaMemcpyHostToDevice);
        }
    }
    
    void download_from_gpu() {
        std::vector<RigidBody> bodies_aos(body_buffer.count);
        cudaMemcpy(bodies_aos.data(), d_bodies, body_buffer.count * sizeof(RigidBody), cudaMemcpyDeviceToHost);
        
        for (size_t i = 0; i < body_buffer.count; ++i) {
            body_buffer.set_body(i, bodies_aos[i]);
        }
    }
#endif
};

// ============================================================================
// PhysicsWorldGPU Implementation
// ============================================================================

PhysicsWorldGPU::PhysicsWorldGPU(const PhysicsWorldConfig& config)
    : impl_(std::make_unique<Impl>(config))
{
}

PhysicsWorldGPU::~PhysicsWorldGPU() = default;

PhysicsWorldGPU::PhysicsWorldGPU(PhysicsWorldGPU&& other) noexcept = default;
PhysicsWorldGPU& PhysicsWorldGPU::operator=(PhysicsWorldGPU&& other) noexcept = default;

// ============================================================================
// Simulation Control
// ============================================================================

void PhysicsWorldGPU::step(float delta_time) {
    impl_->accumulated_time += delta_time;
    
    while (impl_->accumulated_time >= impl_->config.fixed_time_step) {
        step_fixed();
        impl_->accumulated_time -= impl_->config.fixed_time_step;
    }
}

void PhysicsWorldGPU::step_fixed() {
    auto start = std::chrono::high_resolution_clock::now();
    
    const float dt = impl_->config.fixed_time_step;
    
    // 1. Apply external forces (gravity, user forces)
    apply_external_forces();
    
    // 2. Broadphase collision detection
    broadphase();
    
    // 3. Narrowphase collision detection
    narrowphase();
    
    // 4. Solve constraints (contacts + joints)
    solve_constraints();
    
    // 5. Integrate velocities and positions
    integrate();
    
    // 6. Update sleep states
    if (impl_->config.enable_sleeping) {
        update_sleep_states();
    }
    
    // 7. Invoke callbacks
    invoke_callbacks();
    
    // Update statistics
    auto end = std::chrono::high_resolution_clock::now();
    float total_ms = std::chrono::duration<float, std::milli>(end - start).count();
    
    impl_->statistics.total_step_time_ms = total_ms;
    impl_->statistics.total_steps++;
    impl_->statistics.total_simulation_time += dt;
    impl_->statistics.active_body_count = impl_->active_body_count;
    impl_->statistics.contact_count = impl_->contacts.size();
    impl_->statistics.joint_count = impl_->joint_map.size();
}

void PhysicsWorldGPU::reset() {
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        impl_->body_buffer.linear_velocities[i] = Vec3::zero();
        impl_->body_buffer.angular_velocities[i] = Vec3::zero();
        impl_->body_buffer.forces[i] = Vec3::zero();
        impl_->body_buffer.torques[i] = Vec3::zero();
    }
    
    impl_->contacts.clear();
    impl_->statistics = SimulationStatistics();
    impl_->accumulated_time = 0.0f;
}

void PhysicsWorldGPU::clear() {
    impl_->body_buffer.clear();
    impl_->body_active_flags.clear();
    impl_->handle_to_index.clear();
    impl_->index_to_handle.clear();
    impl_->active_body_count = 0;
    
    impl_->ball_socket_joints.clear();
    impl_->hinge_joints.clear();
    impl_->slider_joints.clear();
    impl_->fixed_joints.clear();
    impl_->joint_map.clear();
    
    impl_->contacts.clear();
    impl_->potential_pairs.clear();
    
    reset();
}

// ============================================================================
// Body Management
// ============================================================================

BodyHandle PhysicsWorldGPU::create_body(const RigidBodyDesc& desc) {
    RigidBody body;
    body.position = desc.position;
    body.orientation = desc.orientation;
    body.linear_velocity = desc.linear_velocity;
    body.angular_velocity = desc.angular_velocity;
    body.set_mass(desc.mass);
    body.set_inertia_tensor(desc.inertia_tensor);
    body.type = desc.type;
    body.shape = desc.shape;
    body.user_data = desc.user_data;

    BodyHandle handle = create_body(body);
    // Store half_extents (RigidBody doesn't carry them through SoA)
    size_t index = impl_->handle_to_index[handle.id];
    if (index < impl_->body_half_extents.size())
        impl_->body_half_extents[index] = desc.half_extents;
    return handle;
}

BodyHandle PhysicsWorldGPU::create_body(const RigidBody& body) {
    // Allocate handle
    BodyID id = impl_->next_body_id++;
    BodyHandle handle(id);
    
    // Add to buffer
    size_t index = impl_->body_buffer.add_body(body);
    
    // Track mapping
    if (id >= impl_->handle_to_index.size()) {
        impl_->handle_to_index.resize(id + 1, INVALID_BODY_ID);
    }
    impl_->handle_to_index[id] = static_cast<BodyID>(index);
    
    if (index >= impl_->index_to_handle.size()) {
        impl_->index_to_handle.resize(index + 1);
        impl_->body_active_flags.resize(index + 1, false);
        impl_->body_half_extents.resize(index + 1, Vec3(0.5f));
    }
    impl_->index_to_handle[index] = handle;
    impl_->body_active_flags[index] = true;
    // half_extents set by create_body(RigidBodyDesc) overload; default to 0.5
    impl_->body_half_extents[index] = Vec3(0.5f);

    impl_->active_body_count++;
    
    return handle;
}

void PhysicsWorldGPU::destroy_body(BodyHandle handle) {
    if (!is_body_valid(handle)) return;
    
    size_t index = handle_to_index(handle);
    
    // Mark as inactive (soft delete)
    impl_->body_active_flags[index] = false;
    impl_->handle_to_index[handle.id] = INVALID_BODY_ID;
    
    // Set to static so it doesn't participate in simulation
    impl_->body_buffer.types[index] = BodyType::STATIC;
    impl_->body_buffer.inv_masses[index] = 0.0f;
    
    impl_->active_body_count--;
    
    // TODO: Remove joints connected to this body
}

bool PhysicsWorldGPU::is_body_valid(BodyHandle handle) const {
    if (handle.id >= impl_->handle_to_index.size()) return false;
    
    BodyID index = impl_->handle_to_index[handle.id];
    if (index == INVALID_BODY_ID) return false;
    
    if (index >= impl_->body_active_flags.size()) return false;
    return impl_->body_active_flags[index];
}

size_t PhysicsWorldGPU::get_body_count() const {
    return impl_->active_body_count;
}

// ============================================================================
// Body State Queries
// ============================================================================

Vec3 PhysicsWorldGPU::get_body_position(BodyHandle handle) const {
    if (!is_body_valid(handle)) return Vec3::zero();
    return impl_->body_buffer.positions[handle_to_index(handle)];
}

Quaternion PhysicsWorldGPU::get_body_orientation(BodyHandle handle) const {
    if (!is_body_valid(handle)) return Quaternion::identity();
    return impl_->body_buffer.orientations[handle_to_index(handle)];
}

Vec3 PhysicsWorldGPU::get_body_linear_velocity(BodyHandle handle) const {
    if (!is_body_valid(handle)) return Vec3::zero();
    return impl_->body_buffer.linear_velocities[handle_to_index(handle)];
}

Vec3 PhysicsWorldGPU::get_body_angular_velocity(BodyHandle handle) const {
    if (!is_body_valid(handle)) return Vec3::zero();
    return impl_->body_buffer.angular_velocities[handle_to_index(handle)];
}

float PhysicsWorldGPU::get_body_mass(BodyHandle handle) const {
    if (!is_body_valid(handle)) return 0.0f;
    return impl_->body_buffer.masses[handle_to_index(handle)];
}

BodyType PhysicsWorldGPU::get_body_type(BodyHandle handle) const {
    if (!is_body_valid(handle)) return BodyType::STATIC;
    return impl_->body_buffer.types[handle_to_index(handle)];
}

bool PhysicsWorldGPU::is_body_sleeping(BodyHandle handle) const {
    if (!is_body_valid(handle)) return true;
    return impl_->body_buffer.is_sleeping_flags[handle_to_index(handle)];
}

RigidBody PhysicsWorldGPU::get_body(BodyHandle handle) const {
    if (!is_body_valid(handle)) return RigidBody();
    return impl_->body_buffer.get_body(handle_to_index(handle));
}

// ============================================================================
// Body State Modification
// ============================================================================

void PhysicsWorldGPU::set_body_position(BodyHandle handle, const Vec3& position) {
    if (!is_body_valid(handle)) return;
    impl_->body_buffer.positions[handle_to_index(handle)] = position;
}

void PhysicsWorldGPU::set_body_orientation(BodyHandle handle, const Quaternion& orientation) {
    if (!is_body_valid(handle)) return;
    impl_->body_buffer.orientations[handle_to_index(handle)] = orientation;
}

void PhysicsWorldGPU::set_body_linear_velocity(BodyHandle handle, const Vec3& velocity) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    impl_->body_buffer.linear_velocities[idx] = velocity;
    impl_->body_buffer.is_sleeping_flags[idx] = false;
}

void PhysicsWorldGPU::set_body_angular_velocity(BodyHandle handle, const Vec3& velocity) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    impl_->body_buffer.angular_velocities[idx] = velocity;
    impl_->body_buffer.is_sleeping_flags[idx] = false;
}

void PhysicsWorldGPU::set_body_type(BodyHandle handle, BodyType type) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    impl_->body_buffer.types[idx] = type;
    
    if (type == BodyType::STATIC) {
        impl_->body_buffer.inv_masses[idx] = 0.0f;
        impl_->body_buffer.linear_velocities[idx] = Vec3::zero();
        impl_->body_buffer.angular_velocities[idx] = Vec3::zero();
    }
}

void PhysicsWorldGPU::apply_force(BodyHandle handle, const Vec3& force) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    impl_->body_buffer.forces[idx] = impl_->body_buffer.forces[idx] + force;
    impl_->body_buffer.is_sleeping_flags[idx] = false;
}

void PhysicsWorldGPU::apply_force_at_point(BodyHandle handle, const Vec3& force, const Vec3& point) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    
    impl_->body_buffer.forces[idx] = impl_->body_buffer.forces[idx] + force;
    
    Vec3 r = point - impl_->body_buffer.positions[idx];
    impl_->body_buffer.torques[idx] = impl_->body_buffer.torques[idx] + r.cross(force);
    
    impl_->body_buffer.is_sleeping_flags[idx] = false;
}

void PhysicsWorldGPU::apply_torque(BodyHandle handle, const Vec3& torque) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    impl_->body_buffer.torques[idx] = impl_->body_buffer.torques[idx] + torque;
    impl_->body_buffer.is_sleeping_flags[idx] = false;
}

void PhysicsWorldGPU::apply_impulse(BodyHandle handle, const Vec3& impulse) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    
    if (impl_->body_buffer.types[idx] != BodyType::DYNAMIC) return;
    
    float inv_mass = impl_->body_buffer.inv_masses[idx];
    impl_->body_buffer.linear_velocities[idx] = 
        impl_->body_buffer.linear_velocities[idx] + impulse * inv_mass;
    impl_->body_buffer.is_sleeping_flags[idx] = false;
}

void PhysicsWorldGPU::apply_impulse_at_point(BodyHandle handle, const Vec3& impulse, const Vec3& point) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    
    if (impl_->body_buffer.types[idx] != BodyType::DYNAMIC) return;
    
    float inv_mass = impl_->body_buffer.inv_masses[idx];
    impl_->body_buffer.linear_velocities[idx] = 
        impl_->body_buffer.linear_velocities[idx] + impulse * inv_mass;
    
    Vec3 r = point - impl_->body_buffer.positions[idx];
    Quaternion orientation = impl_->body_buffer.orientations[idx];
    Matrix3 rotation = Matrix3::from_quaternion(orientation);
    Matrix3 inv_inertia_world = rotation * impl_->body_buffer.inv_inertia_tensors[idx] * rotation.transposed();
    
    impl_->body_buffer.angular_velocities[idx] = 
        impl_->body_buffer.angular_velocities[idx] + inv_inertia_world * r.cross(impulse);
    
    impl_->body_buffer.is_sleeping_flags[idx] = false;
}

void PhysicsWorldGPU::wake_body(BodyHandle handle) {
    if (!is_body_valid(handle)) return;
    size_t idx = handle_to_index(handle);
    impl_->body_buffer.is_sleeping_flags[idx] = false;
    impl_->body_buffer.sleep_times[idx] = 0.0f;
}

// ============================================================================
// Joint Management
// ============================================================================

ConstraintHandle PhysicsWorldGPU::create_joint(const JointDescriptor& desc) {
    if (!is_body_valid(desc.body_a) || !is_body_valid(desc.body_b)) {
        return ConstraintHandle();
    }
    
    ConstraintID id = impl_->next_joint_id++;
    ConstraintHandle handle(id);
    
    int index_a = static_cast<int>(handle_to_index(desc.body_a));
    int index_b = static_cast<int>(handle_to_index(desc.body_b));
    
    switch (desc.type) {
        case JointType::BALL_SOCKET: {
            BallSocketJoint joint(index_a, index_b, desc.local_anchor_a, desc.local_anchor_b);
            impl_->ball_socket_joints.push_back(joint);
            impl_->joint_map[id] = {JointType::BALL_SOCKET, impl_->ball_socket_joints.size() - 1};
            break;
        }
        case JointType::HINGE: {
            HingeJoint joint;
            joint.body_a_index = index_a;
            joint.body_b_index = index_b;
            joint.local_anchor_a = desc.local_anchor_a;
            joint.local_anchor_b = desc.local_anchor_b;
            joint.local_axis_a = desc.local_axis_a;
            joint.local_axis_b = desc.local_axis_b;
            joint.enable_limits = desc.enable_limits;
            joint.angle_lower_limit = desc.lower_limit;
            joint.angle_upper_limit = desc.upper_limit;
            joint.enable_motor = desc.enable_motor;
            joint.motor_target_velocity = desc.motor_target_velocity;
            joint.motor_max_torque = desc.motor_max_force_or_torque;
            impl_->hinge_joints.push_back(joint);
            impl_->joint_map[id] = {JointType::HINGE, impl_->hinge_joints.size() - 1};
            break;
        }
        case JointType::SLIDER: {
            SliderJoint joint;
            joint.body_a_index = index_a;
            joint.body_b_index = index_b;
            joint.local_anchor_a = desc.local_anchor_a;
            joint.local_anchor_b = desc.local_anchor_b;
            joint.local_axis_a = desc.local_axis_a;
            joint.enable_limits = desc.enable_limits;
            joint.position_lower_limit = desc.lower_limit;
            joint.position_upper_limit = desc.upper_limit;
            joint.enable_motor = desc.enable_motor;
            joint.motor_target_velocity = desc.motor_target_velocity;
            joint.motor_max_force = desc.motor_max_force_or_torque;
            impl_->slider_joints.push_back(joint);
            impl_->joint_map[id] = {JointType::SLIDER, impl_->slider_joints.size() - 1};
            break;
        }
        case JointType::FIXED: {
            FixedJoint joint;
            joint.body_a_index = index_a;
            joint.body_b_index = index_b;
            joint.local_anchor_a = desc.local_anchor_a;
            joint.local_anchor_b = desc.local_anchor_b;
            
            // Capture reference orientation
            Quaternion q_a = impl_->body_buffer.orientations[index_a];
            Quaternion q_b = impl_->body_buffer.orientations[index_b];
            joint.relative_orientation = q_a.conjugate() * q_b;
            
            impl_->fixed_joints.push_back(joint);
            impl_->joint_map[id] = {JointType::FIXED, impl_->fixed_joints.size() - 1};
            break;
        }
    }
    
    return handle;
}

void PhysicsWorldGPU::destroy_joint(ConstraintHandle handle) {
    auto it = impl_->joint_map.find(handle.id);
    if (it == impl_->joint_map.end()) return;
    
    // Mark joint as invalid (soft delete)
    auto [type, index] = it->second;
    switch (type) {
        case JointType::BALL_SOCKET:
            impl_->ball_socket_joints[index].body_a_index = -1;
            break;
        case JointType::HINGE:
            impl_->hinge_joints[index].body_a_index = -1;
            break;
        case JointType::SLIDER:
            impl_->slider_joints[index].body_a_index = -1;
            break;
        case JointType::FIXED:
            impl_->fixed_joints[index].body_a_index = -1;
            break;
    }
    
    impl_->joint_map.erase(it);
}

bool PhysicsWorldGPU::is_joint_valid(ConstraintHandle handle) const {
    return impl_->joint_map.find(handle.id) != impl_->joint_map.end();
}

size_t PhysicsWorldGPU::get_joint_count() const {
    return impl_->joint_map.size();
}

void PhysicsWorldGPU::set_joint_motor_velocity(ConstraintHandle handle, float velocity) {
    auto it = impl_->joint_map.find(handle.id);
    if (it == impl_->joint_map.end()) return;
    
    auto [type, index] = it->second;
    switch (type) {
        case JointType::HINGE:
            impl_->hinge_joints[index].motor_target_velocity = velocity;
            break;
        case JointType::SLIDER:
            impl_->slider_joints[index].motor_target_velocity = velocity;
            break;
        default:
            break;
    }
}

void PhysicsWorldGPU::set_joint_motor_max_force(ConstraintHandle handle, float max_force) {
    auto it = impl_->joint_map.find(handle.id);
    if (it == impl_->joint_map.end()) return;
    
    auto [type, index] = it->second;
    switch (type) {
        case JointType::HINGE:
            impl_->hinge_joints[index].motor_max_torque = max_force;
            break;
        case JointType::SLIDER:
            impl_->slider_joints[index].motor_max_force = max_force;
            break;
        default:
            break;
    }
}

void PhysicsWorldGPU::set_joint_motor_enabled(ConstraintHandle handle, bool enabled) {
    auto it = impl_->joint_map.find(handle.id);
    if (it == impl_->joint_map.end()) return;
    
    auto [type, index] = it->second;
    switch (type) {
        case JointType::HINGE:
            impl_->hinge_joints[index].enable_motor = enabled;
            break;
        case JointType::SLIDER:
            impl_->slider_joints[index].enable_motor = enabled;
            break;
        default:
            break;
    }
}

// ============================================================================
// World Configuration
// ============================================================================

void PhysicsWorldGPU::set_gravity(const Vec3& gravity) {
    impl_->config.gravity = gravity;
}

Vec3 PhysicsWorldGPU::get_gravity() const {
    return impl_->config.gravity;
}

void PhysicsWorldGPU::set_time_step(float time_step) {
    impl_->config.fixed_time_step = time_step;
}

float PhysicsWorldGPU::get_time_step() const {
    return impl_->config.fixed_time_step;
}

void PhysicsWorldGPU::set_solver_iterations(int iterations) {
    impl_->config.solver_iterations = iterations;
}

int PhysicsWorldGPU::get_solver_iterations() const {
    return impl_->config.solver_iterations;
}

// ============================================================================
// Queries
// ============================================================================

BodyHandle PhysicsWorldGPU::raycast(const Vec3& origin, const Vec3& direction, float max_distance,
                                    Vec3* hit_point, Vec3* hit_normal) const {
    if (!impl_ || impl_->body_buffer.count == 0) return BodyHandle();

    Vec3 d = direction.normalized();

    float best_t = max_distance;
    size_t best_i = SIZE_MAX;
    Vec3   best_n;

    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        if (!impl_->body_active_flags[i]) continue;

        Vec3 center = impl_->body_buffer.positions[i];
        Quaternion q = impl_->body_buffer.orientations[i];
        Vec3 half = (i < impl_->body_half_extents.size())
                        ? impl_->body_half_extents[i]
                        : Vec3(0.5f);
        Matrix3 R = Matrix3::from_quaternion(q);

        // Transform ray to OBB local space
        Vec3 d_loc = R.transposed() * d;
        Vec3 o_loc = R.transposed() * (origin - center);

        auto safe_inv = [](float v) {
            return std::abs(v) > 1e-9f ? 1.0f / v : (v >= 0.0f ? 1e30f : -1e30f);
        };
        Vec3 inv_d(safe_inv(d_loc.x), safe_inv(d_loc.y), safe_inv(d_loc.z));

        float tx1 = (-half.x - o_loc.x) * inv_d.x, tx2 = ( half.x - o_loc.x) * inv_d.x;
        float ty1 = (-half.y - o_loc.y) * inv_d.y, ty2 = ( half.y - o_loc.y) * inv_d.y;
        float tz1 = (-half.z - o_loc.z) * inv_d.z, tz2 = ( half.z - o_loc.z) * inv_d.z;
        float tmin_x = std::min(tx1,tx2), tmax_x = std::max(tx1,tx2);
        float tmin_y = std::min(ty1,ty2), tmax_y = std::max(ty1,ty2);
        float tmin_z = std::min(tz1,tz2), tmax_z = std::max(tz1,tz2);
        float tmin = std::max({tmin_x, tmin_y, tmin_z});
        float tmax = std::min({tmax_x, tmax_y, tmax_z});

        if (tmax < 0.0f || tmin > tmax || tmin > best_t) continue;
        float t = (tmin >= 0.0f) ? tmin : tmax;

        if (t < best_t) {
            best_t = t;
            best_i = i;
            Vec3 loc_n;
            if (tmin == tmin_x)      loc_n = Vec3(tx1 < tx2 ? -1.0f : 1.0f, 0, 0);
            else if (tmin == tmin_y) loc_n = Vec3(0, ty1 < ty2 ? -1.0f : 1.0f, 0);
            else                     loc_n = Vec3(0, 0, tz1 < tz2 ? -1.0f : 1.0f);
            best_n = (R * loc_n).normalized();
        }
    }

    if (best_i == SIZE_MAX) return BodyHandle();
    if (hit_point)  *hit_point  = origin + d * best_t;
    if (hit_normal) *hit_normal = best_n;
    return impl_->index_to_handle[best_i];
}

void PhysicsWorldGPU::query_aabb(const AABB& aabb, QueryCallback callback) const {
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        if (!impl_->body_active_flags[i]) continue;
        Vec3 half = (i < impl_->body_half_extents.size())
                        ? impl_->body_half_extents[i]
                        : Vec3(0.5f);
        Vec3 pos = impl_->body_buffer.positions[i];
        AABB body_box(pos - half, pos + half);
        if (aabb.overlaps(body_box)) {
            if (!callback(impl_->index_to_handle[i])) return;
        }
    }
}

// ============================================================================
// Callbacks
// ============================================================================

void PhysicsWorldGPU::set_contact_callback(ContactCallback callback) {
    impl_->contact_callback = std::move(callback);
}

// ============================================================================
// Statistics
// ============================================================================

SimulationStatistics PhysicsWorldGPU::get_statistics() const {
    return impl_->statistics;
}

void PhysicsWorldGPU::reset_statistics() {
    impl_->statistics = SimulationStatistics();
}

// ============================================================================
// Buffer Access
// ============================================================================

const RigidBodySoA& PhysicsWorldGPU::get_body_buffer() const {
    return impl_->body_buffer;
}

void PhysicsWorldGPU::synchronize() {
#if USE_GPU
    if (impl_->use_gpu) {
        impl_->download_from_gpu();
    }
#endif
}

// ============================================================================
// Internal Pipeline Stages
// ============================================================================

void PhysicsWorldGPU::apply_external_forces() {
    const Vec3 gravity = impl_->config.gravity;
    
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        if (!impl_->body_active_flags[i]) continue;
        if (impl_->body_buffer.types[i] != BodyType::DYNAMIC) continue;
        if (impl_->body_buffer.is_sleeping_flags[i]) continue;
        
        float mass = impl_->body_buffer.masses[i];
        impl_->body_buffer.forces[i] = impl_->body_buffer.forces[i] + gravity * mass;
    }
}

void PhysicsWorldGPU::broadphase() {
    auto start = std::chrono::high_resolution_clock::now();
    
    impl_->broadphase_hash.clear();
    impl_->potential_pairs.clear();
    
    // Build AABBs and insert into spatial hash
    std::vector<AABB> aabbs(impl_->body_buffer.count);
    
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        if (!impl_->body_active_flags[i]) continue;
        
        Vec3 pos = impl_->body_buffer.positions[i];
        
        // Simplified AABB (1 unit half-extent for now)
        // TODO: Compute from actual shape
        float half_extent = 0.5f;
        aabbs[i] = AABB(pos - Vec3(half_extent), pos + Vec3(half_extent));
        
        impl_->broadphase_hash.insert(static_cast<int>(i), aabbs[i]);
    }
    
    // Query for pairs using spatial hash
    std::vector<collision::BroadphasePair> raw_pairs;
    impl_->broadphase_hash.query_pairs(raw_pairs);

    for (const auto& rp : raw_pairs) {
        int i = rp.body_a;
        int j = rp.body_b;
        if (!impl_->body_active_flags[i] || !impl_->body_active_flags[j]) continue;
        if (impl_->body_buffer.types[i] == BodyType::STATIC &&
            impl_->body_buffer.types[j] == BodyType::STATIC) continue;
        if (aabbs[i].overlaps(aabbs[j])) {
            impl_->potential_pairs.emplace_back(
                impl_->index_to_handle[i],
                impl_->index_to_handle[j]
            );
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    impl_->statistics.broadphase_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
}

void PhysicsWorldGPU::narrowphase() {
    auto start = std::chrono::high_resolution_clock::now();
    
    impl_->contacts.clear();
    
    // For each potential pair, run GJK/EPA
    for (const auto& pair : impl_->potential_pairs) {
        size_t idx_a = handle_to_index(pair.body_a);
        size_t idx_b = handle_to_index(pair.body_b);
        
        RigidBody body_a = impl_->body_buffer.get_body(idx_a);
        RigidBody body_b = impl_->body_buffer.get_body(idx_b);
        
        // Simplified collision: Sphere-Sphere for now
        // TODO: Use proper GJK/EPA with shapes
        Vec3 delta = body_b.position - body_a.position;
        float dist = delta.length();
        float combined_radius = 1.0f;  // Simplified
        
        if (dist < combined_radius && dist > EPSILON) {
            ContactConstraint contact;
            contact.bodyA_index = static_cast<int>(idx_a);
            contact.bodyB_index = static_cast<int>(idx_b);
            contact.normal = delta / dist;
            contact.contact_point = body_a.position + contact.normal * (combined_radius * 0.5f);
            contact.penetration = combined_radius - dist;
            contact.friction = config::DEFAULT_FRICTION;
            contact.restitution = config::DEFAULT_RESTITUTION;
            
            impl_->contacts.push_back(contact);
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    impl_->statistics.narrowphase_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
}

void PhysicsWorldGPU::solve_constraints() {
    auto start = std::chrono::high_resolution_clock::now();
    
    const float dt = impl_->config.fixed_time_step;
    const int iterations = impl_->config.solver_iterations;
    
    // Convert SoA to AoS for solver (temporary)
    std::vector<RigidBody> bodies(impl_->body_buffer.count);
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        bodies[i] = impl_->body_buffer.get_body(i);
    }
    
    // Pre-solve contacts
    Solver::pre_solve(impl_->contacts.data(), static_cast<int>(impl_->contacts.size()),
                      bodies.data(), static_cast<int>(bodies.size()), dt);
    
    // Pre-solve joints
    JointSolver::pre_solve_ball_socket(impl_->ball_socket_joints.data(), 
                                        static_cast<int>(impl_->ball_socket_joints.size()),
                                        bodies.data(), static_cast<int>(bodies.size()), dt);
    JointSolver::pre_solve_hinge(impl_->hinge_joints.data(),
                                  static_cast<int>(impl_->hinge_joints.size()),
                                  bodies.data(), static_cast<int>(bodies.size()), dt);
    JointSolver::pre_solve_slider(impl_->slider_joints.data(),
                                   static_cast<int>(impl_->slider_joints.size()),
                                   bodies.data(), static_cast<int>(bodies.size()), dt);
    JointSolver::pre_solve_fixed(impl_->fixed_joints.data(),
                                  static_cast<int>(impl_->fixed_joints.size()),
                                  bodies.data(), static_cast<int>(bodies.size()), dt);
    
    // Velocity iterations
    for (int iter = 0; iter < iterations; ++iter) {
        // Solve contacts
        Solver::solve_velocity(impl_->contacts.data(), static_cast<int>(impl_->contacts.size()),
                               bodies.data(), static_cast<int>(bodies.size()), dt);
        
        // Solve joints
        JointSolver::solve_velocity_ball_socket(impl_->ball_socket_joints.data(),
                                                 static_cast<int>(impl_->ball_socket_joints.size()),
                                                 bodies.data(), static_cast<int>(bodies.size()));
        JointSolver::solve_velocity_hinge(impl_->hinge_joints.data(),
                                           static_cast<int>(impl_->hinge_joints.size()),
                                           bodies.data(), static_cast<int>(bodies.size()));
        JointSolver::solve_velocity_slider(impl_->slider_joints.data(),
                                            static_cast<int>(impl_->slider_joints.size()),
                                            bodies.data(), static_cast<int>(bodies.size()));
        JointSolver::solve_velocity_fixed(impl_->fixed_joints.data(),
                                           static_cast<int>(impl_->fixed_joints.size()),
                                           bodies.data(), static_cast<int>(bodies.size()));
    }
    
    // Write back to SoA
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        impl_->body_buffer.set_body(i, bodies[i]);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    impl_->statistics.solver_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
}

void PhysicsWorldGPU::integrate() {
    auto start = std::chrono::high_resolution_clock::now();
    
    const float dt = impl_->config.fixed_time_step;
    
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        if (!impl_->body_active_flags[i]) continue;
        if (impl_->body_buffer.types[i] != BodyType::DYNAMIC) continue;
        if (impl_->body_buffer.is_sleeping_flags[i]) continue;
        
        // Linear integration
        Vec3 force = impl_->body_buffer.forces[i];
        float inv_mass = impl_->body_buffer.inv_masses[i];
        
        Vec3& velocity = impl_->body_buffer.linear_velocities[i];
        velocity = velocity + force * inv_mass * dt;
        
        // Apply damping
        float linear_damping = 0.99f;  // 1% damping per step
        velocity = velocity * linear_damping;
        
        impl_->body_buffer.positions[i] = impl_->body_buffer.positions[i] + velocity * dt;
        
        // Angular integration
        Vec3 torque = impl_->body_buffer.torques[i];
        Quaternion& orientation = impl_->body_buffer.orientations[i];
        Matrix3 rotation = Matrix3::from_quaternion(orientation);
        Matrix3 inv_inertia_world = rotation * impl_->body_buffer.inv_inertia_tensors[i] * rotation.transposed();
        
        Vec3& angular_velocity = impl_->body_buffer.angular_velocities[i];
        angular_velocity = angular_velocity + inv_inertia_world * torque * dt;
        
        // Apply angular damping
        float angular_damping = 0.99f;
        angular_velocity = angular_velocity * angular_damping;
        
        // Update orientation
        Quaternion q_vel(angular_velocity.x * dt, angular_velocity.y * dt, angular_velocity.z * dt, 0.0f);
        Quaternion spin = q_vel * orientation;
        orientation.x += spin.x * 0.5f;
        orientation.y += spin.y * 0.5f;
        orientation.z += spin.z * 0.5f;
        orientation.w += spin.w * 0.5f;
        orientation.normalize();
        
        // Clear force accumulators
        impl_->body_buffer.forces[i] = Vec3::zero();
        impl_->body_buffer.torques[i] = Vec3::zero();
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    impl_->statistics.integration_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
}

void PhysicsWorldGPU::update_sleep_states() {
    const float linear_threshold_sq = impl_->config.sleep_linear_threshold * impl_->config.sleep_linear_threshold;
    const float angular_threshold_sq = impl_->config.sleep_angular_threshold * impl_->config.sleep_angular_threshold;
    const float time_threshold = impl_->config.sleep_time_threshold;
    const float dt = impl_->config.fixed_time_step;
    
    size_t sleeping_count = 0;
    
    for (size_t i = 0; i < impl_->body_buffer.count; ++i) {
        if (!impl_->body_active_flags[i]) continue;
        if (impl_->body_buffer.types[i] != BodyType::DYNAMIC) continue;
        
        float linear_speed_sq = impl_->body_buffer.linear_velocities[i].length_squared();
        float angular_speed_sq = impl_->body_buffer.angular_velocities[i].length_squared();
        
        if (linear_speed_sq < linear_threshold_sq && angular_speed_sq < angular_threshold_sq) {
            impl_->body_buffer.sleep_times[i] += dt;
            
            if (impl_->body_buffer.sleep_times[i] > time_threshold) {
                impl_->body_buffer.is_sleeping_flags[i] = true;
                impl_->body_buffer.linear_velocities[i] = Vec3::zero();
                impl_->body_buffer.angular_velocities[i] = Vec3::zero();
            }
        } else {
            impl_->body_buffer.sleep_times[i] = 0.0f;
            impl_->body_buffer.is_sleeping_flags[i] = false;
        }
        
        if (impl_->body_buffer.is_sleeping_flags[i]) {
            sleeping_count++;
        }
    }
    
    impl_->statistics.sleeping_body_count = sleeping_count;
}

void PhysicsWorldGPU::invoke_callbacks() {
    if (!impl_->contact_callback) return;
    
    for (const auto& contact : impl_->contacts) {
        BodyHandle body_a = impl_->index_to_handle[contact.bodyA_index];
        BodyHandle body_b = impl_->index_to_handle[contact.bodyB_index];
        
        impl_->contact_callback(body_a, body_b, contact.contact_point, contact.normal);
    }
}

// ============================================================================
// Utility
// ============================================================================

size_t PhysicsWorldGPU::handle_to_index(BodyHandle handle) const {
    if (handle.id >= impl_->handle_to_index.size()) return 0;
    return impl_->handle_to_index[handle.id];
}

BodyHandle PhysicsWorldGPU::index_to_handle(size_t index) const {
    if (index >= impl_->index_to_handle.size()) return BodyHandle();
    return impl_->index_to_handle[index];
}

} // namespace engine
} // namespace basements

#ifndef BASEMENTS_ENGINE_UNIFIED_ENGINE_H
#define BASEMENTS_ENGINE_UNIFIED_ENGINE_H

#include "basements/core/math/vec3.h"
#include "core/physics_state.h"
#include "systems/mechanical_system.h"
#include "systems/optimized_mechanical_system.h"
#include "utils/error_handling.h"
#include "utils/serialization.h"
#include "plugins/plugin_manager.h"
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <cstdint>

namespace basements {
namespace engine {

using namespace basements::math;

/// Object/// Handle for physics objects with validation
struct ObjectHandle {
    uint32_t id;
    
    static constexpr uint32_t INVALID_ID = UINT32_MAX;
    
    ObjectHandle() : id(INVALID_ID) {}
    explicit ObjectHandle(uint32_t object_id) : id(object_id) {}
    
    bool is_valid() const { return id != INVALID_ID; }
    void invalidate() { id = INVALID_ID; }
    
    bool operator==(const ObjectHandle& other) const { return id == other.id; }
    bool operator!=(const ObjectHandle& other) const { return id != other.id; }
};

/// Rigid body description
struct RigidBodyDesc {
    Vec3 position = Vec3::zero();
    Vec3 velocity = Vec3::zero();
    Vec3 angular_velocity = Vec3::zero();
    float mass = 1.0f;
    Vec3 gravity = Vec3(0, -9.81f, 0);
};

/// Backend selection
enum class EngineBackend {
    BASIC,      // Simple, easy to debug (PhysicsState + MechanicalSystem)
    OPTIMIZED,  // Handle-based, fast (OptimizedMechanicalSystem)
    GPU         // CUDA accelerated (future)
};

/// Unified physics engine interface
class UnifiedPhysicsEngine {
public:
    explicit UnifiedPhysicsEngine(EngineBackend backend = EngineBackend::OPTIMIZED);
    ~UnifiedPhysicsEngine();

    // Object management
    ObjectHandle add_rigid_body(const RigidBodyDesc& description);
    void remove_rigid_body(ObjectHandle object_handle);
    size_t get_rigid_body_count() const;

    // Simulation control
    void update_simulation(float delta_time_seconds);
    void reset_simulation();
    void set_fixed_time_step(float time_step_seconds);

    // Global physics settings
    void set_global_gravity(const Vec3& gravity_acceleration);
    Vec3 get_global_gravity() const;

    // State queries (read-only)
    Vec3 get_rigid_body_position(ObjectHandle object_handle) const;
    Vec3 get_rigid_body_velocity(ObjectHandle object_handle) const;
    Vec3 get_rigid_body_acceleration(ObjectHandle object_handle) const;
    
    float get_rigid_body_kinetic_energy(ObjectHandle object_handle) const;
    float get_rigid_body_potential_energy(ObjectHandle object_handle) const;
    float get_rigid_body_total_energy(ObjectHandle object_handle) const;
    
    Vec3 get_rigid_body_momentum(ObjectHandle object_handle) const;
    float get_rigid_body_speed(ObjectHandle object_handle) const;

    // State modification (write)
    void set_rigid_body_position(ObjectHandle object_handle, const Vec3& new_position);
    void set_rigid_body_velocity(ObjectHandle object_handle, const Vec3& new_velocity);
    void apply_force_to_rigid_body(ObjectHandle object_handle, const Vec3& force_vector);
    void apply_impulse_to_rigid_body(ObjectHandle object_handle, const Vec3& impulse_vector);

    // Backend info
    EngineBackend get_backend() const { return backend_; }
    std::string get_backend_name() const;

    // Performance statistics
    struct PerformanceStatistics {
        float last_update_duration_milliseconds;
        size_t total_simulation_updates;
        size_t total_rigid_bodies;
    };
    PerformanceStatistics get_performance_statistics() const;

    // Plugin management
    void add_plugin(std::unique_ptr<PhysicsPlugin> plugin);
    bool has_plugin(const std::string& plugin_name) const;
    std::vector<std::string> list_plugins() const;

    // Serialization
    bool save_state_to_file(const std::string& filename) const;
    bool load_state_from_file(const std::string& filename);

    // Error handling
    void set_error_handler(ErrorHandler handler);

    class Impl;

private:

private:
    std::unique_ptr<Impl> impl_;
    EngineBackend backend_;
    PerformanceStatistics stats_;
    PluginManager plugin_manager_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_UNIFIED_ENGINE_H

#include "basements/engine/unified_engine.h"
#include "basements/engine/utils/error_handling.h"
#include "basements/engine/utils/validation.h"
#include "basements/engine/utils/debug_utils.h"
#include <chrono>
#include <unordered_map>

namespace basements {
namespace engine {

// Implementation details
class UnifiedPhysicsEngine::Impl {
public:
    virtual ~Impl() = default;
    
    virtual ObjectHandle add_rigid_body(const RigidBodyDesc& desc) = 0;
    virtual void remove_object(ObjectHandle handle) = 0;
    virtual size_t object_count() const = 0;
    
    virtual void update(float dt) = 0;
    virtual void reset() = 0;
    
    virtual void set_gravity(const Vec3& g) = 0;
    virtual Vec3 get_gravity() const = 0;
    
    virtual Vec3 get_position(ObjectHandle handle) const = 0;
    virtual Vec3 get_velocity(ObjectHandle handle) const = 0;
    virtual Vec3 get_acceleration(ObjectHandle handle) const = 0;
    
    virtual float get_kinetic_energy(ObjectHandle handle) const = 0;
    virtual float get_potential_energy(ObjectHandle handle) const = 0;
    virtual float get_total_energy(ObjectHandle handle) const = 0;
    
    virtual Vec3 get_momentum(ObjectHandle handle) const = 0;
    virtual float get_speed(ObjectHandle handle) const = 0;
    
    virtual void set_position(ObjectHandle handle, const Vec3& pos) = 0;
    virtual void set_velocity(ObjectHandle handle, const Vec3& vel) = 0;
    virtual void apply_force(ObjectHandle handle, const Vec3& force) = 0;
    virtual void apply_impulse(ObjectHandle handle, const Vec3& impulse) = 0;
};

// Optimized backend implementation
class OptimizedImpl : public UnifiedPhysicsEngine::Impl {
public:
    OptimizedImpl() : next_id_(0) {}
    
    ObjectHandle add_rigid_body(const RigidBodyDesc& desc) override {
        // Validate inputs
        NumericalValidator::validate_mass(desc.mass);
        NumericalValidator::validate_velocity(desc.velocity);
        
        if (DebugUtils::has_invalid_values(desc.position)) {
            ErrorManager::throw_error(
                EngineError::INVALID_STATE,
                "Invalid position: " + DebugUtils::describe_invalid_values(desc.position)
            );
        }
        
        uint32_t id = next_id_++;
        
        auto system = std::make_unique<OptimizedMechanicalSystem>();
        system->initialize(desc.position, desc.velocity, desc.mass);
        system->set_gravity(desc.gravity);
        
        systems_[id] = std::move(system);
        return ObjectHandle(id);
    }
    
    void remove_object(ObjectHandle handle) override {
        systems_.erase(handle.id);
    }
    
    size_t object_count() const override {
        return systems_.size();
    }
    
    void update(float dt) override {
        // Validate time step
        NumericalValidator::validate_time_step(dt);
        
        for (auto& [id, system] : systems_) {
            system->update(dt);
        }
    }
    
    void reset() override {
        systems_.clear();
        next_id_ = 0;
    }
    
    void set_gravity(const Vec3& g) override {
        global_gravity_ = g;
        for (auto& [id, system] : systems_) {
            system->set_gravity(g);
        }
    }
    
    Vec3 get_gravity() const override {
        return global_gravity_;
    }
    
    Vec3 get_position(ObjectHandle handle) const override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) {
            ErrorManager::throw_error(EngineError::INVALID_HANDLE, 
                "Object handle " + std::to_string(handle.id) + " not found");
        }
        return it->second->get_position();
    }
    
    Vec3 get_velocity(ObjectHandle handle) const override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) {
            ErrorManager::throw_error(EngineError::INVALID_HANDLE,
                "Object handle " + std::to_string(handle.id) + " not found");
        }
        return it->second->get_velocity();
    }
    
    Vec3 get_acceleration(ObjectHandle handle) const override {
        return Vec3::zero(); // TODO: Store acceleration
    }
    
    float get_kinetic_energy(ObjectHandle handle) const override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) {
            ErrorManager::throw_error(EngineError::INVALID_HANDLE,
                "Object handle " + std::to_string(handle.id) + " not found");
        }
        return it->second->get_kinetic_energy();
    }
    
    float get_potential_energy(ObjectHandle handle) const override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) throw std::runtime_error("Invalid handle");
        return it->second->get_potential_energy();
    }
    
    float get_total_energy(ObjectHandle handle) const override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) throw std::runtime_error("Invalid handle");
        return it->second->get_total_energy();
    }
    
    Vec3 get_momentum(ObjectHandle handle) const override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) throw std::runtime_error("Invalid handle");
        // p = mv
        return it->second->get_velocity() * 1.0f; // TODO: Get mass
    }
    
    float get_speed(ObjectHandle handle) const override {
        return get_velocity(handle).length();
    }
    
    void set_position(ObjectHandle handle, const Vec3& pos) override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) throw std::runtime_error("Invalid handle");
        // TODO: Add setter
    }
    
    void set_velocity(ObjectHandle handle, const Vec3& vel) override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) throw std::runtime_error("Invalid handle");
        // TODO: Add setter
    }
    
    void apply_force(ObjectHandle handle, const Vec3& force) override {
        // Validate force
        NumericalValidator::validate_force(force);
        
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) {
            ErrorManager::throw_error(EngineError::INVALID_HANDLE,
                "Object handle " + std::to_string(handle.id) + " not found");
        }
        it->second->apply_force(force);
    }
    
    void apply_impulse(ObjectHandle handle, const Vec3& impulse) override {
        auto it = systems_.find(handle.id);
        if (it == systems_.end()) throw std::runtime_error("Invalid handle");
        // TODO: Add impulse support
    }
    
private:
    std::unordered_map<uint32_t, std::unique_ptr<OptimizedMechanicalSystem>> systems_;
    uint32_t next_id_;
    Vec3 global_gravity_;
};

// Constructor
UnifiedPhysicsEngine::UnifiedPhysicsEngine(EngineBackend backend)
    : backend_(backend)
{
    stats_.last_update_duration_milliseconds = 0.0f;
    stats_.total_simulation_updates = 0;
    stats_.total_rigid_bodies = 0;
    
    switch (backend) {
        case EngineBackend::OPTIMIZED:
            impl_ = std::make_unique<OptimizedImpl>();
            break;
        case EngineBackend::BASIC:
            // TODO: Implement basic backend
            throw std::runtime_error("BASIC backend not yet implemented");
        case EngineBackend::GPU:
            // TODO: Implement GPU backend
            throw std::runtime_error("GPU backend not yet implemented");
    }
}

UnifiedPhysicsEngine::~UnifiedPhysicsEngine() = default;

ObjectHandle UnifiedPhysicsEngine::add_rigid_body(const RigidBodyDesc& desc) {
    auto handle = impl_->add_rigid_body(desc);
    stats_.total_rigid_bodies = impl_->object_count();
    return handle;
}

void UnifiedPhysicsEngine::remove_rigid_body(ObjectHandle handle) {
    impl_->remove_object(handle);
    stats_.total_rigid_bodies = impl_->object_count();
}

size_t UnifiedPhysicsEngine::get_rigid_body_count() const {
    return impl_->object_count();
}

void UnifiedPhysicsEngine::update_simulation(float dt) {
    auto start = std::chrono::high_resolution_clock::now();
    
    impl_->update(dt);
    
    auto end = std::chrono::high_resolution_clock::now();
    stats_.last_update_duration_milliseconds = 
        std::chrono::duration<float, std::milli>(end - start).count();
    stats_.total_simulation_updates++;
}

void UnifiedPhysicsEngine::reset_simulation() {
    impl_->reset();
    stats_.total_simulation_updates = 0;
    stats_.total_rigid_bodies = 0;
}

void UnifiedPhysicsEngine::set_fixed_time_step(float dt) {
    // TODO: Store time step
}

void UnifiedPhysicsEngine::set_global_gravity(const Vec3& g) {
    impl_->set_gravity(g);
}

Vec3 UnifiedPhysicsEngine::get_global_gravity() const {
    return impl_->get_gravity();
}

Vec3 UnifiedPhysicsEngine::get_rigid_body_position(ObjectHandle handle) const {
    return impl_->get_position(handle);
}

Vec3 UnifiedPhysicsEngine::get_rigid_body_velocity(ObjectHandle handle) const {
    return impl_->get_velocity(handle);
}

Vec3 UnifiedPhysicsEngine::get_rigid_body_acceleration(ObjectHandle handle) const {
    return impl_->get_acceleration(handle);
}

float UnifiedPhysicsEngine::get_rigid_body_kinetic_energy(ObjectHandle handle) const {
    return impl_->get_kinetic_energy(handle);
}

float UnifiedPhysicsEngine::get_rigid_body_potential_energy(ObjectHandle handle) const {
    return impl_->get_potential_energy(handle);
}

float UnifiedPhysicsEngine::get_rigid_body_total_energy(ObjectHandle handle) const {
    return impl_->get_total_energy(handle);
}

Vec3 UnifiedPhysicsEngine::get_rigid_body_momentum(ObjectHandle handle) const {
    return impl_->get_momentum(handle);
}

float UnifiedPhysicsEngine::get_rigid_body_speed(ObjectHandle handle) const {
    return impl_->get_speed(handle);
}

void UnifiedPhysicsEngine::set_rigid_body_position(ObjectHandle handle, const Vec3& pos) {
    impl_->set_position(handle, pos);
}

void UnifiedPhysicsEngine::set_rigid_body_velocity(ObjectHandle handle, const Vec3& vel) {
    impl_->set_velocity(handle, vel);
}

void UnifiedPhysicsEngine::apply_force_to_rigid_body(ObjectHandle handle, const Vec3& force) {
    impl_->apply_force(handle, force);
}

void UnifiedPhysicsEngine::apply_impulse_to_rigid_body(ObjectHandle handle, const Vec3& impulse) {
    impl_->apply_impulse(handle, impulse);
}

std::string UnifiedPhysicsEngine::get_backend_name() const {
    switch (backend_) {
        case EngineBackend::BASIC: return "Basic";
        case EngineBackend::OPTIMIZED: return "Optimized";
        case EngineBackend::GPU: return "GPU";
        default: return "Unknown";
    }
}

UnifiedPhysicsEngine::PerformanceStatistics UnifiedPhysicsEngine::get_performance_statistics() const {
    return stats_;
}

// Plugin management
void UnifiedPhysicsEngine::add_plugin(std::unique_ptr<PhysicsPlugin> plugin) {
    plugin_manager_.register_plugin(std::move(plugin));
}

bool UnifiedPhysicsEngine::has_plugin(const std::string& plugin_name) const {
    return plugin_manager_.has_plugin(plugin_name);
}

std::vector<std::string> UnifiedPhysicsEngine::list_plugins() const {
    return plugin_manager_.list_plugins();
}

// Serialization
bool UnifiedPhysicsEngine::save_state_to_file(const std::string& filename) const {
    // TODO: Implement full serialization
    std::string state_data = "{\"backend\":\"" + get_backend_name() + "\",";
    state_data += "\"object_count\":" + std::to_string(get_rigid_body_count()) + "}";
    return Serializer::save_to_file(filename, state_data);
}

bool UnifiedPhysicsEngine::load_state_from_file(const std::string& filename) {
    // TODO: Implement full deserialization
    std::string content;
    return Serializer::load_from_file(filename, content);
}

// Error handling
void UnifiedPhysicsEngine::set_error_handler(ErrorHandler handler) {
    ErrorManager::set_error_handler(handler);
}

} // namespace engine
} // namespace basements

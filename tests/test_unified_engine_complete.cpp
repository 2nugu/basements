#include "basements/engine/unified_engine.h"
#include <iostream>

int main() {
    std::cout << "=== UnifiedPhysicsEngine Test ==="  << std::endl;

    try {
        // Create engine with error handler
        basements::engine::UnifiedPhysicsEngine engine(basements::engine::EngineBackend::OPTIMIZED);
        
        engine.set_error_handler([](basements::engine::EngineError error, const std::string& message) {
            std::cout << "[ERROR] " << message << std::endl;
        });

        // Safe string prints
        std::cout << "Backend initialized." << std::endl;

        // Add rigid body
        basements::engine::RigidBodyDesc desc;
        desc.position = basements::math::Vec3(0, 10, 0);
        desc.velocity = basements::math::Vec3::zero(); 
        desc.mass = 1.0f;
        desc.gravity = basements::math::Vec3(0, -9.81f, 0);

        auto obj = engine.add_rigid_body(desc);
        std::cout << "Added object, total: " << engine.get_rigid_body_count() << std::endl;

        // Simulate
        for (int i = 0; i < 10; ++i) {
            engine.update_simulation(0.016f);
            
            basements::math::Vec3 pos = engine.get_rigid_body_position(obj);
            basements::math::Vec3 vel = engine.get_rigid_body_velocity(obj);
            float E_k = engine.get_rigid_body_kinetic_energy(obj);
            float E_p = engine.get_rigid_body_potential_energy(obj);
            
            std::cout << "Frame " << i 
                      << ": pos=" << pos.y 
                      << " vel=" << vel.y
                      << " E_k=" << E_k
                      << " E_p=" << E_p
                      << " E_total=" << (E_k + E_p)
                      << std::endl;
        }

        // Test serialization
        std::cout << "\nTesting serialization..." << std::endl;
        if (engine.save_state_to_file("test_state.json")) {
            std::cout << "State saved successfully" << std::endl;
        }

        // Test plugin system
        std::cout << "\nPlugin count: " << engine.list_plugins().size() << std::endl;

        // Performance stats
        auto stats = engine.get_performance_statistics();
        std::cout << "\nPerformance Statistics:" << std::endl;
        std::cout << "  Last update: " << stats.last_update_duration_milliseconds << " ms" << std::endl;
        std::cout << "  Total updates: " << stats.total_simulation_updates << std::endl;
        std::cout << "  Total objects: " << stats.total_rigid_bodies << std::endl;

        // Test error handling
        std::cout << "\nTesting error handling..." << std::endl;
        try {
            basements::engine::ObjectHandle invalid_handle(9999);
            engine.get_rigid_body_position(invalid_handle);
        } catch (const basements::engine::EngineException& e) {
            std::cout << "Caught exception: " << e.what() << std::endl;
        }

        std::cout << "\n=== Test Complete ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

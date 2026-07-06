#ifndef BASEMENTS_ENGINE_THREAD_SAFETY_H
#define BASEMENTS_ENGINE_THREAD_SAFETY_H

/**
 * Thread Safety Documentation for Basements Physics Engine
 * 
 * IMPORTANT: This engine is designed for SINGLE-THREADED use by default.
 * 
 * Thread Safety Status:
 * =====================
 * 
 * ❌ NOT Thread-Safe (Default):
 * - UnifiedPhysicsEngine
 * - OptimizedMechanicalSystem
 * - QuantityStorage
 * - ComputeGraph
 * - PluginManager
 * 
 * ✅ Thread-Safe:
 * - Formula library (all functions in basements::physics::formulas::*)
 * - Math library (Vec3, Quaternion, Matrix3, Matrix4)
 * - Error handling (ErrorManager with proper callbacks)
 * 
 * Usage Guidelines:
 * =================
 * 
 * 1. SINGLE-THREADED (Recommended):
 *    ```cpp
 *    UnifiedPhysicsEngine engine;
 *    
 *    // Main loop (single thread)
 *    while (running) {
 *        engine.update_simulation(dt);
 *        // ... render, etc
 *    }
 *    ```
 * 
 * 2. MULTI-THREADED (Advanced):
 *    If you need multi-threading, use ThreadSafeState wrapper:
 *    
 *    ```cpp
 *    #include "basements/engine/core/thread_safe_state.h"
 *    
 *    ThreadSafeState<UnifiedPhysicsEngine> engine;
 *    
 *    // Thread 1: Update
 *    std::thread update_thread([&]() {
 *        engine.write([](auto& eng) {
 *            eng.update_simulation(dt);
 *        });
 *    });
 *    
 *    // Thread 2: Read
 *    std::thread read_thread([&]() {
 *        engine.read([](const auto& eng) {
 *            auto pos = eng.get_rigid_body_position(handle);
 *        });
 *    });
 *    ```
 * 
 * 3. GPU EXECUTION:
 *    GPU backend uses async streams but is thread-safe from host side:
 *    
 *    ```cpp
 *    UnifiedPhysicsEngine engine(EngineBackend::GPU);
 *    
 *    // Safe: GPU execution is async but synchronized
 *    engine.update_simulation(dt);  // Launches kernel
 *    auto pos = engine.get_position(h);  // Waits for kernel
 *    ```
 * 
 * Data Race Scenarios:
 * ====================
 * 
 * ❌ UNSAFE:
 * ```cpp
 * // Thread 1
 * engine.update_simulation(dt);
 * 
 * // Thread 2 (concurrent)
 * auto pos = engine.get_position(handle);  // RACE CONDITION!
 * ```
 * 
 * ✅ SAFE:
 * ```cpp
 * // Thread 1
 * {
 *     std::lock_guard<std::mutex> lock(engine_mutex);
 *     engine.update_simulation(dt);
 * }
 * 
 * // Thread 2
 * {
 *     std::lock_guard<std::mutex> lock(engine_mutex);
 *     auto pos = engine.get_position(handle);
 * }
 * ```
 * 
 * Performance Considerations:
 * ===========================
 * 
 * - Single-threaded: Best performance for < 10,000 objects
 * - Multi-threaded: Overhead from locking, use only if needed
 * - GPU: Best for > 10,000 objects
 * 
 * Future Work:
 * ============
 * 
 * - Lock-free data structures for QuantityStorage
 * - Thread-safe ComputeGraph execution
 * - Multi-threaded CPU backend
 */

namespace basements {
namespace engine {

// Thread safety marker (compile-time documentation)
#define THREAD_SAFE
#define NOT_THREAD_SAFE

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_THREAD_SAFETY_H

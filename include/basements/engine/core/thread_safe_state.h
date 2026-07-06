#ifndef BASEMENTS_ENGINE_CORE_THREAD_SAFE_STATE_H
#define BASEMENTS_ENGINE_CORE_THREAD_SAFE_STATE_H

#include "physics_state.h"
#include <shared_mutex>
#include <mutex>

namespace basements {
namespace engine {

/// Thread-safe wrapper for PhysicsState
class ThreadSafePhysicsState {
public:
    ThreadSafePhysicsState() = default;

    /// Get scalar value (thread-safe read)
    float get_scalar(const std::string& name) {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return state_.get_scalar(name);
    }

    /// Set scalar value (thread-safe write)
    void set_scalar(const std::string& name, float value) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        state_.set_scalar(name, value);
    }

    /// Get vector value (thread-safe read)
    Vec3 get_vector(const std::string& name) {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return state_.get_vector(name);
    }

    /// Set vector value (thread-safe write)
    void set_vector(const std::string& name, const Vec3& value) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        state_.set_vector(name, value);
    }

    /// Batch read (lock once for multiple reads)
    template<typename Func>
    auto read_batch(Func&& func) -> decltype(func(state_)) {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return func(state_);
    }

    /// Batch write (lock once for multiple writes)
    template<typename Func>
    void write_batch(Func&& func) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        func(state_);
    }

    /// Get underlying state (unsafe, for single-threaded access)
    PhysicsState& unsafe_state() { return state_; }
    const PhysicsState& unsafe_state() const { return state_; }

private:
    PhysicsState state_;
    mutable std::shared_mutex mutex_;  // Readers-writer lock
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_CORE_THREAD_SAFE_STATE_H

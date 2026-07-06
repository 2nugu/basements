#ifndef BASEMENTS_ENGINE_CORE_QUANTITY_H
#define BASEMENTS_ENGINE_CORE_QUANTITY_H

#include "basements/core/math/vec3.h"
#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

namespace basements {
namespace engine {

/// Forward declaration
class PhysicsQuantity;

/// Callback type for quantity updates
using UpdateCallback = std::function<void()>;

/// A physical quantity that can have dependencies and lazy evaluation
class PhysicsQuantity {
public:
    PhysicsQuantity(const std::string& name)
        : name_(name)
        , value_(0.0f)
        , is_dirty_(true)
        , is_constant_(false)
        , is_computing_(false)
    {}

    /// Get the current value (computes if dirty)
    float get() {
        if (is_dirty_ && compute_func_) {
            // Circular dependency detection
            if (is_computing_) {
                throw std::runtime_error(
                    "Circular dependency detected in quantity: " + name_);
            }
            
            is_computing_ = true;
            try {
                value_ = compute_func_();
                is_dirty_ = false;
                is_computing_ = false;
            } catch (...) {
                is_computing_ = false;
                throw;
            }
        }
        return value_;
    }

    /// Set the value directly (marks dependents as dirty)
    void set(float value) {
        if (is_constant_) return;  // Cannot modify constants
        
        value_ = value;
        is_dirty_ = false;
        mark_dependents_dirty();
    }

    /// Set the computation function
    void set_compute_function(std::function<float()> func) {
        compute_func_ = func;
        is_dirty_ = true;
    }

    /// Add a dependent quantity
    void add_dependent(PhysicsQuantity* dependent) {
        dependents_.push_back(dependent);
    }

    /// Mark this quantity as dirty (needs recomputation)
    void mark_dirty() {
        if (!is_dirty_) {
            is_dirty_ = true;
            mark_dependents_dirty();
        }
    }

    /// Mark as constant (cannot be modified)
    void set_constant(bool is_const) {
        is_constant_ = is_const;
    }

    /// Check if dirty
    bool is_dirty() const { return is_dirty_; }

    /// Get name
    const std::string& name() const { return name_; }

    /// Add update callback
    void add_callback(UpdateCallback callback) {
        callbacks_.push_back(callback);
    }

private:
    void mark_dependents_dirty() {
        for (auto* dep : dependents_) {
            dep->mark_dirty();
        }
        
        // Trigger callbacks
        for (auto& callback : callbacks_) {
            callback();
        }
    }

    std::string name_;
    float value_;
    bool is_dirty_;
    bool is_constant_;
    bool is_computing_;  // Guard against circular dependencies
    
    std::function<float()> compute_func_;
    std::vector<PhysicsQuantity*> dependents_;
    std::vector<UpdateCallback> callbacks_;
};

/// Vector quantity (3D)
class VectorQuantity {
public:
    VectorQuantity(const std::string& name)
        : x_(name + ".x")
        , y_(name + ".y")
        , z_(name + ".z")
        , name_(name)
    {}

    basements::math::Vec3 get() {
        return basements::math::Vec3(x_.get(), y_.get(), z_.get());
    }

    void set(const basements::math::Vec3& value) {
        x_.set(value.x);
        y_.set(value.y);
        z_.set(value.z);
    }

    PhysicsQuantity& x() { return x_; }
    PhysicsQuantity& y() { return y_; }
    PhysicsQuantity& z() { return z_; }

    const std::string& name() const { return name_; }

private:
    PhysicsQuantity x_, y_, z_;
    std::string name_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_CORE_QUANTITY_H

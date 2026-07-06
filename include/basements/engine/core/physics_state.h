#ifndef BASEMENTS_ENGINE_CORE_PHYSICS_STATE_H
#define BASEMENTS_ENGINE_CORE_PHYSICS_STATE_H

#include "quantity.h"
#include "basements/core/math/vec3.h"
#include <unordered_map>
#include <memory>
#include <stdexcept>

namespace basements {
namespace engine {

using namespace basements::math;

/// Central state container for all physical quantities
class PhysicsState {
public:
    PhysicsState() = default;

    /// Define a scalar quantity with a computation function
    void define_scalar(
        const std::string& name,
        std::function<float()> compute_func,
        const std::vector<std::string>& dependencies = {}
    ) {
        auto quantity = std::make_unique<PhysicsQuantity>(name);
        quantity->set_compute_function(compute_func);
        
        // Set up dependencies
        for (const auto& dep_name : dependencies) {
            auto it = scalars_.find(dep_name);
            if (it != scalars_.end()) {
                it->second->add_dependent(quantity.get());
            }
        }
        
        scalars_[name] = std::move(quantity);
    }

    /// Define a vector quantity
    void define_vector(const std::string& name) {
        vectors_[name] = std::make_unique<VectorQuantity>(name);
    }

    /// Get scalar value
    float get_scalar(const std::string& name) {
        auto it = scalars_.find(name);
        if (it == scalars_.end()) {
            throw std::runtime_error("Scalar quantity not found: " + name);
        }
        return it->second->get();
    }

    /// Set scalar value
    void set_scalar(const std::string& name, float value) {
        auto it = scalars_.find(name);
        if (it == scalars_.end()) {
            // Auto-create if doesn't exist
            auto quantity = std::make_unique<PhysicsQuantity>(name);
            quantity->set(value);
            scalars_[name] = std::move(quantity);
        } else {
            it->second->set(value);
        }
    }

    /// Get vector value
    Vec3 get_vector(const std::string& name) {
        auto it = vectors_.find(name);
        if (it == vectors_.end()) {
            throw std::runtime_error("Vector quantity not found: " + name);
        }
        return it->second->get();
    }

    /// Set vector value
    void set_vector(const std::string& name, const Vec3& value) {
        auto it = vectors_.find(name);
        if (it == vectors_.end()) {
            // Auto-create if doesn't exist
            auto quantity = std::make_unique<VectorQuantity>(name);
            quantity->set(value);
            vectors_[name] = std::move(quantity);
        } else {
            it->second->set(value);
        }
    }

    /// Get direct access to scalar quantity
    PhysicsQuantity* get_scalar_quantity(const std::string& name) {
        auto it = scalars_.find(name);
        return (it != scalars_.end()) ? it->second.get() : nullptr;
    }

    /// Get direct access to vector quantity
    VectorQuantity* get_vector_quantity(const std::string& name) {
        auto it = vectors_.find(name);
        return (it != vectors_.end()) ? it->second.get() : nullptr;
    }

    /// Mark a quantity as constant
    void set_constant(const std::string& name, bool is_const = true) {
        auto it = scalars_.find(name);
        if (it != scalars_.end()) {
            it->second->set_constant(is_const);
        }
    }

    /// Check if quantity exists
    bool has_scalar(const std::string& name) const {
        return scalars_.find(name) != scalars_.end();
    }

    bool has_vector(const std::string& name) const {
        return vectors_.find(name) != vectors_.end();
    }

    /// Clear all quantities
    void clear() {
        scalars_.clear();
        vectors_.clear();
    }

    /// Get all scalar names
    std::vector<std::string> get_scalar_names() const {
        std::vector<std::string> names;
        for (const auto& pair : scalars_) {
            names.push_back(pair.first);
        }
        return names;
    }

    /// Get all vector names
    std::vector<std::string> get_vector_names() const {
        std::vector<std::string> names;
        for (const auto& pair : vectors_) {
            names.push_back(pair.first);
        }
        return names;
    }

private:
    std::unordered_map<std::string, std::unique_ptr<PhysicsQuantity>> scalars_;
    std::unordered_map<std::string, std::unique_ptr<VectorQuantity>> vectors_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_CORE_PHYSICS_STATE_H

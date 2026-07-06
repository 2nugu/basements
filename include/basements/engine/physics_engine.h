#ifndef BASEMENTS_ENGINE_PHYSICS_ENGINE_H
#define BASEMENTS_ENGINE_PHYSICS_ENGINE_H

#include "core/physics_state.h"
#include "systems/mechanical_system.h"

namespace basements {
namespace engine {

/// Main physics engine interface
class PhysicsEngine {
public:
    PhysicsEngine()
        : state_()
        , mechanical_(state_)
    {}

    /// Initialize mechanical system
    void init_mechanical(const Vec3& position, const Vec3& velocity, float mass = 1.0f) {
        mechanical_.initialize(position, velocity, mass);
    }

    /// Update all systems
    void update(float dt) {
        mechanical_.update(dt);
    }

    /// Get state access
    PhysicsState& state() { return state_; }
    const PhysicsState& state() const { return state_; }

    /// Get system access
    MechanicalSystem& mechanical() { return mechanical_; }
    const MechanicalSystem& mechanical() const { return mechanical_; }

private:
    PhysicsState state_;
    MechanicalSystem mechanical_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_PHYSICS_ENGINE_H

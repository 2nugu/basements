#ifndef BASEMENTS_DYNAMICS_INTEGRATOR_H
#define BASEMENTS_DYNAMICS_INTEGRATOR_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/rigid_body.h"

namespace basements {
namespace dynamics {

using namespace basements::math;
using namespace basements::physics;

class TimeIntegrator {
public:
    /**
     * @brief Integrate rigid bodies forward in time (Semi-Implicit Euler)
     */
    HOST_DEVICE static void integrate(RigidBody* bodies, int num_bodies, float dt) {
        for (int i = 0; i < num_bodies; ++i) {
            RigidBody& b = bodies[i];
            
            if (b.type == BodyType::STATIC || b.is_sleeping) continue;

            // 1. Integrate Velocities
            // v = v + a * dt
            // a = F * invM
            b.linear_velocity += b.force * b.inv_mass * dt;
            
            // w = w + alpha * dt
            // alpha = I_inv * (T - w x (I * w)) -> Euler's equations
            // Simplified: alpha = I_inv * T (neglecting gyroscopic term for stability)
            b.angular_velocity += b.inv_inertia_tensor * b.torque * dt;

            // Apply Per-Body Damping (frame-rate independent)
            b.linear_velocity *= (1.0f - b.linear_damping * dt);
            b.angular_velocity *= (1.0f - b.angular_damping * dt);

            // 2. Integrate Positions
            // x = x + v * dt
            b.position += b.linear_velocity * dt;

            // q = q + 0.5 * w * q * dt
            // Orientation update using quaternion derivative
            Quaternion q_vel(0, b.angular_velocity.x, b.angular_velocity.y, b.angular_velocity.z);
            b.orientation += (q_vel * b.orientation) * (0.5f * dt);
            b.orientation.normalize();

            // 3. Update Inertia Tensor (CRITICAL FIX)
            // World-space inertia changes as body rotates
            b.update_inertia_tensor();

            // 4. Clear Forces
            b.force = Vec3(0, 0, 0);
            b.torque = Vec3(0, 0, 0);
        }
    }
};

} // namespace dynamics
} // namespace basements

#endif // BASEMENTS_DYNAMICS_INTEGRATOR_H

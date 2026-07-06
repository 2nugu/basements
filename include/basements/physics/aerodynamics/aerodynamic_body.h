/**
 * @file aerodynamic_body.h
 * @brief Aerodynamic Forces for General Bodies (Drag, Lift)
 * 
 * Applies simplified aerodynamic forces based on body velocity and geometry.
 */

#ifndef BASEMENTS_AERODYNAMICS_AERODYNAMIC_BODY_H
#define BASEMENTS_AERODYNAMICS_AERODYNAMIC_BODY_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/rigid_body.h"
#include <cmath>

namespace basements {
namespace aerodynamics {

using namespace basements::math;
using namespace basements::physics;

/**
 * @brief Aerodynamic Surface Configuration
 */
struct AeroSurface {
    Vec3 local_normal;      // Surface normal in body-local coordinates
    float area;             // Surface area (m²)
    float drag_coeff;       // Drag coefficient (Cd)
    float lift_coeff;       // Lift coefficient (Cl) per radian
    
    AeroSurface()
        : local_normal(Vec3(0, 1, 0))
        , area(1.0f)
        , drag_coeff(1.0f)
        , lift_coeff(0.0f)
    {}
    
    // Presets
    static AeroSurface FlatPlate(float area_m2) {
        AeroSurface surf;
        surf.area = area_m2;
        surf.drag_coeff = 1.28f;  // Flat plate perpendicular
        surf.lift_coeff = 2.0f * PI;  // Thin airfoil theory
        return surf;
    }
    
    static AeroSurface Sphere(float radius) {
        AeroSurface surf;
        surf.area = PI * radius * radius;
        surf.drag_coeff = 0.47f;
        surf.lift_coeff = 0.0f;  // Sphere has no lift
        return surf;
    }
    
    static AeroSurface Streamlined(float area_m2) {
        AeroSurface surf;
        surf.area = area_m2;
        surf.drag_coeff = 0.04f;  // Streamlined body
        surf.lift_coeff = 0.0f;
        return surf;
    }
};

/**
 * @brief Aerodynamic Body Component
 * 
 * Attach to RigidBody to enable aerodynamic forces.
 */
struct AeroBody {
    std::vector<AeroSurface> surfaces;
    float air_density;
    
    AeroBody() : air_density(1.225f) {}
    
    /**
     * @brief Add a surface to this body
     */
    void add_surface(const AeroSurface& surf) {
        surfaces.push_back(surf);
    }
    
    /**
     * @brief Calculate and apply aerodynamic forces
     */
    void apply_forces(RigidBody& body) const {
        Vec3 velocity = body.linear_velocity;
        float speed = velocity.length();
        
        if (speed < 0.01f) return;  // No forces at very low speed
        
        Vec3 velocity_dir = velocity / speed;
        float dynamic_pressure = 0.5f * air_density * speed * speed;
        
        Vec3 total_force = Vec3::zero();
        
        for (const auto& surf : surfaces) {
            // Transform surface normal to world
            Vec3 world_normal = body.orientation.rotate(surf.local_normal);
            
            // Angle of attack (angle between velocity and surface normal)
            float dot = velocity_dir.dot(world_normal);
            float sin_alpha = std::sqrt(1.0f - dot * dot);
            float cos_alpha = std::abs(dot);
            
            // Drag force (opposite to velocity)
            float drag_mag = dynamic_pressure * surf.area * surf.drag_coeff * cos_alpha;
            Vec3 drag = -velocity_dir * drag_mag;
            
            // Lift force (perpendicular to velocity, in plane of normal)
            Vec3 lift_dir = world_normal.cross(velocity_dir).cross(velocity_dir);
            if (lift_dir.length() > 0.001f) {
                lift_dir = lift_dir.normalized();
            }
            float lift_mag = dynamic_pressure * surf.area * surf.lift_coeff * sin_alpha * cos_alpha;
            Vec3 lift = lift_dir * lift_mag;
            
            total_force += drag + lift;
        }
        
        body.apply_force(total_force);
    }
};

/**
 * @brief Wind Field
 */
class WindField {
public:
    Vec3 base_velocity;     // Base wind velocity (m/s)
    float turbulence;       // Turbulence intensity (0-1)
    
    WindField() : base_velocity(Vec3::zero()), turbulence(0.0f) {}
    
    /**
     * @brief Get wind velocity at a position
     */
    Vec3 get_velocity(const Vec3& position, float time) const {
        Vec3 wind = base_velocity;
        
        if (turbulence > 0.0f) {
            // Simple turbulence model using sin waves
            float freq = 0.5f;
            wind.x += turbulence * 2.0f * std::sin(time * freq + position.x * 0.1f);
            wind.y += turbulence * 1.0f * std::sin(time * freq * 1.3f + position.y * 0.1f);
            wind.z += turbulence * 2.0f * std::sin(time * freq * 0.7f + position.z * 0.1f);
        }
        
        return wind;
    }
    
    /**
     * @brief Apply wind effect to body's perceived velocity
     */
    Vec3 get_relative_velocity(const RigidBody& body, float time) const {
        Vec3 wind = get_velocity(body.position, time);
        return body.linear_velocity - wind;  // Airspeed
    }
};

} // namespace aerodynamics
} // namespace basements

#endif // BASEMENTS_AERODYNAMICS_AERODYNAMIC_BODY_H

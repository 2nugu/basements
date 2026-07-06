/**
 * @file fracture.h
 * @brief Simple Procedural Fracture System
 * 
 * Implements simplified fracture mechanics by splitting rigid bodies
 * into smaller fragments when impact forces exceed material toughness.
 */

#ifndef BASEMENTS_PHYSICS_FRACTURE_H
#define BASEMENTS_PHYSICS_FRACTURE_H

#include <vector>
#include <random>
#include <cmath>
#include "basements/core/math/common.h"
#include "basements/physics/rigid_body.h"
#include "basements/physics/material_library.h"
#include "basements/physics/collision/shape_manager.h"

namespace basements {
namespace physics {

class FractureManager {
public:
    /**
     * @brief Checks if a body should fracture based on impact impulse.
     * 
     * @param body The rigid body to check.
     * @param impulse_magnitude The magnitude of the impact impulse.
     * @param contact_area Estimated contact area (m^2) to compute stress. Default approx 1cm^2 if unknown.
     * @return true if stress exceeds material toughness/strength.
     */
    static bool should_fracture(const RigidBody& body, float impulse_magnitude, float contact_area = 0.0001f) {
        if (body.inv_mass == 0.0f) return false; // Static bodies don't fracture in this sim

        Material mat = MaterialLibrary::get_by_id(body.material_id);
        if (mat.id == 0) return false; // Invalid material (or default)


        // Approximate Impact Force: F = Impulse / dt
        // Since we don't have exact dt here, we can use Impulse directly as a metric,
        // or assume a collision duration (e.g., 1ms).
        // Let's use Impulse directly proportional check or Stress approximation.
        // Stress = Force / Area = (Impulse / dt) / Area.
        
        float dt_collision = 0.001f; // 1ms duration assumption
        float force = impulse_magnitude / dt_collision;
        float stress = force / contact_area;

        // Use Ultimate Strength or Fracture Toughness? 
        // Fracture Toughness (K_IC) is for cracks. Ultimate Strength is for gross failure.
        // For simple check: use Ultimate Strength.
        
        return stress > mat.ultimate_strength;
    }

    /**
     * @brief Splits a rigid body into smaller fragments.
     * Currently implements 2x2x2 Grid Splitting (8 fragments).
     * 
     * @param original The body to split.
     * @param out_fragments Vector to append new bodies to.
     */
    static void split_body(const RigidBody& original, std::vector<RigidBody>& out_fragments) {
        Material mat = MaterialLibrary::get_by_id(original.material_id);
        if (mat.id == 0) return;


        // 1. Approximate Size from Mass and Density
        // V = Mass / Density
        float volume = original.mass / mat.density;
        float L = std::cbrt(volume); // Assume cube: L^3 = V
        
        // 2. Prepare Fragment Properties
        // 8 fragments -> Each has Volume/8, Mass/8, Size L/2
        float frag_mass = original.mass / 8.0f;
        float frag_size = L / 2.0f;
        float half_offset = L / 4.0f; // Offset from center to fragment center

        // Random generator for velocity perturbation
        static std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

        // 3. Generate 8 Fragments
        for (int x = -1; x <= 1; x += 2) {
            for (int y = -1; y <= 1; y += 2) {
                for (int z = -1; z <= 1; z += 2) {
                    RigidBody frag = original; // Copy properties (material etc)
                    
                    // Update Mass properties
                    frag.mass = frag_mass;
                    frag.inv_mass = 1.0f / frag_mass;
                    
                    // Update Inertia (Box formula: m/12 * (h^2 + d^2))
                    // Simplification: Identity scaled by mass/size relationship?
                    // Or just use 1/8 inertia of original? No, I scales with m*r^2.
                    // m -> 1/8. r -> 1/2. r^2 -> 1/4. I -> 1/32.
                    frag.inertia_tensor = original.inertia_tensor * (1.0f / 32.0f);
                    frag.inv_inertia_tensor = original.inv_inertia_tensor * 32.0f;

                    // Update Position (Local offset rotated to World)
                    Vec3 local_offset(x * half_offset, y * half_offset, z * half_offset);
                    frag.position = original.local_to_world(local_offset);
                    
                    // Update Velocity (Inherit + Expansion)
                    float explosion_speed = 1.0f; // m/s
                    Vec3 expansion_vel = original.orientation.rotate(local_offset).normalized() * explosion_speed;
                    frag.linear_velocity = original.linear_velocity + expansion_vel;
                    
                    // Add random rotation
                    frag.angular_velocity.x += dist(rng) * 5.0f;
                    frag.angular_velocity.y += dist(rng) * 5.0f;
                    frag.angular_velocity.z += dist(rng) * 5.0f;

                    // Handle Shape Resizing using ShapeManager
                    // New size is L/2. Half extents = L/4.
                    float half_size = L / 4.0f;
                    frag.shape = collision::ShapeManager::create_box(Vec3(half_size, half_size, half_size));

                    out_fragments.push_back(frag);
                }
            }
        }
    }
};

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FRACTURE_H

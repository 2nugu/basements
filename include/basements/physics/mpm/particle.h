#pragma once

#include "basements/core/math/vec3.h"
#include "basements/core/math/matrix3.h"

namespace basements {
namespace mpm {

    struct Particle {
        basements::math::Vec3 position;
        basements::math::Vec3 velocity;
        
        // Deformation Gradient (F)
        basements::math::Matrix3 F = basements::math::Matrix3::identity();
        
        // Affine momentum matrix (Cp) for APIC
        basements::math::Matrix3 C = basements::math::Matrix3::zero();
        
        float mass = 1.0f;
        float volume = 1.0f; 
        
        int material_id = 0; 
    };

} // namespace mpm
} // namespace basements

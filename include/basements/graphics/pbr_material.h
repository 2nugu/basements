#pragma once
#include "basements/core/math/vec3.h"
#include <string>

namespace basements::graphics {

/**
 * @brief Physically Based Rendering (PBR) Material structure.
 * 
 * Follows the Metallic-Roughness workflow.
 */
struct PBRMaterial {
    // Base Properties
    math::Vec3 albedo = {1.0f, 1.0f, 1.0f}; ///< Base color (linear RGB)
    float metallic = 0.0f;                 ///< 0.0 = Dielectric, 1.0 = Metal
    float roughness = 0.5f;                ///< 0.0 = Smooth, 1.0 = Rough
    float ao = 1.0f;                       ///< Ambient Occlusion factor
    math::Vec3 emissive = {0.0f, 0.0f, 0.0f}; ///< Emissive color (light emission)

    // Textures (Paths or IDs)
    std::string albedo_map;
    std::string normal_map;            ///< Tangent space normal map
    std::string metallic_roughness_map; ///< Packed (G=Roughness, B=Metallic)
    std::string ao_map;
    std::string emissive_map;

    // Parameters
    float normal_strength = 1.0f;
    bool double_sided = false;
    
    // Transparency
    enum class AlphaMode {
        OPAQUE_MODE,
        MASK,
        BLEND
    } alpha_mode = AlphaMode::OPAQUE_MODE;
    
    float alpha_cutoff = 0.5f; // For MASK mode
};

} // namespace basements::graphics

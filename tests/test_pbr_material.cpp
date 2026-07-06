#include <basements/graphics/pbr_material.h>
#include <iostream>
#include <cassert>
#include <cmath>

void test_defaults() {
    std::cout << "Testing PBRMaterial Defaults..." << std::endl;
    
    basements::graphics::PBRMaterial mat;
    
    // Check standard PBR defaults (Plastic-like white)
    // Albedo should be white (1,1,1)
    assert(std::fabs(mat.albedo.x - 1.0f) < 1e-5f);
    assert(std::fabs(mat.albedo.y - 1.0f) < 1e-5f);
    assert(std::fabs(mat.albedo.z - 1.0f) < 1e-5f);
    
    // Dielectric by default
    assert(mat.metallic == 0.0f);
    
    // Middle roughness
    assert(mat.roughness == 0.5f);
    
    // No emission
    assert(mat.emissive.length() == 0.0f);
    
    // Opaque
    assert(mat.alpha_mode == basements::graphics::PBRMaterial::AlphaMode::OPAQUE_MODE);
    
    std::cout << "Defaults Passed!" << std::endl;
}

void test_texture_paths() {
    std::cout << "Testing Texture Paths..." << std::endl;
    
    basements::graphics::PBRMaterial mat;
    mat.albedo_map = "assets/textures/wood_albedo.png";
    mat.normal_map = "assets/textures/wood_normal.png";
    
    assert(mat.albedo_map == "assets/textures/wood_albedo.png");
    assert(!mat.normal_map.empty());
    assert(mat.emissive_map.empty());
    
    std::cout << "Texture Paths Passed!" << std::endl;
}

int main() {
    std::cout << "===========================" << std::endl;
    std::cout << "   Test: PBR Material      " << std::endl;
    std::cout << "===========================" << std::endl;
    
    test_defaults();
    test_texture_paths();
    
    std::cout << "\nALL TESTS PASSED." << std::endl;
    return 0;
}

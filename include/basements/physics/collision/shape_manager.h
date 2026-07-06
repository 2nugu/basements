/**
 * @file shape_manager.h
 * @brief Simple manager for collision shapes
 * 
 * Provides a central registry for creating and accessing collision shapes
 * referenced by ShapeHandles.
 */

#ifndef BASEMENTS_COLLISION_SHAPE_MANAGER_H
#define BASEMENTS_COLLISION_SHAPE_MANAGER_H

#include <vector>
#include <mutex>
#include "basements/core/math/common.h"
#include "basements/core/types.h"
#include "basements/physics/collision/primitives.h"

namespace basements {
namespace collision {

using namespace basements::physics;

class ShapeManager {
private:
    // Simple storage for primitives
    // In a real engine, this might use a pool allocator or variant vector
    static std::vector<Box> boxes;
    static std::vector<Sphere> spheres;
    static std::vector<Capsule> capsules;
    
    // Mapping from global ID to type-specific index is needed if IDs are unified.
    // For simplicity, let's assume we decode Type from ID or store meta.
    // Let's use a simpler approach: ID = Index | (Type << 24)
    
    static constexpr uint32_t TYPE_MASK = 0xFF000000;
    static constexpr uint32_t INDEX_MASK = 0x00FFFFFF;
    static constexpr int TYPE_SHIFT = 24;

public:
    static ShapeHandle create_box(const Vec3& half_extents) {
        // static std::mutex mutex; // Uncomment if multithreading
        // std::lock_guard<std::mutex> lock(mutex);
        
        uint32_t index = static_cast<uint32_t>(boxes.size());
        boxes.push_back(Box(half_extents));
        
        uint32_t id = index | (static_cast<uint32_t>(ShapeType::BOX) << TYPE_SHIFT);
        return ShapeHandle(id);
    }

    static ShapeHandle create_sphere(float radius) {
        uint32_t index = static_cast<uint32_t>(spheres.size());
        spheres.push_back(Sphere(Vec3(0,0,0), radius));
        
        uint32_t id = index | (static_cast<uint32_t>(ShapeType::SPHERE) << TYPE_SHIFT);
        return ShapeHandle(id);
    }
    
    static const Box* get_box(ShapeHandle handle) {
        if (!handle.is_valid()) return nullptr;
        
        uint32_t type = (handle.id & TYPE_MASK) >> TYPE_SHIFT;
        uint32_t index = (handle.id & INDEX_MASK);
        
        if (static_cast<ShapeType>(type) != ShapeType::BOX) return nullptr;
        if (index >= boxes.size()) return nullptr;
        
        return &boxes[index];
    }
    
    // ... get_sphere, get_capsule ...
    
    static void clear() {
        boxes.clear();
        spheres.clear();
        capsules.clear();
    }
};

// Definition of static members
// In a header-only or inline usage, we need 'inline' or separate cpp.
// Since this is likely header-only for now, we use inline definitions if C++17.
inline std::vector<Box> ShapeManager::boxes;
inline std::vector<Sphere> ShapeManager::spheres;
inline std::vector<Capsule> ShapeManager::capsules;

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_SHAPE_MANAGER_H

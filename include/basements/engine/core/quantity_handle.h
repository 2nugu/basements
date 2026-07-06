#ifndef BASEMENTS_ENGINE_CORE_QUANTITY_HANDLE_H
#define BASEMENTS_ENGINE_CORE_QUANTITY_HANDLE_H

#include <cstdint>
#include <string>

namespace basements {
namespace engine {

/// Type-safe handle for quantities (O(1) access, no string lookups)
struct QuantityHandle {
    uint32_t index;
    
    #ifdef _DEBUG
    std::string debug_name;  // Only in debug builds
    #endif
    
    QuantityHandle() : index(UINT32_MAX) {}
    explicit QuantityHandle(uint32_t idx) : index(idx) {}
    
    bool is_valid() const { return index != UINT32_MAX; }
    
    bool operator==(const QuantityHandle& other) const {
        return index == other.index;
    }
    
    bool operator!=(const QuantityHandle& other) const {
        return index != other.index;
    }
};

/// Vector handle (3 scalar handles)
struct VectorHandle {
    QuantityHandle x, y, z;
    
    VectorHandle() = default;
    VectorHandle(QuantityHandle x_, QuantityHandle y_, QuantityHandle z_)
        : x(x_), y(y_), z(z_) {}
    
    bool is_valid() const {
        return x.is_valid() && y.is_valid() && z.is_valid();
    }
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_CORE_QUANTITY_HANDLE_H

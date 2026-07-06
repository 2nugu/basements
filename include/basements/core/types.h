#ifndef BASEMENTS_CORE_TYPES_H
#define BASEMENTS_CORE_TYPES_H

#include <cstdint>
#include <cstddef>

namespace basements {
namespace physics {

// ============================================================
// Type Aliases
// ============================================================

using BodyID = uint32_t;
using ShapeID = uint32_t;
using ConstraintID = uint32_t;

constexpr BodyID INVALID_BODY_ID = 0xFFFFFFFF;
constexpr ShapeID INVALID_SHAPE_ID = 0xFFFFFFFF;
constexpr ConstraintID INVALID_CONSTRAINT_ID = 0xFFFFFFFF;

// ============================================================
// Handle Types (Type-safe IDs)
// ============================================================

template<typename T, typename IDType>
struct Handle {
    IDType id;
    
    Handle() : id(static_cast<IDType>(-1)) {}
    explicit Handle(IDType id_) : id(id_) {}
    
    bool is_valid() const { return id != static_cast<IDType>(-1); }
    void invalidate() { id = static_cast<IDType>(-1); }
    
    bool operator==(const Handle& other) const { return id == other.id; }
    bool operator!=(const Handle& other) const { return id != other.id; }
    bool operator<(const Handle& other) const { return id < other.id; }
};

struct BodyTag {};
struct ShapeTag {};
struct ConstraintTag {};

using BodyHandle = Handle<BodyTag, BodyID>;
using ShapeHandle = Handle<ShapeTag, ShapeID>;
using ConstraintHandle = Handle<ConstraintTag, ConstraintID>;

// ============================================================
// Configuration Constants
// ============================================================

namespace config {
    // Physics constants
    constexpr float DEFAULT_GRAVITY = -9.81f;  // m/s²
    constexpr float DEFAULT_TIME_STEP = 1.0f / 60.0f;  // 60 FPS
    
    // Solver parameters
    constexpr int DEFAULT_SOLVER_ITERATIONS = 10;
    constexpr float DEFAULT_BAUMGARTE_FACTOR = 0.2f;
    constexpr float DEFAULT_RESTITUTION = 0.5f;  // Bounciness
    constexpr float DEFAULT_FRICTION = 0.5f;
    
    // Sleep/wake thresholds
    constexpr float SLEEP_LINEAR_THRESHOLD = 0.01f;  // m/s
    constexpr float SLEEP_ANGULAR_THRESHOLD = 0.01f;  // rad/s
    constexpr float SLEEP_TIME_THRESHOLD = 0.5f;  // seconds
    
    // Memory
    constexpr size_t DEFAULT_BODY_CAPACITY = 10000;
    constexpr size_t DEFAULT_CONTACT_CAPACITY = 50000;
    
    // Collision detection
    constexpr float DEFAULT_CELL_SIZE = 2.0f;  // Spatial hash grid cell size
    constexpr int MAX_CONTACTS_PER_PAIR = 4;  // Contact manifold size
}

// ============================================================
// Enumerations
// ============================================================

enum class BodyType : uint8_t {
    STATIC = 0,   // Infinite mass, never moves
    DYNAMIC = 1,  // Finite mass, affected by forces
    KINEMATIC = 2 // Infinite mass, but can be moved programmatically
};

enum class ShapeType : uint8_t {
    SPHERE = 0,
    BOX = 1,
    CAPSULE = 2,
    CYLINDER = 3,
    CONE = 4, // [NEW]
    TORUS = 5, // [NEW]
    PLANE = 6, // [NEW]
    CONVEX_HULL = 7, 
    MESH = 8  // Concave mesh
};

// ============================================================
// GPU Compatibility Macros
// ============================================================

#ifdef __CUDACC__
    #define HOST_DEVICE __host__ __device__
    #define DEVICE __device__
    #define HOST __host__
    #define GLOBAL __global__
#else
    #define HOST_DEVICE
    #define DEVICE
    #define HOST
    #define GLOBAL
#endif

} // namespace physics
} // namespace basements

#endif // BASEMENTS_CORE_TYPES_H

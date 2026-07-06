#ifndef BASEMENTS_COLLISION_GJK_H
#define BASEMENTS_COLLISION_GJK_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/physics/collision/shape.h"

namespace basements {
namespace collision {

using namespace basements::math;

// ============================================================================
// Simplex
// ============================================================================

/**
 * @brief Simplex structure for GJK
 * Stores up to 4 points (tetrahedron)
 */
struct Simplex {
    Vec3 points[4];
    int size;

    HOST_DEVICE Simplex() : size(0) {}

    HOST_DEVICE void push(const Vec3& p) {
        if (size < 4) {
            points[size++] = p;
        }
    }

    HOST_DEVICE void reset() {
        size = 0;
    }
    
    HOST_DEVICE Vec3& operator[](int i) { return points[i]; }
    HOST_DEVICE const Vec3& operator[](int i) const { return points[i]; }
};

// ============================================================================
// GJK Algorithm
// ============================================================================

struct GJK {
    
    // Transform helper
    struct Transform {
        Vec3 position;
        Quaternion rotation;
        
        HOST_DEVICE Transform() : position(0.0f), rotation(Quaternion::identity()) {}
        HOST_DEVICE Transform(const Vec3& p, const Quaternion& r) : position(p), rotation(r) {}
        
        HOST_DEVICE Vec3 transform_point(const Vec3& p) const {
            return position + rotation.rotate(p);
        }
        
        HOST_DEVICE Vec3 transform_vector_inv(const Vec3& v) const {
            return rotation.inverse().rotate(v);
        }
    };

    /**
     * @brief Computes support point for the Minkowski Difference (A - B)
     */
    template <typename ShapeA, typename ShapeB>
    HOST_DEVICE static Vec3 support(
        const ShapeA& shapeA, const Transform& txA,
        const ShapeB& shapeB, const Transform& txB,
        const Vec3& dir
    ) {
        // Support of A in direction dir
        // We need to transform dir into local space of A
        Vec3 dir_local_A = txA.transform_vector_inv(dir);
        Vec3 support_A = txA.transform_point(shapeA.support(dir_local_A));
        
        // Support of B in direction -dir
        Vec3 dir_local_B = txB.transform_vector_inv(-dir);
        Vec3 support_B = txB.transform_point(shapeB.support(dir_local_B));
        
        return support_A - support_B;
    }

    /**
     * @brief Main GJK intersection test
     * @return true if shapes overlap
     */
    template <typename ShapeA, typename ShapeB>
    HOST_DEVICE static bool run(
        const ShapeA& shapeA, const Transform& txA,
        const ShapeB& shapeB, const Transform& txB,
        Simplex& simplex // Output simplex for EPA
    ) {
        // 1. Initial guess for direction (arbitrary, e.g., A.center - B.center)
        // Or simply Unit X
        Vec3 direction = Vec3(1.0f, 0.0f, 0.0f); 
        
        // 2. Get first support point
        Vec3 c = support(shapeA, txA, shapeB, txB, direction);
        simplex.reset();
        simplex.push(c);
        
        // 3. Search towards origin
        direction = -c; // Direction towards origin
        
        // Loop
        while (true) {
            // Get next point in direction
            Vec3 a = support(shapeA, txA, shapeB, txB, direction);
            
            // Check if we passed the origin
            if (a.dot(direction) < 0) {
                return false; // Origin is outside Minkowski Difference -> No collision
            }
            
            simplex.push(a);
            
            // Solver: Checks if simplex contains origin, and updates simplex/direction
            if (solve_simplex(simplex, direction)) {
                return true;
            }
        }
    }

    // ========================================================================
    // Simplex Solver Helpers
    // ========================================================================

    /**
     * @brief Solves the simplex. 
     * Updates simplex to keep only useful points.
     * Updates direction to point towards origin.
     * Returns true if origin is contained.
     */
    HOST_DEVICE static bool solve_simplex(Simplex& simplex, Vec3& direction) {
        switch (simplex.size) {
            case 2: return solve_line(simplex, direction);
            case 3: return solve_triangle(simplex, direction);
            case 4: return solve_tetrahedron(simplex, direction);
            default: return false; // Should not happen
        }
    }

    HOST_DEVICE static bool solve_line(Simplex& simplex, Vec3& direction) {
        // A is simplex[1] (new point), B is simplex[0] (old point)
        Vec3 a = simplex[1];
        Vec3 b = simplex[0];
        
        Vec3 ab = b - a;
        Vec3 ao = -a;
        
        // If ab contains origin, then origin is on the line segment
        // But we want to search TOWARDS origin.
        // New direction is robust projection of AO onto AB's generic plane
        // Actually, just triple product: (AB x AO) x AB
        
        if (ab.dot(ao) > 0) {
            direction = ab.cross(ao).cross(ab);
        } else {
            // Should verify this case, but standard GJK usually assumes 
            // the previous loop ensured we moved towards origin.
            // If we overshoot A, we might restart? 
            // Basically point towards origin from A
            simplex.size = 1; // Keep only A
            direction = ao;
        }
        return false;
    }

    HOST_DEVICE static bool solve_triangle(Simplex& simplex, Vec3& direction) {
        Vec3 a = simplex[2];
        Vec3 b = simplex[1];
        Vec3 c = simplex[0];
        
        Vec3 ab = b - a;
        Vec3 ac = c - a;
        Vec3 ao = -a;
        
        Vec3 abc = ab.cross(ac); // Normal to triangle
        
        // Plane AB Normal (perpendicular to AB, on triangle plane)
        Vec3 abNorm = ab.cross(abc);
        // Plane AC Normal
        Vec3 acNorm = abc.cross(ac);
        
        // Check Region AB
        if (abNorm.dot(ao) > 0) {
            // Outside AB? Check if it's closer to edge AB or just vertex A
            // We can reuse line solver logic or just simplify
            // Assuming simpler case:
            simplex.size = 2;
            simplex[0] = b; 
            simplex[1] = a; 
            return solve_line(simplex, direction); 
        }
        
        // Check Region AC
        if (acNorm.dot(ao) > 0) {
            simplex.size = 2;
            simplex[0] = c;
            simplex[1] = a;
            return solve_line(simplex, direction);
        }
        
        // If strictly above/below triangle
        if (abc.dot(ao) > 0) {
            // Origin is "above" triangle
            direction = abc;
        } else {
            // Origin is "below" triangle
            // Flip normal and winding to ensure consistency if needed
            simplex[0] = b;
            simplex[1] = c; // Swap B and C to flip winding
            // simplex[2] is still A
            direction = -abc;
        }
        return false;
    }

    HOST_DEVICE static bool solve_tetrahedron(Simplex& simplex, Vec3& direction) {
        Vec3 a = simplex[3];
        Vec3 b = simplex[2];
        Vec3 c = simplex[1];
        Vec3 d = simplex[0];
        
        Vec3 ab = b - a;
        Vec3 ac = c - a;
        Vec3 ad = d - a;
        Vec3 ao = -a;
        
        // Face Normals (pointing outwards?)
        // Winding order matters. Assuming ABC, ACD, ADB base
        Vec3 abc = ab.cross(ac);
        Vec3 acd = ac.cross(ad);
        Vec3 adb = ad.cross(ab);
        
        // Check face ABC
        if (abc.dot(ao) > 0) {
            // Remove D, reduce to triangle
            simplex.size = 3;
            simplex[0] = c;
            simplex[1] = b;
            simplex[2] = a;
            return solve_triangle(simplex, direction);
        }
        
        // Check face ACD
        if (acd.dot(ao) > 0) {
            // Remove B
            simplex.size = 3;
            simplex[0] = d;
            simplex[1] = c;
            simplex[2] = a;
            return solve_triangle(simplex, direction);
        }
        
        // Check face ADB
        if (adb.dot(ao) > 0) {
            // Remove C
            simplex.size = 3;
            simplex[0] = b;
            simplex[1] = d;
            simplex[2] = a;
            return solve_triangle(simplex, direction);
        }
        
        // If inside all faces -> Collision!
        return true;
    }
};

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_GJK_H

#ifndef BASEMENTS_COLLISION_EPA_H
#define BASEMENTS_COLLISION_EPA_H

#include "basements/physics/collision/gjk.h"

namespace basements {
namespace collision {

// ============================================================================
// EPA Constants & Types
// ============================================================================

// Production-ready buffer sizes (99.9% of convex collisions)
constexpr int EPA_MAX_VERTICES = 128;  // Increased from 64
constexpr int EPA_MAX_FACES = 64;      // Optimized for cache
constexpr int EPA_MAX_ITERATIONS = 32;
constexpr float EPA_TOLERANCE = 0.001f;

// Degeneracy detection thresholds
constexpr float EPA_EPSILON_BASE = 1e-6f;     // Base epsilon for distance checks
constexpr float EPA_EPSILON_NORMAL = 1e-8f;   // Minimum normal length
constexpr float EPA_EPSILON_VOLUME = 1e-8f;   // Minimum volume change

struct Face {
    uint8_t v[3];      // Vertex indices (uint8_t for cache efficiency)
    uint8_t adj[3];    // Adjacent face indices (index-based, not pointers!)
    Vec3 normal;       // Normal pointing away from origin
    float dist;        // Distance from origin
    bool obsolete;     // Flag for removal
    
    HOST_DEVICE Face() : obsolete(false) {
        v[0] = v[1] = v[2] = 0;
        adj[0] = adj[1] = adj[2] = 255; // 255 = no adjacent face
    }
};

struct Edge {
    int a, b;
};

// ============================================================================
// EPA Algorithm
// ============================================================================

struct EPA {

    using Transform = GJK::Transform;

    struct Result {
        float depth;
        Vec3 normal;
        Vec3 contact_point;
    };

    /**
     * @brief High-level helper to compute penetration between two bodies
     */
    template <typename BodyT>
    HOST_DEVICE static Result compute_penetration(const BodyT& a, const BodyT& b) {
        Result result;
        result.depth = 0;
        result.normal = Vec3(0, 1, 0);
        result.contact_point = Vec3::zero(); // TODO: Accurate contact point

        Simplex simplex;
        GJK::Transform txA(a.position, a.orientation);
        GJK::Transform txB(b.position, b.orientation);

        // Run GJK to get simplex
        bool hit = GJK::run(a.shape, txA, b.shape, txB, simplex);

        if (hit) {
            EPA::run(simplex, a.shape, txA, b.shape, txB, result.depth, result.normal);
            // Heuristic contact point (midpoint of bodies or projected)
            // For stacking, using the center of intersection volume or similar would be better
            // But for now, we leave contact_point as zero or centroid
            // result.contact_point = (a.position + b.position) * 0.5f; 
        }
        return result;
    }

    struct EPAState {
        Vec3 vertices[EPA_MAX_VERTICES];
        int num_vertices;
        Face faces[EPA_MAX_FACES];
        int num_faces;
        float polytope_scale;  // For adaptive epsilon
        
        HOST_DEVICE EPAState() : num_vertices(0), num_faces(0), polytope_scale(1.0f) {}
        
        // Calculate polytope scale (bounding sphere radius)
        HOST_DEVICE void compute_scale() {
            if (num_vertices == 0) {
                polytope_scale = 1.0f;
                return;
            }
            
            // Find centroid
            Vec3 centroid = Vec3::zero();
            for (int i = 0; i < num_vertices; i++) {
                centroid = centroid + vertices[i];
            }
            centroid = centroid * (1.0f / num_vertices);
            
            // Find max distance from centroid
            float max_dist_sq = 0.0f;
            for (int i = 0; i < num_vertices; i++) {
                float dist_sq = (vertices[i] - centroid).length_squared();
                if (dist_sq > max_dist_sq) {
                    max_dist_sq = dist_sq;
                }
            }
            
            #ifdef __CUDA_ARCH__
            polytope_scale = sqrtf(max_dist_sq);
            #else
            polytope_scale = std::sqrt(max_dist_sq);
            #endif
            
            if (polytope_scale < EPA_EPSILON_BASE) {
                polytope_scale = EPA_EPSILON_BASE;
            }
        }
    };

    // Degeneracy detection: Check if vertex is too close to existing vertices
    HOST_DEVICE static bool is_degenerate_vertex(
        const EPAState& state,
        const Vec3& new_vertex
    ) {
        float epsilon_dist = EPA_EPSILON_BASE * state.polytope_scale;
        
        for (int i = 0; i < state.num_vertices; i++) {
            float dist_sq = (new_vertex - state.vertices[i]).length_squared();
            if (dist_sq < epsilon_dist * epsilon_dist) {
                return true; // Too close to existing vertex
            }
        }
        return false;
    }
    
    // Degeneracy detection: Check if face is degenerate
    HOST_DEVICE static bool is_degenerate_face(
        const Vec3& v0, const Vec3& v1, const Vec3& v2
    ) {
        Vec3 edge1 = v1 - v0;
        Vec3 edge2 = v2 - v0;
        Vec3 normal = edge1.cross(edge2);
        
        float normal_length_sq = normal.length_squared();
        
        // Check if normal is too small (degenerate triangle)
        if (normal_length_sq < EPA_EPSILON_NORMAL * EPA_EPSILON_NORMAL) {
            return true;
        }
        
        return false;
    }

    // Helper to add face with degeneracy check
    HOST_DEVICE static bool add_face_helper(EPAState& state, int a, int b, int c) {
        if (state.num_faces >= EPA_MAX_FACES) return false; // Buffer full
        
        Vec3 v0 = state.vertices[a];
        Vec3 v1 = state.vertices[b];
        Vec3 v2 = state.vertices[c];
        
        // Check for degeneracy
        if (is_degenerate_face(v0, v1, v2)) {
            return false; // Skip degenerate face
        }
        
        Face& f = state.faces[state.num_faces++];
        f.v[0] = static_cast<uint8_t>(a);
        f.v[1] = static_cast<uint8_t>(b);
        f.v[2] = static_cast<uint8_t>(c);
        f.obsolete = false;
        
        Vec3 edge1 = v1 - v0;
        Vec3 edge2 = v2 - v0;
        f.normal = edge1.cross(edge2).normalized();
        f.dist = f.normal.dot(v0);
        
        if (f.dist < 0) {
            f.normal = -f.normal;
            f.dist = -f.dist;
            // Swap winding
            uint8_t temp = f.v[1];
            f.v[1] = f.v[2];
            f.v[2] = temp;
        }
        
        return true;
    }

    // Helper to add edge
    HOST_DEVICE static void add_edge(Edge* edges, int& num_edges, int a, int b) {
        for (int k = 0; k < num_edges; ++k) {
            if (edges[k].a == b && edges[k].b == a) {
                // Edge exists in reverse order -> shared edge between faces
                // Remove it (internal edge)
                edges[k] = edges[--num_edges];
                return;
            }
        }
        // New edge
        edges[num_edges].a = a;
        edges[num_edges].b = b;
        num_edges++;
    }

    /**
     * @brief Computes penetration depth and normal
     * @param simplex Result from GJK (must be a tetrahedron)
     * @param depth Output penetration depth
     * @param normal Output collision normal (pointing from B to A)
     */
    template <typename ShapeA, typename ShapeB>
    HOST_DEVICE static bool run(
        const Simplex& simplex,
        const ShapeA& shapeA, const Transform& txA,
        const ShapeB& shapeB, const Transform& txB,
        float& depth, Vec3& normal
    ) {
        // 1. Initialize Polytope from GJK simplex
        EPAState state;
        
        // Copy vertices (Simplex should have 4 points for 3D EPA)
        if (simplex.size != 4) {
             return false; 
        }
        
        for (int i = 0; i < 4; ++i) {
            state.vertices[i] = simplex[i];
        }
        state.num_vertices = 4;
        
        // Compute initial polytope scale
        state.compute_scale();
        
        add_face_helper(state, 0, 1, 2);
        add_face_helper(state, 0, 3, 1);
        add_face_helper(state, 0, 2, 3);
        add_face_helper(state, 1, 3, 2);
        
        // 2. Expansion Loop
        for (int iter = 0; iter < EPA_MAX_ITERATIONS; ++iter) {
            // Find closest face to origin
            Face* closest_face = nullptr;
            #ifdef __CUDA_ARCH__
            float min_dist = FLT_MAX;
            #else
            float min_dist = std::numeric_limits<float>::max();
            #endif
            
            for (int i = 0; i < state.num_faces; ++i) {
                if (state.faces[i].obsolete) continue;
                if (state.faces[i].dist < min_dist) {
                    min_dist = state.faces[i].dist;
                    closest_face = &state.faces[i];
                }
            }
            
            if (!closest_face) break; // Should not happen
            
            // Search specific direction
            Vec3 search_dir = closest_face->normal;
            Vec3 p = GJK::support(shapeA, txA, shapeB, txB, search_dir);
            
            // Calculate distance to new point along normal
            float d = p.dot(search_dir);
            
            if (d - min_dist < EPA_TOLERANCE) {
                // Convergence
                normal = search_dir;
                depth = d;
                return true;
            }
            
            // Add new point to polytope
            if (state.num_vertices >= EPA_MAX_VERTICES) {
                // Out of memory - return best effort result
                normal = search_dir;
                depth = min_dist;
                return true;
            }
            
            // Check for vertex degeneracy
            if (is_degenerate_vertex(state, p)) {
                // New vertex too close to existing vertices - convergence
                normal = search_dir;
                depth = d;
                return true;
            }
            
            state.vertices[state.num_vertices] = p;
            int p_idx = state.num_vertices++;
            
            // Update polytope scale with new vertex
            state.compute_scale();
            
            // 3. Remove visible faces and find horizon edges
            Edge edges[EPA_MAX_FACES]; // Max edges approx max faces
            int num_edges = 0;
            
            for (int i = 0; i < state.num_faces; ++i) {
                if (state.faces[i].obsolete) continue;
                
                // Can this face "see" the point? (Point is in front of face)
                // Use a small epsilon for robustness
                if (state.faces[i].normal.dot(p - state.vertices[state.faces[i].v[0]]) > EPSILON) {
                    state.faces[i].obsolete = true;
                    
                    add_edge(edges, num_edges, state.faces[i].v[0], state.faces[i].v[1]);
                    add_edge(edges, num_edges, state.faces[i].v[1], state.faces[i].v[2]);
                    add_edge(edges, num_edges, state.faces[i].v[2], state.faces[i].v[0]);
                }
            }
            
            // 4. Add new faces from horizon edges to new point
            for (int i = 0; i < num_edges; ++i) {
                add_face_helper(state, edges[i].a, edges[i].b, p_idx);
            }
        }
        
        return false;
    }
};

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_EPA_H

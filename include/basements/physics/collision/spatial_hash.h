#ifndef BASEMENTS_COLLISION_SPATIAL_HASH_H
#define BASEMENTS_COLLISION_SPATIAL_HASH_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/collision/aabb.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace basements {
namespace collision {

// Configuration for Spatial Hash
constexpr int HASH_TABLE_SIZE = 4096; // Adjust based on scene scale
constexpr float CELL_SIZE = 2.0f;     // Should be larger than the largest object

struct BroadphasePair {
    int body_a;
    int body_b;

    bool operator==(const BroadphasePair& other) const {
        return (body_a == other.body_a && body_b == other.body_b) ||
               (body_a == other.body_b && body_b == other.body_a);
    }
};

class SpatialHash {
public:
    // Fixed size hash table for simplicity and performance
    // In a real GPU implementation, this would be flat arrays (Cell Start, Cell Count)
    // For CPU reference, we use vector of vectors or a flat linked list simulation.
    // Let's use a "Dense" approach: Hash -> Head Index, and a Node Pool.
    
    struct Node {
        int body_index;
        int next_node_index;
    };

    std::vector<int> buckets;       // Maps Hash -> First Node Index (Head)
    std::vector<Node> node_pool;    // Pre-allocated nodes
    int pool_usage;

    SpatialHash() {
        buckets.resize(HASH_TABLE_SIZE, -1);
        node_pool.resize(10000); // Initial capacity
        pool_usage = 0;
    }

    void clear() {
        std::fill(buckets.begin(), buckets.end(), -1);
        pool_usage = 0;
    }

    // Spatial Hash Function
    // (x, y, z) are Grid Coordinates, not World Coordinates
    HOST_DEVICE unsigned int hash(int x, int y, int z) const {
        const unsigned int p1 = 73856093;
        const unsigned int p2 = 19349663;
        const unsigned int p3 = 83492791;
        
        // Handle negative coordinates by offsetting or bit casting, 
        // but simple modular arithmetic with large primes handles it okay for local areas.
        // Better: XOR with large number if negative? 
        // Let's keep it simple: cast to unsigned directly handles it by wrapping (acceptable for hash)
        
        unsigned int n = (static_cast<unsigned int>(x) * p1) ^ 
                         (static_cast<unsigned int>(y) * p2) ^ 
                         (static_cast<unsigned int>(z) * p3);
        
        return n % HASH_TABLE_SIZE;
    }

    // Insert a body into the grid
    void insert(int body_index, const AABB& aabb) {
        int min_x = static_cast<int>(std::floor(aabb.min.x / CELL_SIZE));
        int min_y = static_cast<int>(std::floor(aabb.min.y / CELL_SIZE));
        int min_z = static_cast<int>(std::floor(aabb.min.z / CELL_SIZE));

        int max_x = static_cast<int>(std::floor(aabb.max.x / CELL_SIZE));
        int max_y = static_cast<int>(std::floor(aabb.max.y / CELL_SIZE));
        int max_z = static_cast<int>(std::floor(aabb.max.z / CELL_SIZE));

        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                for (int z = min_z; z <= max_z; ++z) {
                    unsigned int h = hash(x, y, z);
                    add_to_bucket(h, body_index);
                }
            }
        }
    }

    // Find all potential pairs
    // Helper to avoiding duplicates: only check pairs if body_a < body_b
    // or use a set (slow).
    // Better strategy for "One-Pass":
    // For each body, compute its cells. For each cell, check neighbors ALREADY in the bucket.
    // If we clear and rebuild every frame, this works well.
    void query_pairs(std::vector<BroadphasePair>& pairs) {
        // Naive iteration over buckets
        for (int i = 0; i < HASH_TABLE_SIZE; ++i) {
            int node_idx = buckets[i];
            
            // Traverse list for this bucket
            while (node_idx != -1) {
                int body_a = node_pool[node_idx].body_index;
                
                // Compare with all SUBSEQUENT nodes in this bucket
                int other_node_idx = node_pool[node_idx].next_node_index;
                while (other_node_idx != -1) {
                    int body_b = node_pool[other_node_idx].body_index;
                    
                    // Avoid self-collision and ensure unique order (if same pair in multiple buckets)
                    // Note: This naive approach produces duplicates if pair shares multiple cells.
                    // We need a way to filter duplicates. 
                    // A common trick is to insert and verify, or sort-unique later.
                    // For now, let's just add all and user filters, OR
                    // check if we already added? No that's O(N).
                    // We'll perform the detailed overlap check here or just emit.
                    // Optimization: Only emit if body_a < body_b.
                    
                    if (body_a < body_b) {
                        pairs.push_back({body_a, body_b});
                    } else if (body_b < body_a) {
                        pairs.push_back({body_b, body_a});
                    }

                    other_node_idx = node_pool[other_node_idx].next_node_index;
                }
                node_idx = node_pool[node_idx].next_node_index;
            }
        }
        
        // Remove duplicates
        if (!pairs.empty()) {
            std::sort(pairs.begin(), pairs.end(), [](const BroadphasePair& a, const BroadphasePair& b) {
                if (a.body_a != b.body_a) return a.body_a < b.body_a;
                return a.body_b < b.body_b;
            });
            pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
        }
    }

private:
    void add_to_bucket(unsigned int bucket_index, int body_index) {
        if (pool_usage >= node_pool.size()) {
            node_pool.resize(node_pool.size() * 2);
        }

        int node_idx = pool_usage++;
        node_pool[node_idx].body_index = body_index;
        node_pool[node_idx].next_node_index = buckets[bucket_index];
        buckets[bucket_index] = node_idx;
    }
};

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_SPATIAL_HASH_H

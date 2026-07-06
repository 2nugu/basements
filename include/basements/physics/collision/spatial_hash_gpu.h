#ifndef BASEMENTS_COLLISION_SPATIAL_HASH_GPU_H
#define BASEMENTS_COLLISION_SPATIAL_HASH_GPU_H

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/physics/collision/aabb.h"

namespace basements {
namespace collision {

// ============================================================================
// GPU Spatial Hash Structures
// ============================================================================

struct Node {
    int body_index;
    int next_node_index; // Index in node_pool, -1 if end of list
};

// POD struct to pass to Kernels
struct SpatialHashContext {
    int* buckets;           // Hash table [HASH_SIZE] -> Head Node Index
    Node* node_pool;        // Pre-allocated nodes [MAX_NODES]
    int* pool_counter;      // Atomic counter for node allocation
    int hash_size;
    int max_nodes;
    float cell_size;

    HOST_DEVICE SpatialHashContext() 
        : buckets(nullptr), node_pool(nullptr), pool_counter(nullptr), 
          hash_size(0), max_nodes(0), cell_size(2.0f) {}

    // Hash Function (Same logic as CPU)
    HOST_DEVICE unsigned int hash(int x, int y, int z) const {
        // Optimized for GPU bits
        const unsigned int p1 = 73856093;
        const unsigned int p2 = 19349663;
        const unsigned int p3 = 83492791;
        
        unsigned int n = (static_cast<unsigned int>(x) * p1) ^ 
                         (static_cast<unsigned int>(y) * p2) ^ 
                         (static_cast<unsigned int>(z) * p3);
        
        return n % hash_size;
    }

    // Insert body into a bucket
    HOST_DEVICE void add_to_bucket(int x, int y, int z, int body_index) {
        #ifdef __CUDA_ARCH__
        if (!buckets || !node_pool || !pool_counter) return;

        unsigned int h = hash(x, y, z);

        // 1. Allocate a node (Atomic Add)
        int node_idx = atomicAdd(pool_counter, 1);
        
        // Check overflow
        if (node_idx >= max_nodes) {
            // Pool full!
            return;
        }

        // 2. Setup Node
        node_pool[node_idx].body_index = body_index;

        // 3. Link into bucket (Atomic Exch)
        int old_head = atomicExch(&buckets[h], node_idx);
        node_pool[node_idx].next_node_index = old_head;
        #endif
    }
};

// ============================================================================
// Host-Side Manager Class
// ============================================================================

class SpatialHashGPU {
public:
    int* d_buckets;
    Node* d_node_pool;
    int* d_pool_counter;
    
    int hash_size;
    int max_nodes;
    float cell_size;

    SpatialHashGPU(int size = 4096, int max_n = 10000, float cell_s = 2.0f) 
        : hash_size(size), max_nodes(max_n), cell_size(cell_s) 
    {
        cudaMalloc(&d_buckets, hash_size * sizeof(int));
        cudaMalloc(&d_node_pool, max_nodes * sizeof(Node));
        cudaMalloc(&d_pool_counter, sizeof(int));
    }

    ~SpatialHashGPU() {
        if (d_buckets) cudaFree(d_buckets);
        if (d_node_pool) cudaFree(d_node_pool);
        if (d_pool_counter) cudaFree(d_pool_counter);
    }

    void clear() {
        cudaMemset(d_buckets, -1, hash_size * sizeof(int));
        cudaMemset(d_pool_counter, 0, sizeof(int));
    }

    // Returns a context object to pass to kernels
    SpatialHashContext get_context() const {
        SpatialHashContext ctx;
        ctx.buckets = d_buckets;
        ctx.node_pool = d_node_pool;
        ctx.pool_counter = d_pool_counter;
        ctx.hash_size = hash_size;
        ctx.max_nodes = max_nodes;
        ctx.cell_size = cell_size;
        return ctx;
    }
};

// ============================================================================
// Kernels (Defined here for simplicity, launched by user .cu)
// ============================================================================

__global__ void insert_aabb_kernel(SpatialHashContext ctx, const AABB* d_aabbs, int num_bodies) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_bodies) return;

    AABB aabb = d_aabbs[i];
    
    int min_x = static_cast<int>(floorf(aabb.min.x / ctx.cell_size));
    int min_y = static_cast<int>(floorf(aabb.min.y / ctx.cell_size));
    int min_z = static_cast<int>(floorf(aabb.min.z / ctx.cell_size));

    int max_x = static_cast<int>(floorf(aabb.max.x / ctx.cell_size));
    int max_y = static_cast<int>(floorf(aabb.max.y / ctx.cell_size));
    int max_z = static_cast<int>(floorf(aabb.max.z / ctx.cell_size));

    // Iterate over cells covered by AABB
    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            for (int z = min_z; z <= max_z; ++z) {
                ctx.add_to_bucket(x, y, z, i);
            }
        }
    }
}

} // namespace collision
} // namespace basements

#endif // BASEMENTS_COLLISION_SPATIAL_HASH_GPU_H

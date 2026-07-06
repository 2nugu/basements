#include "basements/core/math/basements.h"
#include "basements/physics/collision/spatial_hash_gpu.h"
#include <iostream>
#include <vector>
#include <cuda_runtime.h>

using namespace basements::collision;
using namespace basements::math;

// Kernel to check results on GPU
__global__ void check_buckets_kernel(SpatialHashContext ctx, int* d_results) {
    // Check bucket for (0,0,0) - Should contain Body 0
    int h0 = ctx.hash(0, 0, 0);
    int node_idx = ctx.buckets[h0];
    
    // Simple check: is there at least one node?
    if (node_idx != -1) {
        d_results[0] = 1;
        // Check body index
        if (ctx.node_pool[node_idx].body_index == 0) {
            d_results[1] = 1;
        }
    }

    // Check bucket for (5,0,0) - Should be empty (if cell size is 2, 5 is index 2)
    // Body 1 is at (10,0,0) -> index 5
    int h1 = ctx.hash(5, 0, 0); 
    int node_idx1 = ctx.buckets[h1];
    
    if (node_idx1 != -1) {
        if (ctx.node_pool[node_idx1].body_index == 1) {
            d_results[2] = 1;
        }
    }
}

int main() {
    std::cout << "[Test] GPU Broadphase (Spatial Hash)" << std::endl;

    // 1. Setup Hash on Host
    SpatialHashGPU spatial_hash(1024, 1000, 2.0f);
    spatial_hash.clear();

    // 2. Create AABBs
    std::vector<AABB> h_aabbs;
    h_aabbs.push_back(AABB(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f)));   // Body 0: Cell (0,0,0)
    h_aabbs.push_back(AABB(Vec3(10.0f, 0.0f, 0.0f), Vec3(11.0f, 1.0f, 1.0f))); // Body 1: Cell (5,0,0)

    AABB* d_aabbs;
    cudaMalloc(&d_aabbs, h_aabbs.size() * sizeof(AABB));
    cudaMemcpy(d_aabbs, h_aabbs.data(), h_aabbs.size() * sizeof(AABB), cudaMemcpyHostToDevice);

    // 3. Launch Insert Kernel
    SpatialHashContext ctx = spatial_hash.get_context();
    insert_aabb_kernel<<<1, 32>>>(ctx, d_aabbs, h_aabbs.size());
    cudaDeviceSynchronize();

    // 4. Verify Results
    int* d_results;
    int* h_results = (int*)calloc(10, sizeof(int));
    cudaMalloc(&d_results, 10 * sizeof(int));
    cudaMemset(d_results, 0, 10 * sizeof(int));

    check_buckets_kernel<<<1, 1>>>(ctx, d_results);
    cudaDeviceSynchronize();

    cudaMemcpy(h_results, d_results, 10 * sizeof(int), cudaMemcpyDeviceToHost);

    bool success = true;
    if (h_results[0] == 1) std::cout << "SUCCESS: Bucket (0,0,0) populated." << std::endl;
    else { std::cout << "FAILURE: Bucket (0,0,0) empty." << std::endl; success = false; }

    if (h_results[1] == 1) std::cout << "SUCCESS: Bucket (0,0,0) contains Body 0." << std::endl;
    else { std::cout << "FAILURE: Bucket (0,0,0) wrong body." << std::endl; success = false; }

    if (h_results[2] == 1) std::cout << "SUCCESS: Bucket (5,0,0) contains Body 1." << std::endl;
    else { std::cout << "FAILURE: Bucket (5,0,0) wrong body." << std::endl; success = false; }

    cudaFree(d_aabbs);
    cudaFree(d_results);
    free(h_results);

    return success ? 0 : 1;
}

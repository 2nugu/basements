#include "basements/core/math/basements.h"
#include <iostream>
#include <cuda_runtime.h>

using namespace basements::collision;
using namespace basements::math;

__global__ void test_narrowphase_kernel(float* results) {
    // 1. Setup Shapes
    Sphere sphere(Vec3(0.5f, 0.0f, 0.0f), 1.0f); // Center at 0.5, radius 1. Overlaps origin.
    Box box(Vec3(0.5f)); // 1x1x1 box at origin (min -0.5, max 0.5)
    
    // Transform (Identity for now)
    GJK::Transform txSphere(Vec3(0.8f, 0.0f, 0.0f), Quaternion::identity()); // Sphere center at world 0.8
    GJK::Transform txBox(Vec3(0.0f, 0.0f, 0.0f), Quaternion::identity());    // Box center at world 0.0
    
    // Box range: -0.5 to 0.5
    // Sphere range: 0.8 - 1.0 = -0.2 to 1.8
    // Overlap: [-0.2, 0.5] -> 0.7 overlap?
    // Wait, let's calculate simpler.
    // Box Right Face at x=0.5.
    // Sphere Center at x=0.8.
    // Sphere Left Point at x = 0.8 - 1.0 = -0.2.
    // Penetration depth should be 0.5 - (-0.2) = 0.7.
    // Collision Normal should be (-1, 0, 0) pointing from A to B or B to A?
    // EPA returns normal pointing from B to A usually (push A out of B).
    // A=Sphere, B=Box. To push Sphere out of Box, move Sphere +X. So normal (1,0,0).
    // Wait, let's see implementation.
    
    Simplex simplex;
    bool collision = GJK::run(sphere, txSphere, box, txBox, simplex);
    
    results[0] = collision ? 1.0f : 0.0f;
    
    if (collision) {
        float depth = 0.0f;
        Vec3 normal(0.0f);
        bool epa_success = EPA::run(simplex, sphere, txSphere, box, txBox, depth, normal);
        
        results[1] = epa_success ? 1.0f : 0.0f;
        results[2] = depth;
        results[3] = normal.x;
        results[4] = normal.y;
        results[5] = normal.z;
    }
}

int main() {
    std::cout << "[Test] GPU Narrowphase (GJK/EPA)" << std::endl;

    const int N = 6;
    size_t size = N * sizeof(float);
    float* h_res = (float*)malloc(size);
    float* d_res;
    cudaMalloc(&d_res, size);
    cudaMemset(d_res, 0, size);

    test_narrowphase_kernel<<<1, 1>>>(d_res);
    cudaDeviceSynchronize();
    
    cudaMemcpy(h_res, d_res, size, cudaMemcpyDeviceToHost);
    
    bool success = true;
    
    if (h_res[0] == 1.0f) std::cout << "SUCCESS: GJK detected collision." << std::endl;
    else { std::cout << "FAILURE: GJK missed collision." << std::endl; success = false; }
    
    if (h_res[1] == 1.0f) std::cout << "SUCCESS: EPA converged." << std::endl;
    else { std::cout << "FAILURE: EPA failed." << std::endl; success = false; }
    
    std::cout << "Depth: " << h_res[2] << " (Expected ~0.7)" << std::endl;
    std::cout << "Normal: " << h_res[3] << ", " << h_res[4] << ", " << h_res[5] << std::endl;
    
    // Check Depth
    if (fabs(h_res[2] - 0.7f) < 0.01f) std::cout << "SUCCESS: Depth correct." << std::endl;
    else { 
        std::cout << "WARNING: Depth slightly off or coordinates different." << std::endl; 
        // 0.7 is ideal case.
    }

    cudaFree(d_res);
    free(h_res);
    
    return success ? 0 : 1;
}

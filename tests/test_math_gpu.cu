#include "basements/core/math/basements.h"
#include <iostream>
#include <cuda_runtime.h>

using namespace basements::math;

__global__ void test_math_kernel(float* result) {
    // Test Vec3
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b; // (5, 7, 9)
    
    // Test Dot product
    float d = a.dot(b); // 4 + 10 + 18 = 32
    
    // Test Cross product
    Vec3 x = a.cross(b); // (-3, 6, -3)
    
    // Test Quaternion
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f); // Identity
    Quaternion q2(0.0f, 1.0f, 0.0f, 0.0f); // i
    Quaternion q3 = q1 * q2; // Should be q2
    
    // Store results for verification
    result[0] = c.x;
    result[1] = c.y;
    result[2] = c.z;
    result[3] = d;
    result[4] = x.x;
    result[5] = q3.x;
}

int main() {
    std::cout << "[Test] Core Math on GPU" << std::endl;
    
    const int N = 6;
    size_t size = N * sizeof(float);
    float* h_res = (float*)malloc(size);
    float* d_res;
    
    cudaMalloc(&d_res, size);
    
    test_math_kernel<<<1, 1>>>(d_res);
    cudaDeviceSynchronize();
    
    cudaMemcpy(h_res, d_res, size, cudaMemcpyDeviceToHost);
    
    bool success = true;
    
    // Check results
    if (h_res[0] != 5.0f) { std::cerr << "Vec3 Add X mismatch: " << h_res[0] << std::endl; success = false; }
    if (h_res[1] != 7.0f) { std::cerr << "Vec3 Add Y mismatch: " << h_res[1] << std::endl; success = false; }
    if (h_res[2] != 9.0f) { std::cerr << "Vec3 Add Z mismatch: " << h_res[2] << std::endl; success = false; }
    if (h_res[3] != 32.0f) { std::cerr << "Vec3 Dot mismatch: " << h_res[3] << std::endl; success = false; }
    if (h_res[4] != -3.0f) { std::cerr << "Vec3 Cross X mismatch: " << h_res[4] << std::endl; success = false; }
    
    // Quaternion q2 is (0, 1, 0, 0) -> x=1
    // Wait, Quaternion constructor is (w, x, y, z) usually.
    // Let's check quaternion.h
    // Standard convention: w is real part.
    // q1(1,0,0,0) -> w=1 (Identity)
    // q2(0,1,0,0) -> w=0, x=1
    // q1 * q2 = q2. So q3.x should be 1.0f
    
    if (h_res[5] != 1.0f) { std::cerr << "Quaternion Mul mismatch: " << h_res[5] << std::endl; success = false; }
    
    if (success) {
        std::cout << "SUCCESS: Math library works on GPU!" << std::endl;
    } else {
        std::cout << "FAILURE: Math library GPU errors." << std::endl;
    }
    
    cudaFree(d_res);
    free(h_res);
    
    return success ? 0 : 1;
}

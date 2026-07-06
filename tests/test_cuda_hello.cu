#include "basements/core/math/common.h"
#include <iostream>
#include <vector>
#include <cuda_runtime.h>

// Simple CUDA Kernel
__global__ void add_vectors(const float* A, const float* B, float* C, int N) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < N) {
        C[i] = A[i] + B[i];
    }
}

void check_cuda_error(cudaError_t err, const char* msg) {
    if (err != cudaSuccess) {
        std::cerr << "CUDA Error [" << msg << "]: " << cudaGetErrorString(err) << std::endl;
        exit(EXIT_FAILURE);
    }
}

int main() {
    std::cout << "[Test] CUDA Hello World (Vector Add)" << std::endl;

    int N = 1 << 20; // 1M elements
    size_t size = N * sizeof(float);

    // Host memory
    std::vector<float> h_A(N, 1.0f);
    std::vector<float> h_B(N, 2.0f);
    std::vector<float> h_C(N, 0.0f);

    // Device memory
    float *d_A, *d_B, *d_C;
    check_cuda_error(cudaMalloc(&d_A, size), "Malloc A");
    check_cuda_error(cudaMalloc(&d_B, size), "Malloc B");
    check_cuda_error(cudaMalloc(&d_C, size), "Malloc C");

    // Copy to device
    check_cuda_error(cudaMemcpy(d_A, h_A.data(), size, cudaMemcpyHostToDevice), "Copy A");
    check_cuda_error(cudaMemcpy(d_B, h_B.data(), size, cudaMemcpyHostToDevice), "Copy B");

    // Launch Kernel
    int threadsPerBlock = 256;
    int blocksPerGrid = (N + threadsPerBlock - 1) / threadsPerBlock;
    
    add_vectors<<<blocksPerGrid, threadsPerBlock>>>(d_A, d_B, d_C, N);
    check_cuda_error(cudaGetLastError(), "Kernel Launch");
    check_cuda_error(cudaDeviceSynchronize(), "Sync");

    // Copy back
    check_cuda_error(cudaMemcpy(h_C.data(), d_C, size, cudaMemcpyDeviceToHost), "Copy C");

    // Verify
    bool success = true;
    for (int i = 0; i < N; ++i) {
        if (abs(h_C[i] - 3.0f) > 1e-5) {
            std::cerr << "Error at " << i << ": " << h_C[i] << " != 3.0" << std::endl;
            success = false;
            break;
        }
    }

    if (success) {
        std::cout << "SUCCESS: Vector addition on GPU correct!" << std::endl;
        std::cout << "Computed " << N << " elements." << std::endl;
    } else {
        std::cout << "FAILURE: Results incorrect." << std::endl;
    }

    // Cleanup
    cudaFree(d_A);
    cudaFree(d_B);
    cudaFree(d_C);

    return success ? 0 : 1;
}

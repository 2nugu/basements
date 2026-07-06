#pragma once
#include "basements/backend/compute_backend.h"

#ifdef BASEMENTS_USE_CUDA
#include <cuda_runtime.h>
#include <string>

namespace basements {
namespace backend {

/**
 * @brief NVIDIA CUDA compute backend
 */
class CUDABackend : public IComputeBackend {
public:
    CUDABackend() : initialized_(false), device_id_(0) {}
    ~CUDABackend() override { shutdown(); }

    // ========== Device Management ==========

    bool initialize() override {
        if (initialized_) return true;

        int device_count = 0;
        cudaError_t err = cudaGetDeviceCount(&device_count);
        if (err != cudaSuccess || device_count == 0) {
            return false;
        }

        // Use device 0 by default
        err = cudaSetDevice(device_id_);
        if (err != cudaSuccess) {
            return false;
        }

        initialized_ = true;
        return true;
    }

    void shutdown() override {
        if (initialized_) {
            cudaDeviceReset();
            initialized_ = false;
        }
    }

    BackendType get_type() const override {
        return BackendType::CUDA;
    }

    std::string get_device_name() const override {
        if (!initialized_) return "CUDA (Not initialized)";

        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, device_id_);
        return std::string("NVIDIA ") + prop.name;
    }

    // ========== Memory Management ==========

    void* allocate(size_t size) override {
        void* ptr = nullptr;
        cudaError_t err = cudaMalloc(&ptr, size);
        return (err == cudaSuccess) ? ptr : nullptr;
    }

    void deallocate(void* ptr) override {
        if (ptr) {
            cudaFree(ptr);
        }
    }

    void copy_to_device(void* dst, const void* src, size_t size) override {
        cudaMemcpy(dst, src, size, cudaMemcpyHostToDevice);
    }

    void copy_to_host(void* dst, const void* src, size_t size) override {
        cudaMemcpy(dst, src, size, cudaMemcpyDeviceToHost);
    }

    // ========== Kernel Execution ==========

    void launch_kernel(const char* kernel_name, void** args, size_t num_threads) override {
        // Kernel dispatch would be implemented here
        // This requires a kernel registry or function pointer mapping
        
        // Example grid/block configuration
        int block_size = 256;
        int grid_size = (num_threads + block_size - 1) / block_size;

        // Actual kernel launch would use cudaLaunchKernel or <<<>>> syntax
        // For now, this is a placeholder
        (void)kernel_name;
        (void)args;
        (void)grid_size;
    }

    // ========== Synchronization ==========

    void synchronize() override {
        cudaDeviceSynchronize();
    }

private:
    bool initialized_;
    int device_id_;
};

} // namespace backend
} // namespace basements

#endif // BASEMENTS_USE_CUDA

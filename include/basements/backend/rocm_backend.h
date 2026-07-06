#pragma once
#include "basements/backend/compute_backend.h"

#ifdef BASEMENTS_USE_ROCM
#include <hip/hip_runtime.h>
#include <string>

namespace basements {
namespace backend {

/**
 * @brief AMD ROCm compute backend using HIP
 * 
 * HIP (Heterogeneous-compute Interface for Portability) allows
 * the same kernel code to compile for both NVIDIA CUDA and AMD ROCm.
 * 
 * This backend uses HIP API which is nearly identical to CUDA API.
 */
class ROCmBackend : public IComputeBackend {
public:
    ROCmBackend() : initialized_(false), device_id_(0) {}
    ~ROCmBackend() override { shutdown(); }

    // ========== Device Management ==========

    bool initialize() override {
        if (initialized_) return true;

        int device_count = 0;
        hipError_t err = hipGetDeviceCount(&device_count);
        if (err != hipSuccess || device_count == 0) {
            return false;
        }

        // Use device 0 by default
        err = hipSetDevice(device_id_);
        if (err != hipSuccess) {
            return false;
        }

        initialized_ = true;
        return true;
    }

    void shutdown() override {
        if (initialized_) {
            hipDeviceReset();
            initialized_ = false;
        }
    }

    BackendType get_type() const override {
        return BackendType::ROCM;
    }

    std::string get_device_name() const override {
        if (!initialized_) return "ROCm (Not initialized)";

        hipDeviceProp_t prop;
        hipGetDeviceProperties(&prop, device_id_);
        return std::string("AMD ") + prop.name;
    }

    // ========== Memory Management ==========

    void* allocate(size_t size) override {
        void* ptr = nullptr;
        hipError_t err = hipMalloc(&ptr, size);
        return (err == hipSuccess) ? ptr : nullptr;
    }

    void deallocate(void* ptr) override {
        if (ptr) {
            hipFree(ptr);
        }
    }

    void copy_to_device(void* dst, const void* src, size_t size) override {
        hipMemcpy(dst, src, size, hipMemcpyHostToDevice);
    }

    void copy_to_host(void* dst, const void* src, size_t size) override {
        hipMemcpy(dst, src, size, hipMemcpyDeviceToHost);
    }

    // ========== Kernel Execution ==========

    void launch_kernel(const char* kernel_name, void** args, size_t num_threads) override {
        // HIP kernel dispatch
        // Same pattern as CUDA but uses HIP API
        
        int block_size = 256;
        int grid_size = (num_threads + block_size - 1) / block_size;

        // Actual kernel launch would use hipLaunchKernel or <<<>>> syntax
        // HIP kernels use the same syntax as CUDA kernels
        (void)kernel_name;
        (void)args;
        (void)grid_size;
    }

    // ========== Synchronization ==========

    void synchronize() override {
        hipDeviceSynchronize();
    }

private:
    bool initialized_;
    int device_id_;
};

} // namespace backend
} // namespace basements

#endif // BASEMENTS_USE_ROCM

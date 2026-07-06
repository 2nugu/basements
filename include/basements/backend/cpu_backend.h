#pragma once
#include "basements/backend/compute_backend.h"
#include <cstdlib>
#include <cstring>
#include <thread>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace basements {
namespace backend {

/**
 * @brief CPU-based compute backend using OpenMP
 */
class CPUBackend : public IComputeBackend {
public:
    CPUBackend() : initialized_(false) {}
    ~CPUBackend() override { shutdown(); }

    // ========== Device Management ==========

    bool initialize() override {
        if (initialized_) return true;

#ifdef _OPENMP
        // Set number of threads (default: hardware concurrency)
        int num_threads = std::thread::hardware_concurrency();
        omp_set_num_threads(num_threads);
#endif
        
        initialized_ = true;
        return true;
    }

    void shutdown() override {
        initialized_ = false;
    }

    BackendType get_type() const override {
        return BackendType::CPU;
    }

    std::string get_device_name() const override {
#ifdef _OPENMP
        int num_threads = omp_get_max_threads();
        return "CPU (OpenMP, " + std::to_string(num_threads) + " threads)";
#else
        return "CPU (Single-threaded)";
#endif
    }

    // ========== Memory Management ==========

    void* allocate(size_t size) override {
        return std::malloc(size);
    }

    void deallocate(void* ptr) override {
        std::free(ptr);
    }

    void copy_to_device(void* dst, const void* src, size_t size) override {
        // CPU: "device" is just host memory
        std::memcpy(dst, src, size);
    }

    void copy_to_host(void* dst, const void* src, size_t size) override {
        // CPU: "device" is just host memory
        std::memcpy(dst, src, size);
    }

    // ========== Kernel Execution ==========

    void launch_kernel(const char* kernel_name, void** args, size_t num_threads) override {
        // Dispatch to CPU kernel implementations
        // This is a simplified dispatch mechanism
        // In practice, you'd use a registry or function pointers
        
        // Example: if (strcmp(kernel_name, "integrate_bodies") == 0) { ... }
        // For now, this is a placeholder
        (void)kernel_name;
        (void)args;
        (void)num_threads;
    }

    // ========== Synchronization ==========

    void synchronize() override {
        // CPU: no async operations, so this is a no-op
    }

private:
    bool initialized_;
};

} // namespace backend
} // namespace basements

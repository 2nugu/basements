#pragma once
#include <cstddef>
#include <string>

namespace basements {
namespace backend {

/**
 * @brief Supported compute backend types
 */
enum class BackendType {
    CPU,      ///< OpenMP-based CPU backend
    CUDA,     ///< NVIDIA CUDA
    ROCM,     ///< AMD ROCm (via HIP)
    METAL,    ///< Apple Metal
    VULKAN,   ///< Vulkan Compute
    ONEAPI    ///< Intel oneAPI
};

/**
 * @brief Abstract interface for compute backends
 * 
 * This interface provides hardware-independent abstraction for:
 * - Memory allocation/deallocation
 * - Data transfer (host ↔ device)
 * - Kernel execution
 * - Synchronization
 * 
 * Implementations: CPUBackend, CUDABackend, ROCmBackend, etc.
 */
class IComputeBackend {
public:
    virtual ~IComputeBackend() = default;

    // ========== Device Management ==========
    
    /**
     * @brief Initialize the backend
     * @return true if successful, false otherwise
     */
    virtual bool initialize() = 0;

    /**
     * @brief Shutdown the backend and release resources
     */
    virtual void shutdown() = 0;

    /**
     * @brief Get the backend type
     * @return BackendType enum value
     */
    virtual BackendType get_type() const = 0;

    /**
     * @brief Get human-readable device name
     * @return Device name (e.g., "NVIDIA RTX 4090", "AMD Radeon RX 7900 XT")
     */
    virtual std::string get_device_name() const = 0;

    // ========== Memory Management ==========

    /**
     * @brief Allocate memory on the device
     * @param size Number of bytes to allocate
     * @return Pointer to allocated memory, or nullptr on failure
     */
    virtual void* allocate(size_t size) = 0;

    /**
     * @brief Deallocate device memory
     * @param ptr Pointer to memory to free
     */
    virtual void deallocate(void* ptr) = 0;

    /**
     * @brief Copy data from host to device
     * @param dst Device pointer
     * @param src Host pointer
     * @param size Number of bytes to copy
     */
    virtual void copy_to_device(void* dst, const void* src, size_t size) = 0;

    /**
     * @brief Copy data from device to host
     * @param dst Host pointer
     * @param src Device pointer
     * @param size Number of bytes to copy
     */
    virtual void copy_to_host(void* dst, const void* src, size_t size) = 0;

    // ========== Kernel Execution ==========

    /**
     * @brief Launch a compute kernel
     * @param kernel_name Name of the kernel to execute
     * @param args Array of pointers to kernel arguments
     * @param num_threads Total number of threads to launch
     * 
     * Note: Grid/block dimensions are determined internally based on backend
     */
    virtual void launch_kernel(const char* kernel_name, void** args, size_t num_threads) = 0;

    // ========== Synchronization ==========

    /**
     * @brief Wait for all pending operations to complete
     */
    virtual void synchronize() = 0;
};

/**
 * @brief Factory function to create a backend instance
 * @param type Desired backend type
 * @return Pointer to backend instance, or nullptr if unavailable
 */
IComputeBackend* create_backend(BackendType type);

/**
 * @brief Get human-readable name for backend type
 * @param type Backend type
 * @return String name (e.g., "CUDA", "ROCm")
 */
const char* backend_type_to_string(BackendType type);

} // namespace backend
} // namespace basements

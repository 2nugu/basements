#include "basements/backend/compute_backend.h"
#include <stdexcept>

namespace basements {
namespace backend {

const char* backend_type_to_string(BackendType type) {
    switch (type) {
        case BackendType::CPU:    return "CPU";
        case BackendType::CUDA:   return "CUDA";
        case BackendType::ROCM:   return "ROCm";
        case BackendType::METAL:  return "Metal";
        case BackendType::VULKAN: return "Vulkan";
        case BackendType::ONEAPI: return "oneAPI";
        default: return "Unknown";
    }
}

// Forward declarations of backend implementations
class CPUBackend;
#ifdef BASEMENTS_USE_CUDA
class CUDABackend;
#endif
#ifdef BASEMENTS_USE_ROCM
class ROCmBackend;
#endif

IComputeBackend* create_backend(BackendType type) {
    switch (type) {
        case BackendType::CPU:
            // CPU backend is always available
            return new CPUBackend();
        
#ifdef BASEMENTS_USE_CUDA
        case BackendType::CUDA:
            return new CUDABackend();
#endif

#ifdef BASEMENTS_USE_ROCM
        case BackendType::ROCM:
            return new ROCmBackend();
#endif

        default:
            // Backend not compiled or not supported
            return nullptr;
    }
}

} // namespace backend
} // namespace basements

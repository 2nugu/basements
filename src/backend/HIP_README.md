# HIP Kernel Example - Portable CUDA/ROCm Code

This directory contains example HIP kernels that compile to both NVIDIA CUDA and AMD ROCm.

## What is HIP?

**HIP (Heterogeneous-compute Interface for Portability)** is AMD's C++ runtime API and kernel language that allows developers to create portable applications for AMD and NVIDIA GPUs from a single source code.

## Key Features

- **Write Once, Run Anywhere**: Same kernel code compiles for CUDA and ROCm
- **CUDA-like Syntax**: Nearly identical to CUDA API
- **Automatic Translation**: `hipify` tool can convert CUDA code to HIP

## Example: Integrate Bodies Kernel

```cpp
// integrate_bodies.hip
#include <hip/hip_runtime.h>

struct RigidBody {
    float3 position;
    float3 velocity;
    float3 force;
    float mass;
};

__global__ void integrate_bodies_kernel(RigidBody* bodies, int n, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        // F = ma → a = F/m
        float3 acceleration;
        acceleration.x = bodies[i].force.x / bodies[i].mass;
        acceleration.y = bodies[i].force.y / bodies[i].mass;
        acceleration.z = bodies[i].force.z / bodies[i].mass;
        
        // v = v + a*dt
        bodies[i].velocity.x += acceleration.x * dt;
        bodies[i].velocity.y += acceleration.y * dt;
        bodies[i].velocity.z += acceleration.z * dt;
        
        // p = p + v*dt
        bodies[i].position.x += bodies[i].velocity.x * dt;
        bodies[i].position.y += bodies[i].velocity.y * dt;
        bodies[i].position.z += bodies[i].velocity.z * dt;
    }
}
```

## Compilation

### For NVIDIA CUDA
```bash
hipcc --cuda integrate_bodies.hip -o integrate_cuda.o
# Or use nvcc directly (HIP is compatible)
nvcc integrate_bodies.hip -o integrate_cuda.o
```

### For AMD ROCm
```bash
hipcc integrate_bodies.hip -o integrate_rocm.o
```

## API Comparison

| CUDA API | HIP API | Description |
|----------|---------|-------------|
| `cudaMalloc` | `hipMalloc` | Allocate device memory |
| `cudaFree` | `hipFree` | Free device memory |
| `cudaMemcpy` | `hipMemcpy` | Copy memory |
| `cudaDeviceSynchronize` | `hipDeviceSynchronize` | Sync device |
| `cudaGetDeviceProperties` | `hipGetDeviceProperties` | Get device info |
| `__global__` | `__global__` | Kernel qualifier (same!) |
| `blockIdx`, `threadIdx` | `blockIdx`, `threadIdx` | Thread indices (same!) |

## Porting CUDA to HIP

AMD provides `hipify-perl` tool to automatically convert CUDA code:

```bash
hipify-perl my_cuda_kernel.cu > my_hip_kernel.hip
```

Most conversions are simple string replacements:
- `cuda` → `hip`
- `CUDA` → `HIP`

## Integration with Basements

The `ROCmBackend` class uses HIP API to provide AMD GPU support:

```cpp
// Same interface as CUDABackend
ROCmBackend* backend = new ROCmBackend();
backend->initialize();
void* gpu_ptr = backend->allocate(1024);
backend->copy_to_device(gpu_ptr, host_data, 1024);
backend->launch_kernel("integrate_bodies", args, num_bodies);
backend->synchronize();
```

## Requirements

- **AMD GPU**: Radeon RX 6000/7000 series, or MI series
- **ROCm SDK**: Version 5.0+ (https://rocm.docs.amd.com)
- **HIP Compiler**: `hipcc` (included with ROCm)

## Testing

```bash
# Check ROCm installation
rocm-smi

# Compile test
hipcc --version

# Run backend test
cmake -DBASEMENTS_BACKEND=ROCM ..
cmake --build . --target test_backend
./test_backend
```

## References

- HIP Documentation: https://rocm.docs.amd.com/projects/HIP
- HIP Programming Guide: https://github.com/ROCm/HIP
- CUDA to HIP Porting Guide: https://rocm.docs.amd.com/en/latest/how-to/hip-porting-guide.html

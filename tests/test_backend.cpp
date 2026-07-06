#include "basements/backend/compute_backend.h"
#include "basements/backend/cpu_backend.h"
#ifdef BASEMENTS_USE_CUDA
#include "basements/backend/cuda_backend.h"
#endif

#include <gtest/gtest.h>
#include <cstring>

using namespace basements::backend;

// ========== Backend Type String Tests ==========

TEST(BackendTest, TypeToString) {
    EXPECT_STREQ(backend_type_to_string(BackendType::CPU), "CPU");
    EXPECT_STREQ(backend_type_to_string(BackendType::CUDA), "CUDA");
    EXPECT_STREQ(backend_type_to_string(BackendType::ROCM), "ROCm");
    EXPECT_STREQ(backend_type_to_string(BackendType::METAL), "Metal");
}

// ========== CPU Backend Tests ==========

TEST(CPUBackendTest, Initialization) {
    IComputeBackend* backend = create_backend(BackendType::CPU);
    ASSERT_NE(backend, nullptr);

    EXPECT_TRUE(backend->initialize());
    EXPECT_EQ(backend->get_type(), BackendType::CPU);
    
    std::string device_name = backend->get_device_name();
    EXPECT_TRUE(device_name.find("CPU") != std::string::npos);

    backend->shutdown();
    delete backend;
}

TEST(CPUBackendTest, MemoryAllocation) {
    IComputeBackend* backend = create_backend(BackendType::CPU);
    ASSERT_NE(backend, nullptr);
    ASSERT_TRUE(backend->initialize());

    // Allocate 1KB
    void* ptr = backend->allocate(1024);
    ASSERT_NE(ptr, nullptr);

    // Write some data
    int test_data[256];
    for (int i = 0; i < 256; ++i) {
        test_data[i] = i;
    }

    backend->copy_to_device(ptr, test_data, sizeof(test_data));

    // Read back
    int result[256];
    backend->copy_to_host(result, ptr, sizeof(result));

    // Verify
    for (int i = 0; i < 256; ++i) {
        EXPECT_EQ(result[i], i);
    }

    backend->deallocate(ptr);
    backend->shutdown();
    delete backend;
}

TEST(CPUBackendTest, Synchronization) {
    IComputeBackend* backend = create_backend(BackendType::CPU);
    ASSERT_NE(backend, nullptr);
    ASSERT_TRUE(backend->initialize());

    // Synchronize should not crash (no-op for CPU)
    backend->synchronize();

    backend->shutdown();
    delete backend;
}

// ========== CUDA Backend Tests (Conditional) ==========

#ifdef BASEMENTS_USE_CUDA
TEST(CUDABackendTest, Initialization) {
    IComputeBackend* backend = create_backend(BackendType::CUDA);
    
    // CUDA might not be available on all systems
    if (backend == nullptr) {
        GTEST_SKIP() << "CUDA backend not available";
    }

    bool init_success = backend->initialize();
    if (!init_success) {
        delete backend;
        GTEST_SKIP() << "CUDA initialization failed (no GPU?)";
    }

    EXPECT_EQ(backend->get_type(), BackendType::CUDA);
    
    std::string device_name = backend->get_device_name();
    EXPECT_TRUE(device_name.find("NVIDIA") != std::string::npos || 
                device_name.find("CUDA") != std::string::npos);

    backend->shutdown();
    delete backend;
}

TEST(CUDABackendTest, MemoryAllocation) {
    IComputeBackend* backend = create_backend(BackendType::CUDA);
    if (backend == nullptr || !backend->initialize()) {
        delete backend;
        GTEST_SKIP() << "CUDA not available";
    }

    // Allocate 1KB on GPU
    void* gpu_ptr = backend->allocate(1024);
    ASSERT_NE(gpu_ptr, nullptr);

    // Host data
    int test_data[256];
    for (int i = 0; i < 256; ++i) {
        test_data[i] = i * 2;
    }

    // Copy to GPU
    backend->copy_to_device(gpu_ptr, test_data, sizeof(test_data));

    // Copy back
    int result[256];
    backend->copy_to_host(result, gpu_ptr, sizeof(result));

    // Verify
    for (int i = 0; i < 256; ++i) {
        EXPECT_EQ(result[i], i * 2);
    }

    backend->deallocate(gpu_ptr);
    backend->shutdown();
    delete backend;
}
#endif

// ========== Factory Tests ==========

TEST(BackendFactoryTest, CreateCPU) {
    IComputeBackend* backend = create_backend(BackendType::CPU);
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->get_type(), BackendType::CPU);
    delete backend;
}

TEST(BackendFactoryTest, UnsupportedBackend) {
    // Metal backend is not implemented yet
    IComputeBackend* backend = create_backend(BackendType::METAL);
    EXPECT_EQ(backend, nullptr);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include <iostream>
#include <chrono>
#include <random>
#include <iomanip>
#include "basements/core/math/vec3.h"

using namespace basements::math;
using namespace std::chrono;

// Benchmark configuration
constexpr int ITERATIONS = 10'000'000;
constexpr int WARMUP = 1'000'000;

// Random number generator
std::mt19937 rng(42);  // Fixed seed for reproducibility
std::uniform_real_distribution<float> dist(-100.0f, 100.0f);

// Generate random Vec3
Vec3 random_vec3() {
    return Vec3(dist(rng), dist(rng), dist(rng));
}

// Benchmark template
template<typename Func>
double benchmark(const std::string& name, Func func, int iterations = ITERATIONS) {
    // Warmup
    for (int i = 0; i < WARMUP; ++i) {
        func();
    }
    
    // Actual benchmark
    auto start = high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        func();
    }
    auto end = high_resolution_clock::now();
    
    auto duration = duration_cast<nanoseconds>(end - start).count();
    double avg_ns = static_cast<double>(duration) / iterations;
    
    std::cout << std::setw(30) << std::left << name 
              << ": " << std::setw(10) << std::right << std::fixed << std::setprecision(2) 
              << avg_ns << " ns/op" << std::endl;
    
    return avg_ns;
}

// Scalar (non-SIMD) implementations for comparison
namespace scalar {
    struct Vec3 {
        float x, y, z;
        
        Vec3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
        
        Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
        Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
        Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
        
        float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
        
        Vec3 cross(const Vec3& o) const {
            return Vec3(
                y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x
            );
        }
        
        float length() const { return std::sqrt(dot(*this)); }
        
        Vec3 normalized() const {
            float len = length();
            if (len < 1e-6f) return Vec3(0, 0, 0);
            return *this * (1.0f / len);
        }
    };
}

int main() {
    std::cout << "=======================================================\n";
    std::cout << "  Basements Physics Engine - Vec3 Performance Benchmark\n";
    std::cout << "=======================================================\n\n";
    
    std::cout << "Configuration:\n";
    std::cout << "  Iterations: " << ITERATIONS << "\n";
    std::cout << "  Warmup: " << WARMUP << "\n";
    std::cout << "  SIMD: AVX2 enabled\n\n";
    
    // Pre-generate random vectors
    Vec3 v1 = random_vec3();
    Vec3 v2 = random_vec3();
    scalar::Vec3 sv1(v1.x, v1.y, v1.z);
    scalar::Vec3 sv2(v2.x, v2.y, v2.z);
    
    // ============================================================
    // Addition Benchmark
    // ============================================================
    std::cout << "--- Addition ---\n";
    double simd_add = benchmark("SIMD Vec3 +", [&]() {
        volatile Vec3 result = v1 + v2;
        (void)result;
    });
    
    double scalar_add = benchmark("Scalar Vec3 +", [&]() {
        volatile scalar::Vec3 result = sv1 + sv2;
        (void)result;
    });
    
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) 
              << (scalar_add / simd_add) << "x\n\n";
    
    // ============================================================
    // Dot Product Benchmark
    // ============================================================
    std::cout << "--- Dot Product ---\n";
    double simd_dot = benchmark("SIMD Vec3 dot", [&]() {
        volatile float result = v1.dot(v2);
        (void)result;
    });
    
    double scalar_dot = benchmark("Scalar Vec3 dot", [&]() {
        volatile float result = sv1.dot(sv2);
        (void)result;
    });
    
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) 
              << (scalar_dot / simd_dot) << "x\n\n";
    
    // ============================================================
    // Cross Product Benchmark
    // ============================================================
    std::cout << "--- Cross Product ---\n";
    double simd_cross = benchmark("SIMD Vec3 cross", [&]() {
        volatile Vec3 result = v1.cross(v2);
        (void)result;
    });
    
    double scalar_cross = benchmark("Scalar Vec3 cross", [&]() {
        volatile scalar::Vec3 result = sv1.cross(sv2);
        (void)result;
    });
    
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) 
              << (scalar_cross / simd_cross) << "x\n\n";
    
    // ============================================================
    // Normalization Benchmark
    // ============================================================
    std::cout << "--- Normalization ---\n";
    double simd_norm = benchmark("SIMD Vec3 normalize", [&]() {
        volatile Vec3 result = v1.normalized();
        (void)result;
    });
    
    double scalar_norm = benchmark("Scalar Vec3 normalize", [&]() {
        volatile scalar::Vec3 result = sv1.normalized();
        (void)result;
    });
    
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) 
              << (scalar_norm / simd_norm) << "x\n\n";
    
    // ============================================================
    // Length Benchmark
    // ============================================================
    std::cout << "--- Length ---\n";
    double simd_len = benchmark("SIMD Vec3 length", [&]() {
        volatile float result = v1.length();
        (void)result;
    });
    
    double scalar_len = benchmark("Scalar Vec3 length", [&]() {
        volatile float result = sv1.length();
        (void)result;
    });
    
    std::cout << "  Speedup: " << std::fixed << std::setprecision(2) 
              << (scalar_len / simd_len) << "x\n\n";
    
    // ============================================================
    // Summary
    // ============================================================
    std::cout << "=======================================================\n";
    std::cout << "Summary:\n";
    std::cout << "  Average SIMD speedup: " << std::fixed << std::setprecision(2)
              << ((scalar_add / simd_add) + (scalar_dot / simd_dot) + 
                  (scalar_cross / simd_cross) + (scalar_norm / simd_norm) + 
                  (scalar_len / simd_len)) / 5.0 << "x\n";
    std::cout << "=======================================================\n";
    
    return 0;
}

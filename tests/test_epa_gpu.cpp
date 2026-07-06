#include \u003cgtest/gtest.h\u003e
#include "basements/physics/collision/epa.h"
#include "basements/physics/collision/primitives.h"
#include \u003cchrono\u003e

using namespace basements;
using namespace basements::collision;

// ============================================================================
// Helper Functions
// ============================================================================

float random_float(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

Vec3 random_vec3(float min, float max) {
    return Vec3(
        random_float(min, max),
        random_float(min, max),
        random_float(min, max)
    );
}

// ============================================================================
// Basic EPA Tests
// ============================================================================

TEST(EPATest, BoxBoxPenetration) {
    // Two overlapping boxes
    Box box_a(Vec3(1.0f, 1.0f, 1.0f)); // 2x2x2 box
    Box box_b(Vec3(1.0f, 1.0f, 1.0f));
    
    GJK::Transform tx_a(Vec3(0, 0, 0), Quaternion::identity());
    GJK::Transform tx_b(Vec3(1.5f, 0, 0), Quaternion::identity()); // Overlapping by 0.5
    
    Simplex simplex;
    bool hit = GJK::run(box_a, tx_a, box_b, tx_b, simplex);
    
    ASSERT_TRUE(hit) << "GJK should detect collision";
    ASSERT_EQ(simplex.size, 4) << "Simplex should be tetrahedron";
    
    float depth;
    Vec3 normal;
    bool epa_success = EPA::run(simplex, box_a, tx_a, box_b, tx_b, depth, normal);
    
    ASSERT_TRUE(epa_success) << "EPA should converge";
    EXPECT_NEAR(depth, 0.5f, 0.1f) << "Penetration depth should be ~0.5";
    EXPECT_NEAR(normal.length(), 1.0f, 0.01f) << "Normal should be unit length";
}

TEST(EPATest, SphereSphereOverlap) {
    Sphere sphere_a(1.0f);
    Sphere sphere_b(1.0f);
    
    GJK::Transform tx_a(Vec3(0, 0, 0), Quaternion::identity());
    GJK::Transform tx_b(Vec3(1.5f, 0, 0), Quaternion::identity()); // Overlapping by 0.5
    
    Simplex simplex;
    bool hit = GJK::run(sphere_a, tx_a, sphere_b, tx_b, simplex);
    
    ASSERT_TRUE(hit);
    
    float depth;
    Vec3 normal;
    bool epa_success = EPA::run(simplex, sphere_a, tx_a, sphere_b, tx_b, depth, normal);
    
    ASSERT_TRUE(epa_success);
    EXPECT_NEAR(depth, 0.5f, 0.1f);
    EXPECT_NEAR(std::abs(normal.x), 1.0f, 0.1f) << "Normal should point along X axis";
}

TEST(EPATest, BoxSphereCollision) {
    Box box(Vec3(1.0f, 1.0f, 1.0f));
    Sphere sphere(0.8f);
    
    GJK::Transform tx_box(Vec3(0, 0, 0), Quaternion::identity());
    GJK::Transform tx_sphere(Vec3(1.5f, 0, 0), Quaternion::identity());
    
    Simplex simplex;
    bool hit = GJK::run(box, tx_box, sphere, tx_sphere, simplex);
    
    ASSERT_TRUE(hit);
    
    float depth;
    Vec3 normal;
    bool epa_success = EPA::run(simplex, box, tx_box, sphere, tx_sphere, depth, normal);
    
    ASSERT_TRUE(epa_success);
    EXPECT_GT(depth, 0.0f) << "Should have positive penetration depth";
    EXPECT_NEAR(normal.length(), 1.0f, 0.01f);
}

// ============================================================================
// Degeneracy Tests
// ============================================================================

TEST(EPATest, NearlyIdenticalVertices) {
    // Test with very small penetration (degeneracy edge case)
    Box box_a(Vec3(1.0f, 1.0f, 1.0f));
    Box box_b(Vec3(1.0f, 1.0f, 1.0f));
    
    GJK::Transform tx_a(Vec3(0, 0, 0), Quaternion::identity());
    GJK::Transform tx_b(Vec3(2.001f, 0, 0), Quaternion::identity()); // Barely touching
    
    Simplex simplex;
    bool hit = GJK::run(box_a, tx_a, box_b, tx_b, simplex);
    
    ASSERT_TRUE(hit);
    
    float depth;
    Vec3 normal;
    bool epa_success = EPA::run(simplex, box_a, tx_a, box_b, tx_b, depth, normal);
    
    ASSERT_TRUE(epa_success) << "EPA should handle near-degenerate case";
    EXPECT_GT(depth, 0.0f);
    EXPECT_LT(depth, 0.1f) << "Depth should be very small";
}

TEST(EPATest, DegenerateFaceDetection) {
    // Create scenario that might produce degenerate faces
    Box box_a(Vec3(0.01f, 1.0f, 1.0f)); // Very thin box
    Box box_b(Vec3(1.0f, 1.0f, 1.0f));
    
    GJK::Transform tx_a(Vec3(0, 0, 0), Quaternion::identity());
    GJK::Transform tx_b(Vec3(0.5f, 0, 0), Quaternion::identity());
    
    Simplex simplex;
    bool hit = GJK::run(box_a, tx_a, box_b, tx_b, simplex);
    
    if (hit) {
        float depth;
        Vec3 normal;
        bool epa_success = EPA::run(simplex, box_a, tx_a, box_b, tx_b, depth, normal);
        
        ASSERT_TRUE(epa_success) << "EPA should handle thin geometry";
        EXPECT_FALSE(std::isnan(depth)) << "Depth should not be NaN";
        EXPECT_FALSE(std::isnan(normal.x) || std::isnan(normal.y) || std::isnan(normal.z)) 
            << "Normal should not contain NaN";
    }
}

// ============================================================================
// Stress Tests
// ============================================================================

TEST(EPATest, RandomCollisions100Pairs) {
    srand(42); // Deterministic random
    
    int num_tests = 100;
    int successes = 0;
    
    for (int i = 0; i < num_tests; i++) {
        Box box_a(random_vec3(0.5f, 2.0f));
        Box box_b(random_vec3(0.5f, 2.0f));
        
        Vec3 pos_a = random_vec3(-5.0f, 5.0f);
        Vec3 pos_b = pos_a + random_vec3(-1.0f, 1.0f); // Likely to overlap
        
        GJK::Transform tx_a(pos_a, Quaternion::identity());
        GJK::Transform tx_b(pos_b, Quaternion::identity());
        
        Simplex simplex;
        bool hit = GJK::run(box_a, tx_a, box_b, tx_b, simplex);
        
        if (hit && simplex.size == 4) {
            float depth;
            Vec3 normal;
            bool epa_success = EPA::run(simplex, box_a, tx_a, box_b, tx_b, depth, normal);
            
            if (epa_success) {
                EXPECT_GT(depth, 0.0f);
                EXPECT_NEAR(normal.length(), 1.0f, 0.01f);
                EXPECT_FALSE(std::isnan(depth));
                successes++;
            }
        }
    }
    
    EXPECT_GT(successes, 50) << "At least 50% of random collisions should succeed";
}

TEST(EPATest, StressTest10KPairs) {
    srand(123);
    
    int num_tests = 10000;
    int successes = 0;
    int failures = 0;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_tests; i++) {
        Sphere sphere_a(random_float(0.5f, 2.0f));
        Sphere sphere_b(random_float(0.5f, 2.0f));
        
        Vec3 pos_a = random_vec3(-10.0f, 10.0f);
        Vec3 pos_b = pos_a + random_vec3(-2.0f, 2.0f);
        
        GJK::Transform tx_a(pos_a, Quaternion::identity());
        GJK::Transform tx_b(pos_b, Quaternion::identity());
        
        Simplex simplex;
        bool hit = GJK::run(sphere_a, tx_a, sphere_b, tx_b, simplex);
        
        if (hit && simplex.size == 4) {
            float depth;
            Vec3 normal;
            bool epa_success = EPA::run(simplex, sphere_a, tx_a, sphere_b, tx_b, depth, normal);
            
            if (epa_success && !std::isnan(depth)) {
                successes++;
            } else {
                failures++;
            }
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    float avg_time_us = static_cast<float>(duration.count()) / num_tests;
    
    std::cout << "EPA Stress Test Results:" << std::endl;
    std::cout << "  Total tests: " << num_tests << std::endl;
    std::cout << "  Successes: " << successes << std::endl;
    std::cout << "  Failures: " << failures << std::endl;
    std::cout << "  Average time: " << avg_time_us << " μs" << std::endl;
    
    EXPECT_GT(successes, num_tests * 0.95) << "At least 95% success rate expected";
    EXPECT_LT(avg_time_us, 100.0f) << "Average time should be < 100μs (0.1ms)";
}

// ============================================================================
// Convergence Tests
// ============================================================================

TEST(EPATest, ConvergenceWithinIterations) {
    Box box_a(Vec3(1.0f, 1.0f, 1.0f));
    Box box_b(Vec3(1.0f, 1.0f, 1.0f));
    
    GJK::Transform tx_a(Vec3(0, 0, 0), Quaternion::identity());
    GJK::Transform tx_b(Vec3(1.0f, 0, 0), Quaternion::identity());
    
    Simplex simplex;
    bool hit = GJK::run(box_a, tx_a, box_b, tx_b, simplex);
    
    ASSERT_TRUE(hit);
    
    float depth;
    Vec3 normal;
    bool epa_success = EPA::run(simplex, box_a, tx_a, box_b, tx_b, depth, normal);
    
    ASSERT_TRUE(epa_success) << "EPA should converge within max iterations";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

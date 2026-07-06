/**
 * @file test_sensor.cpp
 * @brief Unit tests for Sensor System (Open Logic Loop)
 */

#include <gtest/gtest.h>
#include "basements/physics/sensor/sensor.h"
#include "basements/physics/rigid_body.h"
#include "basements/core/math/transform.h"

using namespace basements;
using namespace basements::sensor;
using namespace basements::physics;
using namespace basements::math;

// Mock Raycaster for testing
class MockRaycaster : public IRaycaster {
public:
    bool raycast(const Vec3& origin, const Vec3& direction, float max_dist, Vec3& hit_point) override {
        // Simple mock: hit anything at x=5
        if (direction.x > 0) {
            float dist = (5.0f - origin.x) / direction.x;
            if (dist > 0 && dist < max_dist) {
                hit_point = origin + direction * dist;
                return true;
            }
        }
        return false;
    }
};

TEST(SensorTest, HierarchyTransform) {
    RigidBody body;
    body.position = Vec3(10, 0, 0); // Parent at (10,0,0)
    
    MockRaycaster raycaster;
    LidarSensor lidar("TestLidar", &body, &raycaster);
    
    // Mount lidar at (0, 1, 0) relative to parent
    lidar.local_transform.position = Vec3(0, 1, 0);
    
    Transform global = lidar.get_global_pose();
    
    EXPECT_FLOAT_EQ(global.position.x, 10.0f);
    EXPECT_FLOAT_EQ(global.position.y, 1.0f);
    EXPECT_FLOAT_EQ(global.position.z, 0.0f);
}

TEST(SensorTest, LogicLoop) {
    SensorManager manager;
    RigidBody body;
    body.position = Vec3(0, 0, 0);
    
    MockRaycaster raycaster;
    LidarSensor lidar("TestLidar", &body, &raycaster);
    lidar.rays = 4; // Check 0, 90, 180, 270 deg
    
    manager.add_sensor(&lidar);
    
    // Execute Pre-Step (Scanning)
    manager.update_pre_step(0.016f);
    
    // 0 deg (1,0,0) -> Should hit x=5. Dist=5.
    // 90 deg (0,0,1) -> x direction 0 -> No hit (x=5 plane)
    // 180 deg (-1,0,0) -> x direction -1 -> No hit
    
    // We expect at least one hit from the 0-degree ray
    bool hit_forward = false;
    for(const auto& pt : lidar.buffer) {
        if (std::abs(pt.position.x - 5.0f) < 0.01f) {
            hit_forward = true;
            EXPECT_NEAR(pt.distance, 5.0f, 0.01f);
        }
    }
    
    EXPECT_TRUE(hit_forward);
}

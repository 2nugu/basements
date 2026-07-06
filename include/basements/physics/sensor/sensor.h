/**
 * @file sensor.h
 * @brief Virtual Sensor System Interface (Open Logic Loop)
 * 
 * Allows users to implement custom sensors and logic loops that interact
 * with the physics capability without modifying the core engine.
 */

#ifndef BASEMENTS_SENSOR_SENSOR_H
#define BASEMENTS_SENSOR_SENSOR_H

#include "basements/core/math/common.h"
#include "basements/physics/rigid_body.h"
#include "basements/core/math/transform.h"
#include <vector>
#include <string>
#include <cmath>

#ifndef DEG2RAD
#define DEG2RAD (3.14159265f / 180.0f)
#endif

namespace basements {
namespace sensor {

using namespace basements::physics;
using namespace basements::math;

/**
 * @brief Base interface for all virtual sensors
 */
class ISensor {
public:
    std::string name;
    RigidBody* parent_body; // Attached body (optional)
    Transform local_transform; // Offset from parent
    
    explicit ISensor(std::string name, RigidBody* parent = nullptr)
        : name(name), parent_body(parent), local_transform(Transform::identity()) {}
        
    virtual ~ISensor() = default;
    
    /**
     * @brief Called before physics step (Sensing)
     * Use this to read state and potentially prepare control signals.
     */
    virtual void pre_step(float dt) {}
    
    /**
     * @brief Called after physics step (Validation/Logging)
     * State has been updated by solver.
     */
    virtual void post_step(float dt) {}
    
    // Helper to get global pose
    Transform get_global_pose() const {
        if (parent_body) {
            // World = Parent * Local
            // Need Transform composition. 
            // RigidBody has position/orientation.
            Transform parent_tf(parent_body->position, parent_body->orientation);
            return parent_tf * local_transform; 
        }
        return local_transform;
    }
};

/**
 * @brief Simple Raycast LiDAR Data Structure
 */
struct LidarPoint {
    Vec3 position;
    float distance;
    float intensity;
};

/**
 * @brief Abstract raycaster interface (Engine should provide impl)
 */
class IRaycaster {
public:
    virtual bool raycast(const Vec3& origin, const Vec3& direction, float max_dist, Vec3& hit_point) = 0;
};

/**
 * @brief Example implementation: LiDAR Sensor
 */
class LidarSensor : public ISensor {
public:
    float max_range;
    float fov_deg;
    int rays;
    std::vector<LidarPoint> buffer;
    IRaycaster* raycaster_backend; // Dependency Injection
    
    LidarSensor(std::string name, RigidBody* parent, IRaycaster* backend)
        : ISensor(name, parent), max_range(100.0f), fov_deg(360.0f), rays(360), raycaster_backend(backend) {}
        
    void pre_step(float dt) override {
        if (!raycaster_backend) return;
        
        buffer.clear();
        Transform pose = get_global_pose();
        
        // Naive 2D planar scan for example
        for(int i=0; i<rays; ++i) {
            float angle = (float)i * (360.0f / rays) * DEG2RAD;
            Vec3 local_dir(std::cos(angle), 0, std::sin(angle));
            Vec3 dir = pose.rotation.rotate(local_dir);
            
            Vec3 hit;
            if (raycaster_backend->raycast(pose.position, dir, max_range, hit)) {
                buffer.push_back({hit, (hit - pose.position).length(), 1.0f});
            }
        }
    }
};

/**
 * @brief Logic Loop Manager
 */
class SensorManager {
public:
    std::vector<ISensor*> sensors;
    
    void add_sensor(ISensor* s) {
        sensors.push_back(s);
    }
    
    void update_pre_step(float dt) {
        for(auto* s : sensors) s->pre_step(dt);
    }
    
    void update_post_step(float dt) {
        for(auto* s : sensors) s->post_step(dt);
    }
};

} // namespace sensor
} // namespace basements

#endif // BASEMENTS_SENSOR_SENSOR_H

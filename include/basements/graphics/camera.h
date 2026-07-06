#ifndef BASEMENTS_EDITOR_CAMERA_H
#define BASEMENTS_EDITOR_CAMERA_H

#include "basements/core/math/vec3.h"
#include <vector>

namespace basements {
namespace editor {

enum class CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

class Camera {
public:
    math::Vec3 position;
    math::Vec3 front;
    math::Vec3 up;
    math::Vec3 right;
    math::Vec3 world_up;

    float yaw;
    float pitch;
    
    float movement_speed;
    float mouse_sensitivity;
    float zoom;

    Camera(math::Vec3 position = math::Vec3(0.0f, 0.0f, 3.0f), 
           math::Vec3 up = math::Vec3(0.0f, 1.0f, 0.0f), 
           float yaw = -90.0f, float pitch = 0.0f);

    void get_view_matrix(float* mat4_out) const;
    void get_projection_matrix(float* mat4_out, float aspect_ratio) const;

    void process_keyboard(CameraMovement direction, float delta_time);
    void process_mouse_movement(float xoffset, float yoffset, bool constrain_pitch = true);
    void process_mouse_scroll(float yoffset);

    // Expose position as float pointer for interop with GLM-based code
    void get_position(float* out_xyz) const {
        out_xyz[0] = position.x; out_xyz[1] = position.y; out_xyz[2] = position.z;
    }

    void update_camera_vectors();
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_CAMERA_H

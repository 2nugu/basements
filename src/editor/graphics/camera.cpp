#include "basements/graphics/camera.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace basements {
namespace editor {

Camera::Camera(math::Vec3 position, math::Vec3 up, float yaw, float pitch)
    : front(math::Vec3(0.0f, 0.0f, -1.0f)), movement_speed(2.5f), mouse_sensitivity(0.1f), zoom(45.0f) {
    this->position = position;
    this->world_up = up;
    this->yaw = yaw;
    this->pitch = pitch;
    update_camera_vectors();
}

void Camera::get_view_matrix(float* mat4_out) const {
    glm::vec3 pos(position.x, position.y, position.z);
    glm::vec3 f(front.x, front.y, front.z);
    glm::vec3 u(up.x, up.y, up.z);
    glm::mat4 view = glm::lookAt(pos, pos + f, u);
    memcpy(mat4_out, &view[0][0], 16 * sizeof(float));
}

void Camera::get_projection_matrix(float* mat4_out, float aspect_ratio) const {
    glm::mat4 proj = glm::perspective(glm::radians(zoom), aspect_ratio, 0.1f, 100.0f);
    memcpy(mat4_out, &proj[0][0], 16 * sizeof(float));
}

void Camera::process_keyboard(CameraMovement direction, float delta_time) {
    float velocity = movement_speed * delta_time;
    if (direction == CameraMovement::FORWARD)
        position += front * velocity;
    if (direction == CameraMovement::BACKWARD)
        position -= front * velocity;
    if (direction == CameraMovement::LEFT)
        position -= right * velocity;
    if (direction == CameraMovement::RIGHT)
        position += right * velocity;
}

void Camera::process_mouse_movement(float xoffset, float yoffset, bool constrain_pitch) {
    xoffset *= mouse_sensitivity;
    yoffset *= mouse_sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    if (constrain_pitch) {
        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;
    }

    update_camera_vectors();
}

void Camera::process_mouse_scroll(float yoffset) {
    zoom -= (float)yoffset;
    if (zoom < 1.0f) zoom = 1.0f;
    if (zoom > 45.0f) zoom = 45.0f;
}

void Camera::update_camera_vectors() {
    glm::vec3 f;
    f.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    f.y = sin(glm::radians(pitch));
    f.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    f = glm::normalize(f);
    
    front = math::Vec3(f.x, f.y, f.z);
    
    glm::vec3 r = glm::normalize(glm::cross(f, glm::vec3(world_up.x, world_up.y, world_up.z)));
    right = math::Vec3(r.x, r.y, r.z);
    
    glm::vec3 u = glm::normalize(glm::cross(r, f));
    up = math::Vec3(u.x, u.y, u.z);
}

} // namespace editor
} // namespace basements

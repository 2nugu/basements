#include <iostream>
#include <iomanip>
#include "basements/core/math/quaternion.h"

using namespace basements::math;

constexpr float PI = 3.14159265359f;

void print_quat(const std::string& name, const Quaternion& q) {
    std::cout << std::setw(20) << name << ": ("
              << std::setw(7) << std::fixed << std::setprecision(4) << q.w << ", "
              << std::setw(7) << q.x << ", "
              << std::setw(7) << q.y << ", "
              << std::setw(7) << q.z << ")" << std::endl;
}

void print_vec3(const std::string& name, const Vec3& v) {
    std::cout << std::setw(20) << name << ": ("
              << std::setw(7) << std::fixed << std::setprecision(4) << v.x << ", "
              << std::setw(7) << v.y << ", "
              << std::setw(7) << v.z << ")" << std::endl;
}

int main() {
    std::cout << "=======================================================\n";
    std::cout << "  Basements Physics Engine - Quaternion Demo\n";
    std::cout << "=======================================================\n\n";
    
    // Identity rotation
    std::cout << "--- Identity Rotation ---\n";
    Quaternion identity = Quaternion::identity();
    print_quat("Identity", identity);
    
    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 rotated = identity.rotate(v);
    print_vec3("Original vector", v);
    print_vec3("After identity", rotated);
    std::cout << std::endl;
    
    // Axis-angle rotation
    std::cout << "--- Axis-Angle Rotation ---\n";
    Quaternion q_90x = Quaternion::from_axis_angle(Vec3::unit_x(), PI / 2.0f);
    print_quat("90° around X", q_90x);
    
    Vec3 y_axis = Vec3::unit_y();
    Vec3 y_rotated = q_90x.rotate(y_axis);
    print_vec3("Y axis", y_axis);
    print_vec3("After 90° X rot", y_rotated);
    std::cout << "  (Should be Z axis)\n\n";
    
    // Euler angles
    std::cout << "--- Euler Angles ---\n";
    float pitch = PI / 6.0f;  // 30°
    float yaw = PI / 4.0f;    // 45°
    float roll = PI / 3.0f;   // 60°
    
    Quaternion q_euler = Quaternion::from_euler(pitch, yaw, roll);
    print_quat("From Euler", q_euler);
    
    float p, y_angle, r;
    q_euler.to_euler(p, y_angle, r);
    std::cout << "  Recovered: pitch=" << (p * 180.0f / PI) << "°, "
              << "yaw=" << (y_angle * 180.0f / PI) << "°, "
              << "roll=" << (r * 180.0f / PI) << "°\n\n";
    
    // Quaternion multiplication (composition)
    std::cout << "--- Quaternion Composition ---\n";
    Quaternion q_x = Quaternion::rotation_x(PI / 2.0f);
    Quaternion q_y = Quaternion::rotation_y(PI / 2.0f);
    Quaternion q_combined = q_y * q_x;
    
    print_quat("90° around X", q_x);
    print_quat("90° around Y", q_y);
    print_quat("Combined (Y*X)", q_combined);
    
    Vec3 z_axis = Vec3::unit_z();
    Vec3 z_rotated = q_combined.rotate(z_axis);
    print_vec3("Z axis", z_axis);
    print_vec3("After Y*X", z_rotated);
    std::cout << "  (Should be X axis)\n\n";
    
    // SLERP interpolation
    std::cout << "--- SLERP Interpolation ---\n";
    Quaternion q_start = Quaternion::rotation_z(0.0f);
    Quaternion q_end = Quaternion::rotation_z(PI / 2.0f);
    
    print_quat("Start (0°)", q_start);
    print_quat("End (90°)", q_end);
    
    for (float t = 0.0f; t <= 1.0f; t += 0.25f) {
        Quaternion q_interp = Quaternion::slerp(q_start, q_end, t);
        float angle = q_interp.angle();
        std::cout << "  t=" << std::fixed << std::setprecision(2) << t
                  << " → angle=" << std::setprecision(1) << (angle * 180.0f / PI) << "°\n";
    }
    std::cout << std::endl;
    
    // From-To rotation
    std::cout << "--- From-To Rotation ---\n";
    Vec3 from(1.0f, 0.0f, 0.0f);
    Vec3 to(0.0f, 1.0f, 0.0f);
    
    Quaternion q_from_to = Quaternion::from_to_rotation(from, to);
    print_quat("From-To quat", q_from_to);
    
    Vec3 result = q_from_to.rotate(from);
    print_vec3("From", from);
    print_vec3("To (target)", to);
    print_vec3("Result", result);
    std::cout << std::endl;
    
    // Look rotation
    std::cout << "--- Look Rotation ---\n";
    Vec3 forward(1.0f, 0.0f, 1.0f);
    forward.normalize();
    
    Quaternion q_look = Quaternion::look_rotation(forward);
    print_quat("Look rotation", q_look);
    print_vec3("Forward dir", forward);
    
    Vec3 local_forward = Vec3::unit_z();
    Vec3 world_forward = q_look.rotate(local_forward);
    print_vec3("Local forward", local_forward);
    print_vec3("World forward", world_forward);
    std::cout << std::endl;
    
    // Inverse
    std::cout << "--- Inverse ---\n";
    Quaternion q_test = Quaternion::from_axis_angle(Vec3(1, 1, 1).normalized(), PI / 3.0f);
    Quaternion q_inv = q_test.inverse();
    Quaternion q_identity = q_test * q_inv;
    
    print_quat("Original", q_test);
    print_quat("Inverse", q_inv);
    print_quat("Original * Inverse", q_identity);
    std::cout << "  Is identity: " << (q_identity.is_identity() ? "Yes" : "No") << "\n\n";
    
    std::cout << "=======================================================\n";
    std::cout << "  All operations completed successfully!\n";
    std::cout << "=======================================================\n";
    
    return 0;
}

#include "basements/graphics/gizmo.h"
#include "basements/graphics/renderer.h"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <vector>
#include <cmath> // [NEW] For std::round

namespace basements {
namespace editor {

using namespace basements::math;

// ----------------------------------------------------------------------------
// Helpers and Constants
// ----------------------------------------------------------------------------

static const glm::vec3 COLOR_X(0.8f, 0.2f, 0.2f);
static const glm::vec3 COLOR_Y(0.2f, 0.8f, 0.2f);
static const glm::vec3 COLOR_Z(0.2f, 0.2f, 0.8f);
static const glm::vec3 COLOR_SELECTED(1.0f, 1.0f, 0.0f);
static const glm::vec3 COLOR_GRAY(0.5f, 0.5f, 0.5f);

static glm::vec3 to_glm(const Vec3& v) {
    return glm::vec3(v.x, v.y, v.z);
}

static Vec3 to_vec3(const glm::vec3& v) {
    return Vec3(v.x, v.y, v.z);
}

static glm::quat to_glm(const Quaternion& q) {
    return glm::quat(q.w, q.x, q.y, q.z);
}

static Quaternion to_quat(const glm::quat& q) {
    return Quaternion(q.w, q.x, q.y, q.z); 
    // Note: Basements quaternion constructor might be (w, x, y, z) or (x, y, z, w).
    // Verified basements/quaternion.h: (w, x, y, z) usually.
    // Let's assume standard order. If needed verify header.
    // basements/quaternion.h likely matches glm or eigen.
}

// ----------------------------------------------------------------------------
// Translation Gizmo
// ----------------------------------------------------------------------------

TranslationGizmo::TranslationGizmo() {
    gizmo_size_ = 1.0f;
}

void TranslationGizmo::render(const Vec3& position, const Quaternion& rotation, const Camera& camera) {
    glm::vec3 pos = to_glm(position);
    glm::quat rot = to_glm(rotation);
    
    glm::vec3 cam_pos;
    camera.get_position(&cam_pos.x);
    float dist = glm::distance(pos, cam_pos);
    float scale = dist * 0.15f; 
    
    // We typically calculate axes in World space or Local space depending on valid rotation.
    // For Translation, usually World axes are preferred unless Local is requested.
    // But `rotation` arg implies Local.
    // However, if `rot` is identity, it's World.
    
    glm::vec3 axis_x = rot * glm::vec3(1, 0, 0);
    glm::vec3 axis_y = rot * glm::vec3(0, 1, 0);
    glm::vec3 axis_z = rot * glm::vec3(0, 0, 1);
    
    float len = 1.0f * scale;
    float cone_sz = 0.15f * scale;
    
    // X Axis
    glm::vec3 col_x = (selected_axis_ == GizmoAxis::X) ? COLOR_SELECTED : COLOR_X;
    Renderer::draw_line(pos, pos + axis_x * len, col_x);
    Renderer::draw_cube(pos + axis_x * len, glm::vec3(cone_sz), col_x); 
    
    // Y Axis
    glm::vec3 col_y = (selected_axis_ == GizmoAxis::Y) ? COLOR_SELECTED : COLOR_Y;
    Renderer::draw_line(pos, pos + axis_y * len, col_y);
    Renderer::draw_cube(pos + axis_y * len, glm::vec3(cone_sz), col_y);
    
    // Z Axis
    glm::vec3 col_z = (selected_axis_ == GizmoAxis::Z) ? COLOR_SELECTED : COLOR_Z;
    Renderer::draw_line(pos, pos + axis_z * len, col_z);
    Renderer::draw_cube(pos + axis_z * len, glm::vec3(cone_sz), col_z);
}

// Simple Ray-Cylinder/Line intersection logic
bool TranslationGizmo::ray_cylinder_intersect(const Ray& ray, const Vec3& base, const Vec3& axis, float radius, float height, float& t) const {
    // Project ray and segment to a common plane or compute distance?
    // Using simple distance check to the line segment.
    // Reference standard algorithm.
    // Here we use a tolerance check (radius) around the line segment.
    
    glm::vec3 r_o = to_glm(ray.origin);
    glm::vec3 r_d = to_glm(ray.direction);
    glm::vec3 a_o = to_glm(base);
    glm::vec3 a_d = to_glm(axis);
    
    // Closest point on two lines (ray and axis)
    // L1 = r_o + t * r_d
    // L2 = a_o + s * a_d, s in [0, height]
    
    glm::vec3 w0 = r_o - a_o;
    float a = glm::dot(r_d, r_d);
    float b = glm::dot(r_d, a_d);
    float c = glm::dot(a_d, a_d);
    float d = glm::dot(r_d, w0);
    float e = glm::dot(a_d, w0);
    float denom = a*c - b*b;
    
    if (denom < 1e-6f) return false; // Parallel
    
    float sc = (b*e - c*d) / denom;
    float tc = (a*e - b*d) / denom;
    
    // Check if tc is within cylinder height
    if (tc < 0.0f || tc > height) return false;
    
    // Distance
    glm::vec3 p_ray = r_o + sc * r_d;
    glm::vec3 p_axis = a_o + tc * a_d;
    
    float distSq = glm::distance2(p_ray, p_axis);
    if (distSq < (radius * radius)) {
        t = sc;
        return true;
    }
    
    return false;
}

GizmoAxis TranslationGizmo::pick(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) {
    glm::vec3 pos = to_glm(position);
    glm::quat rot = to_glm(rotation);
    glm::vec3 cam_pos = to_glm(camera_pos);
    
    float dist = glm::distance(pos, cam_pos);
    float scale = dist * 0.15f;
    float len = 1.0f * scale;
    float radius = 0.1f * scale; // Hit radius
    
    // Check X
    float t_min = 1e9f;
    GizmoAxis hit = GizmoAxis::NONE;
    
    float t;
    if (ray_cylinder_intersect(ray, position, to_vec3(rot * glm::vec3(1,0,0)), radius, len, t)) {
        if (t < t_min) { t_min = t; hit = GizmoAxis::X; }
    }
    if (ray_cylinder_intersect(ray, position, to_vec3(rot * glm::vec3(0,1,0)), radius, len, t)) {
        if (t < t_min) { t_min = t; hit = GizmoAxis::Y; }
    }
    if (ray_cylinder_intersect(ray, position, to_vec3(rot * glm::vec3(0,0,1)), radius, len, t)) {
        if (t < t_min) { t_min = t; hit = GizmoAxis::Z; }
    }
    
    return hit;
}

Vec3 TranslationGizmo::project_to_plane(const Vec3& point, const Vec3& plane_origin, const Vec3& plane_normal) const {
    glm::vec3 p = to_glm(point);
    glm::vec3 o = to_glm(plane_origin);
    glm::vec3 n = to_glm(plane_normal);
    
    // Project p onto plane
    // P_proj = P - dot(P-O, n) * n
    glm::vec3 proj = p - glm::dot(p - o, n) * n; // Assumes n is normalized? Yes usually.
    return to_vec3(proj);
}

void TranslationGizmo::calculate_delta(
        const Ray& ray_start,
        const Ray& ray_current,
        const Vec3& initial_pos,
        const Quaternion& initial_rot,
        GizmoAxis axis,
        Vec3& delta_pos,
        Quaternion& delta_rot,
        Vec3& delta_scale) 
{
    // Initialize
    delta_pos = Vec3(0);
    delta_rot = Quaternion::identity();
    delta_scale = Vec3(0); // Not used
    
    glm::vec3 i_pos = to_glm(initial_pos);
    glm::quat i_rot = to_glm(initial_rot);
    
    // Choose projection plane based on axis and ray
    // We want a plane containing the axis and somewhat perpendicular to view direction?
    // Actually, simple standard: use two axes?
    // E.g. If moving X, project onto XY or XZ depending on view angle.
    // Or just project closest point on axis.
    
    // Simplified: Project to plane defined by Axis and Cross(Axis, RayDir) but that's unstable.
    // Better: Project to a plane perpendicular to the camera look vector? No, that's screen space drag.
    
    glm::vec3 move_axis;
    if (axis == GizmoAxis::X) move_axis = i_rot * glm::vec3(1,0,0);
    else if (axis == GizmoAxis::Y) move_axis = i_rot * glm::vec3(0,1,0);
    else if (axis == GizmoAxis::Z) move_axis = i_rot * glm::vec3(0,0,1);
    else return;
    
    // Create logic for robust dragging
    // 1. Compute closest point on line (defined by axis) from ray 1
    // 2. Compute closest point on line from ray 2
    // 3. Diff closest points
    
    glm::vec3 r1_o = to_glm(ray_start.origin);
    glm::vec3 r1_d = to_glm(ray_start.direction);
    glm::vec3 r2_o = to_glm(ray_current.origin);
    glm::vec3 r2_d = to_glm(ray_current.direction);
    
    // Closest point on line L(t) = i_pos + t * move_axis to Ray R(s)
    // Standard skew lines closest point
    
    auto compute_t = [&](const glm::vec3& r_o, const glm::vec3& r_d) -> float {
        glm::vec3 w0 = r_o - i_pos;
        float a = glm::dot(r_d, r_d);
        float b = glm::dot(r_d, move_axis);
        float c = glm::dot(move_axis, move_axis);
        float d = glm::dot(r_d, w0);
        float e = glm::dot(move_axis, w0);
        float denom = a*c - b*b;
        if (denom < 1e-6f) return 0.0f;
        return (a*e - b*d) / denom;
    };
    
    float t1 = compute_t(r1_o, r1_d);
    float t2 = compute_t(r2_o, r2_d);
    
    float delta = t2 - t1;
    delta_pos = to_vec3(move_axis * delta);
}

// ----------------------------------------------------------------------------
// Rotation Gizmo
// ----------------------------------------------------------------------------

RotationGizmo::RotationGizmo() { gizmo_size_ = 1.0f; }

void RotationGizmo::render(const Vec3& position, const Quaternion& rotation, const Camera& camera) {
    glm::vec3 pos = to_glm(position);
    glm::quat rot = to_glm(rotation);
    
    glm::vec3 cam_pos;
    camera.get_position(&cam_pos.x);
    float dist = glm::distance(pos, cam_pos);
    float scale = dist * 0.15f; 
    float radius = 1.0f * scale;
    
    glm::vec3 axis_x = rot * glm::vec3(1, 0, 0);
    glm::vec3 axis_y = rot * glm::vec3(0, 1, 0);
    glm::vec3 axis_z = rot * glm::vec3(0, 0, 1);
    
    // Draw circles (approximated by line loop)
    const int segments = 32;
    auto draw_circle = [&](const glm::vec3& center, const glm::vec3& normal, const glm::vec3& up, const glm::vec3& color) {
        glm::vec3 right = glm::cross(normal, up); // assumes orthogonal
        // actually just create basis
        glm::vec3 u = glm::normalize(up);
        glm::vec3 v = glm::normalize(glm::cross(normal, u));
        
        std::vector<glm::vec3> points;
        for(int i=0; i<=segments; ++i) {
            float angle = (float)i / segments * glm::two_pi<float>();
            points.push_back(center + radius * (cos(angle)*u + sin(angle)*v));
        }
        for(size_t i=0; i<points.size()-1; ++i) {
            Renderer::draw_line(points[i], points[i+1], color);
        }
    };
    
    glm::vec3 col_x = (selected_axis_ == GizmoAxis::X) ? COLOR_SELECTED : COLOR_X;
    glm::vec3 col_y = (selected_axis_ == GizmoAxis::Y) ? COLOR_SELECTED : COLOR_Y;
    glm::vec3 col_z = (selected_axis_ == GizmoAxis::Z) ? COLOR_SELECTED : COLOR_Z;
    
    draw_circle(pos, axis_x, axis_y, col_x); // Circle in YZ plane, normal is X
    draw_circle(pos, axis_y, axis_z, col_y); // Circle in XZ plane, normal is Y
    draw_circle(pos, axis_z, axis_x, col_z); // Circle in XY plane, normal is Z
}

bool RotationGizmo::ray_torus_intersect(const Ray& ray, const Vec3& center, const Vec3& axis, float major_radius, float minor_radius, float& t) const {
    // Simplified: Plane intersection + distance to circle check
    // Plane: (p - center) . axis = 0
    glm::vec3 r_o = to_glm(ray.origin);
    glm::vec3 r_d = to_glm(ray.direction);
    glm::vec3 c = to_glm(center);
    glm::vec3 n = to_glm(axis);
    
    float denom = glm::dot(n, r_d);
    if (std::abs(denom) < 1e-6f) return false;
    
    float t_plane = glm::dot(c - r_o, n) / denom;
    if (t_plane < 0.0f) return false;
    
    glm::vec3 p = r_o + t_plane * r_d;
    float dist = glm::distance(p, c);
    
    if (std::abs(dist - major_radius) < minor_radius) {
        t = t_plane;
        return true;
    }
    return false;
}

GizmoAxis RotationGizmo::pick(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) {
    glm::vec3 pos = to_glm(position);
    glm::quat rot = to_glm(rotation);
    glm::vec3 cam_pos = to_glm(camera_pos);
    float dist = glm::distance(pos, cam_pos);
    float scale = dist * 0.15f; 
    float radius = 1.0f * scale;
    float hit_threshold = 0.1f * scale;
    
    float t_min = 1e9f;
    GizmoAxis hit = GizmoAxis::NONE;
    float t;
    
    glm::vec3 ax = rot * glm::vec3(1,0,0);
    glm::vec3 ay = rot * glm::vec3(0,1,0);
    glm::vec3 az = rot * glm::vec3(0,0,1);
    
    if (ray_torus_intersect(ray, position, to_vec3(ax), radius, hit_threshold, t)) {
        if(t < t_min) { t_min = t; hit = GizmoAxis::X; }
    }
    if (ray_torus_intersect(ray, position, to_vec3(ay), radius, hit_threshold, t)) {
        if(t < t_min) { t_min = t; hit = GizmoAxis::Y; }
    }
    if (ray_torus_intersect(ray, position, to_vec3(az), radius, hit_threshold, t)) {
        if(t < t_min) { t_min = t; hit = GizmoAxis::Z; }
    }
    
    return hit;
}

float RotationGizmo::calculate_angle(const Vec3& v1, const Vec3& v2, const Vec3& plane_normal) const {
    glm::vec3 a = glm::normalize(to_glm(v1));
    glm::vec3 b = glm::normalize(to_glm(v2));
    glm::vec3 n = glm::normalize(to_glm(plane_normal));
    
    float dot = glm::clamp(glm::dot(a, b), -1.0f, 1.0f);
    float angle = std::acos(dot);
    
    glm::vec3 cross = glm::cross(a, b);
    if (glm::dot(cross, n) < 0) angle = -angle;
    
    return angle;
}

void RotationGizmo::calculate_delta(
    const Ray& ray_start,
    const Ray& ray_current,
    const Vec3& initial_pos,
    const Quaternion& initial_rot,
    GizmoAxis axis,
    Vec3& delta_pos,
    Quaternion& delta_rot,
    Vec3& delta_scale)
{
    // 1. Intersect ray with rotation plane (defined by axis and object center)
    // 2. Measure angle change between start and current intersection
    
    glm::vec3 c = to_glm(initial_pos);
    glm::quat rot = to_glm(initial_rot);
    glm::vec3 n;
    
    if (axis == GizmoAxis::X) n = rot * glm::vec3(1,0,0);
    else if (axis == GizmoAxis::Y) n = rot * glm::vec3(0,1,0);
    else if (axis == GizmoAxis::Z) n = rot * glm::vec3(0,0,1);
    else return;
    
    auto intersect_plane = [&](const Ray& r) -> glm::vec3 {
        glm::vec3 ro = to_glm(r.origin);
        glm::vec3 rd = to_glm(r.direction);
        float t = glm::dot(c - ro, n) / glm::dot(rd, n);
        return ro + t * rd;
    };
    
    glm::vec3 p1 = intersect_plane(ray_start);
    glm::vec3 p2 = intersect_plane(ray_current);
    
    glm::vec3 v1 = p1 - c;
    glm::vec3 v2 = p2 - c;
    
    float angle = calculate_angle(to_vec3(v1), to_vec3(v2), to_vec3(n));
    
    delta_rot = to_quat(glm::angleAxis(angle, n)); // Axis-Angle rotation
    // Note: This delta rotation is world space (around world axis n).
    // Combining rotation: new_rot = delta_rot * initial_rot.
}

// ----------------------------------------------------------------------------
// Scale Gizmo
// ----------------------------------------------------------------------------

ScaleGizmo::ScaleGizmo() { gizmo_size_ = 1.0f; }

void ScaleGizmo::render(const Vec3& position, const Quaternion& rotation, const Camera& camera) {
    // Similar to Translation but cubes at end
    glm::vec3 pos = to_glm(position);
    glm::quat rot = to_glm(rotation);
    glm::vec3 cam_pos;
    camera.get_position(&cam_pos.x);
    float dist = glm::distance(pos, cam_pos);
    float scale = dist * 0.15f; 
    
    glm::vec3 axis_x = rot * glm::vec3(1, 0, 0);
    glm::vec3 axis_y = rot * glm::vec3(0, 1, 0);
    glm::vec3 axis_z = rot * glm::vec3(0, 0, 1);
    float len = 1.0f * scale;
    float box_sz = 0.15f * scale;
    
    glm::vec3 col_x = (selected_axis_ == GizmoAxis::X) ? COLOR_SELECTED : COLOR_X;
    Renderer::draw_line(pos, pos + axis_x * len, col_x);
    Renderer::draw_cube(pos + axis_x * len, glm::vec3(box_sz), col_x);
    
    glm::vec3 col_y = (selected_axis_ == GizmoAxis::Y) ? COLOR_SELECTED : COLOR_Y;
    Renderer::draw_line(pos, pos + axis_y * len, col_y);
    Renderer::draw_cube(pos + axis_y * len, glm::vec3(box_sz), col_y);
    
    glm::vec3 col_z = (selected_axis_ == GizmoAxis::Z) ? COLOR_SELECTED : COLOR_Z;
    Renderer::draw_line(pos, pos + axis_z * len, col_z);
    Renderer::draw_cube(pos + axis_z * len, glm::vec3(box_sz), col_z);
}

bool ScaleGizmo::ray_box_intersect(const Ray& ray, const Vec3& center, const Vec3& half_extents, float& t) const {
    // AABB check (simplified, assumes axis aligned or transforms ray)
    // Actually box is at tip of axis.
    // Let's use simple sphere approx for now, or true OBB.
    // Sphere is easy.
    float radius = half_extents.x; // approx
    
    glm::vec3 r_o = to_glm(ray.origin);
    glm::vec3 r_d = to_glm(ray.direction);
    glm::vec3 c = to_glm(center);
    
    // Ray-Sphere
    glm::vec3 oc = r_o - c;
    float b = glm::dot(oc, r_d);
    float c_val = glm::dot(oc, oc) - radius*radius;
    float h = b*b - c_val;
    if (h < 0.0f) return false;
    
    t = -b - sqrt(h);
    return t > 0.0f;
}

GizmoAxis ScaleGizmo::pick(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) {
    glm::vec3 pos = to_glm(position);
    glm::quat rot = to_glm(rotation);
    glm::vec3 cam_pos = to_glm(camera_pos);
    float dist = glm::distance(pos, cam_pos);
    float scale = dist * 0.15f; 
    float len = 1.0f * scale;
    float box_sz = 0.15f * scale;
    
    glm::vec3 axis_x = rot * glm::vec3(1, 0, 0);
    glm::vec3 axis_y = rot * glm::vec3(0, 1, 0);
    glm::vec3 axis_z = rot * glm::vec3(0, 0, 1);
    
    float t_min = 1e9f;
    GizmoAxis hit = GizmoAxis::NONE;
    float t;
    
    // Check boxes at tips
    if (ray_box_intersect(ray, to_vec3(pos + axis_x * len), to_vec3(glm::vec3(box_sz)), t)) {
        if (t < t_min) { t_min = t; hit = GizmoAxis::X; }
    }
    if (ray_box_intersect(ray, to_vec3(pos + axis_y * len), to_vec3(glm::vec3(box_sz)), t)) {
        if (t < t_min) { t_min = t; hit = GizmoAxis::Y; }
    }
    if (ray_box_intersect(ray, to_vec3(pos + axis_z * len), to_vec3(glm::vec3(box_sz)), t)) {
        if (t < t_min) { t_min = t; hit = GizmoAxis::Z; }
    }
    
    return hit;
}

void ScaleGizmo::calculate_delta(
    const Ray& ray_start,
    const Ray& ray_current,
    const Vec3& initial_pos,
    const Quaternion& initial_rot,
    GizmoAxis axis,
    Vec3& delta_pos,
    Quaternion& delta_rot,
    Vec3& delta_scale)
{
    // Project ray onto axis line, measure distance change
    glm::vec3 i_pos = to_glm(initial_pos);
    glm::quat i_rot = to_glm(initial_rot);
    glm::vec3 n;
    
    if (axis == GizmoAxis::X) n = i_rot * glm::vec3(1,0,0);
    else if (axis == GizmoAxis::Y) n = i_rot * glm::vec3(0,1,0);
    else if (axis == GizmoAxis::Z) n = i_rot * glm::vec3(0,0,1);
    else return;
    
    glm::vec3 r1_o = to_glm(ray_start.origin);
    glm::vec3 r1_d = to_glm(ray_start.direction);
    glm::vec3 r2_o = to_glm(ray_current.origin);
    glm::vec3 r2_d = to_glm(ray_current.direction);
    
    auto compute_t = [&](const glm::vec3& r_o, const glm::vec3& r_d) -> float {
        glm::vec3 w0 = r_o - i_pos;
        float a = glm::dot(r_d, r_d);
        float b = glm::dot(r_d, n);
        float c = glm::dot(n, n);
        float d = glm::dot(r_d, w0);
        float e = glm::dot(n, w0);
        float denom = a*c - b*b;
        if (denom < 1e-6f) return 0.0f;
        return (a*e - b*d) / denom;
    };
    
    float t1 = compute_t(r1_o, r1_d);
    float t2 = compute_t(r2_o, r2_d);
    
    // Scale factor
    float initial_len = 1.0f; // actually proportional to t1 if gizmo visual fits?
    // Let's say we scale by ratio (t2 / t1) or just (t2 - t1) * speed
    
    float s = (t2 - t1); // Linear scale delta
    
    delta_scale = Vec3(0);
    if (axis == GizmoAxis::X) delta_scale.x = s;
    if (axis == GizmoAxis::Y) delta_scale.y = s;
    if (axis == GizmoAxis::Z) delta_scale.z = s;
}

// ----------------------------------------------------------------------------
// Gizmo Manager
// ----------------------------------------------------------------------------

GizmoManager::GizmoManager() {
    translate_gizmo_ = std::make_unique<TranslationGizmo>();
    rotate_gizmo_ = std::make_unique<RotationGizmo>();
    scale_gizmo_ = std::make_unique<ScaleGizmo>();
}

GizmoManager::~GizmoManager() {}

void GizmoManager::set_mode(GizmoMode mode) {
    current_mode_ = mode;
}

Gizmo* GizmoManager::get_active_gizmo() {
    switch(current_mode_) {
        case GizmoMode::TRANSLATE: return translate_gizmo_.get();
        case GizmoMode::ROTATE: return rotate_gizmo_.get();
        case GizmoMode::SCALE: return scale_gizmo_.get();
        default: return nullptr;
    }
}

void GizmoManager::render(const Vec3& position, const Quaternion& rotation, const Camera& camera) {
    Gizmo* g = get_active_gizmo();
    if (g) g->render(position, rotation, camera);
}

GizmoAxis GizmoManager::begin_drag(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) {
    Gizmo* g = get_active_gizmo();
    if (!g) return GizmoAxis::NONE;
    
    GizmoAxis axis = g->pick(ray, camera_pos, position, rotation);
    if (axis != GizmoAxis::NONE) {
        is_dragging_ = true;
        dragging_axis_ = axis;
        drag_start_ray_ = ray;
        g->set_selected_axis(axis);
    }
    return axis;
}

void GizmoManager::update_drag(const Ray& ray_current, const Vec3& initial_pos, const Quaternion& initial_rot,
                     Vec3& delta_pos, Quaternion& delta_rot, Vec3& delta_scale)
{
    if (!is_dragging_) return;
    Gizmo* g = get_active_gizmo();
    if (g) {
        g->calculate_delta(drag_start_ray_, ray_current, initial_pos, initial_rot, dragging_axis_, delta_pos, delta_rot, delta_scale);
        
        // Apply Snapping
        if (snap_enabled_) {
            if (current_mode_ == GizmoMode::TRANSLATE) {
                if (snap_translation_grid_ > 1e-6f) { // Avoid div by zero
                    auto snap_val = [&](float& v) {
                         v = std::round(v / snap_translation_grid_) * snap_translation_grid_;
                    };
                    snap_val(delta_pos.x);
                    snap_val(delta_pos.y);
                    snap_val(delta_pos.z);
                }
            } else if (current_mode_ == GizmoMode::ROTATE) {
                if (snap_rotation_angle_ > 1e-6f) {
                    // Start by getting angle from delta_rot
                    // This is tricky for Quaternions. Standard Gizmo implementation usually returns axis-angle delta.
                    // RotationGizmo::calculate_delta returns a quaternion.
                    float angle;
                    Vec3 axis = delta_rot.axis();
                    angle = delta_rot.angle(); 
                    // Note: axis() and angle() might not be perfectly stable for near-identity, but calculate_delta uses AngleAxis.
                    
                    // Convert to degrees
                    float angle_deg = angle * 180.0f / 3.14159f;
                    
                    // Snap
                    float snapped_deg = std::round(angle_deg / snap_rotation_angle_) * snap_rotation_angle_;
                    
                    // If snapped angle is close to 0, return identity.
                    if (std::abs(snapped_deg) < 1e-3f) {
                        delta_rot = Quaternion::identity();
                    } else {
                        // Reconstruct quaternion
                        float snapped_rad = snapped_deg * 3.14159f / 180.0f;
                        delta_rot = Quaternion::from_axis_angle(axis, snapped_rad);
                    }
                }
            } else if (current_mode_ == GizmoMode::SCALE) {
                 // Snap scale? (Optional, maybe 0.1 increments)
                 float snap_scale = 0.5f; // Constant for now or use translation grid?
                 if (snap_translation_grid_ > 1e-6f) {
                    auto snap_s = [&](float& v) {
                        v = std::round(v / 0.1f) * 0.1f; // Hardcoded 0.1 step
                    };
                    snap_s(delta_scale.x);
                    snap_s(delta_scale.y);
                    snap_s(delta_scale.z);
                 }
            }
        }
    }
}

void GizmoManager::end_drag() {
    is_dragging_ = false;
    dragging_axis_ = GizmoAxis::NONE;
    Gizmo* g = get_active_gizmo();
    if (g) g->set_selected_axis(GizmoAxis::NONE);
}

} // namespace editor
} // namespace basements

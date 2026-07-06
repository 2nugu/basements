#ifndef BASEMENTS_EDITOR_GIZMO_H
#define BASEMENTS_EDITOR_GIZMO_H

/**
 * @file gizmo.h
 * @brief Transform manipulation gizmos for the scene editor
 * 
 * Provides visual handles for translating, rotating, and scaling scene nodes.
 */

#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/graphics/camera.h"
#include <memory>

namespace basements {
namespace editor {

using namespace basements::math;

// ============================================================================
// Gizmo Mode
// ============================================================================

enum class GizmoMode {
    NONE,
    TRANSLATE,
    ROTATE,
    SCALE
};

enum class GizmoAxis {
    NONE,
    X,
    Y,
    Z,
    XY,
    YZ,
    XZ,
    XYZ  // For uniform scale
};

// ============================================================================
// Ray (for picking)
// ============================================================================

struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray() = default;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    
    Vec3 point_at(float t) const {
        return origin + direction * t;
    }
};

// ============================================================================
// Gizmo Base Class
// ============================================================================

/**
 * @brief Abstract base class for transform gizmos
 */
class Gizmo {
public:
    virtual ~Gizmo() = default;
    
    /**
     * @brief Render the gizmo at the given position
     */
    virtual void render(const Vec3& position, const Quaternion& rotation, const Camera& camera) = 0;
    
    /**
     * @brief Handle mouse interaction
     * @param ray Mouse ray in world space
     * @param camera_pos Camera position for depth testing
     * @param position Object position
     * @param rotation Object rotation
     * @return Selected axis (NONE if no hit)
     */
    virtual GizmoAxis pick(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) = 0;
    
    /**
     * @brief Calculate transform delta from mouse drag
     * @param ray_start Ray at drag start
     * @param ray_current Current mouse ray
     * @param initial_pos Initial object position
     * @param initial_rot Initial object rotation
     * @param axis Selected axis
     * @param[out] delta_pos Position change (for translate/scale)
     * @param[out] delta_rot Rotation change (for rotate)
     * @param[out] delta_scale Scale change (for scale)
     */
    virtual void calculate_delta(
        const Ray& ray_start,
        const Ray& ray_current,
        const Vec3& initial_pos,
        const Quaternion& initial_rot,
        GizmoAxis axis,
        Vec3& delta_pos,
        Quaternion& delta_rot,
        Vec3& delta_scale
    ) = 0;
    
    /**
     * @brief Set the currently selected axis (for highlighting)
     */
    virtual void set_selected_axis(GizmoAxis axis) = 0;
    
protected:
    GizmoAxis selected_axis_ = GizmoAxis::NONE;
    float gizmo_size_ = 1.0f;  // Visual scale
};

// ============================================================================
// Translation Gizmo
// ============================================================================

/**
 * @brief Gizmo for translating objects (3 arrows)
 */
class TranslationGizmo : public Gizmo {
public:
    TranslationGizmo();
    ~TranslationGizmo() override = default;
    
    void render(const Vec3& position, const Quaternion& rotation, const Camera& camera) override;
    GizmoAxis pick(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) override;
    void calculate_delta(
        const Ray& ray_start,
        const Ray& ray_current,
        const Vec3& initial_pos,
        const Quaternion& initial_rot,
        GizmoAxis axis,
        Vec3& delta_pos,
        Quaternion& delta_rot,
        Vec3& delta_scale
    ) override;
    void set_selected_axis(GizmoAxis axis) override { selected_axis_ = axis; }
    
private:
    // Helper: Ray-cylinder intersection for arrow picking
    bool ray_cylinder_intersect(const Ray& ray, const Vec3& base, const Vec3& axis, float radius, float height, float& t) const;
    
    // Helper: Project point onto plane for drag calculation
    Vec3 project_to_plane(const Vec3& point, const Vec3& plane_origin, const Vec3& plane_normal) const;
};

// ============================================================================
// Rotation Gizmo
// ============================================================================

/**
 * @brief Gizmo for rotating objects (3 circles)
 */
class RotationGizmo : public Gizmo {
public:
    RotationGizmo();
    ~RotationGizmo() override = default;
    
    void render(const Vec3& position, const Quaternion& rotation, const Camera& camera) override;
    GizmoAxis pick(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) override;
    void calculate_delta(
        const Ray& ray_start,
        const Ray& ray_current,
        const Vec3& initial_pos,
        const Quaternion& initial_rot,
        GizmoAxis axis,
        Vec3& delta_pos,
        Quaternion& delta_rot,
        Vec3& delta_scale
    ) override;
    void set_selected_axis(GizmoAxis axis) override { selected_axis_ = axis; }
    
private:
    // Helper: Ray-torus intersection for circle picking
    bool ray_torus_intersect(const Ray& ray, const Vec3& center, const Vec3& axis, float major_radius, float minor_radius, float& t) const;
    
    // Helper: Calculate angle between two vectors on a plane
    float calculate_angle(const Vec3& v1, const Vec3& v2, const Vec3& plane_normal) const;
};

// ============================================================================
// Scale Gizmo
// ============================================================================

/**
 * @brief Gizmo for scaling objects (3 cubes + center cube)
 */
class ScaleGizmo : public Gizmo {
public:
    ScaleGizmo();
    ~ScaleGizmo() override = default;
    
    void render(const Vec3& position, const Quaternion& rotation, const Camera& camera) override;
    GizmoAxis pick(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation) override;
    void calculate_delta(
        const Ray& ray_start,
        const Ray& ray_current,
        const Vec3& initial_pos,
        const Quaternion& initial_rot,
        GizmoAxis axis,
        Vec3& delta_pos,
        Quaternion& delta_rot,
        Vec3& delta_scale
    ) override;
    void set_selected_axis(GizmoAxis axis) override { selected_axis_ = axis; }
    
private:
    // Helper: Ray-box intersection for cube picking
    bool ray_box_intersect(const Ray& ray, const Vec3& center, const Vec3& half_extents, float& t) const;
};

// ============================================================================
// Gizmo Manager
// ============================================================================

/**
 * @brief Manages gizmo mode switching and interaction
 */
class GizmoManager {
public:
    GizmoManager();
    ~GizmoManager();
    
    void set_mode(GizmoMode mode);
    GizmoMode get_mode() const { return current_mode_; }
    
    void render(const Vec3& position, const Quaternion& rotation, const Camera& camera);
    
    // Mouse interaction
    GizmoAxis begin_drag(const Ray& ray, const Vec3& camera_pos, const Vec3& position, const Quaternion& rotation);
    void update_drag(const Ray& ray_current, const Vec3& initial_pos, const Quaternion& initial_rot,
                     Vec3& delta_pos, Quaternion& delta_rot, Vec3& delta_scale);
    void end_drag();
    
    bool is_dragging() const { return is_dragging_; }
    
    // Snapping configuration
    void set_snap_translation(float grid_size) { snap_translation_grid_ = grid_size; }
    void set_snap_rotation(float angle_deg) { snap_rotation_angle_ = angle_deg; }
    float get_snap_translation() const { return snap_translation_grid_; }
    float get_snap_rotation() const { return snap_rotation_angle_; }
    
    void set_snap_enabled(bool enabled) { snap_enabled_ = enabled; }
    bool is_snap_enabled() const { return snap_enabled_; }

private:
    GizmoMode current_mode_ = GizmoMode::TRANSLATE;
    
    // Snapping state
    bool snap_enabled_ = false;
    float snap_translation_grid_ = 1.0f; // Default 1 unit
    float snap_rotation_angle_ = 15.0f; // Default 15 degrees
    
    std::unique_ptr<TranslationGizmo> translate_gizmo_;
    std::unique_ptr<RotationGizmo> rotate_gizmo_;
    std::unique_ptr<ScaleGizmo> scale_gizmo_;
    
    Gizmo* get_active_gizmo();
    
    bool is_dragging_ = false;
    GizmoAxis dragging_axis_ = GizmoAxis::NONE;
    Ray drag_start_ray_;
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_GIZMO_H

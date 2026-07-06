// test_mpm_rigid_cylinder.cpp — Cylinder primitive correctness under rotation.
//
// Round 12 found a trade-off between M4.5a (high support, high slip) and
// M4.5b (high traction, deep sinkage) on the rover_wheel scenario. Before
// trusting that as a scientific finding we need to rule out that the
// cylinder's body-local inside-check / outward-normal helpers misbehave
// when the cylinder is rotated 90° about world X (the rolling configuration
// used by ScenarioRoverWheel::WheelMode::ROLL).
//
// These tests pin down the rotation behaviour for future regressions.

#include <gtest/gtest.h>

#include "basements/physics/coupling/mpm_rigid_coupler.h"

using namespace basements::math;
using namespace basements::physics::coupling;

namespace {

constexpr float kPi = 3.14159265358979f;

RigidColliderState make_cylinder(const Vec3& center, const Quaternion& q,
                                 float r, float L) {
    RigidColliderState c;
    c.center       = center;
    c.orientation  = q;
    c.half_extents = Vec3(r, L, r);
    c.shape        = ShapeKind::CYLINDER;
    c.mass         = 1.0f;
    return c;
}

}  // namespace

// ─── Identity: axis along local Y == world Y. ──────────────────────────────
TEST(MpmRigidCylinder, UprightCenterIsInside) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    // Point at body centre, expressed already in local frame.
    Vec3 local = Vec3::zero();
    EXPECT_TRUE(shape_inside(c, local));
}

TEST(MpmRigidCylinder, UprightOutsideRadius) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    Vec3 local(0.20f, 0.0f, 0.0f);   // outside radius
    EXPECT_FALSE(shape_inside(c, local));
}

TEST(MpmRigidCylinder, UprightOutsideAxialClipping) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    Vec3 local(0.0f, 0.10f, 0.0f);   // past +Y cap
    EXPECT_FALSE(shape_inside(c, local));
}

TEST(MpmRigidCylinder, UprightSideOutwardNormalRadial) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    // local(0.10, 0, 0) hits the axial=radial boundary exactly — floating
    // tie-breaking is implementation-defined. Move closer to the side wall.
    Vec3 local(0.14f, 0.0f, 0.0f);
    Vec3 n = shape_outward_normal_local(c, local);
    EXPECT_NEAR(n.x, 1.0f, 1e-4f);
    EXPECT_NEAR(n.y, 0.0f, 1e-4f);
    EXPECT_NEAR(n.z, 0.0f, 1e-4f);
}

TEST(MpmRigidCylinder, UprightCapOutwardNormalAxial) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    Vec3 local(0.02f, 0.045f, 0.0f);   // close to +Y cap
    Vec3 n = shape_outward_normal_local(c, local);
    EXPECT_NEAR(n.x, 0.0f, 1e-4f);
    EXPECT_NEAR(n.y, 1.0f, 1e-4f);
    EXPECT_NEAR(n.z, 0.0f, 1e-4f);
}

// ─── ROLL configuration: orientation = 90° about world X axis. ─────────────
//   Local Y → World Z.   Cylinder axis is along world Z (a wheel rolling
//   along world X). World query points must be transformed by R_w2l = Rᵀ
//   before being handed to shape_inside.
TEST(MpmRigidCylinder, RolledConfigInsideAtCenter) {
    Quaternion q = Quaternion::from_axis_angle(Vec3(1, 0, 0), kPi * 0.5f);
    auto c = make_cylinder(Vec3::zero(), q, 0.15f, 0.05f);

    Matrix3 R_l2w = Matrix3::from_quaternion(q);
    Matrix3 R_w2l = R_l2w.transposed();
    Vec3 world_pt(0.0f, 0.0f, 0.0f);
    Vec3 local = R_w2l * (world_pt - c.center);
    EXPECT_TRUE(shape_inside(c, local));
}

TEST(MpmRigidCylinder, RolledConfigOutsideAlongWorldZAxis) {
    // After 90° rotation about X, the cylinder axis is world +Z. A point at
    // +0.20 world Z is *past* the cap (only 0.05 m half-length).
    Quaternion q = Quaternion::from_axis_angle(Vec3(1, 0, 0), kPi * 0.5f);
    auto c = make_cylinder(Vec3::zero(), q, 0.15f, 0.05f);

    Matrix3 R_w2l = Matrix3::from_quaternion(q).transposed();
    Vec3 world_pt(0.0f, 0.0f, 0.20f);
    Vec3 local = R_w2l * (world_pt - c.center);
    EXPECT_FALSE(shape_inside(c, local));
}

TEST(MpmRigidCylinder, RolledConfigInsideAlongWorldXAxis) {
    // World +X is *radial* (perpendicular to axis) in this orientation; up to
    // 0.15 m should still be inside.
    Quaternion q = Quaternion::from_axis_angle(Vec3(1, 0, 0), kPi * 0.5f);
    auto c = make_cylinder(Vec3::zero(), q, 0.15f, 0.05f);

    Matrix3 R_w2l = Matrix3::from_quaternion(q).transposed();
    Vec3 world_pt(0.10f, 0.0f, 0.0f);
    Vec3 local = R_w2l * (world_pt - c.center);
    EXPECT_TRUE(shape_inside(c, local));
}

TEST(MpmRigidCylinder, RolledConfigNormalIsRadialInBodyFrame) {
    // A point near the side wall (world X = 0.12) in rolled orientation: in
    // body-local frame the local x = 0.12, local y = 0 (axial), local z = 0.
    // Closest face is the radial side, so outward normal in body-local frame
    // is (+1, 0, 0). After rotating to world we expect (+1, 0, 0) as well
    // (rotation about X leaves x unchanged).
    Quaternion q = Quaternion::from_axis_angle(Vec3(1, 0, 0), kPi * 0.5f);
    auto c = make_cylinder(Vec3::zero(), q, 0.15f, 0.05f);

    Matrix3 R_l2w = Matrix3::from_quaternion(q);
    Matrix3 R_w2l = R_l2w.transposed();
    Vec3 world_pt(0.12f, 0.0f, 0.0f);
    Vec3 local = R_w2l * (world_pt - c.center);

    Vec3 n_local = shape_outward_normal_local(c, local);
    Vec3 n_world = R_l2w * n_local;

    EXPECT_NEAR(n_world.x, 1.0f, 1e-4f);
    EXPECT_NEAR(n_world.y, 0.0f, 1e-4f);
    EXPECT_NEAR(n_world.z, 0.0f, 1e-4f);
}

// ─── AABB tight bound ───────────────────────────────────────────────────────
TEST(MpmRigidCylinder, AabbAxisAligned) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    Matrix3 R = Matrix3::from_quaternion(c.orientation);
    Vec3 h = shape_world_aabb_half(c, R);
    EXPECT_NEAR(h.x, 0.15f, 1e-4f);   // radius along X
    EXPECT_NEAR(h.y, 0.05f, 1e-4f);   // half-length along Y (the axis)
    EXPECT_NEAR(h.z, 0.15f, 1e-4f);   // radius along Z
}

TEST(MpmRigidCylinder, AabbRolled90AboutX) {
    Quaternion q = Quaternion::from_axis_angle(Vec3(1, 0, 0), kPi * 0.5f);
    auto c = make_cylinder(Vec3::zero(), q, 0.15f, 0.05f);
    Matrix3 R = Matrix3::from_quaternion(q);
    Vec3 h = shape_world_aabb_half(c, R);
    // After rotation, axis is world +Z (so half-extent along Z = L), and X/Y
    // are now both radial (= R).
    EXPECT_NEAR(h.x, 0.15f, 1e-4f);
    EXPECT_NEAR(h.y, 0.15f, 1e-4f);
    EXPECT_NEAR(h.z, 0.05f, 1e-4f);
}

// ─── Cell overlap fraction monotonicity ────────────────────────────────────
TEST(MpmRigidCylinder, OverlapFractionMonotoneRadial) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    float f0 = shape_cell_overlap_fraction(c, Vec3(0.00f, 0.0f, 0.0f));
    float f1 = shape_cell_overlap_fraction(c, Vec3(0.05f, 0.0f, 0.0f));
    float f2 = shape_cell_overlap_fraction(c, Vec3(0.10f, 0.0f, 0.0f));
    EXPECT_GT(f0, f1);
    EXPECT_GT(f1, f2);
    EXPECT_GE(f2, 0.0f);
}

TEST(MpmRigidCylinder, OverlapFractionZeroOutsideRadial) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    // Outside the radius — overlap should be 0 (negative argument clamps to 0).
    float f = shape_cell_overlap_fraction(c, Vec3(0.20f, 0.0f, 0.0f));
    EXPECT_NEAR(f, 0.0f, 1e-6f);
}

// ─── O2: axial == radial tie at the exact boundary ─────────────────────────
// At (0.10, 0, 0) with r=0.15, L=0.05, both axial_dist and radial_dist equal
// 0.05 — the closest-face heuristic is ambiguous. The exact branch taken is
// implementation-defined under floating-point tie-breaking. Either decision
// must produce (a) a unit-length vector, (b) along exactly one of the two
// valid axes (radial X or axial Y). The follow-up sweep code does not care
// which is chosen, only that the result is well-formed.
TEST(MpmRigidCylinder, OutwardNormalAtAxialRadialTieIsValid) {
    auto c = make_cylinder(Vec3::zero(), Quaternion::identity(), 0.15f, 0.05f);
    Vec3 local(0.10f, 0.0f, 0.0f);   // axial_dist == radial_dist == 0.05
    Vec3 n = shape_outward_normal_local(c, local);

    // Unit length.
    EXPECT_NEAR(n.length(), 1.0f, 1e-3f);

    // Z component is irrelevant for this point (z=0); the result must be
    // either radial (+X) or axial (+Y), not some arbitrary direction.
    const bool is_radial_x = (std::abs(n.x - 1.0f) < 1e-3f
                              && std::abs(n.y)     < 1e-3f
                              && std::abs(n.z)     < 1e-3f);
    const bool is_axial_y  = (std::abs(n.x)        < 1e-3f
                              && std::abs(n.y - 1.0f) < 1e-3f
                              && std::abs(n.z)     < 1e-3f);
    EXPECT_TRUE(is_radial_x || is_axial_y)
        << "Tie-break returned non-axis-aligned vector: (" << n.x << ", "
        << n.y << ", " << n.z << ")";
}

// ─── Q3: rolling temporal consistency ──────────────────────────────────────
// Sweep a world-space query point through the cylinder while the cylinder
// is rotated as if rolling about world Z. The inside-status must be
// monotone: enter once, exit once, no flip-flop. This rules out a class
// of bugs where intermediate orientations briefly mis-classify points on
// the boundary.
TEST(MpmRigidCylinder, RollingInsideMonotone) {
    // Cylinder centred at the origin, rolling configuration (axis along
    // world Z). We slowly translate a query point along world +X from
    // outside, through the body, to outside on the far side, while
    // simultaneously incrementing the cylinder's rotation about its axis
    // (it should not change the inside/outside status of a point near
    // the equator, since rotation about the cylinder axis is a symmetry).
    const float r = 0.15f, L = 0.05f;
    Quaternion q_base = Quaternion::from_axis_angle(Vec3(1, 0, 0),
                                                    3.14159265f * 0.5f);
    auto c = make_cylinder(Vec3::zero(), q_base, r, L);

    enum class S { OUT_LEFT, IN, OUT_RIGHT };
    S state = S::OUT_LEFT;
    int transitions = 0;

    const int N = 200;
    for (int i = 0; i < N; ++i) {
        const float x = -0.25f + 0.50f * (i / (float)(N - 1));   // -0.25 → +0.25

        // Spin the cylinder about its own axis (world Z). This must NOT
        // change inside-status for an equatorial probe point.
        const float angle_about_axis = i * 0.05f;
        Quaternion q_spin = Quaternion::from_axis_angle(Vec3(0, 0, 1),
                                                        angle_about_axis);
        c.orientation = q_spin * q_base;
        Matrix3 R_w2l = Matrix3::from_quaternion(c.orientation).transposed();

        Vec3 world_pt(x, 0.0f, 0.0f);
        Vec3 local = R_w2l * (world_pt - c.center);
        const bool inside = shape_inside(c, local);

        S next = inside
                ? S::IN
                : (state == S::IN ? S::OUT_RIGHT : state);
        if (next != state) { state = next; ++transitions; }
    }

    // The probe enters once and exits once → 2 transitions exactly.
    EXPECT_EQ(transitions, 2)
        << "Rolling cylinder inside-status flip-flopped (" << transitions
        << " transitions instead of 2). Spin-about-axis is a symmetry — "
           "rotation around the cylinder's own axis must not toggle "
           "inside-status.";
}

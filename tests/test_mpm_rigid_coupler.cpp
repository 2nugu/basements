// test_mpm_rigid_coupler.cpp — Smoke tests for the MPM ↔ rigid coupling MVP.
//
//   BasicReactionForce      — A grid node moving down into an OBB experiences a
//                             clamped velocity and the body sees an upward force.
//   OutsideBodyNoReaction   — A grid node outside the OBB AABB is untouched.
//   TangentialFriction      — Friction = 1 fully transfers tangential velocity.
//   AngularReactionPositive — A node offset from center yields torque, not just force.

#include <gtest/gtest.h>

#include "basements/physics/coupling/mpm_rigid_coupler.h"

using namespace basements::math;
using namespace basements::mpm;
using namespace basements::physics::coupling;

namespace {
RigidColliderState make_box(const Vec3& center, const Vec3& half) {
    RigidColliderState b;
    b.center       = center;
    b.half_extents = half;
    return b;
}
} // namespace

TEST(MPMRigidCoupler, BasicReactionForce) {
    // Particle-jet scenario: a grid node is barely inside the body's bottom
    // face moving upward. The body's closest face is its -Y face, so the BC
    // pushes the node back down and the body is lifted (Newton's third law).
    SPGridCPU grid(0.1f);
    GridNode* n = grid.get_node(0, 0, 0);             // world ≈ (0.05, 0.05, 0.05)
    n->active   = true;
    n->mass     = 1.0f;
    n->velocity = Vec3(0.0f, +5.0f, 0.0f);            // upward jet

    // Body is mostly above the node so the closest face is the bottom (-Y).
    auto body = make_box(Vec3(0.05f, 0.5f, 0.05f),
                         Vec3(0.5f, 0.55f, 0.5f));    // body y ∈ [-0.05, 1.05]
    auto r = MPMRigidCoupler::apply(grid, body, 1.0f / 60.0f);

    EXPECT_GT(r.nodes_inside, 0);
    EXPECT_GT(r.force.y, 0.0f) << "Upward-moving particle should lift the body.";
    EXPECT_LE(n->velocity.y, 0.0f + 1e-5f) << "Node should be clamped at the face.";
}

TEST(MPMRigidCoupler, OutsideBodyNoReaction) {
    SPGridCPU grid(0.1f);
    GridNode* n = grid.get_node(20, 20, 20);   // world ≈ (2.05, 2.05, 2.05)
    n->active   = true;
    n->mass     = 1.0f;
    n->velocity = Vec3(0.0f, -5.0f, 0.0f);

    auto body = make_box(Vec3::zero(), Vec3(0.5f, 0.5f, 0.5f));
    auto r = MPMRigidCoupler::apply(grid, body, 1.0f / 60.0f);

    EXPECT_EQ(r.nodes_inside, 0);
    EXPECT_NEAR(r.force.length(),  0.0f, 1e-6f);
    EXPECT_NEAR(r.torque.length(), 0.0f, 1e-6f);
}

TEST(MPMRigidCoupler, TangentialFrictionTransfers) {
    SPGridCPU grid(0.1f);
    GridNode* n = grid.get_node(0, 0, 0);
    n->active   = true;
    n->mass     = 1.0f;
    n->velocity = Vec3(5.0f, 0.0f, 0.0f);   // pure tangential

    auto body = make_box(Vec3(0.05f, 0.0f, 0.05f), Vec3(0.5f, 0.5f, 0.5f));
    body.linear_velocity = Vec3(0.0f, 0.0f, 0.0f);
    body.friction        = 1.0f;            // full coupling

    MPMRigidCoupler::apply(grid, body, 1.0f / 60.0f);

    // Body is stationary, friction=1 → node tangential velocity must drop to body's (zero).
    EXPECT_NEAR(n->velocity.x, 0.0f, 1e-4f);
}

TEST(MPMRigidCoupler, AngularReactionAtOffset) {
    SPGridCPU grid(0.1f);
    // Active node offset in +x direction from body center, moving down.
    GridNode* n = grid.get_node(3, 0, 0);   // world ≈ (0.35, 0.05, 0.05)
    n->active   = true;
    n->mass     = 1.0f;
    n->velocity = Vec3(0.0f, -5.0f, 0.0f);

    auto body = make_box(Vec3(0.05f, 0.0f, 0.05f), Vec3(0.5f, 0.5f, 0.5f));
    auto r = MPMRigidCoupler::apply(grid, body, 1.0f / 60.0f);

    EXPECT_GT(r.nodes_inside, 0);
    // r × Δp with r ~ +x and Δp ~ +y (reaction up on grid is -Δp ~ -y...) yields nonzero torque.
    EXPECT_GT(r.torque.length(), 1e-3f);
}

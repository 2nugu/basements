// test_urdf_physics_bridge.cpp — Validates URDF → PhysicsWorld spawning.
//
//   FixedBaseTwoLink     — REVOLUTE arm: root static, joint = HINGE with limits.
//   PrismaticGantry      — PRISMATIC joint maps to SLIDER + limits.
//   FloatingBase6DOF     — FLOATING joint creates no constraint; child is free.
//   ForwardKinematics    — child link's spawn pose follows the parent chain.
//   FixedChainNoCycle    — FIXED joint locks pose; integration keeps relative offset.

#include <gtest/gtest.h>

#include "basements/io/urdf_physics_bridge.h"

using namespace basements::io;
using namespace basements::engine;
using namespace basements::math;

static URDFRobot make_two_link_arm() {
    URDFRobot r("two_link_arm");
    r.links.emplace_back("base");
    r.links.emplace_back("link1");

    URDFJoint j("shoulder");
    j.type        = URDFJointType::REVOLUTE;
    j.parent_link = "base";
    j.child_link  = "link1";
    j.origin.position = URDFVector3(0.0f, 1.0f, 0.0f);
    j.axis        = URDFVector3(0.0f, 0.0f, 1.0f);
    j.lower_limit = -1.57f;
    j.upper_limit =  1.57f;
    r.joints.push_back(j);

    r.links[0].inertial.mass = 0.0f;     // base
    r.links[1].inertial.mass = 1.0f;     // link1
    return r;
}

TEST(URDFPhysicsBridge, FixedBaseTwoLink) {
    URDFRobot robot = make_two_link_arm();
    PhysicsWorldGPU world;

    URDFBridgeOptions opts;
    opts.fix_base = true;
    URDFBridgeResult res = URDFPhysicsBridge::spawn(robot, world, opts);

    ASSERT_TRUE(res.ok());
    EXPECT_EQ(res.link_bodies.size(), 2u);
    EXPECT_EQ(res.joint_constraints.size(), 1u);

    // Root is static.
    EXPECT_EQ(world.get_body_type(res.link_by_name["base"]),  BodyType::STATIC);
    EXPECT_EQ(world.get_body_type(res.link_by_name["link1"]), BodyType::DYNAMIC);

    // Joint exists.
    EXPECT_TRUE(world.is_joint_valid(res.joint_by_name["shoulder"]));
    EXPECT_EQ(world.get_joint_count(), 1u);
}

TEST(URDFPhysicsBridge, PrismaticGantry) {
    URDFRobot r("gantry");
    r.links.emplace_back("base");
    r.links.emplace_back("slider");
    URDFJoint j("rail");
    j.type        = URDFJointType::PRISMATIC;
    j.parent_link = "base";
    j.child_link  = "slider";
    j.axis        = URDFVector3(1.0f, 0.0f, 0.0f);
    j.lower_limit = -0.5f;
    j.upper_limit =  0.5f;
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 0.5f;

    PhysicsWorldGPU world;
    URDFBridgeOptions opts; opts.fix_base = true;
    auto res = URDFPhysicsBridge::spawn(r, world, opts);

    ASSERT_TRUE(res.ok());
    EXPECT_TRUE(world.is_joint_valid(res.joint_by_name["rail"]));
}

TEST(URDFPhysicsBridge, FloatingBase6DOF) {
    URDFRobot r("free_body");
    r.links.emplace_back("world");
    r.links.emplace_back("torso");
    URDFJoint j("base_to_torso");
    j.type        = URDFJointType::FLOATING;
    j.parent_link = "world";
    j.child_link  = "torso";
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 5.0f;

    PhysicsWorldGPU world;
    auto res = URDFPhysicsBridge::spawn(r, world);

    ASSERT_TRUE(res.ok());
    // No constraint created for FLOATING.
    EXPECT_FALSE(world.is_joint_valid(res.joint_constraints[0]));
    EXPECT_EQ(world.get_joint_count(), 0u);
    // Warning recorded.
    bool found_warn = false;
    for (const auto& w : res.warnings) {
        if (w.find("FLOATING") != std::string::npos) found_warn = true;
    }
    EXPECT_TRUE(found_warn);
}

TEST(URDFPhysicsBridge, ForwardKinematicsChainsPose) {
    URDFRobot r("chain");
    r.links.emplace_back("a");
    r.links.emplace_back("b");
    r.links.emplace_back("c");

    URDFJoint j1("a_b"), j2("b_c");
    j1.type = j2.type = URDFJointType::FIXED;
    j1.parent_link = "a"; j1.child_link = "b";
    j1.origin.position = URDFVector3(1.0f, 0.0f, 0.0f);
    j2.parent_link = "b"; j2.child_link = "c";
    j2.origin.position = URDFVector3(0.0f, 0.5f, 0.0f);
    r.joints.push_back(j1);
    r.joints.push_back(j2);
    for (auto& l : r.links) l.inertial.mass = 1.0f;

    PhysicsWorldGPU world;
    URDFBridgeOptions opts;
    opts.base_position = Vec3(10.0f, 0.0f, 0.0f);
    auto res = URDFPhysicsBridge::spawn(r, world, opts);

    ASSERT_TRUE(res.ok());

    Vec3 pa = world.get_body_position(res.link_by_name["a"]);
    Vec3 pb = world.get_body_position(res.link_by_name["b"]);
    Vec3 pc = world.get_body_position(res.link_by_name["c"]);

    EXPECT_NEAR(pa.x, 10.0f, 1e-5f);
    EXPECT_NEAR(pb.x, 11.0f, 1e-5f);  // a + (1, 0, 0)
    EXPECT_NEAR(pb.y,  0.0f, 1e-5f);
    EXPECT_NEAR(pc.x, 11.0f, 1e-5f);  // b + (0, 0.5, 0)
    EXPECT_NEAR(pc.y,  0.5f, 1e-5f);
}

TEST(URDFPhysicsBridge, NoLinksReturnsEmpty) {
    URDFRobot empty("");
    PhysicsWorldGPU world;
    auto res = URDFPhysicsBridge::spawn(empty, world);
    EXPECT_FALSE(res.ok());
    EXPECT_FALSE(res.warnings.empty());
}

// Revolute joint at π/2 about z rotates the child position from +x to +y.
TEST(URDFPhysicsBridge, RevoluteInitialAngleAppliesFK) {
    URDFRobot r("arm");
    r.links.emplace_back("base");
    r.links.emplace_back("link1");

    URDFJoint j("shoulder");
    j.type        = URDFJointType::REVOLUTE;
    j.parent_link = "base";
    j.child_link  = "link1";
    j.origin.position = URDFVector3(1.0f, 0.0f, 0.0f);   // 1 m along +x from base
    j.axis        = URDFVector3(0.0f, 0.0f, 1.0f);       // rotate about z
    j.lower_limit = -3.14f; j.upper_limit = 3.14f;
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 1.0f;

    URDFBridgeOptions opts; opts.fix_base = true;
    opts.initial_joint_positions["shoulder"] = 1.57079632f;  // π/2

    PhysicsWorldGPU world;
    auto res = URDFPhysicsBridge::spawn(r, world, opts);
    ASSERT_TRUE(res.ok());

    Vec3 p_link1 = world.get_body_position(res.link_by_name["link1"]);
    // Joint origin (1, 0, 0) followed by π/2 rotation about z — child origin
    // is at the joint origin (zero offset from joint frame), so position
    // equals joint origin: (1, 0, 0). The orientation reflects the angle.
    EXPECT_NEAR(p_link1.x, 1.0f, 1e-4f);
    EXPECT_NEAR(p_link1.y, 0.0f, 1e-4f);

    Quaternion q = world.get_body_orientation(res.link_by_name["link1"]);
    // π/2 about z → q = (cos(π/4), 0, 0, sin(π/4))
    EXPECT_NEAR(q.w, 0.7071f, 1e-3f);
    EXPECT_NEAR(q.z, 0.7071f, 1e-3f);
}

// set_joint_position() can drive a hinge to an arbitrary angle at runtime.
TEST(URDFPhysicsBridge, SetJointPositionHinge) {
    URDFRobot r("arm");
    r.links.emplace_back("base");
    r.links.emplace_back("link1");

    URDFJoint j("shoulder");
    j.type        = URDFJointType::REVOLUTE;
    j.parent_link = "base";
    j.child_link  = "link1";
    j.origin.position = URDFVector3(1.0f, 0.0f, 0.0f);
    j.axis        = URDFVector3(0.0f, 0.0f, 1.0f);
    j.lower_limit = -3.14f; j.upper_limit = 3.14f;
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 1.0f;

    PhysicsWorldGPU world;
    URDFBridgeOptions opts; opts.fix_base = true;
    auto res = URDFPhysicsBridge::spawn(r, world, opts);
    ASSERT_TRUE(res.ok());

    // Drive the joint to π/2 about z.
    world.set_joint_position(res.joint_by_name["shoulder"], 1.57079632f);

    Quaternion q = world.get_body_orientation(res.link_by_name["link1"]);
    // π/2 about z → (cos(π/4), 0, 0, sin(π/4))
    EXPECT_NEAR(q.w, 0.7071f, 5e-3f);
    EXPECT_NEAR(q.z, 0.7071f, 5e-3f);

    // Velocity must be zeroed for kinematic playback.
    EXPECT_NEAR(world.get_body_linear_velocity(res.link_by_name["link1"]).length(),  0.0f, 1e-5f);
    EXPECT_NEAR(world.get_body_angular_velocity(res.link_by_name["link1"]).length(), 0.0f, 1e-5f);
}

// set_joint_position() on a slider moves the child along the joint axis.
TEST(URDFPhysicsBridge, SetJointPositionSlider) {
    URDFRobot r("gantry");
    r.links.emplace_back("base");
    r.links.emplace_back("slider");
    URDFJoint j("rail");
    j.type        = URDFJointType::PRISMATIC;
    j.parent_link = "base";
    j.child_link  = "slider";
    j.axis        = URDFVector3(1.0f, 0.0f, 0.0f);
    j.lower_limit = -1.0f; j.upper_limit = 1.0f;
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 0.5f;

    PhysicsWorldGPU world;
    URDFBridgeOptions opts; opts.fix_base = true;
    auto res = URDFPhysicsBridge::spawn(r, world, opts);
    ASSERT_TRUE(res.ok());

    world.set_joint_position(res.joint_by_name["rail"], 0.4f);
    Vec3 p = world.get_body_position(res.link_by_name["slider"]);
    EXPECT_NEAR(p.x, 0.4f, 1e-4f);
}

// set_joint_velocity() on hinge sets the child's angular velocity.
TEST(URDFPhysicsBridge, SetJointVelocityHinge) {
    URDFRobot r("arm");
    r.links.emplace_back("base");
    r.links.emplace_back("link1");
    URDFJoint j("shoulder");
    j.type        = URDFJointType::REVOLUTE;
    j.parent_link = "base";
    j.child_link  = "link1";
    j.axis        = URDFVector3(0.0f, 0.0f, 1.0f);
    j.lower_limit = -3.14f; j.upper_limit = 3.14f;
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 1.0f;

    PhysicsWorldGPU world;
    URDFBridgeOptions opts; opts.fix_base = true;
    auto res = URDFPhysicsBridge::spawn(r, world, opts);
    ASSERT_TRUE(res.ok());

    // 2 rad/s about z. Base is static (angular_velocity = 0) so child's
    // angular velocity must equal axis_world × qdot = (0,0,1) × 2 = (0,0,2).
    world.set_joint_velocity(res.joint_by_name["shoulder"], 2.0f);

    Vec3 omega = world.get_body_angular_velocity(res.link_by_name["link1"]);
    EXPECT_NEAR(omega.x, 0.0f, 1e-5f);
    EXPECT_NEAR(omega.y, 0.0f, 1e-5f);
    EXPECT_NEAR(omega.z, 2.0f, 1e-5f);
}

// set_joint_velocity() on slider sets the child's linear velocity.
TEST(URDFPhysicsBridge, SetJointVelocitySlider) {
    URDFRobot r("gantry");
    r.links.emplace_back("base");
    r.links.emplace_back("slider");
    URDFJoint j("rail");
    j.type        = URDFJointType::PRISMATIC;
    j.parent_link = "base";
    j.child_link  = "slider";
    j.axis        = URDFVector3(1.0f, 0.0f, 0.0f);
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 0.5f;

    PhysicsWorldGPU world;
    URDFBridgeOptions opts; opts.fix_base = true;
    auto res = URDFPhysicsBridge::spawn(r, world, opts);
    ASSERT_TRUE(res.ok());

    world.set_joint_velocity(res.joint_by_name["rail"], 1.5f);

    Vec3 v = world.get_body_linear_velocity(res.link_by_name["slider"]);
    EXPECT_NEAR(v.x, 1.5f, 1e-5f);
    EXPECT_NEAR(v.y, 0.0f, 1e-5f);
    EXPECT_NEAR(v.z, 0.0f, 1e-5f);
}

// Prismatic joint with initial coord translates the child along axis.
TEST(URDFPhysicsBridge, PrismaticInitialPositionAppliesFK) {
    URDFRobot r("gantry");
    r.links.emplace_back("base");
    r.links.emplace_back("slider");

    URDFJoint j("rail");
    j.type        = URDFJointType::PRISMATIC;
    j.parent_link = "base";
    j.child_link  = "slider";
    j.axis        = URDFVector3(1.0f, 0.0f, 0.0f);
    j.lower_limit = -1.0f; j.upper_limit = 1.0f;
    r.joints.push_back(j);
    r.links[0].inertial.mass = 0.0f;
    r.links[1].inertial.mass = 0.5f;

    URDFBridgeOptions opts; opts.fix_base = true;
    opts.initial_joint_positions["rail"] = 0.3f;

    PhysicsWorldGPU world;
    auto res = URDFPhysicsBridge::spawn(r, world, opts);
    ASSERT_TRUE(res.ok());

    Vec3 p = world.get_body_position(res.link_by_name["slider"]);
    EXPECT_NEAR(p.x, 0.3f, 1e-5f);
    EXPECT_NEAR(p.y, 0.0f, 1e-5f);
    EXPECT_NEAR(p.z, 0.0f, 1e-5f);
}

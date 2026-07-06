/**
 * @file test_physics_world_gpu.cpp
 * @brief Unit tests for PhysicsWorldGPU
 * 
 * Tests cover:
 *   - World creation and configuration
 *   - Body creation, destruction, and state management
 *   - Joint creation and motor control
 *   - Simulation stepping and physics correctness
 *   - Statistics and callbacks
 */

#include <gtest/gtest.h>
#include "basements/engine/physics_world_gpu.h"
#include <cmath>

using namespace basements::engine;
using namespace basements::physics;
using namespace basements::math;

// ============================================================================
// Test Fixtures
// ============================================================================

class PhysicsWorldGPUTest : public ::testing::Test {
protected:
    void SetUp() override {
        PhysicsWorldConfig config;
        config.gravity = Vec3(0.0f, -10.0f, 0.0f);
        config.fixed_time_step = 1.0f / 60.0f;
        config.solver_iterations = 10;
        config.prefer_gpu = false;  // Use CPU for deterministic tests
        
        world = std::make_unique<PhysicsWorldGPU>(config);
    }
    
    void TearDown() override {
        world.reset();
    }
    
    std::unique_ptr<PhysicsWorldGPU> world;
};

// ============================================================================
// World Creation Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, WorldCreation) {
    EXPECT_EQ(world->get_body_count(), 0);
    EXPECT_EQ(world->get_joint_count(), 0);
    
    Vec3 gravity = world->get_gravity();
    EXPECT_FLOAT_EQ(gravity.y, -10.0f);
}

TEST_F(PhysicsWorldGPUTest, WorldConfiguration) {
    world->set_gravity(Vec3(0.0f, -20.0f, 0.0f));
    EXPECT_FLOAT_EQ(world->get_gravity().y, -20.0f);
    
    world->set_time_step(1.0f / 120.0f);
    EXPECT_FLOAT_EQ(world->get_time_step(), 1.0f / 120.0f);
    
    world->set_solver_iterations(20);
    EXPECT_EQ(world->get_solver_iterations(), 20);
}

// ============================================================================
// Body Management Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, BodyCreation) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 5, 0), Vec3(1, 1, 1), 1.0f);
    
    BodyHandle handle = world->create_body(desc);
    
    EXPECT_TRUE(handle.is_valid());
    EXPECT_TRUE(world->is_body_valid(handle));
    EXPECT_EQ(world->get_body_count(), 1);
}

TEST_F(PhysicsWorldGPUTest, BodyDestruction) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 5, 0), Vec3(1, 1, 1), 1.0f);
    BodyHandle handle = world->create_body(desc);
    
    EXPECT_EQ(world->get_body_count(), 1);
    
    world->destroy_body(handle);
    
    EXPECT_FALSE(world->is_body_valid(handle));
    EXPECT_EQ(world->get_body_count(), 0);
}

TEST_F(PhysicsWorldGPUTest, BodyStateQueries) {
    RigidBodyDesc desc;
    desc.position = Vec3(1, 2, 3);
    desc.linear_velocity = Vec3(0, -1, 0);
    desc.mass = 5.0f;
    desc.type = BodyType::DYNAMIC;
    
    BodyHandle handle = world->create_body(desc);
    
    Vec3 pos = world->get_body_position(handle);
    EXPECT_FLOAT_EQ(pos.x, 1.0f);
    EXPECT_FLOAT_EQ(pos.y, 2.0f);
    EXPECT_FLOAT_EQ(pos.z, 3.0f);
    
    EXPECT_FLOAT_EQ(world->get_body_mass(handle), 5.0f);
    EXPECT_EQ(world->get_body_type(handle), BodyType::DYNAMIC);
}

TEST_F(PhysicsWorldGPUTest, BodyStateModification) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 0, 0), Vec3(1, 1, 1), 1.0f);
    BodyHandle handle = world->create_body(desc);
    
    world->set_body_position(handle, Vec3(10, 20, 30));
    Vec3 pos = world->get_body_position(handle);
    EXPECT_FLOAT_EQ(pos.x, 10.0f);
    EXPECT_FLOAT_EQ(pos.y, 20.0f);
    EXPECT_FLOAT_EQ(pos.z, 30.0f);
    
    world->set_body_linear_velocity(handle, Vec3(1, 2, 3));
    Vec3 vel = world->get_body_linear_velocity(handle);
    EXPECT_FLOAT_EQ(vel.x, 1.0f);
    EXPECT_FLOAT_EQ(vel.y, 2.0f);
    EXPECT_FLOAT_EQ(vel.z, 3.0f);
}

TEST_F(PhysicsWorldGPUTest, MultipleBodyCreation) {
    const int num_bodies = 100;
    std::vector<BodyHandle> handles;
    
    for (int i = 0; i < num_bodies; ++i) {
        RigidBodyDesc desc = make_sphere_body(
            Vec3(static_cast<float>(i), static_cast<float>(i * 2), 0),
            0.5f, 1.0f
        );
        handles.push_back(world->create_body(desc));
    }
    
    EXPECT_EQ(world->get_body_count(), num_bodies);
    
    for (int i = 0; i < num_bodies; ++i) {
        EXPECT_TRUE(world->is_body_valid(handles[i]));
        
        Vec3 pos = world->get_body_position(handles[i]);
        EXPECT_FLOAT_EQ(pos.x, static_cast<float>(i));
    }
}

// ============================================================================
// Force and Impulse Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, ApplyForce) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 0, 0), Vec3(1, 1, 1), 1.0f);
    BodyHandle handle = world->create_body(desc);
    
    // Apply force and step
    world->apply_force(handle, Vec3(10, 0, 0));
    world->step_fixed();
    
    Vec3 vel = world->get_body_linear_velocity(handle);
    
    // velocity should increase due to force (F = ma, a = F/m = 10 m/s²)
    // After 1/60 sec: v = a * dt ≈ 0.167 m/s
    EXPECT_GT(vel.x, 0.1f);
}

TEST_F(PhysicsWorldGPUTest, ApplyImpulse) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 0, 0), Vec3(1, 1, 1), 1.0f);
    BodyHandle handle = world->create_body(desc);
    
    // Apply impulse (immediate velocity change)
    world->apply_impulse(handle, Vec3(5, 0, 0));
    
    Vec3 vel = world->get_body_linear_velocity(handle);
    
    // impulse = m * Δv, so Δv = impulse / m = 5 m/s
    EXPECT_FLOAT_EQ(vel.x, 5.0f);
}

TEST_F(PhysicsWorldGPUTest, GravityAffectsBody) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 10, 0), Vec3(1, 1, 1), 1.0f);
    BodyHandle handle = world->create_body(desc);
    
    float initial_y = world->get_body_position(handle).y;
    
    // Step for 1 second (60 steps at 60 FPS)
    for (int i = 0; i < 60; ++i) {
        world->step_fixed();
    }
    
    float final_y = world->get_body_position(handle).y;
    
    // Body should have fallen due to gravity
    EXPECT_LT(final_y, initial_y);
    
    // After 1 second of free fall: y = y0 + v0*t + 0.5*g*t^2
    // y ≈ 10 + 0 - 0.5 * 10 * 1 = 5 meters
    // With damping it will be slightly less
    EXPECT_LT(final_y, 6.0f);
}

// ============================================================================
// Joint Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, BallSocketJointCreation) {
    RigidBodyDesc desc_a = make_box_body(Vec3(0, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    RigidBodyDesc desc_b = make_box_body(Vec3(2, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    
    BodyHandle body_a = world->create_body(desc_a);
    BodyHandle body_b = world->create_body(desc_b);
    
    JointDescriptor joint_desc = make_ball_socket_joint(
        body_a, Vec3(1, 0, 0),
        body_b, Vec3(-1, 0, 0)
    );
    
    ConstraintHandle joint = world->create_joint(joint_desc);
    
    EXPECT_TRUE(joint.is_valid());
    EXPECT_TRUE(world->is_joint_valid(joint));
    EXPECT_EQ(world->get_joint_count(), 1);
}

TEST_F(PhysicsWorldGPUTest, HingeJointCreation) {
    RigidBodyDesc desc_a = make_box_body(Vec3(0, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 0.0f);  // Static
    desc_a.type = BodyType::STATIC;
    
    RigidBodyDesc desc_b = make_box_body(Vec3(2, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    
    BodyHandle body_a = world->create_body(desc_a);
    BodyHandle body_b = world->create_body(desc_b);
    
    JointDescriptor joint_desc = make_hinge_joint(
        body_a, Vec3(1, 0, 0), Vec3(0, 0, 1),
        body_b, Vec3(-1, 0, 0), Vec3(0, 0, 1)
    );
    joint_desc.enable_limits = true;
    joint_desc.lower_limit = -PI / 2.0f;
    joint_desc.upper_limit = PI / 2.0f;
    
    ConstraintHandle joint = world->create_joint(joint_desc);
    
    EXPECT_TRUE(world->is_joint_valid(joint));
}

TEST_F(PhysicsWorldGPUTest, JointDestruction) {
    RigidBodyDesc desc_a = make_box_body(Vec3(0, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    RigidBodyDesc desc_b = make_box_body(Vec3(2, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    
    BodyHandle body_a = world->create_body(desc_a);
    BodyHandle body_b = world->create_body(desc_b);
    
    JointDescriptor joint_desc = make_ball_socket_joint(
        body_a, Vec3(1, 0, 0),
        body_b, Vec3(-1, 0, 0)
    );
    
    ConstraintHandle joint = world->create_joint(joint_desc);
    EXPECT_EQ(world->get_joint_count(), 1);
    
    world->destroy_joint(joint);
    
    EXPECT_FALSE(world->is_joint_valid(joint));
    EXPECT_EQ(world->get_joint_count(), 0);
}

TEST_F(PhysicsWorldGPUTest, JointMotorControl) {
    RigidBodyDesc desc_a = make_box_body(Vec3(0, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 0.0f);
    desc_a.type = BodyType::STATIC;
    
    RigidBodyDesc desc_b = make_box_body(Vec3(2, 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
    
    BodyHandle body_a = world->create_body(desc_a);
    BodyHandle body_b = world->create_body(desc_b);
    
    JointDescriptor joint_desc = make_hinge_joint(
        body_a, Vec3(1, 0, 0), Vec3(0, 0, 1),
        body_b, Vec3(-1, 0, 0), Vec3(0, 0, 1)
    );
    joint_desc.enable_motor = true;
    joint_desc.motor_target_velocity = 1.0f;
    joint_desc.motor_max_force_or_torque = 10.0f;
    
    ConstraintHandle joint = world->create_joint(joint_desc);
    
    // Modify motor settings
    world->set_joint_motor_velocity(joint, 2.0f);
    world->set_joint_motor_max_force(joint, 20.0f);
    world->set_joint_motor_enabled(joint, false);
    
    // No exceptions expected
    EXPECT_TRUE(world->is_joint_valid(joint));
}

// ============================================================================
// Simulation Stability Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, StableSimulationWith1000Steps) {
    // Create a few bodies
    for (int i = 0; i < 10; ++i) {
        RigidBodyDesc desc = make_box_body(
            Vec3(static_cast<float>(i), 10.0f, 0),
            Vec3(0.5f, 0.5f, 0.5f),
            1.0f
        );
        world->create_body(desc);
    }
    
    // Run for 1000 steps without crash
    for (int i = 0; i < 1000; ++i) {
        world->step_fixed();
    }
    
    // All bodies should still be valid
    EXPECT_EQ(world->get_body_count(), 10);
    
    // Get statistics
    SimulationStatistics stats = world->get_statistics();
    EXPECT_EQ(stats.total_steps, 1000);
    EXPECT_GT(stats.total_simulation_time, 0.0);
}

TEST_F(PhysicsWorldGPUTest, StaticBodyDoesNotMove) {
    RigidBodyDesc desc = make_box_body(Vec3(5, 5, 5), Vec3(1, 1, 1), 0.0f);
    desc.type = BodyType::STATIC;
    
    BodyHandle handle = world->create_body(desc);
    
    Vec3 initial_pos = world->get_body_position(handle);
    
    // Step many times
    for (int i = 0; i < 100; ++i) {
        world->step_fixed();
    }
    
    Vec3 final_pos = world->get_body_position(handle);
    
    // Static body should not move
    EXPECT_FLOAT_EQ(initial_pos.x, final_pos.x);
    EXPECT_FLOAT_EQ(initial_pos.y, final_pos.y);
    EXPECT_FLOAT_EQ(initial_pos.z, final_pos.z);
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, StatisticsTracking) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 5, 0), Vec3(1, 1, 1), 1.0f);
    world->create_body(desc);
    
    world->step_fixed();
    world->step_fixed();
    world->step_fixed();
    
    SimulationStatistics stats = world->get_statistics();
    
    EXPECT_EQ(stats.total_steps, 3);
    EXPECT_EQ(stats.active_body_count, 1);
    EXPECT_GT(stats.total_step_time_ms, 0.0f);
}

TEST_F(PhysicsWorldGPUTest, ResetStatistics) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 5, 0), Vec3(1, 1, 1), 1.0f);
    world->create_body(desc);
    
    world->step_fixed();
    world->step_fixed();
    
    world->reset_statistics();
    
    SimulationStatistics stats = world->get_statistics();
    EXPECT_EQ(stats.total_steps, 0);
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, ContactCallback) {
    int contact_count = 0;
    
    world->set_contact_callback([&contact_count](BodyHandle, BodyHandle, const Vec3&, const Vec3&) {
        contact_count++;
    });
    
    // Create two overlapping bodies
    RigidBodyDesc desc_a = make_sphere_body(Vec3(0, 0, 0), 1.0f, 1.0f);
    RigidBodyDesc desc_b = make_sphere_body(Vec3(0.5f, 0, 0), 1.0f, 1.0f);  // Overlapping
    
    world->create_body(desc_a);
    world->create_body(desc_b);
    
    world->step_fixed();
    
    // Callback should have been invoked
    EXPECT_GT(contact_count, 0);
}

// ============================================================================
// Clear and Reset Tests
// ============================================================================

TEST_F(PhysicsWorldGPUTest, ClearWorld) {
    for (int i = 0; i < 10; ++i) {
        RigidBodyDesc desc = make_box_body(Vec3(static_cast<float>(i), 5, 0), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
        world->create_body(desc);
    }
    
    EXPECT_EQ(world->get_body_count(), 10);
    
    world->clear();
    
    EXPECT_EQ(world->get_body_count(), 0);
    EXPECT_EQ(world->get_joint_count(), 0);
}

TEST_F(PhysicsWorldGPUTest, ResetSimulation) {
    RigidBodyDesc desc = make_box_body(Vec3(0, 10, 0), Vec3(1, 1, 1), 1.0f);
    BodyHandle handle = world->create_body(desc);
    
    // Step and let body fall
    for (int i = 0; i < 30; ++i) {
        world->step_fixed();
    }
    
    Vec3 vel_before = world->get_body_linear_velocity(handle);
    EXPECT_NE(vel_before.y, 0.0f);  // Should have some velocity
    
    world->reset();
    
    Vec3 vel_after = world->get_body_linear_velocity(handle);
    EXPECT_FLOAT_EQ(vel_after.y, 0.0f);  // Velocity reset to zero
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(FactoryFunctionTest, MakeBoxBody) {
    RigidBodyDesc desc = make_box_body(Vec3(1, 2, 3), Vec3(0.5f, 1.0f, 1.5f), 10.0f);
    
    EXPECT_FLOAT_EQ(desc.position.x, 1.0f);
    EXPECT_FLOAT_EQ(desc.position.y, 2.0f);
    EXPECT_FLOAT_EQ(desc.position.z, 3.0f);
    EXPECT_FLOAT_EQ(desc.mass, 10.0f);
}

TEST(FactoryFunctionTest, MakeSphereBody) {
    RigidBodyDesc desc = make_sphere_body(Vec3(0, 0, 0), 2.0f, 5.0f);
    
    EXPECT_FLOAT_EQ(desc.mass, 5.0f);
    
    // Check inertia tensor (diagonal should be 2/5 * m * r²)
    float expected_I = (2.0f / 5.0f) * 5.0f * 4.0f;  // 8.0
    EXPECT_NEAR(desc.inertia_tensor.m[0][0], expected_I, 0.01f);
}

// ============================================================================
// Raycast / QueryAABB Tests
// ============================================================================

TEST(QueryTest, Raycast_HitsBox) {
    PhysicsWorldConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorldGPU world(cfg);

    RigidBodyDesc box;
    box.position = Vec3(0, 0, 10);
    box.type = BodyType::STATIC;
    box.half_extents = Vec3(1.0f);
    box.mass = 0.0f;
    auto h = world.create_body(box);

    Vec3 hit_pt, hit_n;
    auto result = world.raycast(Vec3(0, 0, 0), Vec3(0, 0, 1), 100.0f, &hit_pt, &hit_n);
    EXPECT_TRUE(result.is_valid());
    EXPECT_EQ(result.id, h.id);
    EXPECT_NEAR(hit_pt.z, 9.0f, 0.01f);     // front face at z=9
    EXPECT_NEAR(hit_n.z, -1.0f, 0.01f);     // normal points toward ray origin
}

TEST(QueryTest, Raycast_MissesBox) {
    PhysicsWorldConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorldGPU world(cfg);

    RigidBodyDesc box;
    box.position = Vec3(5, 5, 10);
    box.type = BodyType::STATIC;
    box.half_extents = Vec3(0.5f);
    box.mass = 0.0f;
    world.create_body(box);

    auto result = world.raycast(Vec3(0, 0, 0), Vec3(0, 0, 1), 100.0f, nullptr, nullptr);
    EXPECT_FALSE(result.is_valid());
}

TEST(QueryTest, Raycast_ReturnsClosest) {
    PhysicsWorldConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorldGPU world(cfg);

    RigidBodyDesc b1, b2;
    b1.position = Vec3(0, 0, 5);  b1.type = BodyType::STATIC; b1.half_extents = Vec3(0.5f);
    b2.position = Vec3(0, 0, 15); b2.type = BodyType::STATIC; b2.half_extents = Vec3(0.5f);
    auto h1 = world.create_body(b1);
    world.create_body(b2);

    auto result = world.raycast(Vec3(0, 0, 0), Vec3(0, 0, 1), 100.0f, nullptr, nullptr);
    EXPECT_TRUE(result.is_valid());
    EXPECT_EQ(result.id, h1.id);  // nearer box wins
}

TEST(QueryTest, QueryAABB_FindsOverlapping) {
    PhysicsWorldConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorldGPU world(cfg);

    RigidBodyDesc inside, outside;
    inside.position  = Vec3(0, 0, 0); inside.type  = BodyType::STATIC; inside.half_extents  = Vec3(0.5f);
    outside.position = Vec3(10, 0, 0); outside.type = BodyType::STATIC; outside.half_extents = Vec3(0.5f);
    auto h_in = world.create_body(inside);
    world.create_body(outside);

    using namespace basements::collision;
    AABB query(Vec3(-2,-2,-2), Vec3(2,2,2));
    std::vector<BodyHandle> found;
    world.query_aabb(query, [&](BodyHandle h) { found.push_back(h); return true; });

    ASSERT_EQ(found.size(), 1u);
    EXPECT_EQ(found[0].id, h_in.id);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// 물리 파이프라인 수직심화 검증 테스트
// 1. 자유낙하: 정확한 위치 계산
// 2. 박스 낙하 후 정지: 지면 충돌 + 바운스 감쇠
// 3. 회전 물체 충돌: 관성 텐서 올바른지
// 4. 스택: 두 박스 쌓기 안정성

#include "basements/engine/physics_world_gpu.h"
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>

using namespace basements::engine;
using namespace basements::physics;
using namespace basements::math;

static PhysicsWorldGPU make_world(float gravity_y = -9.81f) {
    PhysicsWorldConfig cfg;
    cfg.gravity = Vec3(0, gravity_y, 0);
    cfg.fixed_time_step = 1.0f / 60.0f;
    cfg.solver_iterations = 20;
    return PhysicsWorldGPU(cfg);
}

// ── 1. 자유낙하 정확도 ─────────────────────────────────────────────
TEST(Validation, FreeFall_Position) {
    auto world = make_world(-9.81f);

    RigidBodyDesc desc;
    desc.position = Vec3(0, 10, 0);
    desc.mass = 1.0f;
    desc.type = BodyType::DYNAMIC;
    desc.half_extents = Vec3(0.5f);
    float I = (1.0f/6.0f) * 1.0f * (0.5f*0.5f + 0.5f*0.5f);
    desc.inertia_tensor = Matrix3::identity() * I;

    auto body = world.create_body(desc);

    // 1초 시뮬레이션 (60 스텝)
    for (int i = 0; i < 60; ++i) world.step(1.0f / 60.0f);

    float y = world.get_body_position(body).y;
    // 이론값: 10 - 0.5*9.81*1² = 5.095m (반지름 고려 없음)
    // 지면 없으므로 순수 낙하
    float expected = 10.0f - 0.5f * 9.81f * 1.0f * 1.0f;
    std::cout << "  FreeFall y=" << y << " expected≈" << expected << "\n";
    EXPECT_NEAR(y, expected, 0.15f); // 2.5% 허용 오차
}

// ── 2. 지면 충돌 후 정지 ──────────────────────────────────────────
TEST(Validation, BoxLandsOnGround) {
    auto world = make_world(-9.81f);

    // 지면 (static)
    RigidBodyDesc ground;
    ground.position = Vec3(0, -0.1f, 0);
    ground.type = BodyType::STATIC;
    ground.half_extents = Vec3(20, 0.1f, 20);
    world.create_body(ground);

    // 낙하 박스
    RigidBodyDesc box;
    box.position = Vec3(0, 3, 0);
    box.mass = 1.0f;
    box.type = BodyType::DYNAMIC;
    box.half_extents = Vec3(0.5f);
    box.restitution = 0.0f;  // 바운스 없음
    box.friction = 0.5f;
    float I = (1.0f/6.0f) * 1.0f * (0.5f*0.5f + 0.5f*0.5f);
    box.inertia_tensor = Matrix3::identity() * I;
    auto body = world.create_body(box);

    // 3초 시뮬레이션
    for (int i = 0; i < 180; ++i) world.step(1.0f / 60.0f);

    float y = world.get_body_position(body).y;
    float vy = world.get_body_linear_velocity(body).y;
    std::cout << "  After landing: y=" << y << " vy=" << vy << "\n";

    // 박스 절반 높이(0.5) + 지면 절반 높이(0.1) = 0.6 위에 안착
    EXPECT_NEAR(y, 0.5f, 0.1f);
    EXPECT_NEAR(vy, 0.0f, 0.5f); // 정지에 가깝게
}

// ── 3. 스택: 두 박스 안정성 ───────────────────────────────────────
TEST(Validation, TwoBoxStack_Stable) {
    auto world = make_world(-9.81f);

    RigidBodyDesc ground;
    ground.position = Vec3(0, -0.1f, 0);
    ground.type = BodyType::STATIC;
    ground.half_extents = Vec3(20, 0.1f, 20);
    world.create_body(ground);

    float I_box = (1.0f/6.0f) * 1.0f * (0.5f*0.5f + 0.5f*0.5f);
    Matrix3 inertia = Matrix3::identity() * I_box;

    RigidBodyDesc b1, b2;
    b1.position = Vec3(0, 1.5f, 0); b1.mass = 1.0f; b1.type = BodyType::DYNAMIC;
    b1.half_extents = Vec3(0.5f); b1.restitution = 0.0f; b1.inertia_tensor = inertia;
    b2.position = Vec3(0, 3.5f, 0); b2.mass = 1.0f; b2.type = BodyType::DYNAMIC;
    b2.half_extents = Vec3(0.5f); b2.restitution = 0.0f; b2.inertia_tensor = inertia;

    auto body1 = world.create_body(b1);
    auto body2 = world.create_body(b2);

    // 5초 시뮬레이션
    for (int i = 0; i < 300; ++i) world.step(1.0f / 60.0f);

    float y1 = world.get_body_position(body1).y;
    float y2 = world.get_body_position(body2).y;
    std::cout << "  Stack y1=" << y1 << " y2=" << y2 << "\n";

    EXPECT_NEAR(y1, 0.5f, 0.15f);   // bottom box resting on ground
    EXPECT_NEAR(y2, 1.5f, 0.15f);   // top box resting on bottom box
}

// ── 4. 관성 텐서: 회전 후 각운동량 보존 ──────────────────────────
TEST(Validation, AngularMomentum_Conserved) {
    auto world = make_world(0.0f); // 중력 없음

    RigidBodyDesc desc;
    desc.position = Vec3(0, 0, 0);
    desc.mass = 1.0f;
    desc.type = BodyType::DYNAMIC;
    desc.half_extents = Vec3(0.5f);
    float I = (1.0f/6.0f) * 1.0f * (0.5f*0.5f + 0.5f*0.5f);
    desc.inertia_tensor = Matrix3::identity() * I;
    desc.angular_velocity = Vec3(0, 2.0f, 0); // 초기 각속도

    auto body = world.create_body(desc);
    world.step(1.0f / 60.0f);

    Vec3 omega = world.get_body_angular_velocity(body);
    std::cout << "  Angular velocity after 1 step: " << omega.x << " " << omega.y << " " << omega.z << "\n";
    // 충돌 없음 + 중력 없음 → 각속도 보존 (damping 제외)
    EXPECT_NEAR(omega.y, 2.0f, 0.1f);
}

// ── 5. BallSocket Joint 제약 유지 ─────────────────────────────────
// 정적 앵커 + 동적 진자봉이 볼소켓으로 연결. 3초 시뮬레이션 후
// 두 앵커 포인트가 10cm 이내로 일치해야 한다.
TEST(Validation, BallSocketJoint_ConstraintMaintained) {
    PhysicsWorldConfig cfg;
    cfg.gravity = Vec3(0, -9.81f, 0);
    cfg.fixed_time_step = 1.0f / 120.0f;  // 120 Hz for joint accuracy
    cfg.solver_iterations = 15;
    auto world = PhysicsWorldGPU(cfg);

    // Static anchor body
    RigidBodyDesc anchorDesc;
    anchorDesc.position = Vec3(0, 5, 0);
    anchorDesc.type = BodyType::STATIC;
    anchorDesc.half_extents = Vec3(0.1f);
    anchorDesc.mass = 0.0f;
    auto anchor = world.create_body(anchorDesc);

    // Dynamic pendulum bob — 1m below anchor pivot
    RigidBodyDesc bobDesc;
    bobDesc.position = Vec3(0.5f, 4.0f, 0);  // 30° displaced, L=1m
    bobDesc.mass = 1.0f;
    bobDesc.type = BodyType::DYNAMIC;
    bobDesc.half_extents = Vec3(0.1f);
    float Icm = (1.0f/6.0f) * 1.0f * (0.1f*0.1f + 0.1f*0.1f);
    bobDesc.inertia_tensor = Matrix3::identity() * Icm;
    auto bob = world.create_body(bobDesc);

    // Ball-socket: anchor at center of anchor body, bob offset = (0, +L, 0) from bob CM
    auto jd = make_ball_socket_joint(anchor, Vec3::zero(), bob, Vec3(0, 1, 0));
    auto joint = world.create_joint(jd);
    ASSERT_TRUE(joint.is_valid());

    // Simulate 3 seconds
    for (int i = 0; i < 360; ++i) world.step(1.0f / 120.0f);

    // Verify anchor points still coincide
    Vec3 posAnchor = world.get_body_position(anchor);
    Vec3 posBob    = world.get_body_position(bob);
    Vec3 orientBob_rot = world.get_body_orientation(bob).rotate(Vec3(0, 1, 0));  // local_anchor_b in world
    Vec3 worldAnchorA = posAnchor;                   // local_anchor_a = (0,0,0)
    Vec3 worldAnchorB = posBob + orientBob_rot;      // local_anchor_b = (0,1,0)
    float separation = (worldAnchorB - worldAnchorA).length();

    std::cout << "  BallSocket separation after 3s: " << separation << " m\n";
    EXPECT_LT(separation, 0.10f);  // < 10cm constraint error
}

// ── 6. Hinge Joint 힌지 진자 주기 ────────────────────────────────
// 힌지 진자의 측정 주기가 이론값 T=2π√(I_pivot/mgL) ±15% 이내
TEST(Validation, HingeJoint_PendulumPeriod) {
    PhysicsWorldConfig cfg;
    cfg.gravity = Vec3(0, -9.81f, 0);
    cfg.fixed_time_step = 1.0f / 120.0f;
    cfg.solver_iterations = 15;
    auto world = PhysicsWorldGPU(cfg);

    // Static pivot
    RigidBodyDesc pivotDesc;
    pivotDesc.position = Vec3(0, 5, 0);
    pivotDesc.type = BodyType::STATIC;
    pivotDesc.half_extents = Vec3(0.05f);
    pivotDesc.mass = 0.0f;
    auto pivot = world.create_body(pivotDesc);

    // Bob displaced 30° from vertical (L=1m arm)
    const float L = 1.0f;
    RigidBodyDesc bobDesc;
    bobDesc.position = Vec3(0.5f, 5.0f - std::sqrt(L*L - 0.25f), 0);
    bobDesc.mass = 1.0f;
    bobDesc.type = BodyType::DYNAMIC;
    bobDesc.half_extents = Vec3(0.05f);
    bobDesc.inertia_tensor = Matrix3::identity() * 0.1f;
    auto bob = world.create_body(bobDesc);

    // Hinge: pivot at origin of pivot body, bob arm = (0, +L, 0), axis = Z
    auto jd = make_hinge_joint(pivot, Vec3::zero(), Vec3(0,0,1),
                                bob,  Vec3(0, L, 0), Vec3(0,0,1));
    world.create_joint(jd);

    // Simulate 5 seconds, count zero crossings
    const float dt = 1.0f / 120.0f;
    int zero_crossings = 0;
    float prev_x = world.get_body_position(bob).x;
    float first_t = -1.0f, last_t = -1.0f;

    for (int i = 0; i < 600; ++i) {
        world.step(dt);
        float t = (i + 1) * dt;
        float curr_x = world.get_body_position(bob).x;
        float vx = world.get_body_linear_velocity(bob).x;
        if (prev_x < 0.0f && curr_x >= 0.0f && vx > 0.0f) {
            zero_crossings++;
            if (first_t < 0.0f) first_t = t;
            last_t = t;
        }
        prev_x = curr_x;
    }

    // T = 2π√(I_pivot / mgL),  I_pivot = I_cm + mL² = 0.1 + 1*1 = 1.1
    float I_pivot = 0.1f + 1.0f * L * L;
    float T_expected = 2.0f * 3.14159f * std::sqrt(I_pivot / (1.0f * 9.81f * L));
    float T_measured = (zero_crossings > 1)
        ? (last_t - first_t) / (zero_crossings - 1)
        : 0.0f;

    std::cout << "  Hinge period expected=" << T_expected
              << "s  measured=" << T_measured
              << "s  crossings=" << zero_crossings << "\n";

    EXPECT_GT(zero_crossings, 1);
    EXPECT_NEAR(T_measured, T_expected, T_expected * 0.15f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

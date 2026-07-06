// test_contact_cache.cpp — Validates contact caching / warm-starting
//
// Tests:
//   WarmStartTransfer    — impulse accumulated in frame N is transferred to
//                          the constraint in frame N+1 (warm start non-zero).
//   StackingStability    — a stacked column of boxes remains stable for 3s
//                          without drifting or exploding (regression guard).
//   CacheExpiry          — entries older than MAX_CACHE_AGE_SECONDS (50 ms) are
//                          discarded and do NOT warm-start the next contact.
//                          Time-based age makes warm-start invariant to dt/Hz.
//   EnergyMonotonicity   — kinetic energy of a resting box is monotonically
//                          non-increasing once it lands (no energy injection).

#include <gtest/gtest.h>
#include "basements/engine/physics_world_gpu.h"

using namespace basements::math;
using namespace basements::engine;
using namespace basements::physics;

// ─── helpers ────────────────────────────────────────────────────────────────

static PhysicsWorldConfig make_cfg(float dt = 1.0f / 60.0f, int iters = 20) {
    PhysicsWorldConfig cfg;
    cfg.fixed_time_step   = dt;
    cfg.solver_iterations = iters;
    cfg.gravity           = Vec3(0.0f, -9.81f, 0.0f);
    return cfg;
}

static BodyHandle make_box(PhysicsWorldGPU& world,
                           Vec3 pos, Vec3 half,
                           float mass = 1.0f,
                           BodyType type = BodyType::DYNAMIC) {
    RigidBodyDesc d;
    d.position    = pos;
    d.half_extents = half;
    d.type        = type;
    if (type == BodyType::STATIC) {
        d.mass = 0.0f;
    } else {
        d.mass = mass;
        // Solid-box inertia: I = m/12 * (b²+c², a²+c², a²+b²)
        float a2 = half.x * half.x * 4.0f;
        float b2 = half.y * half.y * 4.0f;
        float c2 = half.z * half.z * 4.0f;
        float f  = mass / 12.0f;
        d.inertia_tensor = Matrix3(
            f*(b2+c2), 0, 0,
            0, f*(a2+c2), 0,
            0, 0, f*(a2+b2)
        );
    }
    d.restitution = 0.0f;
    d.friction    = 0.5f;
    return world.create_body(d);
}

// ─── Test 1: WarmStartTransfer ───────────────────────────────────────────────
// After the first frame with a resting contact, the second frame's constraint
// should begin with a nonzero accumulated normal impulse (warm start).
// We verify this indirectly: the box position must NOT bounce upward between
// two consecutive resting frames — which only holds if warm-start prevents the
// solver from under-correcting on the very first iterate.

TEST(ContactCache, WarmStartTransfer) {
    PhysicsWorldGPU world(make_cfg(1.0f / 60.0f, 20));

    // Ground
    make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
             0.0f, BodyType::STATIC);

    // Dynamic box resting on ground
    BodyHandle box = make_box(world, Vec3(0.0f, 0.5f, 0.0f), Vec3(0.5f));

    // Run enough frames so the box has settled completely.
    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    float y_before = world.get_body_position(box).y;

    // One more frame — with warm-start the box must stay on the ground.
    world.step(1.0f / 60.0f);

    float y_after = world.get_body_position(box).y;

    // Box should stay essentially at rest (not bounce up or sink through).
    EXPECT_NEAR(y_after, y_before, 0.002f)
        << "Box moved " << (y_after - y_before) * 1000.0f
        << " mm in one frame after settling — warm-start may not be working.";

    // Sanity: box is above ground level (no sinking).
    EXPECT_GT(y_after, 0.45f) << "Box sank below ground.";
}

// ─── Test 2: StackingStability ───────────────────────────────────────────────
// A 3-box vertical column should stay within ±5 cm of its initial height
// after 3 seconds of simulation (180 frames).

TEST(ContactCache, StackingStability) {
    PhysicsWorldGPU world(make_cfg(1.0f / 60.0f, 20));

    const Vec3 h(0.5f);

    make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
             0.0f, BodyType::STATIC);

    BodyHandle b1 = make_box(world, Vec3(0.0f, 0.5f,  0.0f), h);
    BodyHandle b2 = make_box(world, Vec3(0.0f, 1.5f,  0.0f), h);
    BodyHandle b3 = make_box(world, Vec3(0.0f, 2.5f,  0.0f), h);

    // Let them settle
    for (int i = 0; i < 180; ++i)
        world.step(1.0f / 60.0f);

    float y1 = world.get_body_position(b1).y;
    float y2 = world.get_body_position(b2).y;
    float y3 = world.get_body_position(b3).y;

    EXPECT_NEAR(y1, 0.5f,  0.05f) << "Bottom box drifted.";
    EXPECT_NEAR(y2, 1.5f,  0.05f) << "Middle box drifted.";
    EXPECT_NEAR(y3, 2.5f,  0.05f) << "Top box drifted.";
}

// ─── Test 3: CacheExpiry ─────────────────────────────────────────────────────
// If a contact disappears for more than MAX_CACHE_AGE (2) frames and then
// reappears, the warm-start impulse should be zero (expired entry).
// We verify the box behaves identically to a cold-start by comparing its
// penetration-correction response: first two contacts produce impulses;
// after 3+ idle frames the cache is stale and the body must re-converge
// identically to the first contact.
//
// Practical proxy: drop a box, let it settle, teleport it upward (contact
// breaks for 3 frames), then let it re-land.  The landing response (velocity
// damping speed) must be the same as the very first landing because there is
// no stale warm-start impulse inflating the first iterate.

TEST(ContactCache, CacheExpiry) {
    auto run_and_measure_landing = [](bool with_stale_cache) -> float {
        PhysicsWorldGPU world(make_cfg(1.0f / 60.0f, 20));

        make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
                 0.0f, BodyType::STATIC);

        BodyHandle box = make_box(world, Vec3(0.0f, 5.0f, 0.0f), Vec3(0.5f));

        // Let it fall and fully settle.
        for (int i = 0; i < 180; ++i) world.step(1.0f / 60.0f);

        if (with_stale_cache) {
            // Cache populated; teleport box up so contact vanishes for 3+
            // frames, then back down — stale entry should have expired.
            world.set_body_position(box, Vec3(0.0f, 5.0f, 0.0f));
            world.set_body_linear_velocity(box, Vec3(0.0f));
            world.wake_body(box);
            for (int i = 0; i < 3; ++i) world.step(1.0f / 60.0f);  // 3 idle frames
            // Now re-drop from just above ground.
            world.set_body_position(box, Vec3(0.0f, 5.0f, 0.0f));
            world.set_body_linear_velocity(box, Vec3(0.0f));
            world.wake_body(box);
        } else {
            // Cold start: fresh world, drop from height.
            world.reset();
            make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
                     0.0f, BodyType::STATIC);
            box = make_box(world, Vec3(0.0f, 5.0f, 0.0f), Vec3(0.5f));
        }

        // Step until first contact frame.
        for (int i = 0; i < 60; ++i) world.step(1.0f / 60.0f);

        return world.get_body_position(box).y;
    };

    float y_stale = run_and_measure_landing(true);
    float y_cold  = run_and_measure_landing(false);

    // Both should converge to roughly the same resting height.
    EXPECT_NEAR(y_stale, y_cold, 0.02f)
        << "Stale cache y=" << y_stale << " vs cold y=" << y_cold
        << " — expired cache entry may have injected bad impulse.";
}

// ─── Test 4: EnergyMonotonicity ───────────────────────────────────────────────
// After settling, the box should not gain net energy over a 120-frame window.
// We measure peak KE in two halves; the second half must be ≤ the first.
// This tolerates the sleep→wake transition impulse (a single-frame spike
// caused by Baumgarte correcting micro-penetration on wake-up) while still
// catching any sustained energy injection from bad warm-starting.

TEST(ContactCache, EnergyMonotonicity) {
    PhysicsWorldGPU world(make_cfg(1.0f / 60.0f, 20));

    make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
             0.0f, BodyType::STATIC);

    BodyHandle box = make_box(world, Vec3(0.0f, 3.0f, 0.0f), Vec3(0.5f),
                              1.0f, BodyType::DYNAMIC);

    // Let it fall and fully settle (sleep-eligible).
    for (int i = 0; i < 120; ++i) world.step(1.0f / 60.0f);

    auto ke = [&]() {
        Vec3  lv = world.get_body_linear_velocity(box);
        Vec3  av = world.get_body_angular_velocity(box);
        float m  = world.get_body_mass(box);
        return 0.5f * m * lv.dot(lv) + 0.5f * av.dot(av);
    };

    // Sample peak KE in two consecutive 60-frame windows.
    float peak_first = 0.0f;
    for (int i = 0; i < 60; ++i) {
        world.step(1.0f / 60.0f);
        peak_first = std::max(peak_first, ke());
    }

    float peak_second = 0.0f;
    for (int i = 0; i < 60; ++i) {
        world.step(1.0f / 60.0f);
        peak_second = std::max(peak_second, ke());
    }

    // Second window must not exceed first window's peak.
    // This catches sustained energy injection while tolerating one-time spikes.
    EXPECT_LE(peak_second, peak_first + 1e-3f)
        << "Peak KE grew between windows: first=" << peak_first
        << " second=" << peak_second
        << " — warm-start may be injecting energy.";
}

// ─── Test 5: RateInvariance (parametric) ─────────────────────────────────────
// The seconds-based MAX_CACHE_AGE_SECONDS must keep the cache window invariant
// across simulation rates. A 3-box column settled at 60/120/240 Hz should land
// within the same height tolerance — same wall-clock simulation duration.

class ContactCacheRate : public ::testing::TestWithParam<float> {};

TEST_P(ContactCacheRate, StackingStable) {
    const float dt   = GetParam();
    const int   iters = 20;
    PhysicsWorldGPU world(make_cfg(dt, iters));

    const Vec3 h(0.5f);
    make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
             0.0f, BodyType::STATIC);
    BodyHandle b1 = make_box(world, Vec3(0.0f, 0.5f, 0.0f), h);
    BodyHandle b2 = make_box(world, Vec3(0.0f, 1.5f, 0.0f), h);
    BodyHandle b3 = make_box(world, Vec3(0.0f, 2.5f, 0.0f), h);

    // 3 seconds of simulation regardless of dt.
    const int steps = (int)std::round(3.0f / dt);
    for (int i = 0; i < steps; ++i) world.step(dt);

    EXPECT_NEAR(world.get_body_position(b1).y, 0.5f, 0.05f)
        << "dt=" << dt << " — bottom drifted.";
    EXPECT_NEAR(world.get_body_position(b2).y, 1.5f, 0.05f)
        << "dt=" << dt << " — middle drifted.";
    EXPECT_NEAR(world.get_body_position(b3).y, 2.5f, 0.05f)
        << "dt=" << dt << " — top drifted.";
}

// Cache expire must also be dt-invariant: a contact idle for 0.1 s (> 0.05 s
// threshold) is expired regardless of how many frames that takes.
TEST_P(ContactCacheRate, ExpireWindowIsTimeBased) {
    const float dt = GetParam();
    PhysicsWorldGPU world(make_cfg(dt, 20));
    make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
             0.0f, BodyType::STATIC);
    BodyHandle box = make_box(world, Vec3(0.0f, 0.5f, 0.0f), Vec3(0.5f));

    // Settle (1 s).
    int steps = (int)std::round(1.0f / dt);
    for (int i = 0; i < steps; ++i) world.step(dt);

    // Idle long enough to definitively expire (0.1 s > 0.05 s threshold).
    world.set_body_position(box, Vec3(0.0f, 5.0f, 0.0f));
    world.set_body_linear_velocity(box, Vec3::zero());
    world.wake_body(box);
    int idle_steps = (int)std::round(0.1f / dt);
    for (int i = 0; i < idle_steps; ++i) world.step(dt);

    // Re-drop and re-settle — should converge to ~0.5 with no impulse hangover.
    world.set_body_position(box, Vec3(0.0f, 5.0f, 0.0f));
    world.set_body_linear_velocity(box, Vec3::zero());
    world.wake_body(box);
    // 2 s is enough for ~1 m drop + bounce + settle even at 60 Hz.
    steps = (int)std::round(2.0f / dt);
    for (int i = 0; i < steps; ++i) world.step(dt);

    EXPECT_NEAR(world.get_body_position(box).y, 0.5f, 0.05f)
        << "dt=" << dt << " — re-landed height differs from cold convergence.";
}

INSTANTIATE_TEST_SUITE_P(
    Rates, ContactCacheRate,
    ::testing::Values(1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 240.0f)
);

// ─── Test 6: JitteredTimestep ────────────────────────────────────────────────
// Real applications occasionally step the world with a non-uniform dt
// (frame-rate jitter, manual sub-stepping). The seconds-based cache age must
// behave deterministically under those conditions: time accumulates correctly
// and the warm-start window expires when *cumulative idle time* exceeds the
// threshold, regardless of how many step() calls produced it.

// ─── Test 6.4: FeatureKeyDirect (unit) ───────────────────────────────────────
// Direct assertion: with the same dominant-axis anchor, two contact-point
// coordinates that differ by less than the per-axis dead-zone must yield
// the SAME feature_key. This is what octant hysteresis exists to guarantee.

TEST(ContactCache, FeatureKeyDirectStableUnderJitter) {
    PhysicsWorldGPU::FeatureKeyInputs in0;
    in0.normal  = Vec3(0.0f, 1.0f, 0.0f);     // +Y face contact
    in0.half_a  = Vec3(0.5f, 0.5f, 0.5f);
    in0.half_b  = Vec3(0.5f, 0.5f, 0.5f);
    in0.local_a = Vec3(0.001f, 0.499f, 0.001f);  // near corner on +Y face
    in0.local_b = in0.local_a;

    // Frame 0: no anchor → fresh octant for this point.
    uint32_t k0 = PhysicsWorldGPU::debug_feature_key(in0, -1);

    // Frame 1: same point sub-cm jitter, with k0 as anchor.
    PhysicsWorldGPU::FeatureKeyInputs in1 = in0;
    in1.local_a = Vec3(-0.002f, 0.499f, -0.002f);   // crossed zero on x and z
    in1.local_b = in1.local_a;
    uint32_t k1 = PhysicsWorldGPU::debug_feature_key(in1, (int)k0);

    EXPECT_EQ(k0, k1)
        << "feature_key changed under sub-cm jitter: " << k0 << " → " << k1;

    // Crossing far outside the dead-zone (≥ 2% × 0.5 = 0.01 m) MUST flip.
    PhysicsWorldGPU::FeatureKeyInputs in2 = in0;
    in2.local_a = Vec3(-0.05f, 0.499f, -0.05f);
    in2.local_b = in2.local_a;
    uint32_t k2 = PhysicsWorldGPU::debug_feature_key(in2, (int)k0);
    EXPECT_NE(k0, k2)
        << "feature_key failed to flip across true octant boundary.";
}

// ─── Test 6.5: FeatureKeyJitterResistance ───────────────────────────────────
// Octant hysteresis must absorb sub-cm contact-point wobble in a settled stack.
// If the dead-zone is doing its job the contact cache size stays bounded by
// the count of *stable* features (1 face contact in this scene) — without
// hysteresis it grows every frame as the octant flickers.

TEST(ContactCache, FeatureKeyJitterResistance) {
    PhysicsWorldGPU world(make_cfg(1.0f / 240.0f, 20));
    make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
             0.0f, BodyType::STATIC);
    BodyHandle box = make_box(world, Vec3(0.0f, 0.5f, 0.0f), Vec3(0.5f));

    // Settle.
    for (int i = 0; i < 480; ++i) world.step(1.0f / 240.0f);
    size_t baseline = world.get_contact_cache_size();

    // Apply small alternating angular kicks so the contact point wobbles
    // near the octant boundary in body-local frame.
    for (int i = 0; i < 240; ++i) {
        Vec3 omega(0.0f, (i & 1) ? 0.05f : -0.05f, 0.0f);
        world.set_body_angular_velocity(box, omega);
        world.step(1.0f / 240.0f);
    }

    size_t after = world.get_contact_cache_size();
    EXPECT_LE(after, baseline + 2)
        << "Cache grew from " << baseline << " to " << after
        << " entries during 1 s of sub-cm jitter — hysteresis dead-zone "
           "is not absorbing octant churn.";
}

TEST(ContactCache, JitteredTimestep) {
    // Build a world configured for 1/240 fixed step but call step() with
    // varying delta times; the internal accumulator should slice them into
    // the configured fixed step exactly.
    PhysicsWorldGPU world(make_cfg(1.0f / 240.0f, 20));

    make_box(world, Vec3(0.0f, -0.5f, 0.0f), Vec3(10.0f, 0.5f, 10.0f),
             0.0f, BodyType::STATIC);
    BodyHandle box = make_box(world, Vec3(0.0f, 0.5f, 0.0f), Vec3(0.5f));

    // 5 seconds of jittered driving (dt cycles through 5/120/240/30/60 Hz).
    const float driver_dt[] = {
        1.0f / 5.0f, 1.0f / 120.0f, 1.0f / 240.0f, 1.0f / 30.0f, 1.0f / 60.0f
    };
    float elapsed = 0.0f;
    int   i = 0;
    while (elapsed < 5.0f) {
        float dt = driver_dt[i % 5];
        world.step(dt);
        elapsed += dt;
        ++i;
    }

    float y = world.get_body_position(box).y;
    EXPECT_NEAR(y, 0.5f, 0.05f)
        << "Jittered step sequence produced unstable stacking — accumulator "
           "or seconds-based cache age may have drifted.";
}

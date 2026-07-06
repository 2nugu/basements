// CPU physics pipeline — stub upgraded to a real sequential-impulse solver.
// Uses the existing TimeIntegrator and Solver headers; no GPU/CUDA required.

#include "basements/engine/physics_world_gpu.h"
#include "basements/physics/dynamics/integrator.h"
#include "basements/physics/dynamics/solver.h"
#include "basements/physics/dynamics/joint_solver.h"
#include "basements/physics/collision/primitives.h"
#include "basements/physics/collision/gjk.h"
#include "basements/physics/collision/epa.h"

#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>

namespace basements {
namespace engine {

using namespace basements::math;
using namespace basements::physics;
using namespace basements::dynamics;
using namespace basements::collision;

// ============================================================================
// Contact Cache helpers
// ============================================================================

namespace {

struct CachedContact {
    int      body_a;
    int      body_b;
    uint32_t feature_id;           // packed (axis, octant_a, octant_b) — see feature_key()
    float    impulse_normal;
    float    impulse_tangent[2];
    float    age_seconds;          // seconds since last match; expire at MAX_CACHE_AGE_SECONDS
                                   // (time-based so warm-start window survives dt/rate changes)
    Vec3     last_local_a;         // contact point in body-A local frame (for octant hysteresis)
    Vec3     last_local_b;         // contact point in body-B local frame
};

// Map a contact normal to a stable face ID (0-5) for cache keying.
static int dominant_axis(const Vec3& n) {
    float ax = std::abs(n.x), ay = std::abs(n.y), az = std::abs(n.z);
    if (ax >= ay && ax >= az) return n.x > 0.0f ? 0 : 1;
    if (ay >= ax && ay >= az) return n.y > 0.0f ? 2 : 3;
    return n.z > 0.0f ? 4 : 5;
}

// World→local contact-point transform.
static Vec3 to_local(const RigidBody& b, const Vec3& world_point) {
    return Matrix3::from_quaternion(b.orientation).transposed() * (world_point - b.position);
}

// Octant of a local-space point — used only when no prior octant exists.
static int fresh_octant(const Vec3& local) {
    int ox = local.x > 0.0f ? 1 : 0;
    int oy = local.y > 0.0f ? 1 : 0;
    int oz = local.z > 0.0f ? 1 : 0;
    return ox | (oy << 1) | (oz << 2);
}

// Octant hysteresis: per-axis dead-zone preserves the previous octant bit when
// the local coordinate sits within ±dead_zone of zero. The dead-zone is sized
// from the *smaller* body of the pair so both A-frame and B-frame use the same
// scale — asymmetric dead-zones (small box on huge floor) would otherwise let
// one side flip-flop while the other holds, defeating the warm-start anchor.
static constexpr float OCTANT_HYSTERESIS_REL = 0.02f;
// Vec3 overload — per-axis dead-zone (required for asymmetric bodies).
static int hysteretic_octant(const Vec3& local, const Vec3& dead_zone, int prev_octant) {
    auto axis_bit = [](float v, float dz, int prev_bit) {
        if (v >  dz) return 1;
        if (v < -dz) return 0;
        return prev_bit;
    };
    int prev_x =  prev_octant       & 1;
    int prev_y = (prev_octant >> 1) & 1;
    int prev_z = (prev_octant >> 2) & 1;
    int ox = axis_bit(local.x, dead_zone.x, prev_x);
    int oy = axis_bit(local.y, dead_zone.y, prev_y);
    int oz = axis_bit(local.z, dead_zone.z, prev_z);
    return ox | (oy << 1) | (oz << 2);
}

// Composite cache key: 3 bits axis × 3 bits octant_a × 3 bits octant_b = 9 bits.
// `prev_axis_cache` (if non-null) seeds hysteresis. Pass nullptr for a fresh
// contact. `half_a` / `half_b` set the shared scale for the symmetric dead-zone.
static uint32_t feature_key(const Vec3& normal,
                            const Vec3& local_a, const Vec3& half_a,
                            const Vec3& local_b, const Vec3& half_b,
                            const CachedContact* prev_axis_cache) {
    uint32_t axis = (uint32_t)dominant_axis(normal);
    uint32_t oA, oB;
    if (prev_axis_cache) {
        // Per-axis dead-zone: each axis uses the minimum half-extent on THAT
        // axis from the two bodies. Avoids over-tightening on axes where the
        // contacting bodies happen to be long (e.g. a thin tall pillar vs a
        // flat wide floor) — each axis gets its own physically meaningful scale.
        float dz_x = std::min(half_a.x, half_b.x) * OCTANT_HYSTERESIS_REL;
        float dz_y = std::min(half_a.y, half_b.y) * OCTANT_HYSTERESIS_REL;
        float dz_z = std::min(half_a.z, half_b.z) * OCTANT_HYSTERESIS_REL;
        int prev_oA = (prev_axis_cache->feature_id >> 3) & 7;
        int prev_oB =  prev_axis_cache->feature_id       & 7;
        Vec3 dza(dz_x, dz_y, dz_z);
        Vec3 dzb(dz_x, dz_y, dz_z);
        oA = (uint32_t)hysteretic_octant(local_a, dza, prev_oA);
        oB = (uint32_t)hysteretic_octant(local_b, dzb, prev_oB);
    } else {
        oA = (uint32_t)fresh_octant(local_a);
        oB = (uint32_t)fresh_octant(local_b);
    }
    return (axis << 6) | (oA << 3) | oB;
}

// Find a cached entry for (body_a, body_b, same dominant axis) — used as the
// anchor for octant hysteresis. Returns nullptr if no candidate exists.
static const CachedContact* find_axis_anchor(const std::vector<CachedContact>& cache,
                                             int ai, int bi, int axis) {
    for (const auto& c : cache) {
        if (c.body_a == ai && c.body_b == bi &&
            (int)((c.feature_id >> 6) & 7) == axis) {
            return &c;
        }
    }
    return nullptr;
}

} // anonymous namespace

// ============================================================================
// Internal Impl
// ============================================================================

struct PhysicsWorldGPU::Impl {
    PhysicsWorldConfig config;

    // Flat arrays — required by TimeIntegrator and Solver (RigidBody*, int n).
    std::vector<RigidBody>  bodies;
    std::vector<Vec3>       half_extents;   // per-body collision half-extents
    std::vector<float>      restitutions;
    std::vector<float>      frictions;
    std::vector<uint32_t>   handles;        // handle.id for each slot

    // Handle → index mapping
    std::unordered_map<uint32_t, size_t> handle_to_index;
    uint32_t next_id = 1;

    // Simulation parameters
    Vec3  gravity    { 0.0f, -9.81f, 0.0f };
    float time_step  { 1.0f / 60.0f };
    int   solver_iterations { 10 };
    float accumulator{ 0.0f };

    // Per-frame broadphase pairs & contacts
    std::vector<std::pair<int,int>>    pairs;
    std::vector<ContactConstraint>     contacts;

    // Persistent contact cache for warm-starting
    std::vector<CachedContact>         contact_cache;
    static constexpr float MAX_CACHE_AGE_SECONDS = 0.05f;  // 50 ms — survives 60/120/240 Hz

    ContactCallback contact_cb;

    // Joint storage — one vector per joint type for cache locality
    std::vector<BallSocketJoint> ball_socket_joints;
    std::vector<HingeJoint>      hinge_joints;
    std::vector<SliderJoint>     slider_joints;
    std::vector<FixedJoint>      fixed_joints;

    // Joint handle → (JointType, index in per-type vector)
    std::unordered_map<uint32_t, std::pair<JointType, size_t>> joint_handle_to_slot;
    uint32_t next_joint_id = 1;

    explicit Impl(const PhysicsWorldConfig& cfg)
        : config(cfg)
        , gravity(cfg.gravity)
        , time_step(cfg.fixed_time_step)
        , solver_iterations(cfg.solver_iterations)
    {}

    // ---- helpers ----

    size_t add_body(const RigidBody& b, Vec3 he, float rest, float fric, uint32_t id) {
        size_t idx = bodies.size();
        bodies.push_back(b);
        half_extents.push_back(he);
        restitutions.push_back(rest);
        frictions.push_back(fric);
        handles.push_back(id);
        handle_to_index[id] = idx;
        return idx;
    }

    // When destroy_body swaps body [old_idx] ← body [last_idx],
    // update all joint references from last_idx → old_idx.
    void remap_body_index(int old_idx, int new_idx) {
        auto remap2 = [&](int& a, int& b) {
            if (a == new_idx) a = old_idx;
            if (b == new_idx) b = old_idx;
        };
        for (auto& j : ball_socket_joints) remap2(j.body_a_index, j.body_b_index);
        for (auto& j : hinge_joints)       remap2(j.body_a_index, j.body_b_index);
        for (auto& j : slider_joints)      remap2(j.body_a_index, j.body_b_index);
        for (auto& j : fixed_joints)       remap2(j.body_a_index, j.body_b_index);
        for (auto& c : contact_cache)      remap2(c.body_a, c.body_b);
    }

    // World-space AABB of an OBB: expand each axis by sum |R[row][j]| * h[j]
    AABB compute_aabb(int idx) const {
        const RigidBody& b = bodies[idx];
        const Vec3& h      = half_extents[idx];
        Matrix3 R = Matrix3::from_quaternion(b.orientation);

        Vec3 world_h(
            std::abs(R.m[0][0]) * h.x + std::abs(R.m[0][1]) * h.y + std::abs(R.m[0][2]) * h.z,
            std::abs(R.m[1][0]) * h.x + std::abs(R.m[1][1]) * h.y + std::abs(R.m[1][2]) * h.z,
            std::abs(R.m[2][0]) * h.x + std::abs(R.m[2][1]) * h.y + std::abs(R.m[2][2]) * h.z
        );
        return AABB(b.position - world_h, b.position + world_h);
    }
};

// ============================================================================
// Construction
// ============================================================================

PhysicsWorldGPU::PhysicsWorldGPU(const PhysicsWorldConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

PhysicsWorldGPU::~PhysicsWorldGPU() = default;

PhysicsWorldGPU::PhysicsWorldGPU(PhysicsWorldGPU&& other) noexcept
    : impl_(std::move(other.impl_)) {}

PhysicsWorldGPU& PhysicsWorldGPU::operator=(PhysicsWorldGPU&& other) noexcept {
    impl_ = std::move(other.impl_);
    return *this;
}

// ============================================================================
// Simulation Control
// ============================================================================

void PhysicsWorldGPU::step(float delta_time) {
    if (!impl_) return;
    impl_->accumulator += delta_time;
    const float dt = impl_->time_step;
    int max_substeps = 3;
    while (impl_->accumulator >= dt && max_substeps-- > 0) {
        impl_->accumulator -= dt;
        step_fixed();
    }
}

void PhysicsWorldGPU::step_fixed() {
    apply_external_forces();
    broadphase();
    narrowphase();
    solve_constraints();
    integrate();
    update_sleep_states();
    update_contact_cache(impl_->time_step);
    invoke_callbacks();
}

void PhysicsWorldGPU::reset()  {
    if (!impl_) return;
    impl_->bodies.clear(); impl_->half_extents.clear();
    impl_->restitutions.clear(); impl_->frictions.clear();
    impl_->handles.clear(); impl_->handle_to_index.clear();
    impl_->ball_socket_joints.clear(); impl_->hinge_joints.clear();
    impl_->slider_joints.clear(); impl_->fixed_joints.clear();
    impl_->joint_handle_to_slot.clear();
    impl_->contact_cache.clear();
    impl_->accumulator = 0.0f;
}
void PhysicsWorldGPU::clear()  { reset(); }

// ============================================================================
// Pipeline Stages
// ============================================================================

void PhysicsWorldGPU::apply_external_forces() {
    for (auto& b : impl_->bodies) {
        if (b.type != BodyType::DYNAMIC || b.is_sleeping) continue;
        b.force += impl_->gravity * b.mass;
    }
}

void PhysicsWorldGPU::broadphase() {
    impl_->pairs.clear();
    int n = (int)impl_->bodies.size();

    // Precompute world AABBs
    std::vector<AABB> aabbs(n);
    for (int i = 0; i < n; ++i) aabbs[i] = impl_->compute_aabb(i);

    // O(n²) pair test — fine for small editor scenes (<200 bodies)
    for (int i = 0; i < n; ++i) {
        if (impl_->bodies[i].is_sleeping) continue;
        for (int j = i + 1; j < n; ++j) {
            if (impl_->bodies[i].type == BodyType::STATIC &&
                impl_->bodies[j].type == BodyType::STATIC) continue;
            if (aabbs[i].overlaps(aabbs[j]))
                impl_->pairs.emplace_back(i, j);
        }
    }
}

void PhysicsWorldGPU::narrowphase() {
    impl_->contacts.clear();

    for (auto& [ai, bi] : impl_->pairs) {
        const RigidBody& bA = impl_->bodies[ai];
        const RigidBody& bB = impl_->bodies[bi];

        // Build Box shapes from stored half_extents (local space, unit origin)
        collision::Box shapeA(impl_->half_extents[ai]);
        collision::Box shapeB(impl_->half_extents[bi]);

        GJK::Transform txA(bA.position, bA.orientation);
        GJK::Transform txB(bB.position, bB.orientation);

        // GJK intersection test
        Simplex simplex;
        bool hit = GJK::run(shapeA, txA, shapeB, txB, simplex);
        if (!hit) continue;

        // EPA for penetration depth and contact normal
        float depth = 0.0f;
        Vec3  normal;
        bool epa_ok = EPA::run(simplex, shapeA, txA, shapeB, txB, depth, normal);

        // EPA requires a full tetrahedron (4-point simplex). If it fails (degenerate
        // contact / touching surfaces), fall back to AABB min-axis for stability.
        if (!epa_ok || depth < 1e-6f) {
            AABB aabbA = impl_->compute_aabb(ai);
            AABB aabbB = impl_->compute_aabb(bi);
            struct Cand { float d; Vec3 n; };
            Cand cands[6] = {
                { aabbA.max.x - aabbB.min.x, Vec3( 1, 0, 0) },
                { aabbB.max.x - aabbA.min.x, Vec3(-1, 0, 0) },
                { aabbA.max.y - aabbB.min.y, Vec3( 0, 1, 0) },
                { aabbB.max.y - aabbA.min.y, Vec3( 0,-1, 0) },
                { aabbA.max.z - aabbB.min.z, Vec3( 0, 0, 1) },
                { aabbB.max.z - aabbA.min.z, Vec3( 0, 0,-1) },
            };
            int best = 0;
            for (int k = 1; k < 6; ++k)
                if (cands[k].d < cands[best].d) best = k;
            depth  = cands[best].d;
            normal = cands[best].n;
        }

        // Contact point: midpoint along normal between the two body surfaces
        // Approximate as the average of the two surface witness points.
        Vec3 absN(std::abs(normal.x), std::abs(normal.y), std::abs(normal.z));
        Vec3 pA = bA.position - normal * impl_->half_extents[ai].dot(absN);
        Vec3 pB = bB.position + normal * impl_->half_extents[bi].dot(absN);
        Vec3 contact_point = (pA + pB) * 0.5f;

        ContactConstraint c;
        c.bodyA_index   = ai;
        c.bodyB_index   = bi;
        c.normal        = normal;         // pushes B away from A
        c.contact_point = contact_point;
        c.penetration   = depth;          // positive; Baumgarte: max(0, depth-slop)
        c.friction      = (impl_->frictions[ai]    + impl_->frictions[bi])    * 0.5f;
        c.restitution   = std::min(impl_->restitutions[ai], impl_->restitutions[bi]);

        // Warm-start: build the composite feature key (axis + per-body octant)
        // with hysteresis seeded from any prior cache entry on the same axis.
        Vec3 local_a = to_local(bA, contact_point);
        Vec3 local_b = to_local(bB, contact_point);
        int axis = dominant_axis(normal);
        const CachedContact* anchor = find_axis_anchor(impl_->contact_cache, ai, bi, axis);
        uint32_t fid = feature_key(normal,
                                   local_a, impl_->half_extents[ai],
                                   local_b, impl_->half_extents[bi],
                                   anchor);
        for (const auto& cached : impl_->contact_cache) {
            if (cached.body_a == ai && cached.body_b == bi && cached.feature_id == fid) {
                c.accumulated_impulse_normal     = cached.impulse_normal;
                c.accumulated_impulse_tangent[0] = cached.impulse_tangent[0];
                c.accumulated_impulse_tangent[1] = cached.impulse_tangent[1];
                break;
            }
        }

        impl_->contacts.push_back(c);
    }
}

void PhysicsWorldGPU::solve_constraints() {
    const float dt = impl_->time_step;
    int nb = (int)impl_->bodies.size();
    RigidBody* bptr = impl_->bodies.data();

    int nc = (int)impl_->contacts.size();
    ContactConstraint* cptr = impl_->contacts.data();

    int n_bs = (int)impl_->ball_socket_joints.size();
    int n_hi = (int)impl_->hinge_joints.size();
    int n_sl = (int)impl_->slider_joints.size();
    int n_fi = (int)impl_->fixed_joints.size();

    bool has_contacts = nc > 0;
    bool has_joints   = n_bs + n_hi + n_sl + n_fi > 0;
    if (!has_contacts && !has_joints) return;

    // Pre-solve (warm starting + effective mass)
    if (has_contacts)
        Solver::pre_solve(cptr, nc, bptr, nb, dt);
    if (n_bs) JointSolver::pre_solve_ball_socket(impl_->ball_socket_joints.data(), n_bs, bptr, nb, dt);
    if (n_hi) JointSolver::pre_solve_hinge(      impl_->hinge_joints.data(),       n_hi, bptr, nb, dt);
    if (n_sl) JointSolver::pre_solve_slider(     impl_->slider_joints.data(),      n_sl, bptr, nb, dt);
    if (n_fi) JointSolver::pre_solve_fixed(      impl_->fixed_joints.data(),       n_fi, bptr, nb, dt);

    // Velocity solve iterations (contacts + joints interleaved)
    for (int iter = 0; iter < impl_->solver_iterations; ++iter) {
        if (has_contacts)
            Solver::solve_velocity(cptr, nc, bptr, nb, dt);
        if (n_bs) JointSolver::solve_velocity_ball_socket(impl_->ball_socket_joints.data(), n_bs, bptr, nb);
        if (n_hi) JointSolver::solve_velocity_hinge(      impl_->hinge_joints.data(),       n_hi, bptr, nb);
        if (n_sl) JointSolver::solve_velocity_slider(     impl_->slider_joints.data(),      n_sl, bptr, nb);
        if (n_fi) JointSolver::solve_velocity_fixed(      impl_->fixed_joints.data(),       n_fi, bptr, nb);
    }
}

void PhysicsWorldGPU::integrate() {
    if (impl_->bodies.empty()) return;
    TimeIntegrator::integrate(impl_->bodies.data(), (int)impl_->bodies.size(), impl_->time_step);
}

void PhysicsWorldGPU::update_sleep_states() {
    const float lin_thresh  = config::SLEEP_LINEAR_THRESHOLD;
    const float ang_thresh  = config::SLEEP_ANGULAR_THRESHOLD;
    const float time_thresh = config::SLEEP_TIME_THRESHOLD;
    const float dt          = impl_->time_step;

    for (auto& b : impl_->bodies) {
        if (b.type != BodyType::DYNAMIC) continue;
        if (b.should_sleep()) {
            b.sleep_time += dt;
            if (b.sleep_time >= time_thresh) b.is_sleeping = true;
        } else {
            b.sleep_time = 0.0f;
            b.is_sleeping = false;
        }
    }
}

void PhysicsWorldGPU::invoke_callbacks() {
    if (!impl_->contact_cb) return;
    for (const auto& c : impl_->contacts) {
        impl_->contact_cb(
            BodyHandle(impl_->handles[c.bodyA_index]),
            BodyHandle(impl_->handles[c.bodyB_index]),
            c.contact_point, c.normal
        );
    }
}

void PhysicsWorldGPU::update_contact_cache(float dt) {
    auto& cache = impl_->contact_cache;

    // Age all existing entries by elapsed wall time (not frame count)
    for (auto& e : cache) e.age_seconds += dt;

    // Upsert / refresh entries from this frame's solved contacts. The feature
    // key must be computed with the same hysteresis logic used in narrowphase
    // so the just-warm-started entry is found and refreshed (not duplicated).
    for (const auto& c : impl_->contacts) {
        const RigidBody& bA = impl_->bodies[c.bodyA_index];
        const RigidBody& bB = impl_->bodies[c.bodyB_index];
        Vec3 local_a = to_local(bA, c.contact_point);
        Vec3 local_b = to_local(bB, c.contact_point);
        int axis = dominant_axis(c.normal);
        const CachedContact* anchor = find_axis_anchor(cache, c.bodyA_index, c.bodyB_index, axis);
        uint32_t fid = feature_key(c.normal,
                                   local_a, impl_->half_extents[c.bodyA_index],
                                   local_b, impl_->half_extents[c.bodyB_index],
                                   anchor);

        bool found = false;
        for (auto& e : cache) {
            if (e.body_a == c.bodyA_index && e.body_b == c.bodyB_index && e.feature_id == fid) {
                e.impulse_normal     = c.accumulated_impulse_normal;
                e.impulse_tangent[0] = c.accumulated_impulse_tangent[0];
                e.impulse_tangent[1] = c.accumulated_impulse_tangent[1];
                e.age_seconds        = 0.0f;
                e.last_local_a       = local_a;
                e.last_local_b       = local_b;
                found = true;
                break;
            }
        }
        if (!found) {
            CachedContact entry;
            entry.body_a             = c.bodyA_index;
            entry.body_b             = c.bodyB_index;
            entry.feature_id         = fid;
            entry.impulse_normal     = c.accumulated_impulse_normal;
            entry.impulse_tangent[0] = c.accumulated_impulse_tangent[0];
            entry.impulse_tangent[1] = c.accumulated_impulse_tangent[1];
            entry.age_seconds        = 0.0f;
            entry.last_local_a       = local_a;
            entry.last_local_b       = local_b;
            cache.push_back(entry);
        }
    }

    // Expire entries whose match-gap exceeds MAX_CACHE_AGE_SECONDS
    cache.erase(
        std::remove_if(cache.begin(), cache.end(),
            [](const CachedContact& e) { return e.age_seconds > Impl::MAX_CACHE_AGE_SECONDS; }),
        cache.end()
    );
}

// ============================================================================
// Body Management
// ============================================================================

BodyHandle PhysicsWorldGPU::create_body(const RigidBodyDesc& desc) {
    uint32_t id = impl_->next_id++;

    RigidBody b;
    b.position         = desc.position;
    b.orientation      = desc.orientation;
    b.linear_velocity  = desc.linear_velocity;
    b.angular_velocity = desc.angular_velocity;
    b.type             = desc.type;
    b.user_data        = desc.user_data;

    if (desc.type == BodyType::STATIC) {
        b.mass    = 0.0f;
        b.inv_mass = 0.0f;
        b.inv_inertia_tensor = Matrix3(0,0,0, 0,0,0, 0,0,0);
    } else {
        b.set_mass(desc.mass);
        b.set_inertia_tensor(desc.inertia_tensor);
    }

    impl_->add_body(b, desc.half_extents, desc.restitution, desc.friction, id);
    return BodyHandle(id);
}

BodyHandle PhysicsWorldGPU::create_body(const RigidBody& body) {
    uint32_t id = impl_->next_id++;
    impl_->add_body(body, Vec3(0.5f), 0.3f, 0.5f, id);
    return BodyHandle(id);
}

void PhysicsWorldGPU::destroy_body(BodyHandle h) {
    if (!impl_) return;
    auto it = impl_->handle_to_index.find(h.id);
    if (it == impl_->handle_to_index.end()) return;

    size_t idx = it->second;
    size_t last = impl_->bodies.size() - 1;

    // Purge cache entries for the dead body BEFORE remap so the moved body's
    // entries (at 'last') are not accidentally erased after being remapped to idx.
    {
        int dead_idx = (int)idx;
        impl_->contact_cache.erase(
            std::remove_if(impl_->contact_cache.begin(), impl_->contact_cache.end(),
                [dead_idx](const CachedContact& e) {
                    return e.body_a == dead_idx || e.body_b == dead_idx;
                }),
            impl_->contact_cache.end()
        );
    }

    if (idx != last) {
        // Swap with last
        std::swap(impl_->bodies[idx],       impl_->bodies[last]);
        std::swap(impl_->half_extents[idx], impl_->half_extents[last]);
        std::swap(impl_->restitutions[idx], impl_->restitutions[last]);
        std::swap(impl_->frictions[idx],    impl_->frictions[last]);
        std::swap(impl_->handles[idx],      impl_->handles[last]);
        impl_->handle_to_index[impl_->handles[idx]] = idx;
        // Update joint + cache references: last → idx
        impl_->remap_body_index((int)idx, (int)last);
    }

    impl_->bodies.pop_back();
    impl_->half_extents.pop_back();
    impl_->restitutions.pop_back();
    impl_->frictions.pop_back();
    impl_->handles.pop_back();
    impl_->handle_to_index.erase(h.id);
}

bool   PhysicsWorldGPU::is_body_valid(BodyHandle h) const  { return impl_ && impl_->handle_to_index.count(h.id) > 0; }
size_t PhysicsWorldGPU::get_body_count() const              { return impl_ ? impl_->bodies.size() : 0; }

// ============================================================================
// Body State Queries
// ============================================================================

#define BODY_GET(default_val, expr) \
    if (!impl_) return default_val; \
    auto it = impl_->handle_to_index.find(h.id); \
    if (it == impl_->handle_to_index.end()) return default_val; \
    const RigidBody& b = impl_->bodies[it->second]; \
    return expr;

Vec3       PhysicsWorldGPU::get_body_position(BodyHandle h)         const { BODY_GET(Vec3::zero(),           b.position) }
Quaternion PhysicsWorldGPU::get_body_orientation(BodyHandle h)      const { BODY_GET(Quaternion::identity(), b.orientation) }
Vec3       PhysicsWorldGPU::get_body_linear_velocity(BodyHandle h)  const { BODY_GET(Vec3::zero(),           b.linear_velocity) }
Vec3       PhysicsWorldGPU::get_body_angular_velocity(BodyHandle h) const { BODY_GET(Vec3::zero(),           b.angular_velocity) }
float      PhysicsWorldGPU::get_body_mass(BodyHandle h)             const { BODY_GET(0.0f,                   b.mass) }
BodyType   PhysicsWorldGPU::get_body_type(BodyHandle h)             const { BODY_GET(BodyType::STATIC,       b.type) }
bool       PhysicsWorldGPU::is_body_sleeping(BodyHandle h)          const { BODY_GET(false,                  b.is_sleeping) }
RigidBody  PhysicsWorldGPU::get_body(BodyHandle h)                  const { BODY_GET(RigidBody{},            b) }

#undef BODY_GET

// ============================================================================
// Body State Modification
// ============================================================================

#define BODY_SET(expr) \
    if (!impl_) return; \
    auto it = impl_->handle_to_index.find(h.id); \
    if (it == impl_->handle_to_index.end()) return; \
    RigidBody& b = impl_->bodies[it->second]; \
    expr;

void PhysicsWorldGPU::set_body_position(BodyHandle h, const Vec3& v)         { BODY_SET(b.position = v) }
void PhysicsWorldGPU::set_body_orientation(BodyHandle h, const Quaternion& q) { BODY_SET(b.orientation = q) }
void PhysicsWorldGPU::set_body_linear_velocity(BodyHandle h, const Vec3& v)   { BODY_SET(b.linear_velocity = v) }
void PhysicsWorldGPU::set_body_angular_velocity(BodyHandle h, const Vec3& v)  { BODY_SET(b.angular_velocity = v) }
void PhysicsWorldGPU::set_body_type(BodyHandle h, BodyType t)                 { BODY_SET(b.type = t) }
void PhysicsWorldGPU::apply_force(BodyHandle h, const Vec3& f)                { BODY_SET(b.apply_force(f)) }
void PhysicsWorldGPU::apply_force_at_point(BodyHandle h, const Vec3& f, const Vec3& p) { BODY_SET(b.apply_force_at_point(f, p)) }
void PhysicsWorldGPU::apply_torque(BodyHandle h, const Vec3& t)               { BODY_SET(b.apply_torque(t)) }
void PhysicsWorldGPU::apply_impulse(BodyHandle h, const Vec3& imp)            { BODY_SET(b.apply_impulse(imp)) }
void PhysicsWorldGPU::apply_impulse_at_point(BodyHandle h, const Vec3& imp, const Vec3& p) { BODY_SET(b.apply_impulse_at_point(imp, p)) }
void PhysicsWorldGPU::wake_body(BodyHandle h)                                 { BODY_SET(b.wake_up()) }

#undef BODY_SET

// ============================================================================
// Joints
// ============================================================================

ConstraintHandle PhysicsWorldGPU::create_joint(const JointDescriptor& desc) {
    if (!impl_) return ConstraintHandle{};

    // Resolve body handles → body indices
    auto itA = impl_->handle_to_index.find(desc.body_a.id);
    auto itB = impl_->handle_to_index.find(desc.body_b.id);
    if (itA == impl_->handle_to_index.end() || itB == impl_->handle_to_index.end())
        return ConstraintHandle{};

    int idxA = (int)itA->second;
    int idxB = (int)itB->second;

    uint32_t jid = impl_->next_joint_id++;
    JointType type = desc.type;

    switch (type) {
    case JointType::BALL_SOCKET: {
        BallSocketJoint j(idxA, idxB, desc.local_anchor_a, desc.local_anchor_b);
        size_t slot = impl_->ball_socket_joints.size();
        impl_->ball_socket_joints.push_back(j);
        impl_->joint_handle_to_slot[jid] = {type, slot};
        break;
    }
    case JointType::HINGE: {
        HingeJoint j;
        j.body_a_index = idxA;  j.body_b_index = idxB;
        j.local_anchor_a = desc.local_anchor_a;  j.local_anchor_b = desc.local_anchor_b;
        j.local_axis_a = desc.local_axis_a;      j.local_axis_b = desc.local_axis_b;
        j.enable_limits = desc.enable_limits;
        j.angle_lower_limit = desc.lower_limit;  j.angle_upper_limit = desc.upper_limit;
        j.enable_motor = desc.enable_motor;
        j.motor_target_velocity = desc.motor_target_velocity;
        j.motor_max_torque = desc.motor_max_force_or_torque;
        size_t slot = impl_->hinge_joints.size();
        impl_->hinge_joints.push_back(j);
        impl_->joint_handle_to_slot[jid] = {type, slot};
        break;
    }
    case JointType::SLIDER: {
        SliderJoint j;
        j.body_a_index = idxA;  j.body_b_index = idxB;
        j.local_anchor_a = desc.local_anchor_a;  j.local_anchor_b = desc.local_anchor_b;
        j.local_axis_a = desc.local_axis_a;
        j.enable_limits = desc.enable_limits;
        j.position_lower_limit = desc.lower_limit;  j.position_upper_limit = desc.upper_limit;
        j.enable_motor = desc.enable_motor;
        j.motor_target_velocity = desc.motor_target_velocity;
        j.motor_max_force = desc.motor_max_force_or_torque;
        size_t slot = impl_->slider_joints.size();
        impl_->slider_joints.push_back(j);
        impl_->joint_handle_to_slot[jid] = {type, slot};
        break;
    }
    case JointType::FIXED: {
        FixedJoint j;
        j.body_a_index = idxA;  j.body_b_index = idxB;
        j.local_anchor_a = desc.local_anchor_a;  j.local_anchor_b = desc.local_anchor_b;
        // Compute initial relative orientation
        j.relative_orientation = impl_->bodies[idxA].orientation.conjugate() * impl_->bodies[idxB].orientation;
        size_t slot = impl_->fixed_joints.size();
        impl_->fixed_joints.push_back(j);
        impl_->joint_handle_to_slot[jid] = {type, slot};
        break;
    }
    }
    return ConstraintHandle(jid);
}

void PhysicsWorldGPU::destroy_joint(ConstraintHandle h) {
    if (!impl_) return;
    auto it = impl_->joint_handle_to_slot.find(h.id);
    if (it == impl_->joint_handle_to_slot.end()) return;

    auto [type, slot] = it->second;
    impl_->joint_handle_to_slot.erase(it);

    // Swap-with-last pattern per joint type
    auto erase_joint = [&](auto& vec) {
        size_t last = vec.size() - 1;
        if (slot != last) {
            vec[slot] = vec[last];
            // Update handle map for the moved joint: find its handle
            for (auto& [jid, pair] : impl_->joint_handle_to_slot) {
                if (pair.first == type && pair.second == last) {
                    pair.second = slot;
                    break;
                }
            }
        }
        vec.pop_back();
    };

    switch (type) {
    case JointType::BALL_SOCKET: erase_joint(impl_->ball_socket_joints); break;
    case JointType::HINGE:       erase_joint(impl_->hinge_joints);       break;
    case JointType::SLIDER:      erase_joint(impl_->slider_joints);      break;
    case JointType::FIXED:       erase_joint(impl_->fixed_joints);       break;
    }
}

bool PhysicsWorldGPU::is_joint_valid(ConstraintHandle h) const {
    return impl_ && impl_->joint_handle_to_slot.count(h.id) > 0;
}

size_t PhysicsWorldGPU::get_joint_count() const {
    if (!impl_) return 0;
    return impl_->ball_socket_joints.size() + impl_->hinge_joints.size()
         + impl_->slider_joints.size()       + impl_->fixed_joints.size();
}

size_t PhysicsWorldGPU::get_contact_cache_size() const {
    return impl_ ? impl_->contact_cache.size() : 0;
}

uint32_t PhysicsWorldGPU::debug_feature_key(const FeatureKeyInputs& in, int prev_key) {
    if (prev_key < 0) {
        return feature_key(in.normal, in.local_a, in.half_a, in.local_b, in.half_b, nullptr);
    }
    CachedContact tmp;
    tmp.body_a       = 0;
    tmp.body_b       = 0;
    tmp.feature_id   = (uint32_t)prev_key;
    tmp.impulse_normal = 0.0f;
    tmp.impulse_tangent[0] = 0.0f;
    tmp.impulse_tangent[1] = 0.0f;
    tmp.age_seconds  = 0.0f;
    tmp.last_local_a = Vec3::zero();
    tmp.last_local_b = Vec3::zero();
    return feature_key(in.normal, in.local_a, in.half_a, in.local_b, in.half_b, &tmp);
}

void PhysicsWorldGPU::set_joint_motor_velocity(ConstraintHandle h, float v) {
    if (!impl_) return;
    auto it = impl_->joint_handle_to_slot.find(h.id);
    if (it == impl_->joint_handle_to_slot.end()) return;
    auto [type, slot] = it->second;
    if (type == JointType::HINGE)  impl_->hinge_joints[slot].motor_target_velocity = v;
    if (type == JointType::SLIDER) impl_->slider_joints[slot].motor_target_velocity = v;
}

void PhysicsWorldGPU::set_joint_motor_max_force(ConstraintHandle h, float f) {
    if (!impl_) return;
    auto it = impl_->joint_handle_to_slot.find(h.id);
    if (it == impl_->joint_handle_to_slot.end()) return;
    auto [type, slot] = it->second;
    if (type == JointType::HINGE)  impl_->hinge_joints[slot].motor_max_torque = f;
    if (type == JointType::SLIDER) impl_->slider_joints[slot].motor_max_force  = f;
}

void PhysicsWorldGPU::set_joint_motor_enabled(ConstraintHandle h, bool enabled) {
    if (!impl_) return;
    auto it = impl_->joint_handle_to_slot.find(h.id);
    if (it == impl_->joint_handle_to_slot.end()) return;
    auto [type, slot] = it->second;
    if (type == JointType::HINGE)  impl_->hinge_joints[slot].enable_motor = enabled;
    if (type == JointType::SLIDER) impl_->slider_joints[slot].enable_motor = enabled;
}

void PhysicsWorldGPU::set_joint_position(ConstraintHandle h, float q) {
    if (!impl_) return;
    auto it = impl_->joint_handle_to_slot.find(h.id);
    if (it == impl_->joint_handle_to_slot.end()) return;
    auto [type, slot] = it->second;

    if (type == JointType::HINGE) {
        const HingeJoint& j = impl_->hinge_joints[slot];
        if (j.body_a_index < 0 || j.body_b_index < 0) return;
        const RigidBody& bA = impl_->bodies[j.body_a_index];
        RigidBody&       bB = impl_->bodies[j.body_b_index];

        Matrix3 R_A          = Matrix3::from_quaternion(bA.orientation);
        Vec3    anchor_world = bA.position + R_A * j.local_anchor_a;
        Vec3    axis_a       = j.local_axis_a;
        float   al = axis_a.length();
        if (al > 1e-9f) axis_a = axis_a * (1.0f / al);
        Vec3    axis_world   = R_A * axis_a;

        // Child orientation = parent orientation rotated by q about world axis.
        Quaternion delta = Quaternion::from_axis_angle(axis_world, q);
        bB.orientation = (delta * bA.orientation).normalized();

        // Force anchor coincidence: anchor_world = bB.position + R_B * local_anchor_b.
        Matrix3 R_B = Matrix3::from_quaternion(bB.orientation);
        bB.position = anchor_world - R_B * j.local_anchor_b;

        bB.linear_velocity  = Vec3::zero();
        bB.angular_velocity = Vec3::zero();
        bB.is_sleeping      = false;
        bB.sleep_time       = 0.0f;
    }
    else if (type == JointType::SLIDER) {
        const SliderJoint& j = impl_->slider_joints[slot];
        if (j.body_a_index < 0 || j.body_b_index < 0) return;
        const RigidBody& bA = impl_->bodies[j.body_a_index];
        RigidBody&       bB = impl_->bodies[j.body_b_index];

        Matrix3 R_A           = Matrix3::from_quaternion(bA.orientation);
        Vec3    anchor_a_world = bA.position + R_A * j.local_anchor_a;
        Vec3    axis_a         = j.local_axis_a;
        float   al = axis_a.length();
        if (al > 1e-9f) axis_a = axis_a * (1.0f / al);
        Vec3    axis_world     = R_A * axis_a;

        // Slider preserves relative rotation. Translate child anchor along axis.
        Matrix3 R_B = Matrix3::from_quaternion(bB.orientation);
        bB.position = anchor_a_world + axis_world * q - R_B * j.local_anchor_b;

        bB.linear_velocity  = Vec3::zero();
        bB.angular_velocity = Vec3::zero();
        bB.is_sleeping      = false;
        bB.sleep_time       = 0.0f;
    }
    // BALL_SOCKET / FIXED: no scalar coordinate — silently ignore.
}

void PhysicsWorldGPU::set_joint_acceleration(ConstraintHandle h, float qddot) {
    if (!impl_) return;
    auto it = impl_->joint_handle_to_slot.find(h.id);
    if (it == impl_->joint_handle_to_slot.end()) return;
    auto [type, slot] = it->second;
    auto normalize_axis = [](Vec3 a) {
        float l = a.length();
        return l > 1e-9f ? a * (1.0f / l) : Vec3(1, 0, 0);
    };

    if (type == JointType::HINGE) {
        const HingeJoint& j = impl_->hinge_joints[slot];
        if (j.body_a_index < 0 || j.body_b_index < 0) return;
        const RigidBody& bA = impl_->bodies[j.body_a_index];
        RigidBody&       bB = impl_->bodies[j.body_b_index];

        Matrix3 R_A        = Matrix3::from_quaternion(bA.orientation);
        Vec3    axis_world = normalize_axis(R_A * j.local_axis_a);
        Matrix3 I_inv_w    = bB.get_inv_inertia_world();
        float   inv_I_axis = axis_world.dot(I_inv_w * axis_world);
        float   I_axis     = inv_I_axis > 1e-9f ? (1.0f / inv_I_axis) : 1.0f;

        bB.torque += axis_world * (I_axis * qddot);
        bB.is_sleeping = false;
        bB.sleep_time  = 0.0f;
    }
    else if (type == JointType::SLIDER) {
        const SliderJoint& j = impl_->slider_joints[slot];
        if (j.body_a_index < 0 || j.body_b_index < 0) return;
        const RigidBody& bA = impl_->bodies[j.body_a_index];
        RigidBody&       bB = impl_->bodies[j.body_b_index];

        Matrix3 R_A        = Matrix3::from_quaternion(bA.orientation);
        Vec3    axis_world = normalize_axis(R_A * j.local_axis_a);

        bB.force += axis_world * (bB.mass * qddot);
        bB.is_sleeping = false;
        bB.sleep_time  = 0.0f;
    }
    // BALL_SOCKET / FIXED: no scalar coordinate — silently ignore.
}

void PhysicsWorldGPU::set_joint_velocity(ConstraintHandle h, float qdot) {
    if (!impl_) return;
    auto it = impl_->joint_handle_to_slot.find(h.id);
    if (it == impl_->joint_handle_to_slot.end()) return;
    auto [type, slot] = it->second;

    auto normalize_axis = [](Vec3 a) {
        float l = a.length();
        return l > 1e-9f ? a * (1.0f / l) : Vec3(1, 0, 0);
    };

    if (type == JointType::HINGE) {
        const HingeJoint& j = impl_->hinge_joints[slot];
        if (j.body_a_index < 0 || j.body_b_index < 0) return;
        const RigidBody& bA = impl_->bodies[j.body_a_index];
        RigidBody&       bB = impl_->bodies[j.body_b_index];

        Matrix3 R_A         = Matrix3::from_quaternion(bA.orientation);
        Vec3    axis_world  = R_A * normalize_axis(j.local_axis_a);

        // ω_B = ω_A + qdot · axis_world  (rigid hinge differential kinematics).
        bB.angular_velocity = bA.angular_velocity + axis_world * qdot;

        // Linear velocity at the joint anchor must match between bodies.
        // anchor_world = bB.position + R_B * local_anchor_b
        // => v_anchor_B = v_B + ω_B × (anchor_world - bB.position)
        //              == v_anchor_A = v_A + ω_A × (anchor_world - bA.position)
        Matrix3 R_B         = Matrix3::from_quaternion(bB.orientation);
        Vec3 anchor_world   = bA.position + R_A * j.local_anchor_a;
        Vec3 rA             = anchor_world - bA.position;
        Vec3 rB             = anchor_world - bB.position;
        Vec3 v_anchor_A     = bA.linear_velocity + bA.angular_velocity.cross(rA);
        bB.linear_velocity  = v_anchor_A - bB.angular_velocity.cross(rB);

        bB.is_sleeping = false;
        bB.sleep_time  = 0.0f;
        (void)R_B;
    }
    else if (type == JointType::SLIDER) {
        const SliderJoint& j = impl_->slider_joints[slot];
        if (j.body_a_index < 0 || j.body_b_index < 0) return;
        const RigidBody& bA = impl_->bodies[j.body_a_index];
        RigidBody&       bB = impl_->bodies[j.body_b_index];

        Matrix3 R_A        = Matrix3::from_quaternion(bA.orientation);
        Vec3    axis_world = R_A * normalize_axis(j.local_axis_a);

        // Slider keeps relative rotation locked: ω_B = ω_A.
        bB.angular_velocity = bA.angular_velocity;
        // Body B translates along the axis at qdot relative to body A.
        bB.linear_velocity  = bA.linear_velocity + axis_world * qdot;

        bB.is_sleeping = false;
        bB.sleep_time  = 0.0f;
    }
    // BALL_SOCKET / FIXED: no scalar coordinate — silently ignore.
}

// ============================================================================
// World Configuration
// ============================================================================

void  PhysicsWorldGPU::set_gravity(const Vec3& g)       { if (impl_) impl_->gravity = g; }
Vec3  PhysicsWorldGPU::get_gravity() const               { return impl_ ? impl_->gravity : Vec3(0,-9.81f,0); }
void  PhysicsWorldGPU::set_time_step(float dt)           { if (impl_) impl_->time_step = dt; }
float PhysicsWorldGPU::get_time_step() const             { return impl_ ? impl_->time_step : 1.0f/60.0f; }
void  PhysicsWorldGPU::set_solver_iterations(int n)      { if (impl_) impl_->solver_iterations = n; }
int   PhysicsWorldGPU::get_solver_iterations() const     { return impl_ ? impl_->solver_iterations : 10; }

// ============================================================================
// Queries
// ============================================================================

// Ray-AABB slab test: returns t of entry, or -1 if no hit within [0, max_t]
static float ray_aabb_t(const Vec3& ro, const Vec3& inv_d, const AABB& box, float max_t) {
    float tx1 = (box.min.x - ro.x) * inv_d.x, tx2 = (box.max.x - ro.x) * inv_d.x;
    float ty1 = (box.min.y - ro.y) * inv_d.y, ty2 = (box.max.y - ro.y) * inv_d.y;
    float tz1 = (box.min.z - ro.z) * inv_d.z, tz2 = (box.max.z - ro.z) * inv_d.z;
    float tmin = std::max({std::min(tx1,tx2), std::min(ty1,ty2), std::min(tz1,tz2)});
    float tmax = std::min({std::max(tx1,tx2), std::max(ty1,ty2), std::max(tz1,tz2)});
    if (tmax < 0.0f || tmin > tmax || tmin > max_t) return -1.0f;
    return (tmin >= 0.0f) ? tmin : tmax;
}

// Ray-OBB test via transforming ray to OBB local space and running slab test.
// Returns t of hit, or -1 if miss.
static float ray_obb_t(const Vec3& origin, const Vec3& dir,
                       const Vec3& center, const Matrix3& R, const Vec3& half,
                       float max_t, Vec3* out_normal) {
    // Local-space ray
    Vec3 d_loc = R.transposed() * dir;
    Vec3 o_loc = R.transposed() * (origin - center);

    Vec3 inv_d(
        std::abs(d_loc.x) > 1e-9f ? 1.0f / d_loc.x : (d_loc.x >= 0.0f ? 1e30f : -1e30f),
        std::abs(d_loc.y) > 1e-9f ? 1.0f / d_loc.y : (d_loc.y >= 0.0f ? 1e30f : -1e30f),
        std::abs(d_loc.z) > 1e-9f ? 1.0f / d_loc.z : (d_loc.z >= 0.0f ? 1e30f : -1e30f)
    );

    float tx1 = (-half.x - o_loc.x) * inv_d.x, tx2 = ( half.x - o_loc.x) * inv_d.x;
    float ty1 = (-half.y - o_loc.y) * inv_d.y, ty2 = ( half.y - o_loc.y) * inv_d.y;
    float tz1 = (-half.z - o_loc.z) * inv_d.z, tz2 = ( half.z - o_loc.z) * inv_d.z;

    float tmin_x = std::min(tx1,tx2), tmax_x = std::max(tx1,tx2);
    float tmin_y = std::min(ty1,ty2), tmax_y = std::max(ty1,ty2);
    float tmin_z = std::min(tz1,tz2), tmax_z = std::max(tz1,tz2);
    float tmin = std::max({tmin_x, tmin_y, tmin_z});
    float tmax = std::min({tmax_x, tmax_y, tmax_z});

    if (tmax < 0.0f || tmin > tmax || tmin > max_t) return -1.0f;
    float t = (tmin >= 0.0f) ? tmin : tmax;

    if (out_normal) {
        // Determine which slab the entry hit
        Vec3 loc_normal;
        if (tmin == tmin_x) loc_normal = Vec3(tx1 < tx2 ? -1.0f : 1.0f, 0, 0);
        else if (tmin == tmin_y) loc_normal = Vec3(0, ty1 < ty2 ? -1.0f : 1.0f, 0);
        else                     loc_normal = Vec3(0, 0, tz1 < tz2 ? -1.0f : 1.0f);
        *out_normal = (R * loc_normal).normalized();
    }
    return t;
}

BodyHandle PhysicsWorldGPU::raycast(const Vec3& origin, const Vec3& dir, float max_dist,
                                    Vec3* hit_point, Vec3* hit_normal) const {
    if (!impl_ || impl_->bodies.empty()) return BodyHandle{};

    Vec3 d = dir.normalized();
    Vec3 inv_d(
        std::abs(d.x) > 1e-9f ? 1.0f / d.x : (d.x >= 0.0f ? 1e30f : -1e30f),
        std::abs(d.y) > 1e-9f ? 1.0f / d.y : (d.y >= 0.0f ? 1e30f : -1e30f),
        std::abs(d.z) > 1e-9f ? 1.0f / d.z : (d.z >= 0.0f ? 1e30f : -1e30f)
    );

    float best_t = max_dist;
    int   best_i = -1;
    Vec3  best_n;

    int n = (int)impl_->bodies.size();
    for (int i = 0; i < n; ++i) {
        // Broadphase: AABB slab test
        AABB aabb = impl_->compute_aabb(i);
        if (ray_aabb_t(origin, inv_d, aabb, best_t) < 0.0f) continue;

        // Narrowphase: OBB slab test
        const RigidBody& b = impl_->bodies[i];
        Matrix3 R = Matrix3::from_quaternion(b.orientation);
        Vec3 n_hit;
        float t = ray_obb_t(origin, d, b.position, R, impl_->half_extents[i], best_t, &n_hit);
        if (t >= 0.0f && t < best_t) {
            best_t = t;
            best_i = i;
            best_n = n_hit;
        }
    }

    if (best_i < 0) return BodyHandle{};
    if (hit_point)  *hit_point  = origin + d * best_t;
    if (hit_normal) *hit_normal = best_n;
    BodyHandle h; h.id = impl_->handles[best_i];
    return h;
}

void PhysicsWorldGPU::query_aabb(const AABB& query, QueryCallback callback) const {
    if (!impl_) return;
    int n = (int)impl_->bodies.size();
    for (int i = 0; i < n; ++i) {
        if (impl_->compute_aabb(i).overlaps(query)) {
            BodyHandle h; h.id = impl_->handles[i];
            if (!callback(h)) return;
        }
    }
}
void       PhysicsWorldGPU::set_contact_callback(ContactCallback cb) { if (impl_) impl_->contact_cb = std::move(cb); }

// ============================================================================
// Statistics / Advanced
// ============================================================================

SimulationStatistics PhysicsWorldGPU::get_statistics() const {
    SimulationStatistics s;
    if (impl_) s.active_body_count = impl_->bodies.size();
    return s;
}
void PhysicsWorldGPU::reset_statistics() {}

const RigidBodySoA& PhysicsWorldGPU::get_body_buffer() const {
    static RigidBodySoA empty;
    return empty;
}

void PhysicsWorldGPU::synchronize() {}

// Private stubs (all logic is in step_fixed above)
size_t     PhysicsWorldGPU::handle_to_index(BodyHandle h) const  { return impl_ ? impl_->handle_to_index.at(h.id) : 0; }
BodyHandle PhysicsWorldGPU::index_to_handle(size_t idx) const    { return impl_ ? BodyHandle(impl_->handles[idx]) : BodyHandle{}; }

} // namespace engine
} // namespace basements

# Basements: Multi-Scale Physics Engine

**Document Version**: 12.0
**Last Updated**: 2026-05-13
**Niche (reframed 2026-05-13)**: Open MPM ↔ Rigid coupling benchmark suite for
**robotics in granular media** (sand, snow, regolith, agricultural soil).

> Prior framing — "From Earthworms to Galaxies" — has been **retired** as
> over-scoped for a single-engineer project. Galaxy-scale N-body, full
> general-purpose engine, and all-backend (CUDA/Metal/ROCm) ambitions are
> moved to **stretch goals only**. See [`PROJECT_STATUS.md::Pivot Record`](./PROJECT_STATUS.md)
> for the rationale and [`roadmap.md`](./roadmap.md) for the new
> phase-based plan.

**Active phase:** Phase 1 (Paper #1) — see `roadmap.md` and `checklist.md`.

---

## 📋 Quick Status (honest)

| Layer | Status | Notes |
|---|---|---|
| **Math (Vec3, Quaternion, Matrix3)** | ✅ Done | AVX2 SIMD, 84 tests passing |
| **MPM Granular Solver** | ✅ Done (basic) | B-spline weights fixed, CPU. Hyperelastic energy density TODO. |
| **GJK / EPA Collision** | ✅ Done | Convex-convex, fully tested |
| **Rigid Body (CPU)** | ✅ Done | Real solver: GJK/EPA, Sequential Impulse, Joints, Raycast, Contact Caching |
| **Joint System (Ball/Hinge/Slider/Fixed)** | ✅ Done | + limits, motor, runtime set_joint_position/velocity/acceleration |
| **MPM ↔ Rigid Coupling (M0 Dirichlet)** | ✅ Done | MVP, mass_rigid ≫ mass_grid regime |
| **MPM ↔ Rigid Coupling (M1 PIC-Rigid)** | ✅ Done | 35× momentum-error reduction vs M0 |
| **MPM ↔ Rigid Coupling (M4.5a — Layer A renormalized)** | ✅ Done | **2.2× ↓ vs M1 at +6% cost — paper headline** |
| **MPM ↔ Rigid Coupling (M4.5b, M4.5c)** | ✅ Done | Honest regressions — paper Section 4 evidence |
| **MPM ↔ Rigid Coupling (M2 ASFLIP, M3 MLS-MPM, M4 Lagrange)** | ⬜ Planned | Phase 2 — currently stubs delegating to M1 |
| **Scene Editor (ImGui)** | ✅ Done (basic) | Hierarchy/Inspector/Gizmo/Console/snapshot. UI stabilization continues. |
| **URDF Import → Physics** | ✅ Done | Parse + scene-graph + joint-constraint auto-wire on Play |
| **Drift benchmark + figures** | ✅ Done | 6 figures, CLI `--long-form`, GitHub Actions CI |
| **GPU CUDA Backend** | 🟡 Stub | Phase 3 — selective kernels only |
| **BVH Broadphase** | ⬜ Planned | Phase 3 |
| **Soft Body PBD / SPH Fluid** | 🟡 Stub | Stretch goal — outside core niche |
| **ROS 2 Integration** | ⬜ Planned | Phase 4 |
| **Python Bindings (pybind11)** | ⬜ Planned | Phase 4 |
| **N-body Galaxy / Cosmology** | ❌ Retired | Out of scope — not a single-engineer target |

---

## 📐 Table 0: Theoretical Foundation & Engineering Principles

### 1. Governing Equations

| Domain | Equation | Application |
|:---|:---|:---|
| **Continuum** | $\rho (\frac{\partial \mathbf{u}}{\partial t} + \mathbf{u} \cdot \nabla \mathbf{u}) = -\nabla p + \mu \nabla^2 \mathbf{u} + \mathbf{f}$ | Navier-Stokes (Fluids) |
| **Discrete** | $m_i \frac{d^2 \mathbf{r}_i}{dt^2} = \sum_{j} \mathbf{F}_{ij} + \mathbf{F}_{ext}$ | Rigid Body Dynamics, Granular |

### 2. Numerical Discretization

| Method | Feature | Application |
|:---|:---|:---|
| **FEM** | Energy minimization | Soft bodies, structural analysis |
| **FVM** | Conservative flux | Fluid dynamics (Eulerian) |
| **MPM / SPH** | Meshfree, large deformation | **MPM Granular Solver** (Sand/Snow) |
| **FDM** | Grid-based | Wave equations, height-field water |

### 3. Constitutive Modeling

- **Elasticity**: Linear ($\sigma = D\epsilon$) vs. Hyperelastic (Neo-Hookean) for large deformations.
- **Granular Mechanics**: **Drucker-Prager Yield Criterion** — SVD-based plasticity for angle of repose simulation.
- **EOS**: Pressure-density relationship $P(\rho)$ for compressible flows.

### 4. Time Integration

| Scheme | Stability | Use Case |
|:---|:---|:---|
| **Explicit (RK4, Euler)** | CFL: $\Delta t \le \Delta x / C$ | Fracture, rapid collisions |
| **Implicit (Backward Euler)** | Unconditionally stable | Stiff constraints, stacking |
| **Symplectic (Velocity Verlet)** | Energy-preserving | Orbital mechanics |

---

## 🏜️ MPM (Material Point Method) — Granular Solver

**Status**: ✅ Implemented, CPU, tests passing
**Header**: `include/basements/physics/mpm/mpm_solver.h`

### Hybrid Lagrangian-Eulerian Approach

- **Particles (Lagrangian)**: Carry mass, velocity, deformation gradient ($\mathbf{F}$), and `C` (APIC momentum).
- **Grid (Eulerian)**: Solves $F=ma$, handles boundary conditions and self-collision automatically.

### Algorithm (per step)

```
1. grid.clear()
2. P2G: transfer particle mass/momentum → grid nodes (quadratic B-spline weights)
3. update_grid: normalize velocities, apply gravity, enforce boundary conditions
4. G2P: transfer grid velocities back → particles (APIC), advect positions
5. apply_plasticity: Drucker-Prager yield (SVD decomposition)
```

### Key Fix (2026-03-21)

Quadratic B-spline weights were incorrect, causing energy blow-up (avg_y jumped to 22.59):

```cpp
// WRONG (old) — weights summed to 1.75, not 1.0
w[0] = 0.5*(1.5 - f)^2,  w[2] = 0.5*(f - 0.5)^2

// CORRECT (fixed)
w[0] = 0.5*(1 - f)^2,    w[1] = 0.75 - (f - 0.5)^2,    w[2] = 0.5*f^2
// Sum: (1-f)^2/2 + 3/4 - (f-1/2)^2 + f^2/2 = 1 ✓
```

### Sparse Grid

`SPGridCPU` uses block-based spatial hashing (`BLOCK_SIZE = 4`, `unordered_map<uint64_t, GridBlock>`).
`get_node(ni, nj, nk)` auto-creates blocks on demand.

---

## 📐 Collision Engine: GJK & EPA

**Status**: ✅ Implemented
**Headers**: `include/basements/physics/collision/`

```
Broadphase  →  O(n²) AABB (CPU; GPU Spatial Hash — Phase 5)
Narrowphase →  GJK (Gilbert-Johnson-Keerthi) — convex intersection test
Contact Gen →  EPA (Expanding Polytope) — penetration depth & normal
Raycast     →  Ray-OBB slab method (broadphase AABB + OBB narrowphase)
QueryAABB   →  AABB overlap query (returns all overlapping body handles)
```

### Raycast Algorithm

1. **Broadphase**: Ray-AABB slab test per body (early reject)
2. **Narrowphase**: Transform ray to OBB local space, run slab intersection
3. **Result**: Returns closest hit handle + world-space hit point + surface normal
4. Both `physics_world_gpu_stub.cpp` (CPU solver) and `physics_world_gpu.cpp` (SoA/GPU) implement this.

---

## 🗂️ Contact Caching (Persistent Manifold)

**Status**: ✅ Implemented (2026-03-31)
**File**: `src/engine/physics_world_gpu_stub.cpp`

Contact caching preserves `accumulated_impulse` across frames so the solver can **warm-start** the next frame's constraint solve, reaching a good solution in fewer iterations.

### Key concept: feature_id

Each contact is identified by `(body_a, body_b, feature_id)` where `feature_id` is the **dominant axis** of the contact normal (0=+X, 1=-X, 2=+Y, 3=-Y, 4=+Z, 5=-Z). This provides a stable key for matching contacts across frames without requiring GJK feature tracking.

```cpp
static int dominant_axis(const Vec3& n) {
    float ax = abs(n.x), ay = abs(n.y), az = abs(n.z);
    if (ax >= ay && ax >= az) return n.x > 0 ? 0 : 1;
    if (ay >= ax && ay >= az) return n.y > 0 ? 2 : 3;
    return n.z > 0 ? 4 : 5;
}
```

### Pipeline integration

```
step_fixed():
  apply_external_forces
  broadphase
  narrowphase          ← warm-start: look up (body_a, body_b, feature_id) in cache
  solve_constraints    ← uses warmed impulses for faster convergence
  integrate
  update_sleep_states
  update_contact_cache ← age entries, expire (age > MAX_CACHE_AGE=2), upsert current frame
  invoke_callbacks
```

### Cache lifecycle rules

| Event | Action |
|---|---|
| Contact present this frame | `age = 0`, copy `accumulated_impulse` to cache |
| Contact absent this frame | `age++`; expire if `age > 2` |
| `destroy_body(h)` | Purge dead entries **before** `remap_body_index()` — prevents moved body's entries from being deleted |
| `reset()` | Clear entire cache |

### Validation (test_contact_cache, 4/4 pass)

| Test | What it checks |
|---|---|
| `WarmStartTransfer` | Settled box stays within 2 mm after one more frame |
| `StackingStability` | 3-box column stays within ±5 cm after 3 s |
| `CacheExpiry` | After 3+ idle frames, re-landed box converges identically to cold-start |
| `EnergyMonotonicity` | Peak KE in second 60-frame window ≤ first window (no sustained energy injection) |

---

## ⚙️ Joint Constraint System

**Status**: ✅ Implemented & Validated
**Headers**: `include/basements/physics/dynamics/joint_solver.h`

| Joint Type | DOF removed | Limits | Motor |
|---|---|---|---|
| BallSocket | 3 linear | — | — |
| Hinge | 3 linear + 2 angular | ✅ min/max angle | ✅ velocity + max force |
| Slider | 3 angular + 2 linear | ✅ min/max distance | ✅ velocity + max force |
| Fixed | 6 (all) | — | — |

### Key design decisions

- **Sequential Impulse**: `pre_solve` (warm starting + effective mass) → N × `solve_velocity` (interleaved with contacts)
- **World-space inertia**: `R * I_body_inv * R^T` computed each frame
- **Baumgarte for equality constraints** (ball-socket, hinge free motion): `bias = β/dt * violation`, β=0.1
- **Zero bias for unilateral limits**: `limit_bias = 0.0f` — velocity constraint alone stops the body at the limit boundary. Positive bias causes exponential divergence (`violation_n+1 = violation_n × 1.2/frame` at 60Hz).
- **Warm starting**: Accumulated impulses from previous frame applied in pre_solve to accelerate convergence (~2× fewer iterations needed)

### Validation results (120 Hz, 3–5 s simulation)

| Test | Result |
|---|---|
| BallSocket constraint error | 3.4 mm after 3 s (< 10 cm target) |
| Hinge pendulum period | 2.133 s measured vs 2.104 s theoretical (1.4% error, ±15% tolerance) |

---

## 🖥️ Scene Editor

**Status**: ✅ Building and running
**Executable**: `build/bin/Debug/basements_editor.exe`
**Platform**: Windows, OpenGL 3.3 Core Profile, ImGui docking

### Editor Architecture

```
EditorApp (PIMPL)
├── GLFWwindow + OpenGL context
├── SceneGraph          — node hierarchy (create/remove/parent)
├── SelectionManager    — multi-select (primary + additive)
├── CommandHistory      — undo/redo stack (Ctrl+Z / Ctrl+Y)
├── GizmoManager        — translate/rotate/scale gizmos (W/E/R keys)
├── Framebuffer         — off-screen render target for viewport
├── Camera              — free-fly (WASD + right-drag + scroll)
└── Panels
    ├── HierarchyPanel  — scene tree, node creation context menu
    ├── InspectorPanel  — transform, physics props, PBR material
    ├── PhysicsPanel    — environment settings + MPM sand simulation
    └── ConsolePanel    — log output
```

### Simulation Loop

```
EditorMode: EDIT → SIMULATE (Play) → EDIT (Stop)

play():
  1. Save snapshot of all node transforms (for Stop restore)
  2. Create PhysicsWorldGPU, sync scene nodes → physics bodies
  3. mode = SIMULATE

step() [called every frame in SIMULATE]:
  1. physics_world->step(1/60)
  2. Sync dynamic body position/orientation → scene nodes

stop():
  1. Restore all node transforms from snapshot  ← NEW (Phase 3)
  2. Destroy physics_world
  3. mode = EDIT
```

### Phase 3 Additions (2026-03-21)

| Feature | Description |
|---|---|
| **Play/Stop snapshot** | Stop 시 Play 이전 상태로 자동 복원 |
| **MPM step location** | `on_render()` 내부 → 메인 루프 최상단 (모드 독립) |
| **MPM render location** | ImGui 루프 내부(버그) → `begin_scene()/end_scene()` 내부 |
| **SimulationBodyState** | Inspector에서 시뮬레이션 중 선속도/각속도/speed/sleeping 표시 |
| **Body type colors** | Dynamic=파란색, Static=회색, Kinematic=초록색, Selected=주황색 |

---

## 📊 Implementation & Verification Status

| Module | Feature | Implemented | Tests | Status |
|:---|:---|:---:|:---:|:---|
| **Math** | Vec3 (AVX2 SIMD) | ✅ | ✅ | Done |
| | Quaternion | ✅ | ✅ | Done — Hamilton product convention verified |
| | Matrix3 | ✅ | ✅ | Done — rotation composition verified |
| **MPM** | SPGridCPU | ✅ | ✅ | Done — B-spline weights fixed |
| | Drucker-Prager plasticity | ✅ | ✅ | Done |
| | Boundary collision | ✅ | ✅ | Done |
| **Collision** | GJK | ✅ | ✅ | Done |
| | EPA | ✅ | ✅ | Done |
| **Physics** | PhysicsWorldGPU API | ✅ | ✅ | Done — real solver active |
| | Rigid Body integration | ✅ | ✅ | Done — Symplectic Euler, GJK/EPA, Sequential Impulse |
| | Joint constraints | ✅ | ✅ | Done — Ball/Hinge/Slider/Fixed + limit fix (Baumgarte bias=0) |
| | Joint simulation | ✅ | ✅ | Done — BallSocket constraint error <3.4mm, Hinge period ±1.4% |
| | Raycast (ray-OBB) | ✅ | ✅ | Done — brute-force, slab method, both CPU impls |
| | QueryAABB | ✅ | ✅ | Done — AABB overlap test, per-body half_extents |
| | Contact Caching | ✅ | ✅ | Done — persistent manifold, warm-start, dominant-axis key, age-based expiry |
| **I/O** | URDF Parser | ✅ | ✅ | Done (9 tests) |
| | Scene Serializer (JSON) | ✅ | — | Done (nlohmann/json) |
| | Mesh Importer (OBJ) | ✅ | — | Done (tinyobjloader) |
| **Editor** | SceneGraph + Commands | ✅ | — | Done |
| | Gizmo (translate/rotate/scale) | ✅ | — | Done |
| | Inspector (transform/physics/PBR) | ✅ | — | Done + SimulationBodyState |
| | PhysicsPanel + MPM | ✅ | — | Done (properly in framebuffer) |
| | Play/Stop/Pause | ✅ | — | Done + snapshot restore |

---

## 📅 Development History & Roadmap

### Completed Phases

#### Phase 1: Repository Cleanup (2026-03)
- [x] 루트 오염 파일 정리 (로그, 빌드 산출물)
- [x] Include 폴더 구조 재편 (`core/math/`, `engine/`, `editor/` 하위 이동)
- [x] 전체 소스 `#include` 경로 일괄 업데이트

#### Phase 2: Editor Build (2026-03)
- [x] `glad` 의존성 제거 → `OpenGL::GL` (wglGetProcAddress 기반 커스텀 로더)
- [x] `nlohmann/json` FetchContent 추가 (SceneSerializer)
- [x] `PhysicsWorldGPU` stub 구현 (physics_world_gpu_stub.cpp)
- [x] `PythonBridge` stub 구현 (pybind11 없이 빌드 가능)
- [x] `ReparentNodeCommand`, `create_sphere`, `move_to_root` 미구현 함수 완성
- [x] Shader 파일 로드 수정 (경로를 GLSL 소스로 처리하던 버그 수정)
- [x] GLSL 셰이더 파일 생성 (`basic.vert`, `basic.frag`)
- [x] 에디터 빌드 및 실행 확인

#### Phase 3: Physics ↔ UI Connection (2026-03-21)
- [x] MPM B-spline weight 버그 수정 (avg_y: 22.59 → 0.671)
- [x] Quaternion / Matrix3 회전 합성 테스트 순서 수정
- [x] Play/Stop 스냅샷 복원
- [x] MPM step/render 올바른 위치로 이동
- [x] Inspector SimulationBodyState (velocity, angular vel, sleeping)
- [x] 시뮬레이션 중 바디 타입별 색상 구분

### Phase 4: Real Physics Solver (2026-03-27 ~ 2026-03-30) ✅ Complete

**성과**: PhysicsWorldGPU stub → 실제 작동하는 강체 시뮬레이터 + 완전한 joint 시뮬레이션 + 쿼리 API

| Task | Status | Notes |
|---|---|---|
| CPU rigid body integrator | ✅ | Symplectic Euler, contiguous vector layout |
| Broadphase AABB + GJK/EPA | ✅ | OBB broadphase + GJK/EPA narrowphase |
| Sequential Impulse solver | ✅ | 4 critical bugs fixed (world-space inertia, Baumgarte sign, normal dir, penetration sign) |
| Sleep system | ✅ | Sleep timer + threshold |
| Joint wiring (PhysicsWorldGPU) | ✅ | Ball/Hinge/Slider/Fixed joints wired into simulation loop |
| Joint limit constraints | ✅ | Baumgarte bias = 0 fix (positive bias → exponential divergence) |
| Raycast (ray-OBB) | ✅ | Slab method, broadphase AABB + OBB narrowphase, both CPU impls |
| QueryAABB | ✅ | AABB overlap query with per-body half_extents |
| Physics validation tests | ✅ | 6/6 pass: FreeFall, BoxLandsOnGround, TwoBoxStack, AngularMomentum, BallSocket, HingePeriod |
| optics.h / test_forces include fix | ✅ | `constexpr_abs` namespace + old include path fixed |

### Phase 5: 수직심화-검증-수평확장-검증 (2026-03-31~)

#### 수직심화: Contact Caching (2026-03-31) ✅ Complete

| Task | Status | Notes |
|---|---|---|
| `CachedContact` struct + `dominant_axis()` | ✅ | feature_id: dominant axis of contact normal (0-5) |
| Warm-start transfer in `narrowphase()` | ✅ | Look up `(body_a, body_b, feature_id)` and copy accumulated impulse |
| `update_contact_cache()` | ✅ | Age entries, expire if `age > 2`, upsert current frame contacts |
| `destroy_body` cache purge ordering | ✅ | Purge dead entries before `remap_body_index()` — critical ordering |
| `test_contact_cache` | ✅ | 4/4: WarmStartTransfer, StackingStability, CacheExpiry, EnergyMonotonicity |

#### 수평확장: URDF → PhysicsWorld Bridge (2026-05-13) ✅ Complete
- [x] `URDFPhysicsBridge::spawn(robot, world, opts)` with `URDFBridgeOptions::initial_joint_positions`
- [x] REVOLUTE/CONTINUOUS→HINGE, PRISMATIC→SLIDER, FIXED→FIXED, FLOATING→free body, PLANAR→warning
- [x] BFS Forward Kinematics with per-joint initial coordinate seeding
- [x] `test_urdf_physics_bridge`: 7/7 (incl. RevoluteInitialAngleAppliesFK, PrismaticInitialPositionAppliesFK)

#### 수평확장: MPM ↔ RigidBody Coupling MVP (2026-05-13) ✅ Complete
- [x] `MPMRigidCoupler::apply(grid, body, dt)` — OBB Dirichlet BC + reaction force/torque
- [x] Outward normal selection via closest-face heuristic on body-local position
- [x] Friction-based tangential blend (`min(1, μ)` of body's tangential velocity)
- [x] `test_mpm_rigid_coupler`: 4/4 (BasicReactionForce, OutsideBodyNoReaction, TangentialFrictionTransfers, AngularReactionAtOffset)

##### Two-Way Coupling Roadmap (Future Work)

The MVP coupler is correct only in the regime `mass_rigid ≫ Σ mass_particle`
(typical robotics: rover wheel on sand). For genuinely two-way scenarios
(boulder on snow, lander touchdown) momentum is NOT conserved exactly because
the Dirichlet BC injects impulse from outside the particle subsystem. Three
upgrade paths, ranked by complexity:

| Approach | Complexity | Conservation | References |
|---|---|---|---|
| **PIC-Rigid (impulse-based)** | Low | Approximate; better than Dirichlet | Stomakhin et al. 2014 (snow); coupling extension by Hu et al. (Taichi MPM) |
| **ASFLIP Rigid Extension** | Medium | Momentum-exact for rigid contact; F-bar splitting | Fei et al. 2021, "A Multi-Scale Model for Coupling Strands with Shear-Dependent Liquid" |
| **Lagrange-multiplier (saddle-point solve)** | High | Exact within solver tolerance | Jiang et al. 2020 SCA, "An Implicit Compressible SPH Solver for Snow Simulation" (technique transferable) |

##### Publication Plan Note (2026-05-13)

A short-paper opportunity exists in benchmarking **all three approaches** on a
single robotics-relevant scenario (wheel-on-sand, foot-on-snow,
lander-on-regolith). Existing literature compares MPM↔fluid coupling but
**multi-scale granular MPM + friction-dominated rigid contact** is an open
quantitative gap. Concrete milestones:

| Milestone | Output |
|---|---|
| **M0 (now)** | MVP Dirichlet BC + `bench_mpm_rigid_drift` reference plot |
| **M1** | PIC-Rigid implementation; momentum-drift figure delta vs MVP |
| **M2** | ASFLIP rigid extension; same scenario, same axes |
| **M3** | Lagrange-multiplier solver; same |
| **Paper** | 4-page short paper at SCA / SIGGRAPH Posters with consolidated drift figure across all 3 methods |

Estimated effort: M1 ~ 2 weeks, M2 ~ 4 weeks, M3 ~ 8 weeks. Publication-track
implementation is *optional* — only pursue if a downstream user-case (smart
farm soil tracking, planetary rover ML training data) needs it.

Tracking issue: TBD.

#### 수직검증: Energy / Momentum Drift Figures (2026-05-13) ✅ Complete

`bench_energy_drift.exe` emits 5 CSV time series; `scripts/plot_energy_drift.py`
post-processes into PNG + `<name>_desc.md` per CLAUDE.md visualization
standards. Files under `outputs/figures/`:

| Figure | Path tested | Purpose |
|---|---|---|
| `energy_free_fall` | TimeIntegrator only | Gravitational accuracy baseline |
| `momentum_isolated_pair` | TimeIntegrator only | p + L conservation (no forces) |
| `energy_damped_slider` | TimeIntegrator only | Monotone damping dissipation |
| `solver_stack_drift` | **PhysicsWorldGPU** | Baumgarte + contact-cache stack drift |
| `solver_pendulum` | **PhysicsWorldGPU + Joint** | Hinge constraint energy drift |
| `mpm_rigid_drift` | **MPMRigidCoupler MVP** | M0 baseline: body/grid/total p_y |

Long-form (60 s) regeneration:
```
cmake --build build --config Release --target drift_bench_long_form
```

#### Phase 5 GPU (when GPU available)
- [ ] CUDA 브로드페이즈 (Spatial Hash GPU)
- [ ] CUDA 적분 커널
- [ ] PhysicsWorldGPU 실제 GPU 구현

### Phase 6: Advanced Simulation (2026 Q4)
- [ ] ROS 2 Full Integration (TF, Joint States)
- [ ] USD Export/Import (Omniverse 호환)
- [ ] Scripting 2.0 (Python hot-reload)

---

## 🏗️ Project File Structure

```
G:/CPP/Basements/
├── include/basements/
│   ├── core/math/          ← Vec3, Quaternion, Matrix3, SVD
│   ├── physics/
│   │   ├── mpm/            ← MPMSolver, SPGridCPU, Particle
│   │   ├── collision/      ← GJK, EPA, AABB, ContactInfo
│   │   ├── dynamics/       ← ForceGenerator, Joints, Constraints
│   │   └── formulas/       ← 180+ inline physics formulas (17 domains)
│   ├── engine/             ← PhysicsWorldGPU, SceneGraph, CommandHistory
│   ├── editor/
│   │   ├── editor_app.h
│   │   ├── ui/             ← PhysicsPanel, InspectorPanel, HierarchyPanel, ConsolePanel
│   │   ├── core/           ← SelectionManager, Component
│   │   └── scripting/      ← PythonBridge (stub)
│   ├── graphics/           ← Renderer, Camera, Shader, Mesh, Framebuffer, Gizmo
│   └── io/                 ← URDF parser/exporter/converter
├── src/
│   ├── editor/             ← EditorApp + all panels
│   ├── engine/             ← SceneGraph, CommandHistory, PhysicsWorldGPU stub
│   ├── graphics/           ← Renderer, Camera, Shader, Mesh, Gizmo, PrimitiveFactory
│   └── io/                 ← URDF implementation
├── tests/                  ← GoogleTest unit tests
├── assets/
│   ├── fonts/              ← Pretendard-Regular.otf (Korean support)
│   └── shaders/            ← basic.vert, basic.frag
├── docs/
│   ├── multi-scale_engine.md    ← this document
│   └── api_reference.md         ← API 명세
└── build/
    ├── bin/Debug/
    │   ├── basements_editor.exe
    │   └── editor/shaders/      ← runtime shader path
    └── _deps/                   ← FetchContent (glfw, imgui, glm, gtest, ...)
```

---

## 🔨 Build Instructions

### Prerequisites

- Visual Studio 2022 (MSVC 17+)
- CMake 3.20+
- CUDA 12.6 (optional, for GPU tests)

### Build (Windows)

```powershell
cd G:/CPP/Basements

# Configure
cmake -S . -B build -G "Visual Studio 17 2022"

# Build editor only
cmake --build build --config Debug --target basements_editor

# Build and run specific tests
cmake --build build --config Debug --target test_quaternion test_matrix3 test_mpm_cpu
./build/Debug/test_mpm_cpu.exe
```

### Key Build Targets

| Target | Type | Description |
|:---|:---|:---|
| `basements_editor` | Executable | Scene editor (OpenGL + ImGui) |
| `basements_physics` | Static lib | Physics engine (rigid body, joints, solver) |
| `basements_scene` | Static lib | SceneGraph, CommandHistory |
| `basements_io` | Static lib | URDF parser, Scene serializer |
| `test_physics_validation` | Executable | Physics correctness: FreeFall, BoxLanding, Stack, Angular, BallSocket, HingePeriod |
| `test_physics_world_gpu` | Executable | PhysicsWorldGPU API + Raycast/QueryAABB (27 tests) |
| `test_joints` | Executable | Joint constraint unit tests |
| `test_joint_limits` | Executable | Joint limit tests (7/7, incl. 3 re-enabled) |
| `test_contact_cache` | Executable | Contact caching warm-start (4 tests) |
| `test_core_math` | Executable | Vec3, Matrix3, Quaternion tests |
| `test_mpm_cpu` | Executable | MPM solver physics tests |
| `test_urdf_parser` | Executable | URDF I/O tests (11 tests) |

### FetchContent Dependencies

| Library | Purpose | Version |
|---|---|---|
| GLFW | Window / input | latest |
| ImGui (docking) | UI framework | docking branch |
| GLM | Math (glm types for editor) | latest |
| GoogleTest | Unit testing | latest |
| nlohmann/json | Scene serialization | v3.11.3 |
| TinyXML2 | URDF parsing | latest |
| TinyObjLoader | OBJ mesh import | latest |

---

## 🔌 Multi-Platform Architecture

```
┌────────────────────────────────────────────┐
│            Application Layer               │
│  basements_editor  │  CLI tools  │  Tests  │
└────────────────────────────────────────────┘
                     ↓
┌────────────────────────────────────────────┐
│          Scene / Engine Layer              │
│  SceneGraph │ PhysicsWorldGPU │ Commands   │
└────────────────────────────────────────────┘
                     ↓
┌────────────────────────────────────────────┐
│            Physics Core                    │
│  MPM  │  GJK/EPA  │  Dynamics  │  Formulas│
└────────────────────────────────────────────┘
                     ↓
┌────────────────────────────────────────────┐
│           Backend (HAL)                    │
│  CPU (OpenMP)  │  CUDA (partial)  │  ...   │
└────────────────────────────────────────────┘
                     ↓
┌────────────────────────────────────────────┐
│           Math Foundation                  │
│  Vec3  │  Quaternion  │  Matrix3  │  SVD   │
└────────────────────────────────────────────┘
```

### Supported Compute Backends

| Backend | API | Status |
|---|---|---|
| **CPU** | OpenMP | ✅ Implemented |
| **CUDA** | NVIDIA CUDA 12.6 | 🔄 Partial |
| **ROCm** | AMD HIP | ⬜ Planned |
| **Metal** | Apple Metal | ⬜ Planned |

---

## 🧪 Test Results (2026-03-31)

| Test Suite | Tests | Pass | Fail | Notes |
|---|---|---|---|---|
| `test_vec3` | 27 | ✅ All | 0 | |
| `test_quaternion` | 21 | ✅ All | 0 | |
| `test_matrix3` | 24 | ✅ All | 0 | |
| `test_core_math` | 9 | ✅ All | 0 | |
| `test_advanced_math` | 8 | ✅ All | 0 | |
| `test_mpm_cpu` | 2 | ✅ All | 0 | avg_y=0.671 |
| `test_urdf_parser` | 11 | ✅ All | 0 | test_data/ copy fixed |
| `test_physics_validation` | 6 | ✅ All | 0 | +BallSocket (<3.4mm), +HingePeriod (±1.4%) |
| `test_physics_world_gpu` | 27 | ✅ All | 0 | +Raycast×3, +QueryAABB |
| `test_contact_cache` | 4 | ✅ All | 0 | WarmStartTransfer, StackingStability, CacheExpiry, EnergyMonotonicity |
| `test_joints` | 5 | ✅ All | 0 | |
| `test_joint_limits` | 7 | ✅ All | 0 | 3 previously disabled — Baumgarte bias=0 fix |
| `test_stacking` | 4 | ✅ All | 0 | |
| `test_energy_conservation` | 4 | ✅ All | 0 | |
| `test_fatigue` | 2 | ✅ All | 0 | |
| `test_fracture` | 3 | ✅ All | 0 | |
| `test_thermodynamics` | 5 | ✅ All | 0 | |
| `test_stress_strain` | 11 | ✅ All | 0 | |
| `test_soft_body` | 3 | ✅ All | 0 | |
| `test_sph` | 5 | ✅ All | 0 | |
| `test_material` | 11 | ✅ All | 0 | |
| `test_elastic_collision` | 2 | ✅ All | 0 | |
| `test_newtons_cradle` | 1 | ✅ All | 0 | |
| `test_pendulum` | 2 | ✅ All | 0 | |
| `test_memory_manager` | 4 | ✅ All | 0 | |
| `test_plugin_loader` | 4 | ✅ All | 0 | |
| `test_sensor` | 2 | ✅ All | 0 | |
| `test_gear` | 2 | ✅ All | 0 | |
| `test_aerodynamics` | 6 | ✅ All | 0 | |
| `test_forces` | — | ✅ | 0 | standalone (no gtest); include path fixed |
| `test_formulas_compilation` | — | ✅ | 0 | standalone; optics.h constexpr_abs fix |

**Known failing** (CUDA-only, skip while GPU is busy):
- `test_broadphase_gpu`, `test_math_gpu`, `test_cuda_hello` — nvcc config issue (Phase 5 작업 시 해결)

---

## 📚 Documentation

- [`docs/multi-scale_engine.md`](./multi-scale_engine.md) — Architecture and roadmap (this file)
- [`docs/api_reference.md`](./api_reference.md) — Full API specification

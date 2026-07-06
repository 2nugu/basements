# Integration Notes

Cross-system "gotchas" — things to remember when wiring two subsystems together.
Each note: which two systems, the surprise, the resolution.

---

## Quaternion convention mismatch

- **System A:** `basements::math::Quaternion` ctor is `(w, x, y, z)` — w first.
- **System B:** URDF `URDFQuaternion` struct stores `(x, y, z, w)`.
- **Resolution:** `URDFPhysicsBridge` uses `Quaternion(q.w, q.x, q.y, q.z)`.
  Easy to flip by accident — every link silently 180° rotated if you get this
  wrong. Round 4 caught this; verify any future I/O bridge does the same.

---

## SceneNode JOINT → PhysicsWorld constraint wiring

- **System A:** Editor SceneGraph has `NodeType::JOINT` nodes with
  `JointData::connected_body_a/b` (NodeID).
- **System B:** PhysicsWorldGPU has `ConstraintHandle` keyed by an internal
  joint id; no SceneNode awareness.
- **Resolution:** `EditorApp::play()` iterates JOINT nodes after spawning
  RIGID_BODY nodes, calls `create_joint`, stores result in
  `Impl::node_to_constraint_map`. **Order matters** — bodies before joints.
- **Caveat:** `URDFConverter::to_scene_graph` did NOT create JOINT nodes
  until Round 7. If you write a new SceneGraph importer, remember to emit
  JOINT nodes or `play()` will silently drop the constraints.

---

## MPM grid coupling with PhysicsPanel

- **System A:** `PhysicsPanel` owns the `MPMSolver` instance (private impl).
- **System B:** `EditorApp::step` needs grid access for `MPMRigidCoupler::apply`.
- **Resolution:** `PhysicsPanel::get_solver()` returns the raw pointer;
  `MPMSolver::get_grid_mutable()` returns the SP-Grid reference. Both were
  added in Round 5.
- **Ordering:** PhysicsPanel runs MPM `step()` in the main `EditorApp::run`
  loop BEFORE `EditorApp::step()`. So when coupling fires, the grid already
  holds the post-update velocities. **Do not move the coupling call into
  PhysicsPanel** — it would run on stale (pre-update) grid data.

---

## Contact cache feature key stability

- **System A:** Narrowphase generates contact_point + normal each frame.
- **System B:** Cache lookup uses `feature_key(normal, local_a, half_a, local_b, half_b, prev_anchor)`.
- **Resolution:** Octant hysteresis with per-axis dead-zone =
  `min(half_a[axis], half_b[axis]) * 0.02f`. **Symmetric** — both bodies use
  the same axis dz to avoid one side flipping while the other holds.
- **Caveat:** If you ever introduce a body type with `half_extents == 0` on
  some axis (e.g. a plane), the dead-zone collapses on that axis and
  hysteresis vanishes. Cap `dead_zone = max(dead_zone, 1e-4f)` if/when this
  becomes real.

---

## Editor cwd dependency

- **System A:** Editor exe loads `assets/fonts/...` and `editor/shaders/...`
  by relative path.
- **System B:** Windows shell launches exe with the *user's* cwd, not the
  exe's directory.
- **Resolution:** `run_editor.bat` (and `run_editor_console.bat`) do
  `cd /d %~dp0` before launching. Always recommend the bat file, never
  direct double-click.
- **Symptom of failure:** "Cannot open: editor/shaders/basic.vert" + segfault
  inside the GL program link.

---

## Stale incremental build → editor crash

- **Trigger:** `editor_app.cpp` Impl struct field added.
- **Symptom:** Editor initializes ("✅ Editor initialized successfully.")
  then segfaults in `run()`. Looks like a runtime bug but is actually a
  build artifact mismatch.
- **Resolution:** Full rebuild — `cmake --build build --config Release
  --target basements_editor`. Verified by file size diff (~5 KB).
- **Rule:** After any header/Impl change, refresh portable bundle from the
  build directory; never trust an incremental binary.

---

## ImGui menu pattern restrictions

- **Don't:** nest `ImGui::InputText` inside a `BeginMenu/EndMenu`.
- **Reason:** state transitions on menu close vs widget focus produced flaky
  behaviour on some Windows configs.
- **Do:** put input widgets in `BeginPopupModal`. We migrated the snapshot
  label editor for this reason in Round 6.

---

## CSV → figure filename collision

- **Trigger:** `bench_mpm_rigid_drift --method dirichlet` and `--method pic`
  both wrote the same figure filename, overwriting.
- **Resolution:** `plot_energy_drift.py::plot_mpm_rigid` uses
  `csv_path.stem` for the figure basename, automatically separating
  `mpm_rigid_drift.png` and `mpm_rigid_drift_pic.png`.

---

## RigidColliderState backward compatibility

- **Field added in Round 7:** `mass` (default 1.0).
- **Why backward compatible:** all M0 Dirichlet code paths ignore it.
- **Caveat:** PIC-Rigid (M1) treats it as load-bearing. If a future call site
  forgets to set `col.mass = body.mass`, PIC will silently use 1.0 kg and
  produce wrong forces.
- **Rule:** Always set every RigidColliderState field explicitly in new
  call sites, even when defaults look reasonable. Round 7 added the field
  to both `bench_mpm_rigid_drift` and `EditorApp::step` simultaneously.

---

## Scenario time misalignment for supplementary video

- **Trigger:** building a side-by-side video of `box_drop`, `rover_wheel`,
  `foot_step` from per-method `*_frames.csv`.
- **Issue:** scenarios have different `total_time` values (3 s, 3 s, 2 s)
  and different `frame_every_n` settings (all now 5 → 48 fps). Concatenating
  CSVs naively produces frames that drift out of sync.
- **Resolution options:**
  1. **Trim** every clip to `min(total_time)` = 2 s (loses information).
  2. **Time-normalise** each clip to t ∈ [0, 1] by scaling frame indices —
     video lengths equal, but absolute time labels diverge.
  3. **Pad** shorter clips with their last frame held — simplest, used by
     most "scenario comparison" videos in graphics papers.
- **Decision (deferred to video-editing phase):** option 3 (pad), but only
  documented once `render_animation.py` actually produces multi-scenario
  composites.
- **Source-of-truth rule:** scenario durations are *parameters* in
  `scenario_*.h`; never hard-code "2 s" or "3 s" in plot or video scripts.

## Time-based contact cache age + variable dt

- **Trigger:** simulation rate change (60 → 120 → 240 Hz) or jittered dt
  (user-driven step calls).
- **Resolution:** `MAX_CACHE_AGE_SECONDS = 0.05f`, age accumulates in
  wall-clock seconds (not frame count). Round 5 added parametric test;
  Round 6 added jitter test.
- **Caveat:** If you ever introduce sub-stepping inside `step_fixed`, age
  must accumulate `internal_dt`, not the outer step's dt.

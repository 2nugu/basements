# Risk Log

Active risks per phase, ranked by impact × probability. Update at phase exits
or whenever a risk materialises (move to "Resolved" or "Materialised").

---

## Active risks

### R-1 — Scoop risk: Taichi or similar group publishes PIC-Rigid first

- **Phase:** ~~1~~ → **SUPERSEDED at Pivot 6 (2026-05-16).** Paper #1
  dropped; this risk does not apply to paper #2 (differentiable MPM
  + adjoint method has different reviewer audience and novelty surface).
  Refer to **R-8 / R-9 / R-10** below for paper #2 scoop risks.
- *(original analysis retained for audit trail)*
- **Impact:** ★★★★ — paper #1 reduced to "concurrent / independent" framing.
- **Probability:** ★★★ — Hu's group is the most likely competitor; no public
  preprint yet but logical next step from their 2018 SIGGRAPH paper.
- **Mitigation:**
  1. arXiv preprint within 3 months (Phase 1.3).
  2. Weekly check of arXiv `cs.GR` + `cs.RO` for "MPM" + "rigid".
  3. Emphasise our **friction-dominated robotics regime** — Hu's prior work
     focuses on water/snow softer scenarios, so a different reviewer audience.

### R-2 — Editor crash bug from user input

- **Phase:** 1 (UX track)
- **Impact:** ★★ — blocks robotics demo, doesn't block paper.
- **Probability:** ★★★★ — user has reported it twice in Round 6 / 7.
- **Mitigation:**
  1. `editor_log.txt` + trace mirror are in place — need user to send the
     log line for actual fix.
  2. Until then, paper figures can be regenerated CLI-only without touching
     Editor — paper path is not blocked.

### R-3 — MPM solver constitutive model is "elastic-only"

- **Phase:** 1 → **RESOLVED at Pivot 4 (2026-05-15).** Hencky-Kirchhoff
  stress force loop added; Drucker-Prager projection corrected to
  Klar 2016 eq. 3.1 form with the missing kohesion factor. See
  `PROJECT_STATUS.md::Pivot 4` for verification (5/5 tests PASS,
  Lube H/D reproduced, repose 35° calibrated).
- *(original analysis retained for audit trail)*
- **Impact:** ★★★ — reviewer may flag missing hyperelastic energy density.
- **Probability:** ★★ — reviewers may accept the simplification at short-paper
  scope, less likely at full-paper scope.
- **Mitigation:** Add fixed-corotated energy + cauchy stress derivation in
  Phase 1.2 (~1 day work). Defers to Phase 2 if reviewers don't flag.

### R-4 — Time underestimation (1-person calendar variability)

- **Phase:** all
- **Impact:** ★★★ — schedule slippage cascades.
- **Probability:** ★★★★ — almost certain.
- **Mitigation:**
  1. Roadmap times are *calendar*, not full-time.
  2. Phase boundaries are pivot points (continue/stop), not deadlines.
  3. Each phase has a fallback exit criterion that ships *something* even if
     full plan slips.

### R-5 — GPU CUDA backend never lands

- **Phase:** 3
- **Impact:** ★★ — kills "general engine" framing, doesn't kill paper.
- **Probability:** ★★★★ — Phase 3 is 4–6 months of single-discipline work
  competing with paper revision time.
- **Mitigation:**
  1. Don't claim GPU support in Phase 1 paper text. Paper benches are CPU.
  2. Phase 3 is optional — failure to land triggers Phase 5 pivot.

### R-6 — Genesis or other engine subsumes MPM-rigid niche before release

- **Phase:** 4
- **Impact:** ★★★ — Phase 4 release becomes "yet another coupling library".
- **Probability:** ★★ — Genesis focuses on differentiable, not granular accuracy.
- **Mitigation:** Differentiate on **measurable conservation error figures**
  (paper #1) — community will check before adoption.

### R-7 — Quaternion convention mismatch in future I/O bridges

- **Phase:** 2 (asset pipeline)
- **Impact:** ★★ — silently breaks all imported assets.
- **Probability:** ★★★ — common bug pattern across formats (glTF uses xyzw,
  USD uses wxyz, FBX uses xyzw, BVH motion capture varies).
- **Mitigation:** `notes/integration_notes.md::Quaternion convention mismatch`
  is the canonical check. Add a unit test that loads a sample of each format
  and verifies axis directions.

### R-8 — Differentiable MPM scoop (Pivot 6 paper #2 risk)

- **Phase:** 2 (paper #2)
- **Impact:** ★★★★ — paper #2 main novelty is plastic projection adjoint +
  M4.5a non-local coupling gradient. If Huang group (DiffMPM 2022) or
  similar publishes coupling-aware DiffMPM before us, novelty is contested.
- **Probability:** ★★ — Huang focuses on cloth/soft body, not rigid-coupling;
  DiffTaichi line targets soft sim. Our angle (rigid coupling + agricultural
  scenario) is differentiated.
- **Mitigation:**
  1. Weekly arXiv `cs.LG` + `cs.RO` + `cs.GR` scan for "differentiable MPM",
     "DiffMPM", "differentiable simulator + rigid coupling".
  2. M3 / M4 adjoint math published as arXiv preprint as soon as
     finite-difference verification passes (don't wait for full paper).
  3. Agricultural-robot framing + myCobot sim-to-real differentiates
     from any pure-CG DiffMPM paper.

### R-9 — Plastic projection adjoint math correctness

- **Phase:** 2 (M3)
- **Impact:** ★★★★ — wrong gradient = paper rejected; reviewers will check.
- **Probability:** ★★★ — SVD gradient + D-P cone subgradient is non-trivial;
  Klar 2016 forward projection has measure-zero discontinuity at cone surface.
- **Mitigation:**
  1. Finite-difference verification declared as paper §3.3 ground truth
     (see `notes/pivot6_differentiable_mpm.md` §6).
  2. Pre-registered FD test schedule on Zenodo before M3 figure renders.
  3. Cross-check against Huang 2022 DiffMPM hard-projection adjoint where
     paths overlap.

### R-10 — myCobot sim-to-real gap exceeds publishable threshold

- **Phase:** 2 (M7)
- **Impact:** ★★★ — §4.5 hardware section weakened or dropped; paper #2
  becomes "sim-to-sim" instead of "sim-to-real".
- **Probability:** ★★★ — sim-to-real gaps of 50-100% are routine in granular
  domains.
- **Mitigation:**
  1. Hardware fallback declared (synthetic domain-randomization sweep
     replaces §4.5 if gap is unmanageable).
  2. Paper title remains "sim-to-real" only if §4.5 PASS criterion
     (predicted F_x within ±30% of measured) is met; otherwise reframe
     title pre-submission.
  3. Pivot-4 meta-finding repackaged as "domain randomization signal" to
     turn measurement noise into a methodology contribution.

---

## Resolved risks (kept for audit)

- **R-X (Round 5):** Stale incremental build segfaults Editor → Resolved by
  full-rebuild rule (notes/integration_notes.md).
- **R-X (Round 6):** ImGui InputText inside menu sub-menu → Resolved by
  moving label editor to BeginPopupModal.
- **R-X (Round 5):** Per-axis hysteresis asymmetry between mismatched bodies
  → Resolved by min(half_a[i], half_b[i]) per-axis scheme (Round 5/7).

---

## Risk policy

- **★★★★** active = blocker for current phase, work on it before everything else.
- **★★★** active = mitigated but unresolved; revisit weekly.
- **★★** active = monitor; mitigation plan in place.
- A risk moves to "Resolved" only when the mitigation is *in code or in docs*,
  not just "discussed".

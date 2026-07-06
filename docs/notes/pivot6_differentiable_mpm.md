# Pivot 6 — Paper #2 Design: Differentiable MPM-Rigid Coupling for Agricultural Sim-to-Real

**Status:** Design draft. Implementation kick-off post Pivot 6
(2026-05-15). Replaces dropped paper #1 — see
[`../PROJECT_STATUS.md::Pivot 6`](../PROJECT_STATUS.md) for the
audit and re-ordering rationale.

**Working title:** *Differentiable Material Point Method for
Agricultural Robot Sim-to-Real: Soil-Machine Interaction Learning
with Validated Forward Physics*

**Author:** Hong-Gu Lee (이홍구), KNU Smart Agriculture + Biosystems
Engineering. `hgl@kangwon.ac.kr`.

**Target venues:** ICRA / CoRL / NeurIPS / IROS. arXiv preprint as
`cs.RO` primary + `cs.LG` secondary cross-list.

---

## 1. Why this paper

Three converging forces:

1. **Sim-to-real RL for agricultural robotics** is a hot, growing
   subfield with insufficient differentiable simulators for
   soil-machine interaction. Production pipelines use Bekker
   empirical fits (Wong 1989) — not differentiable — or DEM (not
   continuum, slow). MPM is the natural middle ground but no
   open differentiable MPM-rigid coupling exists.
2. **The Basements forward sim** (paper #1 carry-forward) provides
   a validated MLS-MPM + Hencky-Drucker-Prager continuum with a
   layered rigid-coupling ablation (M0/M1/M4.5{a,b,c}). It is the
   *forward-pass* foundation; only the *backward pass* is new.
3. **User identity match.** KNU Smart Agriculture + Biosystems
   Engineering = agricultural-robot policy learning is the natural
   research direction. ML expertise (deep learning, RL) is required
   for the policy-training portion. myCobot hardware available for
   sim-to-real validation.

## 2. Algorithmic contributions (4 novel)

### 2.1 Adjoint method through Hencky-Drucker-Prager plastic projection

The forward plastic projection (Klar 2016 eq. 3.1) is:

$$
\mathbf{F}_{\text{trial}} = (\mathbf{I} + \nabla\mathbf{v}\,\Delta t)\,\mathbf{F}^n, \quad
\mathbf{F}_{\text{trial}} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^\top
$$

with projection onto the yield cone when
$\|\hat{\boldsymbol{\varepsilon}}\| > -\alpha k\,\mathrm{tr}(\boldsymbol{\varepsilon})$,
$k = (3\lambda+2\mu)/(2\mu)$, $\boldsymbol{\varepsilon} = \ln\boldsymbol{\Sigma}$.

The backward pass requires the gradient
$\partial \mathbf{F}^{n+1} / \partial \mathbf{F}^n$ through this
projection. Two paths:

- **Hard projection (Klar)** → subgradient at the cone surface;
  non-differentiable on a measure-zero set, but well-defined
  smooth gradient elsewhere. We adopt this and document the
  measure-zero discontinuity in §6.
- **Soft projection (Huang 2022 DiffMPM)** → smooth cone via
  sigmoid-like rounding; differentiable everywhere but introduces
  forward-pass error. Rejected: would invalidate paper #1's
  forward-sim calibration.

**Novelty:** the explicit adjoint through SVD (logarithmic strain
basis) + cone projection in our hard-projection regime, with proof
of consistency at the yield surface boundary.

### 2.2 Differentiable two-pass sweep through M4.5a

M4.5a's two-pass normalisation
$m_b^{(i)} = M_{\text{body}} \cdot \mathrm{frac}_i / \sum_j \mathrm{frac}_j$
is itself differentiable, but the dependency on
$\sum_j \mathrm{frac}_j$ (sum over all overlap nodes) creates
*non-local* gradient flow between grid nodes.

**Novelty:** the *non-local rigid-coupling gradient*. Existing
DiffMPM implementations use local kernel weights without
explicit normalisation, missing this dependency.

### 2.3 Multi-phase constitutive (dry / moist / cohesive) as domain randomization axis

Real agricultural soils span a parameter space rather than a single
calibrated point. Wet sand adds capillary pressure
$\sigma_{\text{cap}} = f(\theta_w)$, cohesive soil adds cohesion
$c$ to the Drucker-Prager yield surface. We parameterise all three
in a single differentiable continuum:

$$
\boldsymbol{\sigma} = \boldsymbol{\sigma}_{\text{elastic}}(E, \nu, \boldsymbol{\varepsilon})
+ \boldsymbol{\sigma}_{\text{capillary}}(\theta_w)
- c\,\mathbf{I}
$$

with yield projection adjusted to
$\|\hat{\boldsymbol{\varepsilon}}\| - \alpha k\,(\mathrm{tr}(\boldsymbol{\varepsilon}) - c/K) \le 0$.

**Novelty:** unified differentiable parameterisation across
$(E, \nu, \alpha, c, \theta_w)$ enables *domain randomization* for
sim-to-real transfer in a single forward-backward sim instance.

### 2.4 Sim-to-real policy learning on agricultural implement control

Policy network $\pi_\theta(s) \to a$ takes (soil state, implement
pose) and outputs (implement velocity, force command). Trained in
sim via differentiable rollouts (Brax-style, Freeman 2021), deployed
on myCobot for the `tine_drag` scenario.

**Novelty:** first reported sim-to-real RL on an agricultural
implement using differentiable MPM. Benchmark and code released.

## 3. Paper structure (8 pages, ICRA / CoRL format)

```
§1 Introduction (1 page)
   - Agricultural robot policy learning gap
   - Existing sim alternatives (Bekker, DEM, MuJoCo grids) — what's missing
   - 4 contributions

§2 Related Work (¾ page)
   - DiffTaichi (Hu 2020), DiffMPM (Huang 2022), gradSim (Murthy
     2021), Brax (Freeman 2021), differentiable sim survey.
   - MPM-rigid coupling (Klar 2016, Hu 2018, Jiang 2017) — forward
     sim only, no differentiable coupling.
   - Sim-to-real RL (Höfer 2021), domain randomization (Tobin 2017).
   - Agricultural robot policy learning (recent papers).

§3 Method (2 pages) — *the paper's core*
   3.1 Forward MLS-MPM + Hencky-DP + M4.5a (Basements carry-forward,
       1 paragraph + Appendix A pointer)
   3.2 Backward gradient flow (DiffTaichi pattern, our extension to
       rigid coupling)
   3.3 Plastic projection adjoint (§2.1 derivation, 1 page)
   3.4 Rigid coupling gradient (§2.2 derivation, ½ page)
   3.5 Multi-phase constitutive (§2.3 unified parameterisation)

§4 Experiments (2 pages)
   4.1 Forward validation (carry from paper #1: Lube H/D,
       repose, tine_drag, M0/M1/M4.5 ablation tables)
   4.2 Gradient correctness (finite-difference check on M4.5a)
   4.3 Policy training on tine_drag (RL convergence curves)
   4.4 Domain randomization sensitivity (sweep over E, c, θ_w)
   4.5 Sim-to-real on myCobot (real vs predicted draft force)

§5 Results (1 page)
   - Forward consistency tables (Basements §5 carry-forward)
   - Gradient accuracy metrics
   - Policy reward curves (sim) + sim-to-real gap

§6 Discussion / Limitations (¾ page)
   - Pivot-4 meta-finding repackaged as domain-randomization
     signal (not a bug, a feature).
   - Plastic projection measure-zero discontinuity.
   - myCobot single-platform validation; field-scale = paper #4.

§7 Future work + Conclusion (½ page)
   - Paper #3 (GPU + adaptive real-time)
   - Paper #4 (field deployment)
```

## 4. Architecture decision — in-place autodiff (not firewall)

**Rejected pattern:** separate `extensions/diff_mpm/DiffMPMSolver`
class that duplicates forward-sim math with its own tape. Originally
proposed to keep paper #1 forward sim "untouched", but paper #1 is
dropped at Pivot 6, removing the justification. Two-codebase pattern
is real maintenance burden for a solo engineer.

**Adopted pattern:** `MPMSolver` gains an opt-in `step(dt, tape*)`
overload. When `tape == nullptr`, hot path is unchanged (only one
branch added, ~ns cost). When `tape != nullptr`, p2g / update_grid /
g2p record intermediate state for backward pass. PyTorch / JAX /
DiffTaichi all use this pattern; it is the *least-overhead* way to
introduce autodiff without doubling the codebase.

Consequences:
- One source of truth for forward math.
- Forward-only consumers (external validation submissions, paper #1
  archive consistency, existing tests) unaffected — they call the
  no-tape overload.
- Gradient tests sit alongside forward tests in `tests/`; no parallel
  test suite to maintain.
- New work goes inside `mpm_solver.h` plus a small
  `mpm_autodiff_tape.h` header. No new top-level directory.

## 5. Implementation roadmap

```
M1 (month 1) — pybind11 binding + AutodiffTape skeleton
              ├─ basements_py module: forward sim Python callable
              ├─ torch.Tensor in/out (position, velocity, F, grid mass)
              ├─ AutodiffTape struct + step(dt, tape*) overload
              │  (forward-only path identical to today)
              └─ smoke test: PyTorch forward call matches C++ output

M2 (month 2) — Backward gradient via DiffTaichi pattern
              ├─ taichi-style reverse-mode autodiff for p2g, g2p,
              │  update_grid (Hu 2020 reference)
              ├─ test_diff_mpm: finite-difference gradient check
              │  on simple compression (∂position / ∂F at small ε)
              └─ verify < 1% deviation from numerical gradient

M3 (month 3) — Plastic projection adjoint (§2.1)
              ├─ SVD gradient (well-known, e.g. Papadopoulo & Lourakis
              │  2000)
              ├─ Log-strain → projection → exp-strain chain
              ├─ Cone-surface subgradient handling
              └─ test_diff_plastic: gradient check at yield surface

M4 (month 4) — M4.5a coupling gradient (§2.2)
              ├─ Two-pass sweep backward pass
              ├─ Non-local Σ frac dependency tracked
              └─ test_diff_coupling: gradient check on tine_drag setup

M5 (month 5) — Multi-phase constitutive (§2.3)
              ├─ Capillary pressure term
              ├─ Cohesion in yield projection
              └─ Forward validation against wet-sand literature

M6 (month 6) — RL infrastructure
              ├─ Policy network (small MLP + image input from sim
              │  state via SP-Grid → tensor)
              ├─ PPO or SAC training loop on tine_drag
              ├─ Hyperparameter sweep
              └─ Reward design: minimize draft force at constant depth

M7 (month 7) — Real myCobot experiments
              ├─ ArUco verification (paper #1 W4-prep work, executed)
              ├─ Tine_drag hardware setup (modified probe + soil container)
              ├─ Trained-policy deployment on real hardware
              └─ Sim-to-real gap measurement (predicted vs measured F_x)

M8-9 (months 8-9) — Paper draft, revisions, submission
              ├─ §1-§7 LaTeX (largely carrying forward from paper #1)
              ├─ Adjoint derivation in §3 (math-heavy, careful)
              ├─ External validation submissions (§6, ongoing)
              └─ ICRA or CoRL submission deadline
```

## 5. Carry-forward checklist from paper #1

What survives:
- [x] MPMSolver (Hencky + Klar + M4.5a) → §3.1 forward sim
- [x] scenarios/*.h (box_drop, rover_wheel, foot_step, tine_drag,
      repose_pile) → §4 validation suite
- [x] test_mpm_constitutive, test_mpm_stress_isolation,
      test_mpm_rigid_sweep → §4.2 gradient correctness baseline
- [x] Lube H/D sweep results → §4.1 validation
- [x] paper_outline.md §3-§5 prose → §3.1 + §4 source
- [x] paper/references.bib → extend with DiffTaichi etc.
- [x] L3 ros2_bridge scaffold → §4.5 hardware bridge
- [x] external_validation_protocol → §6 community section
- [x] literature_uncertainty + sim_validity_domain → §6 limitations
- [x] AUTHORS.md, CITATION.cff, CONTRIBUTING.md → unchanged

What's new (must build):
- [ ] pybind11 binding `basements_py`
- [ ] PyTorch autograd integration
- [ ] DiffTaichi-style reverse-mode autodiff
- [ ] Plastic projection adjoint
- [ ] M4.5a non-local coupling gradient
- [ ] Multi-phase constitutive (capillary + cohesion)
- [ ] RL policy training infrastructure
- [ ] Hardware sim-to-real pipeline
- [ ] New §3.2-§3.5 LaTeX (math-heavy adjoint derivations)
- [ ] New §4.2-§4.5 LaTeX (gradient + policy + sim-to-real)

## 6. Reference-grade validation of novel gradients

Three pieces of math in this paper have differently-mature reference
status. Defending each requires a different validation strategy
declared *up front* (paper §3.X for each):

| Component | Reference | Validation |
|-----------|-----------|------------|
| SVD gradient (used in plastic adjoint chain) | Papadopoulo & Lourakis 2000 (closed form) | Cross-check against numpy / PyTorch SVD autograd on identical inputs |
| Drucker-Prager cone projection adjoint | Huang et al. 2022 (DiffMPM) for the hard-projection variant | Finite-difference check at points *not on the cone surface* (smooth interior); explicit subgradient demonstration at the surface |
| **M4.5a non-local coupling gradient** | **None — paper #2 own contribution** | **Finite-difference numerical ground truth declared as the validation tool in paper §3.4**. Pre-register the FD step sizes, particle counts, and tolerances in the pre-registration deposit. |

The M4.5a non-local gradient is the *highest reviewer attack risk*
because no published reference exists. The mitigation is to declare,
in paper §3.4 itself, that the numerical (finite-difference) gradient
is the *ground truth* against which our analytical implementation is
verified — and to report the relative L₂ error across a sweep of
particle counts, kernel widths, and operating depths. A reviewer who
attacks the analytical derivation has to also attack the numerical
verification, which is mechanical (less attackable).

Pre-registration deposit (Zenodo) for paper #2 §3.4 will include:
- FD step size schedule (h ∈ {1e-3, 1e-4, 1e-5, 1e-6})
- Particle count sweep (N ∈ {10, 100, 1000})
- Pass criterion: relative L₂ error < 1% at the optimal h

This converts a reviewer-fragile novelty claim into a
reviewer-defensible *measurement* claim.

## 7. Risks and mitigations

| Risk | Mitigation |
|------|-----------|
| Plastic projection adjoint non-trivial math | Reference DiffMPM (Huang 2022) hard projection adjoint as starting point; consult solid-mechanics literature on yield-surface subgradients |
| Sim-to-real gap large | Pivot-4 lessons → domain randomization range; report gap honestly |
| pybind11 + PyTorch performance prohibitive for RL training | Plan paper #3 (GPU + adaptive) for real-time; paper #2 uses offline rollouts |
| myCobot validation late | tine_drag sim alone is publishable; hardware §4.5 is bonus |
| 6-9 month timeline overruns | Re-scope to skip §2.3 multi-phase (defer to paper #3) if at M5 not converged |

## 7. References to add to `paper/references.bib`

```bibtex
@inproceedings{hu2020difftaichi, ...}
@article{huang2022diffmpm, ...}
@article{murthy2021gradsim, ...}
@article{freeman2021brax, ...}
@article{hofer2021sim2real, ...}
@inproceedings{tobin2017domain, ...}
@article{papadopoulo2000svd_gradient, ...}
```
(populated when paper-#2 LaTeX work begins)

## 8. Change log

- 2026-05-15 — Initial design doc. Captures Pivot 6 paper #2
  scope, novelty claims, paper structure, implementation roadmap,
  carry-forward checklist, risks. Replaces paper #1 outline.

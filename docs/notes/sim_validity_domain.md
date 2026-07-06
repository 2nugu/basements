# Simulator Validity Domain

**Purpose:** Pre-declare the regimes inside which our MPM-rigid coupling
solver is expected to predict physical behaviour, and the regimes
outside which we *expect failure*. Pass criteria from
`literature_uncertainty.md` apply only inside the green regime.
Paper §7 figures color-code every measurement by regime, so cherry-picking
is structurally impossible.

**Cross-references:**
- `docs/notes/literature_uncertainty.md` — pass criteria per experiment
- `docs/notes/l3_pipeline_design.md` §6 — experiment regime classification
- `docs/paper_outline.md` §3 / §7 — domain claim in paper body

---

## 1. The three regimes

| Regime | Color | Pass criterion applies | Definition |
|--------|-------|-----------------------|------------|
| **Green** | green-marker | YES | All four validity axes satisfied |
| **Yellow** | yellow-marker | reported, excluded from pass count | Exactly one axis violated, by < 2× |
| **Red** | red-marker | reported, marked out-of-scope | Two or more axes violated, OR one axis violated by ≥ 2× |

---

## 2. Validity axes (declared before any measurement)

### Axis 1 — Spatial resolution
- **Green:** characteristic feature length $L_{\text{feat}} \ge 3 \cdot d_{50}$
  where $d_{50}$ is median grain diameter
- **Yellow:** $L_{\text{feat}} \in [1, 3] \cdot d_{50}$
- **Red:** $L_{\text{feat}} < d_{50}$ (continuum assumption breaks)

**Rationale:** Below ~3 grains, discrete-grain effects dominate
(Roux & Combe 2002, Goldhirsch 2010). Our MPM is continuum.

### Axis 2 — Kinematic regime (impact velocity)
- **Green:** $v_{\text{impact}} < v_{\text{shock}}$ where
  $v_{\text{shock}} \approx \sqrt{(λ+2μ)/ρ}$ is the bulk sound speed
  (≈ 21 m/s for our W2 sand: E=5×10⁵, ρ=1500)
- **Yellow:** $v_{\text{impact}} \in [0.5, 1.0] \cdot v_{\text{shock}}$
- **Red:** $v_{\text{impact}} \ge v_{\text{shock}}$ (shock-wave regime
  unmodelled; explicit Euler diverges)

### Axis 3 — Material model assumption
- **Green:** dry, single-phase, near-zero cohesion
- **Yellow:** mildly moist (relative humidity < 30%, no capillary
  bridging) or slightly cohesive (cohesion < 1 kPa)
- **Red:** wet sand, fluidised, or strongly cohesive (clay, snow)

**Out of scope entirely.** This is a *scope statement*, not a sim
limitation. Future Paper #2 would add multi-phase or cohesion models.

### Axis 4 — Statistical convergence
- **Green:** particle count $N \ge 100$ in the region of interest
  AND $N$ particles cover $\ge 10$ grid cells
- **Yellow:** $N \in [30, 100]$ OR $N$ in $< 10$ grid cells
- **Red:** $N < 30$ (statistical noise dominates the measurement)

---

## 3. Worked example — sphere drop heights (Experiment A)

Material: Ottawa F-65, $d_{50} = 0.21$ mm. Sandbox 200×200×100 mm,
dx = 25 mm. Sphere radius = 6 mm.

| Drop height | $v_{\text{impact}}$ | Pred. depth | $d_{\text{pred}} / d_{50}$ | $v / v_{\text{shock}}$ | Regime |
|------------|---------------------|-------------|----------------------------|------------------------|---------|
| 5 cm       | 1.0 m/s             | ~0.4 mm     | 1.9                        | 0.05                   | **Yellow** (Axis 1) |
| 10 cm      | 1.4 m/s             | ~1.0 mm     | 4.8                        | 0.07                   | **Green** |
| 15 cm      | 1.7 m/s             | ~1.6 mm     | 7.6                        | 0.08                   | **Green** |
| 20 cm      | 2.0 m/s             | ~2.3 mm     | 11.0                       | 0.09                   | **Green** |
| 25 cm      | 2.2 m/s             | ~3.0 mm     | 14.3                       | 0.10                   | **Green** |
| 30 cm      | 2.4 m/s             | ~3.7 mm     | 17.6                       | 0.11                   | **Green** |
| 35 cm      | 2.6 m/s             | ~4.4 mm     | 21.0                       | 0.12                   | **Green** |

(Predicted depths from preliminary sim runs; will update with
measured values after Experiment A.)

**Pre-registered claim:** point at h=5 cm should show ≥ 15%
deviation between sim and measurement, *and* the predicted-green
points should pass. This makes the validity-domain boundary itself
a positive measurement, not an assumption.

---

## 4. Paper §7 figure protocol

```
log F_pred vs log F_meas                  (or log d_pred vs log d_meas)
─────────────────────────────────────────────
│      ◇   ◇    ◇   ◇                       │
│    ◇             ◇                         │
│   ◇                  ▲    ✕               │
│  ◇   y = x diagonal              ✕         │
│    ±15% band shaded                        │
└─────────────────────────────────────────────

Legend:
  ◇ Green (pass criterion applies)
  ▲ Yellow (reported, not counted)
  ✕ Red (out-of-scope, demonstrates validity boundary)
```

Caption text template:
> "Markers color-coded by pre-registered regime classification
> (Section 3, `sim_validity_domain.md`). Pass criterion ±X%
> (see `literature_uncertainty.md` for source) applied to green
> markers only. Yellow / red points reported as positive evidence
> of the validity-boundary location."

This pattern *defends against cherry-picking* and *converts a
limitation into a positive contribution* (the boundary itself is
measured).

---

## 5. Change log

- 2026-05-15 — Initial draft. Worked example for sphere drop locked
  in pre-registration; will be tagged on Zenodo before W4 measurement.

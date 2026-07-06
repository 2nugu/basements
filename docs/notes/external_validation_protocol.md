# External Validation Protocol

**Purpose:** Specification any third-party lab can follow to validate
the Basements MPM-Rigid coupling solver against their own physical
measurements. Cited by paper #1 §7 + Appendix C as the
*"first pre-registered MPM-rigid coupling validation protocol"*.
Brand-neutral by design — if N≥5 external reproductions accumulate,
paper #2 will formalise a registry; until then this document is the
authoritative reference.

**Used by:**
- `validation/external_submissions/README.md` — submission entry point
- `scripts/external_validator.py` — automated PASS/FAIL report
- Paper #1 §7 + Appendix C — citation reference

---

## 1. Minimum hardware

| Item | Spec / model | Approx. cost |
|------|--------------|--------------|
| Robot arm | myCobot 280 / 320 / Pro 600 (any) | $349 – $1,499 |
| Smartphone or camera | ≥ 240 fps slow-mo capture | already owned (most modern phones) |
| Sandbox | ≥ 200 × 200 × 100 mm, rigid walls | $10 |
| Reference sand | Ottawa F-65 (1 kg) **OR** equivalent standardised | $50 – $100 |
| Drop sphere | Diameter ≈ 12 mm, mass ≈ 7 g (plastic or aluminum) | $1 |
| ArUco marker printout (driver verification) | Standard 6×6 markers, ≥ 50 mm side | $0 (laser print) |

**Equivalent sand** = any sand with measured $E$, $\nu$, $\phi$, $\rho_{\text{bulk}}$,
$d_{50}$ (from a geotech lab or paid service) submitted in
`config/sandbox_USER.yaml` per the schema in §4.

---

## 2. Five-step protocol (gating)

```
Step 1  Hardware procurement
        └─ Verify each row of §1 minimum-hardware table

Step 2  Sand parameter measurement
        └─ Geotech lab (preferred) OR paid service (KICT-equivalent)
        └─ Output: sandbox_USER.yaml with measured values + dates
        └─ See §4 for YAML schema

Step 3  Driver verification (GATING STEP)
        ├─ Print 4× ArUco markers, attach to end-effector + sandbox corners
        ├─ Calibrate smartphone camera (chessboard, OpenCV standard)
        ├─ Run scripts/verify_driver.py
        │   ├─ Commands arm to 20 random poses
        │   ├─ Reads ArUco-measured pose vs driver-reported pose
        │   └─ Computes RMS position error, RMS orientation error,
        │       repeatability
        └─ PASS condition (all three):
            ▸ RMS position error < 1.0 mm
            ▸ RMS orientation error < 1.0°
            ▸ Repeatability < 0.5 mm
        └─ If FAIL: post issue with verify_driver_report.csv. DO NOT proceed.

Step 4  Sphere drop experiment (per pre-registration)
        ├─ Per height h ∈ {5, 10, 15, 20, 25, 30, 35} cm, repeat n = 10
        ├─ Record raw video per drop
        ├─ Process via scripts/measure_penetration_depth.py
        └─ Output: sphere_drop_results.csv

Step 5  Submit data
        └─ git fork our repo, add data under
            validation/external_submissions/lab-<name>-<date>/
        └─ Open pull request
        └─ scripts/external_validator.py auto-runs in CI; report attached.
```

Steps 3 and 4 are **pre-registered**: the parameter ranges, regime
boundaries, and pass criteria are deposited on Zenodo with a DOI prior
to any data collection (see `docs/notes/l3_pipeline_design.md` §6).

---

## 3. Pass criteria

Imported from `docs/notes/literature_uncertainty.md` §7:

| Experiment | Criterion | Validity domain |
|-----------|-----------|----------------|
| Step 3 — driver | RMS_pos < 1 mm, RMS_rot < 1°, repeat < 0.5 mm | required for Step 4 |
| Step 4 — green points only | depth within ±15% of sim prediction, mean of n=10 | per `sim_validity_domain.md` Axis 1 |
| Step 4 — boundary check | predicted-yellow points show ≥ 15% deviation | positive evidence of regime boundary |

A submission **passes** if:
1. Step 3 PASS, **AND**
2. ≥ 75% of green-regime sphere-drop points satisfy ±15%, **AND**
3. ≥ 1 yellow-regime point shows the predicted deviation
   (cherry-picking defence: green-only submissions fail this check)

---

## 4. YAML schema — sandbox_USER.yaml

```yaml
# Required fields
sand:
  name: "Ottawa F-65"                # free-text identifier
  measurement_lab: "Kangwon NU Geotech Lab"
  measurement_date: "2026-XX-XX"
  measurement_certificate: "kn-gt-2026-xxx.pdf"  # included in submission
  E_pa:          3.5e7               # Young's modulus [Pa]
  nu:            0.30                # Poisson ratio
  phi_deg:       32.0                # friction angle [°]
  rho_bulk:      1650                # bulk density [kg/m³]
  d50_m:         2.1e-4              # median grain diameter [m]
  cohesion_pa:   0                   # if non-zero, may exit green regime

container:
  size_x_m:      0.20
  size_y_m:      0.20
  size_z_m:      0.10
  wall_friction: 0.5                 # estimated, not critical

sphere:
  diameter_m:    1.2e-2
  mass_kg:       7.0e-3

camera:
  fps:           240                 # ≥ 240 required for ±15% pass
  resolution_x:  1920
  resolution_y:  1080

robot:
  model:         "myCobot 280 Pi"
  serial:        "MC280-XXXXXX"
  driver_version: "pymycobot 3.6.1"
```

Validator will reject submissions missing any required field.

---

## 5. CSV schemas

### `verify_driver_report.csv`
```
pose_idx, cmd_x_m, cmd_y_m, cmd_z_m, cmd_roll_deg, cmd_pitch_deg, cmd_yaw_deg,
          rep_x_m, rep_y_m, rep_z_m, rep_roll_deg, rep_pitch_deg, rep_yaw_deg,
          aru_x_m, aru_y_m, aru_z_m, aru_roll_deg, aru_pitch_deg, aru_yaw_deg
```
- `cmd_*` — commanded pose
- `rep_*` — driver-reported pose (from forward kinematics)
- `aru_*` — ArUco-measured pose (ground truth)

### `sphere_drop_results.csv`
```
drop_idx, height_m, trial_idx, t_release_sec, depth_measured_m,
          notes_optional
```
- 7 heights × 10 trials = 70 rows minimum
- `depth_measured_m` from frame-by-frame video analysis
- `notes_optional` for "ball bounced", "ball rolled" exclusion flags

---

## 6. Authorship / credit policy

Submissions accepted as PRs are credited in two ways:
1. The lab name + date appears in `validation/external_submissions/index.md`
2. If the submission contributes to paper #2 (community paper), the
   submitting lab is offered authorship (decided per case; co-authorship
   for substantial contributions, acknowledgment for routine submissions)

No paywall, no exclusivity. CC-BY-4.0 license for data, MIT for code.

---

## 7. Pre-registration commitment (for paper #1)

Authors commit that:
- The protocol in this document was finalised and tagged on Zenodo
  (DOI: 10.5281/zenodo.XXXXXXX, deposit date 2026-XX-XX) **before** any
  paper §7 measurement.
- Deviations from the protocol during measurement are reported in
  paper §7.X "Protocol Deviations".
- All raw data (video, ArUco logs, joint-state logs) are released on
  the same Zenodo DOI as the paper.

---

## 8. Change log

- 2026-05-15 — Initial draft. Mirrors structure in `l3_pipeline_design.md`
  §6 + new external-lab-facing onboarding flow.

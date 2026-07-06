---
experiment_id: TEMPLATE
version: v0.0
authors:
  - name: ""
    affiliation: ""
    orcid: ""
deposit_date: ""
zenodo_doi: ""
commitment_signature: |
  This document was finalised on [date] prior to any data collection.
  The author commits to reporting all results from experiments
  conducted under this protocol, including those that fail to satisfy
  the pass criteria. Deviations from the protocol must be reported
  in paper §7.X "Protocol Deviations".
---

# Pre-registration — Experiment [ID]

**Status:** template. Copy to `case_studies/hardware/<experiment_id>/preregistration.md`
and fill in every field before tagging on git.

## 1. Equipment
- Robot: [model + serial number]
- Camera: [phone model + native fps]
- Sandbox: [container length × width × height, wall material]
- Drop sphere (if applicable): [diameter, mass, material]

## 2. Material parameters
- Sand identifier: [Ottawa F-65 / Jumunjin / other]
- $E$ = [value] Pa  (source: [measurement laboratory + date / literature reference])
- $\nu$ = [value]   (source: ...)
- $\phi$ = [value]° (source: ...)
- $\rho_{\text{bulk}}$ = [value] kg/m³ (source: ...)
- $d_{50}$ = [value] m (source: ...)
- Cohesion = [value] Pa  (zero for dry granular; non-zero exits green regime)

## 3. Experimental schedule
- Independent variable: [e.g. drop height, insertion depth]
- Values tested: { [list of numbers + units] }
- Trials per value: n = [value]
- Total measurements: [N]
- Order: [randomised / sequential / blocked]
- Exclusion criteria (pre-defined):
  - [Criterion 1, e.g. "trial discarded if sphere spin > 30 rpm at impact"]
  - [Criterion 2, ...]

## 4. Regime classification (per `docs/notes/sim_validity_domain.md`)
- Green (pass criterion applies): [list of pre-classified conditions]
- Yellow (reported, not counted): [list]
- Red (out-of-scope, demonstrates boundary): [list, if any]

## 5. Pass criteria
- Green points: [mean within ±X% of sim prediction, per
  `docs/notes/literature_uncertainty.md` Table 7]
- Boundary check (positive evidence): [predicted-yellow points must
  show ≥ Y% deviation between sim and measurement]
- Aggregate pass: [≥ Z% of green points satisfy criterion]
- A submission **passes** if all three conditions above are met.

## 6. Statistical analysis
- Per-condition aggregation: [mean ± SD across n trials]
- Comparison test: [paired t-test / Mann-Whitney / Welch's t-test]
- Significance threshold: α = [0.05 by default]
- Effect size: [Cohen's d / relative error]
- Multiple comparisons: [Bonferroni / Holm / none, with justification]

## 7. Data products
After measurement, the following files must be deposited alongside
this pre-registration on the same Zenodo DOI:
- raw video files for at least 3 sample trials
- `*_results.csv` per the schema in `external_validation_protocol.md` §5
- analysis notebook (`*.ipynb`) reproducing the figures
- final figure `*.png` + `*.csv` per the visualisation standard

## 8. Pre-registration timestamp
- Commit hash: (filled in by `scripts/lock_preregistration.py` on git tag)
- Git tag: `preregistration-<experiment_id>-v<version>`
- Zenodo DOI: (assigned by Zenodo on archive)
- Deposit date: (yyyy-mm-dd)

## 9. Sign-off

Author signature: ________________________  Date: ____________

(Digital signature via PGP recommended for high-stakes experiments;
plaintext name + date acceptable for in-house validation.)

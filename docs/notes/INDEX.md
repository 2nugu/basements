# docs/notes — Index

Working lab notebook organised by category. Each file is a focused
note; the registry below groups them by topic so navigation does not
require reading file names linearly.

## Paper #2 active design (Pivot 6)

- [`pivot6_differentiable_mpm.md`](./pivot6_differentiable_mpm.md) — paper #2 architecture, adjoint method sketch, 9-month implementation roadmap, paper-#1 carry-forward checklist

## Theory / engineering

- [`element_tech.md`](./element_tech.md) — element technology lab notebook
- [`integration_notes.md`](./integration_notes.md) — cross-system integration gotchas
- [`risk_log.md`](./risk_log.md) — active risks and mitigations per phase
- [`limitations.md`](./limitations.md) — structural limits + adoption / validation paths

## Validation methodology (Pivot 4 + Pivot 5)

- [`literature_uncertainty.md`](./literature_uncertainty.md) — per-experiment measurement uncertainty floors; source of every pass criterion
- [`sim_validity_domain.md`](./sim_validity_domain.md) — green / yellow / red regime classification for cherry-picking defence
- [`external_validation_protocol.md`](./external_validation_protocol.md) — third-party lab reproduction protocol (paper Appendix C)
- [`zenodo_setup.md`](./zenodo_setup.md) — DOI-backed pre-registration workflow setup guide
- [`geotech_outreach.md`](./geotech_outreach.md) — Tier 1-4 cascade for measured sand parameters (deferred to paper #2)

## Hardware sim2real path (L3)

- [`l3_pipeline_design.md`](./l3_pipeline_design.md) — myCobot sim2real bridge design contract (Phase 1 offline, Phase 3 runtime)

## Bibliography

- [`references.md`](./references.md) — working bibliography (synced with `paper/references.bib`)

## Cross-file invariants

- Pass criteria in `literature_uncertainty.md` §7 are the single source
  for every `validity_domain` decision in `sim_validity_domain.md` and
  for every `expected_accuracy_pct` field in
  `extensions/ros2_bridge/config/*.yaml`.
- The directory tree in `l3_pipeline_design.md` §7 is the source of
  truth for the W3 ros2_bridge scaffold layout; deviations require
  updating the anti-foreclosure checklist (§8).
- The Tier 1-4 outreach cascade in `geotech_outreach.md` is **deferred
  to paper #2 scope** (paper #1 uses ASTM-standardised sand + literature
  values); kept on file for Phase 2 trigger.

## Top-level docs (above this directory)

For new readers — start here, then come back to specific notes:

| Doc | Purpose |
|-----|---------|
| [`../PROJECT_STATUS.md`](../PROJECT_STATUS.md) | Current phase, Pivot Record (1–6), Open Questions, Docs Registry |
| [`../roadmap.md`](../roadmap.md) | Phase plan + 3-paper series (Pivot 6 re-ordered) |
| [`../checklist.md`](../checklist.md) | Phase 2 active milestones + Phase 1 historical record |
| [`../FORK_NOTE.md`](../FORK_NOTE.md) | Phase 1 fork rationale (superseded at Pivot 6, kept as record) |
| [`../paper_outline.md`](../paper_outline.md) | Paper #1 prose **(ARCHIVED at Pivot 6; paper #2 §3.1 + Appendix A source)** |
| [`../../paper/README.md`](../../paper/README.md) | LaTeX paper #1 source **(ARCHIVED at Pivot 6)** |
| [`../../README.md`](../../README.md) | Project tagline + Pivot 6 framing + quick reproduction |

## Maintenance

When adding a new note:
1. Place it under one of the categories above.
2. Register it in this INDEX.md.
3. Register it in `docs/PROJECT_STATUS.md::Docs Registry`.
4. Cross-link from related notes (the L3 design doc and the external
   protocol cross-cite each other, for example).

## Pivot 6 (2026-05-16) — what changed in this index

- Added "Paper #2 active design" category at top (pointing to
  `pivot6_differentiable_mpm.md`).
- Added "Top-level docs" pointer table for new readers.
- Existing four categories (Theory / Validation methodology /
  Hardware sim2real / Bibliography) unchanged — they are
  Pivot-6-stable.

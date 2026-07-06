# Contributing to Basements

Basements is a single-engineer research project actively building toward a
paper publication and an external community-validation network. We welcome
contributions in four areas:

1. **External validation submissions** — reproduce paper §7 sphere-drop
   measurements on your own myCobot (or compatible) hardware and submit
   data via [`validation/external_submissions/`](validation/external_submissions/README.md).
   This is the highest-priority contribution path for the community paper
   (paper #2).
2. **Case-study additions** — add a new physical scenario (silo discharge,
   shear cell, triaxial compression, etc.) via the registry pattern in
   `research/mpm_rigid_coupling/bench_compare_all.cpp`.
3. **Bug reports** — open a GitHub issue with reproduction command +
   environment.
4. **Algorithmic improvements** — open a discussion before opening a PR
   for non-trivial changes (M2 ASFLIP real impl, GPU backend, adaptive
   resolution, etc.).

## Code-of-conduct expectations

- Pull requests must pass all tests (`cmake --build build --config Release --target check`)
- New features need at least one regression test
- Match the existing code style; do not reformat unrelated files
- Discuss before refactoring shared headers

## Pre-registration commits — DO NOT REWRITE

Commits tagged `preregistration-*` are deposited on Zenodo with permanent
DOIs. Their git hash is the immutable third-party record for the
cherry-picking defence in paper §6 and Appendix C.

Forbidden operations on these commits:
- `git rebase` that touches the commit
- `git push --force` that overwrites the tag
- `git filter-branch` / `git filter-repo` over the commit
- `git commit --amend` against the tag

Permitted:
- Adding new commits AFTER the tag
- Creating a v2 pre-registration if the experimental design changes
  materially. This triggers a NEW Zenodo deposit with a NEW DOI; the v1
  remains as the original commitment.

If a pre-registration document is found to have a typo or factual error
after deposit, the policy is: file an erratum as a separate commit and
new pre-registration version. Do NOT amend the original.

## Authorship policy (validation submissions)

For external lab submissions accepted into `validation/external_submissions/`:

- Listed in `validation/external_submissions/index.md` with lab name + date
- Acknowledgment in paper #2 (community paper)
- **Substantial contributions** offered co-authorship on paper #2
  (decided case-by-case; substantial = N ≥ 5 independent trials with
  full driver + sand parameter measurements)

For code contributions to the core solver or research scaffold:

- Listed in `AUTHORS.md` (created when first contribution lands)
- Acknowledgment in relevant papers
- Co-authorship offered for algorithmic novelty contributions
  (new coupling method, new constitutive model, new validation
  experiment design)

## Reporting security issues

Open a GitHub security advisory or email `hgl@kangwon.ac.kr` directly.

## Maintainer

Hong-Gu Lee (이홍구) — Kangwon National University, Smart Agriculture
+ Biosystems Engineering. `hgl@kangwon.ac.kr`.

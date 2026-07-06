# Zenodo + GitHub Pre-Registration Workflow

**Purpose:** Establish a DOI-backed pre-registration pipeline that
deposits each experiment's protocol on Zenodo before any data
collection. The DOI's third-party timestamp is the cherry-picking
defence cited in paper #1 §6.

**Status:** Manual setup pending user action (Zenodo account + GitHub
permission cannot be granted by code). All file templates below are
ready; once the user completes the one-time setup, the workflow runs
on every `git tag preregistration-*` push.

---

## 1. One-time setup (user action, ~30 minutes)

### Step 1 — Zenodo account
1. Visit https://zenodo.org/ and sign up (free, GitHub OAuth supported)
2. Verify email
3. In account settings, set "Default community" if desired (optional)

### Step 2 — Link GitHub repository
1. Go to https://zenodo.org/account/settings/github/
2. Click "Sync now" to refresh the GitHub repo list
3. Find this repository (Basements) in the list
4. Toggle the switch to ON

   → Zenodo now watches the repo for new releases and creates an
   archive + DOI for each one.

### Step 3 — Verify with a test tag
```
git tag test-zenodo-link-v0
git push origin test-zenodo-link-v0
```

Within ~5 minutes, Zenodo dashboard at https://zenodo.org/account/
should show the test deposit. After confirming, delete it on
Zenodo (deposit not yet published) and remove the tag:

```
git tag -d test-zenodo-link-v0
git push origin :refs/tags/test-zenodo-link-v0
```

### Step 4 — Reserved DOI (optional but recommended for paper)
On Zenodo, click "New Upload" → "Reserve DOI" → save the DOI string.
This DOI can be cited in paper drafts before the experiment publishes.

---

## 2. Per-experiment pre-registration workflow

For each experiment (A sphere drop, B probe insertion, etc.):

### Step 1 — Author the pre-registration document

Template at `case_studies/hardware/_template/preregistration.md`.
Required fields (see template):
- Equipment list (model, serial, fps)
- Material parameters with measurement source
- Experimental schedule (heights, trials, exclusions)
- Regime classification rule
- Pass criteria per regime
- Statistical analysis method
- Pre-defined exclusion criteria

Copy template to the experiment directory:
```
case_studies/hardware/A_sphere_drop/preregistration.md
```

Fill in all fields. Sign with PGP or document author signature + date.

### Step 2 — Commit and tag
```
git add case_studies/hardware/A_sphere_drop/preregistration.md
git commit -m "preregistration A sphere drop v1.0"
git tag preregistration-A-sphere-drop-v1.0
git push origin preregistration-A-sphere-drop-v1.0
```

### Step 3 — Zenodo auto-archives
Within ~5 minutes Zenodo creates the archive and assigns DOI.
Retrieve from https://zenodo.org/account/settings/github/.

### Step 4 — Record DOI in repo
Add the assigned DOI to:
- The experiment's `preregistration.md` (front-matter or footer)
- `docs/notes/external_validation_protocol.md` §7
- Paper §6 citation entry

### Step 5 — Lock the commit
Once deposited, the commit referenced by the tag must NEVER be
rewritten (force-push, rebase). The `.gitignore` line in §3 below
plus the policy in `CONTRIBUTING.md` enforce this.

---

## 3. Repo policy — immutable pre-registration commits

Add to `CONTRIBUTING.md`:

```markdown
## Pre-registration commits — DO NOT REWRITE

Commits tagged `preregistration-*` are deposited on Zenodo with
permanent DOIs. Their git hash is the immutable third-party record
for cherry-picking defence in paper §6.

Forbidden operations on these commits:
- `git rebase` that touches the commit
- `git push --force` that overwrites the tag
- `git filter-branch` / `git filter-repo` over the commit

Permitted:
- Adding new commits AFTER the tag
- Creating a v2 pre-registration if the experiment design changes
  materially (this triggers a NEW Zenodo deposit, NEW DOI; the v1
  remains as the original commitment)

If a pre-registration document is found to have a typo or factual
error after deposit, the policy is: file an erratum as a separate
commit + new pre-registration version, NOT amend the original.
```

---

## 4. Template — `preregistration.md`

```markdown
---
experiment_id: A_sphere_drop
version: v1.0
authors:
  - name: [Author Name]
    affiliation: [Institution]
    orcid: 0000-0000-0000-0000   # if available
deposit_date: 2026-XX-XX
zenodo_doi: 10.5281/zenodo.XXXXXXX   # assigned after deposit
commitment_signature: |
  This document was finalised on [date] prior to any data collection.
  The author commits to reporting all results from experiments
  conducted under this protocol, including those that fail to satisfy
  the pass criteria. Deviations from the protocol must be reported
  in paper §7.X "Protocol Deviations".
---

# Pre-registration — Experiment [ID]

## 1. Equipment
- Robot: [model + serial]
- Camera: [phone model + fps]
- Sandbox: [container size + material]
- Drop sphere: [diameter + mass + material]

## 2. Material parameters
- Sand: [type]
- $E$ = [value] Pa  (source: [measurement / literature])
- $\nu$ = [value]   (source: [...])
- $\phi$ = [value]° (source: [...])
- $\rho_{\text{bulk}}$ = [value] kg/m³ (source: [...])
- $d_{50}$ = [value] m (source: [...])

## 3. Experimental schedule
- Heights tested: { [list] } cm
- Trials per height: n = [value]
- Total measurements: [N]
- Exclusion criteria: [e.g. "discard trial if sphere spin > 30 rpm"]

## 4. Regime classification (per `sim_validity_domain.md`)
- Green: [pre-classified heights]
- Yellow: [pre-classified heights]
- Red: [pre-classified heights, if any]

## 5. Pass criteria
- Green points: [mean depth within ±X% of sim prediction]
- Boundary check: [predicted-yellow points must show ≥ Y% deviation]
- Aggregate: [≥ Z% of green points pass]

## 6. Statistical analysis
- Test: [paired t-test / Mann-Whitney / ...]
- Significance threshold: α = [value]
- Effect size measure: [Cohen's d / ...]

## 7. Pre-registration timestamp
Commit hash: (assigned by git tag)
Tag: preregistration-[id]-v[version]
Zenodo DOI: (assigned by Zenodo on deposit)
```

---

## 5. Action items (user)

- [ ] Create Zenodo account
- [ ] Link this repo to Zenodo
- [ ] Optionally reserve a DOI for paper #1
- [ ] Verify with a test tag (Step 3 above)
- [ ] Add the CONTRIBUTING.md section (text in §3)
- [ ] On first experiment design: copy template (§4) to
      `case_studies/hardware/A_sphere_drop/preregistration.md`
      and fill in

After these steps the workflow is fully autonomous on subsequent
experiments.

---

## 6. Change log

- 2026-05-15 — Initial draft. All file templates ready; Zenodo account
  setup is the single user action gating the workflow.

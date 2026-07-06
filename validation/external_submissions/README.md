# External Validation Submissions

This directory accepts third-party laboratory reproductions of the
Basements MPM-Rigid coupling validation experiments. Submissions are
opened as pull requests; an automated validator runs in CI and posts a
PASS/FAIL report.

**Protocol authority:** [`../../docs/notes/external_validation_protocol.md`](../../docs/notes/external_validation_protocol.md)

**Pre-registration record:** Zenodo DOI 10.5281/zenodo.XXXXXXX (to be
populated on first deposit; see protocol §7).

---

## How to submit

1. Read the full protocol document above.
2. Procure hardware per protocol §1.
3. Measure your sand parameters (geotech lab or paid service).
4. Run driver verification — must PASS before proceeding.
5. Run sphere-drop experiment per the height schedule.
6. Fork this repository.
7. Create a new directory under `external_submissions/`:

   ```
   external_submissions/
   └── lab-<lowercase-name>-<YYYY-MM-DD>/
       ├── sandbox_USER.yaml
       ├── verify_driver_report.csv
       ├── sphere_drop_results.csv
       ├── raw_video/                 # at least 1 sample drop video
       └── lab_metadata.md            # PI name, institution, contact
   ```

8. Open a pull request.
9. CI runs `scripts/external_validator.py` and posts the report as a
   PR comment.
10. Maintainers review within 14 days; merged on PASS, comments on FAIL.

---

## Current submissions

| Lab | Date | Sand | PASS | DOI |
|-----|------|------|------|-----|
| (none yet) | | | | |

---

## Credit policy (mirrored from protocol §6)

- Submission accepted → lab name in `index.md` and paper #2
  acknowledgments.
- Substantial contributions → offered co-authorship on paper #2.
- Data CC-BY-4.0; code MIT.

---

## FAQ

**Q. Can I submit without ArUco verification?**
A. No. Step 3 is gating. Driver accuracy bounds Step 4 results;
   without it, your measurement uncertainty is unknown and we cannot
   validate.

**Q. My driver fails Step 3. What now?**
A. Open an issue with `verify_driver_report.csv`. Two common causes:
   (a) gripper backlash — recalibrate with a tighter offset, (b) camera
   distortion — re-run OpenCV chessboard calibration with more samples.

**Q. Can I use a different robot arm (UR3, Franka, KUKA)?**
A. Yes, in principle. Adapt the protocol's hardware section. We expect
   paper #2 to incorporate non-myCobot submissions; until then,
   myCobot reproductions form the reference set for paper #1's claims.

**Q. Can I use non-Ottawa sand?**
A. Yes, provided you submit measured ($E$, $\nu$, $\phi$, $\rho$,
   $d_{50}$) values per the YAML schema. The validator does not require
   a specific sand; it requires *characterised* sand. Literature values
   alone are NOT accepted — your specific sample must be measured.

**Q. What about non-standard scenarios (probe, drag, F/T comparison)?**
A. Phase-1 reproduction is sphere-drop only. Probe / drag / F/T
   experiments are paper #1 §7.X stretch goals; their protocols will
   be added to this directory when the corresponding experiments
   are pre-registered.

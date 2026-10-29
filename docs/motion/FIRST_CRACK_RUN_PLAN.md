# Plan: the first crack-pipeline run on the powered arm with a printed crack (2026-10-29)

Goal: when the operator switches the arm on, everything else is already done. One preflight command says go or
no-go, one bench command has already shown the real print is detected by the real camera, and one session command
walks the operator through search -> scan -> plan -> trace with the right defaults. Every arm step stays behind
`~/rebot_setup/run-trajectory.sh` (guard pre-flight, hold at the fold, Enter to go).

## State found 2026-10-29 (workstation)

| | |
|---|---|
| D405 | on USB (8086:0b5b), enumerates from the crackvision env (pyrealsense2 2.58.4, 1 device); last validated capture 2026-10-22 |
| USB2CAN / arm | unplugged (no /dev/ttyACM*), as expected |
| arm_guard | hooks OK in rebot and lerobot envs; config approved **for offline testing only, "not yet reviewed by the operator"** |
| runner | `~/rebot_setup/demo_move.py` 2026-10-25.1 (`--state-file`, `--ff-bias`); kd 4 on J2/J3 still an override (`DEMO_EXTRA="--gains j2=100/4,j3=80/4"`) |
| plans | `data/crack_trace/search.json` and `scan.json` from 2026-10-23, before the crack_filter change (trajectories are unaffected, regenerate anyway) |
| stale | `data/crack_trace/capture.pid` points at a dead process |
| prints | `data/crack_targets/crack_prints.pdf` built 2026-10-22; whether it has been printed is unknown |
| crack gate | `crack_filter` on by default in capture / search-capture / plan (CRACK_SELECTION_RESEARCH.md); never seen a real print through the real camera |
| camera mount | `end_effector.yaml` wrist_camera is a CAD prior; the mount side (up-facing gripper face at the fold) is an assumption the operator confirms by eye |
| offline chain | `scripts/crack_pipeline.sh demo` passes end to end (2026-10-29) |

Timeouts worth knowing: search-capture waits up to 300 s per stop and capture 600 s for the arm to settle; the runner
switches off after 5 minutes without a go at the fold.

## What gets built (workstation, no arm)

1. **`scripts/crack_pipeline.sh preflight`**: a single go/no-go. Checks: project env (`check_env`), model files, guard
   hook-status and approval record (WARN if not operator-reviewed), D405 present + USB 3 + a short validated capture,
   nothing else holding the camera, stale pid/state files cleaned, disk space, runner file present and its sha256
   recorded, `search.json` and `scan.json` regenerated and rehearsed x1 and x2 with the real guard config, the print PDF
   present. Prints a PASS/WARN/FAIL table and writes `data/crack_trace/preflight_<UTC>.json`. Exit 0 only on no FAIL.
2. **`scripts/crack_pipeline.sh bench [CASE]`**: the real print under the real camera, **arm off**. The operator holds
   or props the camera about 25 cm over the print (or lays the print under the folded arm's camera). A 10-frame burst,
   the crack gate with depth, the component table and an overlay; says clearly whether the print's crack is accepted,
   its length/width in mm and the depth. Also `bench --live`: the filtered live view in the browser for aiming. This is
   the step that proves detection before anything moves.
3. **`scripts/crack_pipeline.sh session CASE [slow]`**: walks search -> scan-plan --from-search -> scan -> plan ->
   (operator looks at the preview) -> trace, with slow=2 and the kd 4 override as defaults (`DEMO_EXTRA` still wins),
   a typed `yes` between steps, and a stop on any failed step with the reason and the log path. It must not add any
   way to move the arm that bypasses run-trajectory.sh.
4. **`scripts/crack_pipeline.sh archive CASE`**: collects the case (search/scan/plan outputs, component tables,
   previews, capture logs, the runner's run CSV/JSON for each arm step, the preflight JSON, guard events) into
   `evidence/operator/first-crack-run-<date>/` with a README template for the operator's notes.
5. **`docs/motion/FIRST_CRACK_RUN.md`**: the runbook, in the style of FIRST_GUARDED_RUN.md: at the desk (preflight,
   bench), physical setup (print #1 flat and taped at about x 0.30 m, y 0, inside the search area x 0.22-0.40 /
   y +-0.20; matte, evenly lit, no glare; camera on the up-facing gripper face), at the arm (power on, read-only
   `--check`, session), a "you see / it means / do this" table (no crack found, crack touches the border, every
   component rejected and why, the arm did not settle, guard fault, trace offset = calibration, hover 14 cm is
   expected), and after (archive, what to file).
6. Tests and checks: `bash -n` on the script, unit tests for any new Python, the full suite still green, the offline
   demo still passing, and preflight run for real on this workstation with its output in the report.

## Operator decisions the runbook must surface (not decide)

- Re-approve the guard config under the operator's own name (`python -m arm_guard approve --by ...`).
- Whether kd 4 on J2/J3 is the default for this run (the plan assumes yes, as an override).
- Firmware 5.17 for the D405: not needed for this run.
- Print #1 (single clean crack) is the recommended first target; print the PDF "fit to page" on matte paper.

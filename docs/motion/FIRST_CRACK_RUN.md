# First crack run on the powered arm — operator runbook

**Status: ready, not yet run on the arm (2026-10-29).** Everything that can be done without the arm is done. Preflight
on the workstation says **GO** with one WARN: the guard config is still approved *for offline testing only* and
waits for your review. The D405 is on USB 3: 30 fps, no dropped frames, validated. Pointed at the room with no print
under it, bench accepted nothing (expected). The plan behind this runbook is
[FIRST_CRACK_RUN_PLAN.md](FIRST_CRACK_RUN_PLAN.md); how each step works inside is in
[CRACK_TRACE_PIPELINE.md](CRACK_TRACE_PIPELINE.md).

Every arm step goes through `~/rebot_setup/run-trajectory.sh`, as in [FIRST_GUARDED_RUN.md](FIRST_GUARDED_RUN.md):
the guard pre-flight, the hold at the fold, Enter to go, `stop`/`off`. Nothing in this runbook adds another way to
move the arm.

## What runs

```
crack_pipeline.sh preflight        desk checks, rehearsals only           (no arm)
crack_pipeline.sh bench [CASE]     the print under the camera, arm OFF     (camera)
crack_pipeline.sh session CASE     search -> scan-plan -> scan -> plan -> trace, a typed yes before each step
   search / scan / trace  ─► run-trajectory.sh ─► demo_move.py ─► arm_guard ─► motorbridge ─► USB2CAN ─► motors
   search / scan          ─► crack_trace search-capture / capture in the background (camera + model, reads the
                             runner's measured pose from ~/rebot_setup/runs/arm_state.json)
crack_pipeline.sh archive CASE     copies everything into evidence/operator/first-crack-run-<date>/
```

| Step | Arm | What it does | Time at x2 |
|---|---|---|---|
| search | yes | The camera, looking straight down from 25 cm, visits 6 stops over the table (x 0.26 / 0.36 m, y −0.15 / 0 / +0.15 m) and rests 3 s at each. The model runs at every stop. Result: where the crack is | about 70 s plus the fold moves |
| scan-plan | no | Solves the scan pose: the camera 25 cm above the crack the search found, looking straight down. Rehearsed in simulation | seconds |
| scan | yes | The arm goes to the scan pose and holds it 15 s at x1 (30 s at x2). The capture grabs 10 frames once the arm has rested and runs the crack gate | about 50 s with the fold moves |
| plan | no | Lifts the best accepted crack to the table and plans the trace. Rehearsed x1 and x2 in simulation. Writes the **preview** | seconds |
| trace | yes | The tool tip follows the crack **about 14 cm above the table**, then retracts and folds | about 30 s for a 20 cm crack, plus the fold moves |

**Timeouts:**
- search-capture waits up to **300 s at each stop** for the arm to rest there.
- capture waits up to **600 s** for the arm to rest at the scan pose.
- The runner switches every motor off at the fold if you have not pressed Enter within **5 minutes**.

## Decisions that are yours

The tools do not make these for you:

1. **Re-approve the guard config under your own name.** It is approved *for offline testing only* ("not yet reviewed
   by the operator"), and preflight shows that as a WARN. To review and approve:
   ```bash
   cd ~/Projects/rebot_crack_vision/packages/arm_guard/src
   ../../../env.sh python -m arm_guard check      # the limits the guard enforces; read them
   ../../../env.sh python -m arm_guard approve --by <your name> --note "reviewed for the first crack run"
   ```
2. **kd 4 on J2/J3 for this run.** `session` passes `DEMO_EXTRA="--gains j2=100/4,j3=80/4"` to the runner unless
   you set `DEMO_EXTRA` yourself. Making kd 4 the permanent default is a separate decision (TODO.md item 0).
   **How `DEMO_EXTRA` combines** (the runner *replaces* its gains with whatever `--gains` it is given; nothing is
   merged on the runner side, so `session` merges for you):
   - unset → `--gains j2=100/4,j3=80/4`;
   - set **without** `--gains` (e.g. `DEMO_EXTRA="--friction j4=0.18"`) → the kd 4 gains are **prepended**:
     `--gains j2=100/4,j3=80/4 --friction j4=0.18` (the session's first line says "default gains prepended");
   - set **with** `--gains` → used exactly as given, so include the kd 4 gains yourself, e.g.
     `DEMO_EXTRA="--gains j2=100/4,j3=80/4,j4=40/3"`;
   - set to empty (`DEMO_EXTRA= bash scripts/crack_pipeline.sh session …`) → nothing: the stock kd 1.

   When you call `run-trajectory.sh` or `crack_pipeline.sh search|scan|trace` by hand, nothing merges: put the kd 4
   gains in `DEMO_EXTRA` yourself. **No quotes and no spaces inside a value:** `DEMO_EXTRA` is split on spaces,
   unquoted (`--gains j2=100/4,j3=80/4` works; `--gains "j2=100/4, j3=80/4"` does not).
   **No `--ff-bias` in `DEMO_EXTRA` for the session:** the capture's settle check needs |v| < 0.05 rad/s on every
   joint, and the bias keeps J2 moving above that, so the search stops and the scan may never settle (the session
   prints a WARNING if it sees `--ff-bias`).
3. **D405 firmware 5.17.** This run does not need it. The camera ran at 30 fps with no dropped frames on 5.15.1.55.
4. **Which print.** Print #1, the single clean crack, is the recommended first target. In
   `data/crack_targets/crack_prints.pdf` it is page 1. Print it "fit to page" on matte paper. The crack then runs
   across the whole page, about 26 cm.

## 1. At the desk (arm off, about 5 minutes)

```bash
cd ~/Projects/rebot_crack_vision
bash scripts/crack_pipeline.sh preflight
```

Preflight takes about 90 s. Nothing moves: the runner is only called with `--rehearse`. It prints one row per check,
each PASS, WARN or FAIL with a reason. A row that is not PASS also gets a fix line. The record is
`data/crack_trace/preflight_<UTC>.json`, and `session` refuses to start without a GO preflight from the last 12 hours.

| Row | What it checks |
|---|---|
| env, model | `scripts/check_env.py`: Python, torch + CUDA, GPU, nnU-Net, pyrealsense2; the OpenCrack checkpoint and plans |
| guard approval | `python -m arm_guard check` passes. WARN when the approval does not name a person |
| guard hooks | the guard is installed in the `rebot` env (the runner's; FAIL if not) and in `lerobot` (WARN) |
| runner files | `run-trajectory.sh` and `demo_move.py` present, `--state-file` supported, version and sha256 of each runner file recorded |
| arm idle | no runner, `basic-demo.sh`, LeRobot or motor tool running. It also says whether the USB2CAN is plugged in (never opened) |
| stale files | removes a `capture.pid` whose process is gone and a leftover `~/rebot_setup/runs/arm_state.json` |
| disk | free space under `data/` (WARN under 10 GB, FAIL under 2 GB) |
| camera free | no other process has the D405's `/dev/video*` or USB node open |
| D405 | on USB (`lsusb` 8086:0b5b), found by pyrealsense2, on a USB 3 link |
| D405 capture | 30 frames recorded and validated (`validate_recording`): fps, dropped frames, depth |
| search.json, scan.json | regenerated, then rehearsed x1 and x2 with the real guard config |
| print PDF | the crack print PDF is there |
| crack gate | `crack_filter.crack_trace: true` in `config/project.yaml` |

Exit 0 means GO: no FAIL. Read the WARN rows: they are your decisions.

**Bench: the real print under the real camera, arm off.** This is the step that proves detection before anything
moves.
```bash
bash scripts/crack_pipeline.sh bench --live     # aim: http://127.0.0.1:8090, red = accepted, yellow = rejected
                                                 # Ctrl-C in that terminal to stop it
bash scripts/crack_pipeline.sh bench print01    # 10 frames, the crack gate, the table, an overlay, a verdict
```
- Hold the camera about **25 cm** over the print, square to it, or lay the print under the folded arm's camera.
  25 cm is the scan distance.
- Use even light, no glare. The whole crack must be inside the picture.
- **Stop `bench --live` (Ctrl-C) before `bench`, `search` or `scan`:** only one program can have the camera.
  `bench` and `session` refuse to start while something holds it, and say which process.

`bench` prints one row per component: accepted or rejected, length and width in mm, the camera-to-surface distance,
mean p, and in plain words why a rejected one was rejected. Then a verdict line:

```
VERDICT: CRACK ACCEPTED: component #1, 212 mm long, 3.0 mm wide, at 24.8 cm, mean p 0.91; camera to scene 25 cm
```

The overlay is `data/crack_trace/bench/<CASE>/overlay.png`: accepted components red, rejected ones yellow, the
number at each component. Exit 0 = accepted, 3 = nothing accepted, **or** the camera is busy (another program, e.g.
`bench --live`, holds it: the message says which process) or could not be opened. Exit 2 = that bench CASE folder
already exists: pick another CASE name, or pass `--force` to overwrite it (`bench print01 --force`). **Do not go to
the arm until bench accepts the print's crack at about 25 cm.**

## 2. Physical setup

- **Print #1, flat and taped, centred at about x 0.30 m, y 0.** x is straight ahead of the base, measured from the
  J1 axis; y is to the side. The search covers x 0.22–0.40 m and y ±0.20 m. Run the long side left to right, as
  seen from the base.
- Matte paper, flat (tape all four corners: a curled edge is a depth step), evenly lit, no glare, no arm shadow on it.
- **Camera on the up-facing gripper face**, on Seeed's stock 15° mount. At the fold, the D405 should be on the side
  of the gripper that faces the ceiling. `end_effector.yaml` assumes this, and it is not yet confirmed on this arm.
  If it is on the other side, stop: the search would look the wrong way.
- The camera cable has slack at every joint, J4–J6 especially. J6 can roll, and neither the planner nor the guard
  knows the cable exists.
- Base clamped, area clear, nothing else within reach of the arm.

## 3. At the arm

**Preconditions:** the desk steps passed (preflight GO, bench accepted the print). The arm is folded with J1 facing
forward. `bench --live` is stopped. Nothing else is using the arm or the camera.

1. Switch on the 24 V. Plug in the USB2CAN; `/dev/ttyACM0` appears.
2. **Read-only check** (opens the bus, reads, enables nothing). The runner needs an absolute path:
   ```bash
   cd ~/rebot_setup
   DEMO_EXTRA="--gains j2=100/4,j3=80/4" ./run-trajectory.sh --check ~/Projects/rebot_crack_vision/data/crack_trace/search.json 2
   ```
   Expected: `pre-checks OK; path OK: …` and `arm_guard pre-flight OK: …`. Exit 4 means the J1 reference is unknown.
   The session's first step then asks you to turn the base to forward and type `yes`.
3. **The session:**
   ```bash
   cd ~/Projects/rebot_crack_vision
   bash scripts/crack_pipeline.sh session print01        # slow 2 and the kd 4 override by default
   ```
   Before each step the session prints what will happen and what you will see, and waits for a typed **`yes`**.
   Anything else stops the session, and nothing more runs. At each arm step, run-trajectory.sh enables, moves to
   the fold and holds; **you press Enter** at its prompt (only an empty line starts the run; any other text is
   answered with "type Enter to run, stop, or off"). Typed while the arm is still moving to zero: `stop` or `off`
   is acted on as soon as it holds (stop = park + off, off = off at the fold); an Enter or anything else is discarded.
   `stop` + Enter parks to the fold and switches off; `off` + Enter switches off at once (at the fold prompt: off
   at the fold; while moving: the arm drops). The **24 V switch is the e-stop at all times: a raised arm
   drops.**

   1. **search.** The camera visits the 6 stops. You get one `stop N:` line per stop, then
      `search: crack found at stop N, centre [x, y, z]`.
   2. **scan-plan.** No arm. Rehearsal `outcome: completed`.
   3. **scan.** The arm holds over the crack for 30 s at x2. You see `captured case …` and the crack filter table.
   4. **plan.** No arm. Then the session prints the **preview path** and the accepted component numbers, and which
      component the trace follows. **Open the preview** (`data/crack_trace/<CASE>_trace_preview.png`). Green, the
      planned path, must sit on red, an accepted crack, on the crack you laid down. The circle marks the start. If
      it does not, type anything but `yes`.
   5. **trace.** The tool tip draws the crack about 14 cm above the print, then folds.

   The session stops at the first step that fails: a non-zero exit, a runner outcome other than `completed`, or
   no runner summary at all. It prints the log path and the runner summary. To go on from there after fixing the
   cause, run the remaining subcommands by hand: `bash scripts/crack_pipeline.sh scan print01 2`, and so on
   (CRACK_TRACE_PIPELINE.md).

**Expected, not a fault:**
- **The tool hovers about 14 cm above a table crack.** The guard keeps gripper_link at least 12 cm above the base
  plane, and the planner adds 1.5 cm.
- **The trace is a few cm off the crack.** The camera mount and the tool tip are CAD values until GEOM-05
  (hand-eye) and GEOM-07 (tool pivot) calibrate them. The hover is sized for this. Measure the offset and write it
  down; do not lower the hover to compensate.

## 4. What can happen, and what to do

| You see | What it means | What to do |
|---|---|---|
| preflight `NO-GO` | At least one FAIL row. | Do what its fix line says, then run preflight again. |
| bench `outside the 7-50 cm working distance` | The camera is too close or too far: the D405's depth works from 7 to 50 cm. | Hold it about 25 cm over the print. |
| bench `the model found nothing crack-like` | No candidate at all. | Check the print is in view (`bench --live`), the light, glare. |
| bench `the model is not confident enough (mean p < 0.8)` | A faint or blurred line. | More even light, hold still, about 25 cm. |
| bench `no valid depth around it` | No depth next to the crack: under 7 cm, glare, or a glossy print. | Matte paper, no glare, 25 cm. |
| bench `on a depth step or an object edge` | The surface around it is not flat: a curled print or the edge of the sheet. | Tape the print flat on the table. The sheet's own edge is often rejected this way, which is correct. |
| bench `wider than 10 mm` / `shorter than 10 mm` | A blob, a shadow or a speck, not a crack. | Nothing, if the print's crack is accepted beside it. |
| a component `flagged: straight and even, could be a joint` | Flagged, not rejected: straight and even like a seam or a paper edge. | Check in the preview that the trace follows the crack, not the edge. |
| `(touches the image edge)` | Part of the crack is outside the picture. | Move the print or the camera so the whole crack is in view. The search prefers stops where it is not cut off. |
| `the camera is busy: pid N (...)` | Another program has the D405 open, usually `bench --live`. | Ctrl-C in its terminal, then run the step again. |
| `search: no crack found at N/6 stops` | No stop saw an accepted crack. | Look at `data/crack_trace/<CASE>_search/stop_NN_overlay.png`: was the print in view? Move it to x 0.30, y 0 and start the session again. |
| `stop N skipped: the arm did not settle … within 300 s` | The arm never rested at that stop: it did not move, or is still oscillating. | If the runner refused or timed out, see those rows. Otherwise, after the run, look at the run CSV for wobble. |
| capture `the arm did not settle at the scan pose within 600 s` or `left the scan pose without settling` | The scan pose was never held still; nothing was captured. | Same as above. Run `scan` again once the cause is clear. |
| capture `no crack found in the view: nothing to trace` (exit 3) | The scan view has no accepted crack. | Look at the capture's overlay. The search target may be off: run search again. |
| plan `no trace: …` | The crack cannot be traced within reach, clearance and joint limits. | Move the print closer to x 0.30, y 0. |
| the preview's green is not on red, or the red is not your crack | The trace would follow the wrong thing. | Type anything but `yes`. Archive the case and look at the component table. |
| `REFUSED (… nothing was enabled)` | A runner pre-check or the guard pre-flight failed, or you pressed Ctrl-C / typed `stop` before anything was enabled. Nothing moved. | Read the reason. "not starting from the folded rest": fold the arm by hand. J1 not forward: turn the base. "gravity margin": plan the pose with less reach (or run without `--gravity-model`). |
| `J1 reference unknown … type yes` | J1 lost its turn count over a 24 V cycle. | Turn the base by hand to face forward, then type `yes`. Ctrl-C or anything else: not confirmed, exit 4, nothing enabled. |
| `no go within 300 s: disabling` | You did not press Enter within 5 minutes. Switched off at the fold. | The session stops (outcome `no_go_timeout`). Start again when ready. |
| `ABORT: [tracking/torque/velocity …]` then `PARK` | The runner's own abort. | Nothing: it parks slowly and switches off. The session stops. |
| `WARNING while holding after the guard fault: [stream] …; still holding` | The 200 Hz loop skipped a cycle while holding. The hold is re-sent; the motors never stopped holding. | Nothing. `stop` / `off` still work. |
| `GUARD FAULT, HOLDING: <reason>` | The guard refused a command and holds the arm. **Nothing is switched off.** | Look at the arm. `stop` + Enter: reset (logged) and slow park. If parking faults again: support the arm, `off` + Enter. |
| `uncontrollable fault` | Bus error, stale feedback or a motor fault code. | It holds 0.5 s, then switches off: **a raised arm drops.** |
| the trace is a few cm off the crack | Calibration error (GEOM-05/07), expected. | Measure the offset, write it in the archive README. Do not lower the hover. |
| the tool is about 14 cm above the print | The guard's floor plus the planner's margin. | Expected. |
| J2/J3 wobble | The 3.3 Hz ring seen on 2026-10-22, reduced by kd 4. | Note when and how much in the archive README. |

## 5. After the run

```bash
bash scripts/crack_pipeline.sh archive print01
```

This **copies** (never moves) into `evidence/operator/first-crack-run-<date>/`:
- the case: search result and per-stop overlays and tables, capture, component table, paths, path3d, trace,
  preview, logs;
- the session record, the preflight record and the day's bench runs;
- the runner's run CSV and JSON for every arm step;
- the guard events from the session window, and the guard approval record.

`MANIFEST.json` and `SHA256SUMS` list every file with its sha256. A second archive of the same day gets `-2`; nothing
is overwritten. Then fill in the README's "What the operator saw" section:
- the trace's offset from the crack;
- the hover height;
- wobble: which joint, when;
- anything refused or stopped;
- whether the camera mount side matched.

File it with `claude-auto evidence add <card> <file>` if wanted. TODO.md item 0b.2 is closed by this run.

## Known limits

- **Nominal geometry.** The camera mount and the tool tip are CAD priors, so expect cm-level offsets (above).
  path3d marks every real case `execution_eligible: false`, and the trace carries that flag.
- **The crack gate has never seen a real print through the real camera at the scan distance.** The thresholds come
  from renders and a desk recording ([CRACK_SELECTION_RESEARCH.md](../perception/CRACK_SELECTION_RESEARCH.md)).
  Bench is the check.
- **The guard's limits are the vendor URDF's**, not measured (TODO.md item 2). There is **no CAN watchdog**
  (TODO.md item 1).
- **The rehearsal's plant is crude.** It proves the logic and the limits, not the dynamics.

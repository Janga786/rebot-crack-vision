# CRACK_TRACE_PIPELINE.md — show trajectories, and the camera-to-trace pipeline (2026-10-23)

Two things are built on the guarded runner that already moves the arm (`~/rebot_setup/run-trajectory.sh` →
`demo_move.py`, arm_guard in front of every motor command, [FIRST_GUARDED_RUN.md](FIRST_GUARDED_RUN.md)):

1. **Show trajectories**: the gripper tip draws figures in the air (`tools/make_fancy_trajectory.py`).
2. **The crack pipeline**: the wrist D405 sees a crack, then the arm traces it (`scripts/crack_pipeline.sh`).

Both produce ordinary `crackvision.joint_trajectory/1` files, so the guard's pre-flight, the hold at the fold, Enter to
go and stop/off all work as before. **Nothing here has run on the powered arm yet.** Every arm step is the operator's.

## The shared piece: `crackvision.arm_ik`

- Inverse kinematics for the tool tip on the repo URDF (the file arm_guard checks against), solved by damped least
  squares, with an optional pointing direction for the tool's +X axis. The roll about that axis is left free.
- Paths are followed point by point, each solve starting from the previous one. A joint jump between neighbouring
  points is refused, not smoothed over.
- The tool moves along the path at a set speed with cosine ramps, so it starts and stops smoothly. Pieces marked
  `stop` (a star's corners) come to rest.
- Each trajectory is then slowed uniformly until every joint stays under a fraction of the runner's planned-speed caps
  (0.6 rad/s for J1–J3, 1.2 rad/s for J4–J6): 70 % for the shapes, 60 % for crack traces.
- gripper_link is kept at least 1 cm above arm_guard's 0.12 m floor.

**Workspace fact found on the way:** the tool can point **straight down** only with its tip between z ≈ 0.13 and
0.18 m. Above that, J4 reaches its 1.57 rad limit. Pointing forward, the tip reaches roughly x 0.25–0.45 m and
z 0.3–0.6 m.

## 1. Show trajectories

```bash
./env.sh python tools/make_fancy_trajectory.py --all --out-dir ~/rebot_setup/trajectories/fancy
./env.sh python tools/make_fancy_trajectory.py --shape heart --out OUT.json --x 0.32 --zc 0.45 --size 0.16
```

| Shape | What | Time at x1 |
|---|---|---|
| figure8, heart, rose, lissajous | closed curves drawn on a "wall" 32 cm in front, tool pointing forward | 10–23 s |
| star | 5-point star; stops at every point | 21 s |
| spiral | 3 turns out, then back in | 31 s |
| wave | a sine wave whose amplitude swells and fades, there and back | 19 s |
| helix | a corkscrew moving 8 cm toward the wall (3D) | 15 s |
| showreel | figure8, star and heart in one run | 41 s |
| raster | tool pointing down: a 5-pass lawnmower scan of a 16 × 10 cm patch at z 0.15 m | 30 s |
| crack | tool pointing down: a jagged crack-like line, the motion the pipeline produces | 6 s |

**Gate:** each shape was rehearsed with `run-trajectory.sh --rehearse` (fakebus plus the real guard config) at x1
and x2. That is 22 runs: all completed with no guard fault and a keep-out margin of at least 30 mm (41 mm for the
wall shapes). The IK tip error is under 0.1 mm. Peak joint speed is at most 70 % of the caps; only the helix needed
slowing, by 1.08×. Stills of the MuJoCo replays are in [figures/fancy/](figures/fancy/). The videos are in
`~/rebot_setup/renders/`; to make your own:
`MUJOCO_GL=egl ~/miniconda3/envs/navila/bin/python tools/render_arm_run.py RUN.csv OUT.mp4 --trail`.

**Running one on the arm (operator):**
```bash
cd ~/rebot_setup
DEMO_EXTRA="--gains j2=100/4,j3=80/4" ./run-trajectory.sh trajectories/fancy/figure8.json 2   # half speed first
```
The kd 4 override is recommended: these shapes change direction continuously, and that is what excites the 3.3 Hz
J2/J3 wobble seen on 2026-10-22. Whether kd 4 becomes the default is still the operator's decision.

## 2. Camera → crack → trace

```
scan-plan   IK: camera 25 cm above the target (default table x=0.30, y=0), looking straight down
            -> data/crack_trace/scan.json (holds that pose 15 s), rehearsed
scan CASE   ARM + CAMERA. run-trajectory.sh drives the arm to the scan pose, with demo_move.py --state-file
            publishing the measured pose. Meanwhile crack_trace capture waits until the arm has rested at the scan
            pose, grabs a 10-frame burst, keeps the sharpest frame and writes the case: RGB, aligned depth, and the
            §9 capture record with the MEASURED joint angles. Then segment + paths (the accepted stages). The arm
            returns to the fold.
plan CASE   path3d (pixels -> base_link surface points) -> trace: the tool tip follows each crack (longest first,
            up to 3) at a hover height, pointing into the surface; approach and retract 4 cm above.
            Writes data/crack_trace/CASE_trace.json, a preview, and a plan report. Rehearsed at x1 and x2.
trace CASE  ARM. run-trajectory.sh plays the trace.
```

```bash
bash scripts/crack_pipeline.sh scan-plan                 # once per target position
bash scripts/crack_pipeline.sh scan  crack01             # arm + camera (Enter at the fold, as usual)
bash scripts/crack_pipeline.sh plan  crack01             # then LOOK at data/crack_trace/crack01_trace_preview.png
bash scripts/crack_pipeline.sh trace crack01 2           # arm, half speed
bash scripts/crack_pipeline.sh demo                      # all of it offline, no hardware (about 1 minute)
```

**The preview is the human check.** It shows the camera image with the model's crack mask in red and the planned
path projected back into the image in green; a circle marks the start. If green does not sit on red, do not run it.

### How high the tool hovers, and why

- The hover is at least `--hover-m` (default 5 cm).
- It is at least 3σ of path3d's uncertainty along the surface normal. With today's **nominal** camera mount
  (CAD prior, σ 1 cm / 5°) that is about 7.5 cm at a 25 cm view.
- It is raised until the tool tip (= gripper_link while the tool is nominal) stays 1.5 cm above arm_guard's
  0.12 m floor. **For a crack on the table this is the binding rule: the tool traces about 14 cm above it.**
- Where the wrist cannot point straight down at that height, the whole segment is tilted 10°, 20° or 30° (away
  from the base). The tilt is chosen per segment, so the motion still mirrors the crack. The demo needed 10°.

The camera mount and the tool tip stay **nominal** until GEOM-05 (hand-eye) and GEOM-07 (tool pivot) calibrate them.
path3d therefore marks every real case `execution_eligible: false`, and the trace carries that flag and its reasons
in `source`. Expect the traced path to be offset from the crack by up to a few centimetres. The hover is sized for
this.

### Verification (offline, 2026-10-23)

- **Geometry chain against ground truth** (`tools/synth_scene3d.py` case, `crack_trace check`): the planned surface
  path is within **0.51 mm** of the true crack (mean 0.25 mm).
- **Real model, rendered scene, exact pose** (`make-replay` of crack test print #1 on the table, capture against a
  stationary state stream): the model found the crack. The planned surface path lies on the print's plane to
  **0.04 mm**, and the preview overlays the mask.
- **`demo`, end to end** ([evidence](../../evidence/crack_trace/offline_demo_2026-10-23/)):
  - the scan pose is rehearsed with the real guard config while demo_move.py publishes its simulated measured pose;
  - capture settles on that stream, and the model finds one 208 mm crack;
  - plan: hover 14.3 cm, tilt 10°, 14.6 s;
  - rehearsal at x1 and x2 completes with no guard fault and a keep-out margin of at least 18 mm.

  The surface comes out 5 mm low here, and that is expected. The image was rendered at the commanded scan pose, but
  the simulated arm sags 0.03 rad under gravity, and the capture correctly records the measured pose. On the real
  arm, image and pose come from the same moment.
- **The real D405 recording of 2026-10-22** (a desk, no crack): capture works on it and correctly reports *no crack
  found: nothing to trace* (exit 3).
- Tests: `tests/test_arm_ik.py` and `tests/test_crack_trace.py` (34). Runner: `test_guarded_runs.StateFile` (2), and
  all 43 runner tests pass with the change.

### Runner change (`~/rebot_setup/demo_move.py` 2026-10-23.1)

`--state-file FILE`: about 20 times a second the runner writes the measured pose as `rebot.arm_state/1` JSON, with an
atomic replace. The file holds physical joint angles (the J1 reference is removed), the segment kind
(approach/trajectory/return), the velocities and the wall time. The option only adds output: a write error stops the
publishing, never the run. A stale file is deleted before the run starts. Snapshot:
[evidence/operator/rebot_setup_runner_2026-10-23](../../evidence/operator/rebot_setup_runner_2026-10-23/README.md).

### Before the first real crack run

1. The D405 must be on the wrist mount as end_effector.yaml assumes: the gripper face that points up at the fold,
   with the stock 15° Seeed mount. Check it visually.
2. Lay the print flat at about x 0.30 m, y 0 (in front of the base), lit evenly, matte paper.
3. Run `scan-plan` and rehearse it, then run `scan` at x2 and look at the overlay. Run `plan`, look at the preview,
   then `trace` at x2 with the kd 4 override.
4. If the trace is offset from the crack, that offset is the calibration error, which GEOM-05/07 fix. Do not lower
   the hover to compensate.

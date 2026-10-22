# First guarded trajectory on the physical arm — 2026-10-22

The operator asked for a trajectory on the physical arm (not crack-following), shown in simulation first, run on
their "go". It ran through `~/rebot_setup/demo_move.py` 2026-10-21.1 (the 2026-10-17 controller) with arm_guard
in front of every motor command.

| | |
|---|---|
| Trajectory | `smoke_raise.json` (the repo's MOT-05.2 fixture: fold → [0.1, −0.6, −0.9, 0.1, 0.1, 0.1] rad), `--slow 2`: 6.0 s up, 6.2 s back to the fold, peak planned speed 0.27 rad/s |
| Simulation first | Guarded rehearsal (fakebus + the real guard config) completed; rendered with `tools/render_arm_run.py` (MuJoCo, repo URDF) and shown to the operator |
| Operator gos | "go" (the read-only check, then enabling and holding at zero); "yes" (J1 faces forward); "go" (the trajectory) |

## Sequence

1. **Read-only check 1** (`…190445Z.json`): refused, nothing enabled. J1 read +6.2766 rad, one whole turn
   (2π) plus −0.007 off its saved reference, after a 24 V cycle. The other six joints read within 0.05 rad of the
   fold.
2. **The operator confirmed J1 faces forward.** Read-only check 2 with `--j1-here-is-zero` (`…190548Z.json`)
   passed. The new reference (+6.2766) is in `~/rebot_setup/j1_reference.json`. The guard's startup check
   unwrapped J1 by one turn on its own.
3. **Attempt 1** (`…190559Z.*`): aborted safely at the fold. J1 and J2 had been enabled when arm_guard refused
   J3: its CTRL_MODE register reply did not arrive within 200 ms. The runner held 0.5 s, then disabled J1 and J2
   (confirmed). Nothing moved.
   - Fix, commit 5d8f4cc: enable-time register reads are retried up to 3 times (the USB2CAN adapter drops an
     occasional reply; this was also seen 2026-10-17).
4. **Read-only check 3** (`…190917Z.json`) passed. **Attempt 2** (`…190924Z.*`): **completed.**
   - All 7 motors enabled with no register retries needed, zeroed and held.
   - On the second go: up, back to the fold, held, all 7 motors disabled (confirmed).

## Attempt 2 results (`traj_smoke_raise_20261022T190924Z.csv`, 200 Hz)

| Joint | Reached (target) | Max \|ref − pos\| | Max \|vel\| | Max \|torque\| (guard abort) |
|---|---|---|---|---|
| J1 | +0.087 (+0.1) | 0.008 | 0.13 | 1.40 (22) |
| J2 | −0.608 (−0.6) | 0.017 | 0.40 | 2.76 (22) |
| J3 | −0.889 (−0.9) | 0.023 | 0.42 | 10.73 (22) |
| J4 | +0.109 (+0.1) | 0.020 | 0.30 | 2.32 (6.5) |
| J5 | +0.098 (+0.1) | 0.008 | 0.11 | 0.46 (6.5) |
| J6 | +0.098 (+0.1) | 0.004 | 0.21 | 0.19 (6.5) |

- J1 is shown in the URDF frame: the raw reading minus one whole turn.
- arm_guard events during the run: bus opened, startup check passed (whole-turn offset J1 = 1). There were no
  faults, no adjustments and no latch.
- `measured_pose_at_top.jpg` is the measured pose at maximum elbow bend, rendered on the URDF.

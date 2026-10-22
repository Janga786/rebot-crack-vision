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

## The wobble (operator: "it wasn't a smooth trajectory, it kind of wobbled")

The log confirms it: **J2 and J3 oscillated at about 3.3 Hz all the way through the move**, up and down. The
measured velocity rippled around the commanded one by 0.065 / 0.070 rad/s RMS against a peak commanded 0.18 /
0.27 rad/s, and the motor torque oscillated with it (`wobble_j2_j3_velocity_torque.png`). The wrist (J4–J6) shows
a smaller ripple at about 7 Hz.

- **Cause (most likely):** an underdamped servo plus stick-slip friction.
  - The 2026-10-17 gains give J2 kp 100 / kd 1 and J3 kp 80 / kd 1. With the arm's inertia about those axes
    (~0.25 kg·m²), that is a natural frequency of √(kp/I) ≈ 20 rad/s ≈ 3.2 Hz, matching the log, at a damping
    ratio near 0.1.
  - The gearbox's stick-slip friction keeps re-exciting it, and slow motion makes stick-slip worse. The same
    3.3 Hz ripple is in the show-demo runs at 2× (2026-10-17 / 10-21), but at 0.026–0.037 rad/s, well under half.
- **Why the tuning missed it:** the kd sweeps (CONTROLLER_TUNING §8) were single-joint 0.1 rad moves near the fold,
  and oscillation was scored only during holds.
- **The simulator cannot show it.** fakebus gives a 10× smaller ripple at the same gains (its friction model is too
  idealised), so the fix has to be tested on the arm.
- **Proposed fix, to A/B on the arm:** J2/J3 kd 1 → 4. J1 already runs kd 4; the vendor default is 10; arm_guard
  allows ≤ 5. Then run at the file's own speed rather than slowed down. `demo_move.py --gains j2=100/4,j3=80/4`
  (added 2026-10-22, this run only, recorded in the summary) makes the A/B possible without touching the tuned
  defaults.

## A/B: J2/J3 kd 1 → 4 (attempt 3, `…192035Z.*`)

The same trajectory, speed and everything else, with `--gains j2=100/4,j3=80/4`, on the operator's gos. It
completed: all 7 motors disabled (confirmed); arm_guard recorded no fault, adjustment or latch.

| | kd 1 (attempt 2) | kd 4 (attempt 3) | Change |
|---|---|---|---|
| J2 velocity ripple (RMS) | 0.062 rad/s | **0.024** rad/s | −61 % |
| J3 velocity ripple (RMS) | 0.067 rad/s | **0.035** rad/s | −48 % |
| J2 / J3 ripple energy, 2–5 Hz | 0.0150 / 0.0098 | 0.0041 / 0.0047 | −73 % / −52 % |
| J2 / J3 max tracking error | 0.017 / 0.023 rad | 0.010 / 0.021 rad | |
| J2 / J3 torque ripple (std) | 0.30 / 0.42 Nm | 0.13 / 0.33 Nm | |

`kd1_vs_kd4_velocity.png` plots both against the commanded velocity. More damping removes most of the 3.3 Hz
ringing.

What remains:
- J3 keeps some jitter: higher-frequency, noise-like, still friction.
- The commanded J3 velocity has a small dip at the middle waypoint. `smoke_raise.json` has only 3 points, and the
  monotone cubic through them is continuous in velocity, not in acceleration.

Next candidates: make kd 4 the J2/J3 default (an operator decision; CONTROLLER_TUNING and `tuned_gains.yaml`), try
the file's own speed, and possibly a smoother interpolation for sparse waypoints.

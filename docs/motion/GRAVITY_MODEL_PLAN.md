# Plan: a pose-dependent gravity correction for the B601-DM (2026-10-29)

Goal: remove the steady, pose-dependent sag so the arm tracks trajectories more precisely, without touching the gains
and without changing anything on the arm until the operator runs it.

## What the sag is, and what we know (from CONTROLLER_TUNING.md §5-7, the 2026-10-23/25 show-run analysis, the logs)

- Feed-forward today: URDF gravity x per-joint scale (J2 0.6, J3 2.5, J4 2.5; identified on 0.1 rad holds near the
  fold) + Coulomb friction on the commanded velocity (+ a per-run constant `--ff-bias`). The scale stands in for mass
  the URDF does not have: the wrist motors (link4/5/6 are 19-190 g in the URDF, a DM4310 is roughly 0.3 kg) and the
  D405 + mount. CONTROLLER_TUNING §6.4 already shows that adding ~0.3 kg at link5, link6 and the gripper plus the
  camera reproduces the identified J3 and J4 loads at the fold (-3.86 vs -4.01, -1.48 vs -1.01 Nm).
- The sag in the show runs is a feed-forward error: the P term supplied a steady J2 +1.0 Nm (+10.6 mrad), J3 -0.85 Nm
  (-10 mrad), J4 -0.24 Nm (-5.4 mrad). A constant bias halved the means but not the rms, because the error depends on
  pose and the approach/return from the fold dominate.
- Tracking error at the tuned gains: e = (G_true - G_ff) / kp, so every 1 Nm of model error is 10 mrad on J2 (kp 100),
  12.5 mrad on J3 (kp 80), 25 mrad on J4 (kp 40). A model within 0.3 Nm everywhere would put the sag under 3-8 mrad.
- Friction: J3 +-2.3 Nm static band, so a single stationary sample only brackets gravity. The direction of the
  preceding move splits bias from friction (analyze_run_gains.py --directions): the same offset in both directions is
  gravity, opposite signs are friction.
- `~/rebot_setup/gravity.py` is the URDF model (0.32 ms per call) and ALREADY accepts extra point masses
  (`GravityModel(extra=[(link, (x,y,z), kg), ...])`). `demo_move.py` evaluates it every cycle on the measured q.
- The fake bus used by rehearsals gives each joint a CONSTANT load from the rest pose (`bus.sims[n].load`), so a
  rehearsal cannot validate a gravity model; it only proves the command stream stays inside the guard.

## The data, and the gap that drives the design

Logged runs with all joints (200 Hz, `~/rebot_setup/runs/*.csv[.gz]`; columns ref/pos/vel/torq/ff per joint; mode
settle/path/...; `torq` is the motor's reported torque):

| set | runs | stationary samples | J2 range | J3 range | J4 range |
|---|---|---|---|---|---|
| show runs 2026-10-23/25 (figure8, heart x3, showreel, box) | 7 | ~44 000 | -1.00 .. +0.07 | -1.46 .. 0 | -0.01 .. +0.51 |
| demo_show / demo_wave 2026-10-17..21 | 11 | ~80 000 | -0.80 .. 0 | -1.60 .. 0 | 0 .. +0.61 |
| smoke_raise 2026-10-22 | 2 | ~24 000 | -0.61 .. 0 | -0.89 .. 0 | 0 .. +0.11 |
| first_move_* hold-others sweeps 2026-10-17 | 12 | ~25 000 | near the fold only | | |

The crack pipeline's trajectories (search.json, scan.json, the demo trace): **J2 -1.13 .. -1.89, J3 -1.04 .. -1.86,
J4 +1.12 .. +1.36.** No logged sample has J2 below -1.0 or J4 above +0.61. A per-joint lookup or polynomial fitted on
the logs would be pure extrapolation where the crack work happens. Therefore:

1. the model must be **physical** (rigid-body gravity is linear in each body's first moment m*c and mass m, computed
   through the URDF kinematics), so it extrapolates by geometry rather than by curve shape; and
2. the crack workspace needs its **own data**: a survey trajectory the operator runs once, holding ~20 poses that span
   J2/J3/J4 of the crack work, each reached from both directions.

## What gets built (workstation; the arm runs are the operator's)

1. **`tools/fit_gravity_model.py`** (repo, offline). Loads any set of run CSVs; selects stationary samples (|v| small
   for J2-J4 over a window, mode path/hold, not the first second after enable); tags each with the direction of the
   preceding move; builds the regressor from the URDF kinematics for a small set of unknowns: an extra point mass at
   link3, link4, link5, link6 and the gripper (m and m*c, or a point mass at a fixed CAD position) plus the D405+mount
   from end_effector.yaml; solves a regularised linear least squares toward the URDF (ridge) with the friction band
   handled by fitting on up/down pairs or by a Huber loss; reports per joint the residual rms/max in Nm and in mrad at
   the tuned kp, on the training runs AND leave-one-run-out, against three baselines: URDF x1, today's scales, and
   today's scales + the 2026-10-25 bias. Writes `config/robot/gravity_model.yaml` (schema, fitted masses, the URDF
   sha256 it belongs to, the runs it was fitted on, the residuals, `valid_range` per joint = the data's convex hull,
   and `extrapolated: true` for the crack workspace until the survey exists). Plots: residual vs q per joint, before
   and after. Must also report the predicted feed-forward along search.json, scan.json and the latest demo trace
   against the caps (10 Nm for J1-J3, 2.5 Nm for J4-J6) and say whether any pose would be refused.
2. **`~/rebot_setup/demo_move.py --gravity-model FILE`** (opt-in, per run, like --ff-bias; default behaviour
   byte-identical without it). Loads the yaml, checks the URDF sha256, builds `GravityModel(extra=...)`, sets the
   gravity scales to 1.0 for the joints the model covers, keeps the 1 s fade-in for the part above the plain URDF
   value (the forearm rests on the J2 motor at the fold; §7 explains the kick), keeps every cap and the guard
   pre-flight. Logs the model file and its sha256 in the run summary. Snapshot the runner into
   `evidence/operator/rebot_setup_runner_<date>/` as before and keep `test_demo_move` / `test_guarded_runs` green
   (add tests for the loader, the scale override and the sha256 check). Also `--gravity-model` must be accepted by
   `run-trajectory.sh --rehearse` via DEMO_EXTRA (it already passes DEMO_EXTRA through).
3. **`tools/make_gravity_survey.py`**: the survey trajectory for the crack workspace (joint_trajectory/1): about 20
   poses on a grid over J2 -0.6..-1.9, J3 -0.6..-1.9, J4 0..1.4 (J1 0, J5/J6 0 unless the wrist pointing needs them),
   each visited twice (arriving from above and from below, in joint space), held 3 s, min-jerk between, speeds under
   60 % of the caps; every pose checked with arm_ik.check_motion_clearance (camera, mount, gripper >= 3 cm from the
   table, gripper_link >= 13.5 cm) and the trajectory rehearsed x1 and x2 with the real guard config. Also a short
   "A/B" trajectory: the heart (already run 7 times) is the regression reference, and the crack demo trace is the
   target.
4. **`tools/analyze_run_gains.py`**: add `--gravity-model FILE` so a past run can be re-scored against the new model
   (what the P term would have had to supply), and a `--compare` of two runs (A/B).
5. **Docs**: `docs/motion/GRAVITY_MODEL.md` (the method, the fit, the numbers, the limits: extrapolation flag, the
   fake-bus caveat, the caps), a section in CONTROLLER_TUNING.md §12 pointing at it, and the operator steps in order:
   (a) rehearse and run the survey (x2 then x1), (b) refit with the survey, (c) A/B the heart with and without
   `--gravity-model`, (d) the crack demo trace with it. Every arm step on the operator's go.

## Acceptance (offline, before any arm step)

- Leave-one-run-out residual on the show runs: J2/J3/J4 mean |error| under 0.3 Nm and rms better than today's scales
  + bias on every held-out run; the predicted mean sag on the heart under 3 mrad per joint.
- Predicted feed-forward along the crack trajectories inside the caps with margin, or the report says where not.
- Default runner behaviour unchanged (tests + a rehearsal of the heart without the flag give the same command stream).
- `run-trajectory.sh --rehearse` of the survey and of the heart with `DEMO_EXTRA="--gravity-model ..."` completes.

## Operator decisions the docs must surface

- Running the survey on the arm (the only way to make the crack-workspace numbers measured rather than extrapolated).
- Whether the fitted model becomes the default after the A/B (today: per-run flag only).
- Weighing the camera + mount and a spare DM4310 would pin two of the unknowns; optional.

# J4 (wrist pitch, DM4310) — refined tuning: research and plan (2026-10-29)

Scope: what the logs say about J4's tracking error, why, and what to build (workstation) and run (operator) to
reduce it. Companion to [CONTROLLER_TUNING.md](CONTROLLER_TUNING.md) §6.2/§8, the show-run analysis
[powered-show-runs-2026-10-23_25](../../evidence/operator/powered-show-runs-2026-10-23_25/README.md) and
[GRAVITY_MODEL.md](GRAVITY_MODEL.md) §5.2/§5.4. Nothing here runs the arm.

## 1. Where J4 stands

| item | value | source |
|---|---|---|
| gains | kp 40 / kd 1 (sweep 2026-10-17: 20/0.5, 30/0.5, 40/1; kp 40 kd 1 best, score 0.0157) | `runs/first_move_j4_20261017T191836Z.json` |
| feed-forward | URDF gravity x 2.5 + **Coulomb 0.5 Nm** x tanh(v_ref / 0.02) | `demo_move.py` FRICTION = TUNED[j4][3] |
| caps | tau_ff 2.5 Nm (runner + guard), guard kp_max 50 / kd_max 5, torque_abort 6.5 Nm, vel_abort 1.6 rad/s (runner), 2.5 (guard) | `arm_guard.toml` [joints.j4], `demo_move.py` VEL_ABORT |
| motor | DM4310 10:1, rated 3 Nm, peak 7 Nm; TMAX mapping 10 Nm | CONTROLLER_TUNING §1 |
| show runs | mean error -2.4..-7.7 mrad, rms 7.6-8.1, max 16.6-22 mrad; 3 Hz ripple 1.1-1.6 mrad (largest of J2-J4) | `analysis.txt` |
| ff headroom | peak ff -2.19..-2.38 Nm near the fold (J4 level), 0 % of samples at the 2.5 cap; in the crack workspace (J4 +1.1..+1.4) URDF gravity is only -0.20..-0.26 Nm (x2.5 = -0.5..-0.65) | computed from heart/search/scan/trace files |

## 2. What the logs say (new analysis, six runs, 200 Hz CSVs)

Regression of the J4 error e = ref - pos (+ = lagging) and of the torque the P/D terms supplied (tq - ff) on
[1, tanh(v_ref/0.02), v_ref, a_ref] over the path segment (seg 1):

| run | sgn term (mrad) | = Nm at kp 40 | viscous (Nm.s/rad) | inertia (Nm.s2/rad) | R2 (e) | corr e4-e3 |
|---|---|---|---|---|---|---|
| heart 10-23 205033 (kd 4 on J2/J3) | -8.1 | -0.32 | -0.16 | -0.002 | 0.93 | -0.40 |
| heart 10-23 205318 (kd 5) | -7.9 | -0.32 | -0.13 | -0.001 | 0.93 | -0.54 |
| heart 10-25 214843 (ff-bias j4 -0.2) | -8.5 | -0.35 | +0.46 | -0.001 | 0.91 | -0.55 |
| figure-8 x1 | -7.6 | -0.32 | -0.27 | -0.011 | 0.91 | -0.35 |
| showreel | -8.4 | -0.34 | -0.15 | 0.000 | 0.90 | -0.35 |
| box (|v| <= 0.07) | -4.7 | -0.19 | -2.2 (ill-conditioned) | -0.001 | 0.78 | +0.36 |

Reading:
1. **About 90 % of J4's in-path error is a sign-of-velocity term of -8 mrad (-0.32 Nm).** The sign is negative: when
   J4 is commanded positive the wrist runs *ahead* of the reference. That is the 0.5 Nm Coulomb feed-forward pushing
   harder than the joint's real moving friction. Implied real Coulomb = 0.5 - 0.32 = **~0.18 Nm**, the same number the
   gravity fit found independently (0.15 Nm, GRAVITY_MODEL §5.2). The hold hysteresis agrees: stationary after moving
   up tq-ff -0.22 Nm, down +0.10 Nm → half band ~0.16 Nm.
2. **No inertia term** (|a_ref| <= 4 rad/s2 gives < 0.01 Nm) and **no usable linear-velocity term** at these speeds
   (|v_ref| <= 0.18 rad/s). Nothing to feed forward beyond friction and gravity.
3. **The approach from the fold is the worst segment** (mean -11 mrad, rms 13-14 mrad; sgn term -0.45..-0.68 Nm there):
   J4 moves slowly (0.03-0.08 rad/s) near the fold, where it also carries its largest gravity load. The return is
   fine (+1.7 mrad). The gravity model covers the pose part; the friction part is the same over-compensation.
4. **The 3 Hz ripple is partly shared with J3**: coherence 0.5-0.56 at 3.3 Hz in the hearts and the box (0.12 in the
   showreel). kd on J4 can only damp J4's own part. Band rms is 1.1-1.6 mrad; at 0.5 m that is < 1 mm.
5. **Velocity readings are clean on J4 in these runs**: 0 jumps > 0.3 rad/s in 19 559 samples, reading-vs-finite-
   difference rms 0.02 rad/s. kd 4 on that noise is 0.08 Nm. The DM4310 spike history is the gripper at enable.
6. **The constant part** (-0.15..-0.33 Nm, -3..-8 mrad) is the pose-dependent gravity bias already addressed by
   `--gravity-model` (held-out J4 |mean| 0.29 → 0.05 Nm).

So, in order of expected gain: (a) Coulomb feed-forward 0.5 → ~0.15-0.2 Nm (removes ~8 mrad of signed error, the
dominant term: path rms 7.4 → ~2.5-3 mrad predicted); (b) the gravity model (removes the -3..-8 mrad offset);
(c) kd 2-4 for J4's own share of the 3 Hz ripple and the step response; (d) kp 50 (guard cap) is a small, last step
(e = dtau/kp: 40 → 50 only shrinks residual errors by 20 %).

## 3. Constraints

- `first_move_j1.py` (the sweep tool) is **not** behind arm_guard. New J4 experiments go through
  `demo_move.py` / `run-trajectory.sh` (guarded), with per-run overrides, like every run since 2026-10-21.
- `demo_move.py` has `--gains jN=KP/KD` and `--ff-bias`, but **no per-run friction override**: FRICTION comes from
  `first_move_j1.TUNED`. `tuned_gains.yaml` / TUNED stay unchanged until an A/B on the arm says otherwise.
- `run-trajectory.sh --rehearse` does not pass `DEMO_EXTRA` (GRAVITY_MODEL §9, operator decision pending), so
  rehearsals with overrides are done by test code calling the runner directly.
- fakebus J4 is the generic plant (inertia 0.07, viscous 0.5, Coulomb 0.3): rehearsal proves guard safety of the
  command stream, not the tuning.
- Guard caps for J4: kp <= 50, kd <= 5, |tau_ff| <= 2.5. Torque abort 6.5 Nm (guard). With kd 4 the runner's own
  torque abort bound grows by kd x vel_abort = 4 x 1.6 = 6.4 Nm; the guard's 6.5 Nm stays the binding limit.

## 4. Build (workstation, Opus agent)

1. **Analysis tool**: `tools/analyze_run_gains.py --friction-fit` (or a new `tools/analyze_j4.py`): per joint and
   per segment the regression above (constant, sgn, viscous, inertia; implied Coulomb = runner ff + sgn coefficient),
   its confidence across runs, the J4-J3 coherence at the ripple line, and "what the error would have been" with a
   given Coulomb value (the sgn term rescaled). Output for all seven show runs into
   `evidence/operator/j4-tuning-2026-10-29/analysis.txt`; tests.
2. **Runner**: `demo_move.py --friction jN=X[,jM=Y]` per run (0 <= X <= FRICTION_MAX_NM 3.0; J1-J6; logged in the
   summary and the banner; restored after the run; default command stream byte-identical, proven like the gravity
   flag). Snapshot before/after into `evidence/operator/rebot_setup_runner_2026-10-29/` (append a section + sha256s)
   and `~/rebot_setup/backup_pre_j4_friction_2026-10-29/`. Tests in `~/rebot_setup/test_demo_move.py`.
3. **J4 characterisation trajectory**: `tools/make_j4_tuning_trajectory.py` → `~/rebot_setup/trajectories/j4_tune.json`.
   Wrist-only moves (J1/J5/J6 at 0, J2/J3 fixed) at two poses: the heart's mid pose (J4 ~0.2, logged workspace) and a
   crack-workspace pose from the survey's accepted set (J4 ~1.2), if clearances allow. At each pose: triangle sweeps of
   +-0.15 rad at 0.05, 0.1 and 0.2 rad/s (friction vs speed, both directions), then 0.05 rad steps (min-jerk, 0.5 s)
   with 2 s holds (static band, settling, overshoot, ripple). <= 4 min at x1, min-jerk, <= 60 % of the J4 speed cap,
   every sample through the arm_guard windows + keep-outs + table clearance like the survey builder. Rehearsed x1 and x2.
4. **Predictions written down before the runs** (falsifiable): with `--friction j4=0.18` the sgn term → ~0 (+-2 mrad),
   path rms 7.4 → ~3 mrad on the heart; with kd 3 the J4 band rms 1.2 → <= 0.8 mrad, no new oscillation flips;
   with kp 50 residual rms x 0.8. Put them in the doc with the test that will check them (`--compare`).
5. **Docs**: `docs/motion/J4_TUNING.md` (findings, the regression method, the runner flag, the A/B matrix below,
   acceptance criteria, decisions), pointers in CONTROLLER_TUNING §12 and TODO.md; memory of the tool's limits.

## 5. Operator A/B (never auto-run; each run on its own go, x2 first)

| run | j4_tune.json with | purpose |
|---|---|---|
| A | defaults (kp 40 / kd 1 / fric 0.5) | baseline on the new trajectory |
| B | `--friction j4=0.18` | the dominant fix; expect the sgn term to vanish |
| C | B + `--gains j4=40/3` | J4's own ripple / step damping |
| D | B + `--gains j4=50/3` | kp at the guard cap |
| E | heart with the best of B-D (+ `--gravity-model` once the survey is refitted) | confirm on the real shape; `--compare` with heart_20261025T214843Z |

Adopt into `tuned_gains.yaml` / TUNED only after E, as a separate, reviewed change.

## 6. Acceptance (offline)

- Tests green (repo + runner); default runner stream byte-identical without the new flag.
- Analysis tool reproduces the table in §2 within rounding.
- `j4_tune.json` rehearses at x1 and x2 with no guard fault; every pose clearance-checked.
- Predictions in §4.4 recorded before any arm run.

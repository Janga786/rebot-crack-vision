# Operator runner snapshot — `~/rebot_setup`, 2026-10-29

Copies of the operator runner around the change that added `demo_move.py --gravity-model FILE`
([docs/motion/GRAVITY_MODEL.md](../../../docs/motion/GRAVITY_MODEL.md) §7, plan
[GRAVITY_MODEL_PLAN.md](../../../docs/motion/GRAVITY_MODEL_PLAN.md) item 2). Previous snapshot:
[2026-10-25](../rebot_setup_runner_2026-10-25/README.md). Workstation only: the arm was not moved, the CAN bus was not
opened, the USB2CAN stayed unplugged; the only runner invocations were `run-trajectory.sh --rehearse` (simulation) and
the unit tests.

- `before/` — the six files as they were before any edit (copied first). Checksums: `SHA256SUMS.before`. The same
  pre-change `demo_move.py` / `test_demo_move.py` are also kept in `~/rebot_setup/backup_pre_gravity_model_2026-10-29/`
  (the byte-identity test loads the copy there).
- top level — the files that changed (`demo_move.py`, `test_demo_move.py`), **now at the state after the second change of
  the day (`--friction`, section below)**. Checksums of the files as they are now: `SHA256SUMS` (updated with that
  change); the state after the gravity-model change: `SHA256SUMS.before_j4_friction` and `before_j4_friction/`.

| File | sha256 before | sha256 after | What changed |
|---|---|---|---|
| `demo_move.py` | `eb02a568…247f83` | `e80f2026…a7e29a` | 2026-10-25.1 → **2026-10-29.1**: new opt-in `--gravity-model FILE` (below) |
| `test_demo_move.py` | `6265e73d…98960c` | `9a5b35e0…acfc665` | new `GravityModelFlag` class, 6 tests (below) |
| `run-trajectory.sh` | `75866449…39f76d` | unchanged | – (not to be edited) |
| `gravity.py` | `8400f5b4…c6801f` | unchanged | – |
| `fakebus.py` | `43e54371…773e42` | unchanged | – |
| `test_guarded_runs.py` | `c0ce9566…67f5b2` | unchanged | – |

(Full digests in `SHA256SUMS.before` / `SHA256SUMS`.)

## `demo_move.py` 2026-10-29.1 — `--gravity-model FILE`

- **Off by default; per run**, like `--ff-bias`. Without the flag nothing in the command stream changes (proof below).
- Loads a `crackvision.gravity_model/1` file (the repo's `config/robot/gravity_model.yaml`, written by
  `tools/fit_gravity_model.py`). The file is YAML in JSON style because the `rebot` env has no PyYAML: `#` comment lines,
  then a JSON object.
- **Refuses** (usage error, before the bus is opened) a file whose `urdf.sha256` is not the sha256 of
  `model/reBot_B601_DM_with_gripper.urdf` here (`f808f6f0…5fa6`), a wrong schema, a body that is not on a moving link,
  has |xyz| > 1 m or kg outside [−0.5, 2], more than 5 kg in all, duplicate bodies, or scales outside J2-J6 / 0..1.5.
- Builds `gravity.GravityModel(extra=...)` from the bodies; `GRAV_SCALE` takes the file's scales (1.0 on J2-J6) for
  this run and is restored afterwards. A covered joint's gravity = s x model, the part above min(s, 1) x plain URDF
  fading in over the first second after enable (the forearm rests on the J2 motor at the fold, CONTROLLER_TUNING §7);
  other joints keep plain URDF x the tuned scale.
- **New, only with the flag:** the Coulomb friction term is reduced where gravity (+ `--ff-bias`) + Coulomb would come
  within 0.05 Nm of the feed-forward cap (10 Nm J1-J3, 2.5 Nm J4-J6). Reason: with the fitted model J3 needs 8.3 Nm
  near the fold, and the 2 Nm Coulomb term would put the total at 10.3 Nm, so the heart (and every crack trajectory)
  would otherwise abort at the start of the approach / be refused by the arm_guard pre-flight. Gravity (+ bias) alone
  over the cap is **not** trimmed: the existing cap check (`ff_for`) and the guard pre-flight still stop it. Every cap,
  the gains, the fade-in, the aborts, the guard pre-flight and the guard itself are unchanged.
- The run summary gets `gravity_model` (`null` without the flag; with it: file, sha256, URDF sha256, bodies, scales,
  `created_utc`, `extrapolated`, the 0.05 Nm margin) and `gravity_scale` as used; the log gets one line. The run CSV
  has no header comment (its first line is read as column names by several tools), so nothing was added to it.
- `--simulate` only: the fake plant's constant load stays URDF x the tuned scales at the rest pose with or without the
  flag (only the commands change). With the model's own 8.2 Nm as the load, the fake J3 (no rest contact at the fold)
  sank to +0.05 rad during the fade-in and the guard faulted; the real forearm sits on its rest there.

## Evidence that the default is unchanged

- **Rehearsal diff:** `run-trajectory.sh --rehearse trajectories/fancy/heart.json` before the edit and after the final
  edit wrote byte-identical CSVs (every joint's ref/pos/vel/torq/ff every 5 ms; md5 `29beb50f31108975c2421428fb0e3d8c`;
  the fake bus is deterministic: two pre-change rehearsals were already identical). The summaries differ only in
  `version`, guard event timestamps and the new `gravity_model: null`.
- **Test:** `test_without_the_flag_the_command_stream_is_byte_identical_to_the_pre_change_runner` runs a guarded smoke
  trajectory through the pre-change script and the new one and compares every `enable`, `send_mit(pos, vel, kp, kd,
  tau)` and `disable` the fake bus received (> 10 000 calls): equal.

## `run-trajectory.sh --rehearse` and `DEMO_EXTRA`

The plan assumed `--rehearse` passes `DEMO_EXTRA` through; it does not (only `--check` and real runs do), and the
wrapper was not to be edited. `DEMO_EXTRA="--gravity-model …" run-trajectory.sh --rehearse heart.json` therefore
rehearses **without** the model (verified: identical CSV, `gravity_model: null`). The with-model rehearsal is the test
`test_heart_rehearsal_with_the_model_completes_under_the_guard`: `demo_move.run` with exactly the `--rehearse`
arguments (`--simulate --guard-sim --wait-go … --sim-auto-go 2 --trajectory heart.json --slow 1`) plus
`--gravity-model ~/Projects/rebot_crack_vision/config/robot/gravity_model.yaml`: completed, no guard fault, guard
pre-flight clean, |J3 ff| ≤ 9.95 Nm throughout. A future one-line change to the wrapper's rehearse branch
(`${DEMO_EXTRA:-}`) would make the two the same; that is the operator's call.

## Tests (`GravityModelFlag`, 6 new)

loader (a synthetic model and the repo's file); URDF sha256 mismatch and eight kinds of bad file refused, the bus never
opened; feed-forward composition (fade-in from plain URDF to the model, friction unchanged below the cap, the Coulomb
trim at the cap, gravity over the cap left for the cap check); a guarded run applies the scales, records file and
sha256, and restores the defaults for the next run; the heart with the model under the guard; the byte-identical
default command stream. Runner suite: **53 tests, all pass** (47 before + 6):
`cd ~/rebot_setup && env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python -m unittest test_guarded_runs test_demo_move`.

## Rehearsals run for this change (simulation, real guard config)

| Trajectory | Flag | Slow | Outcome |
|---|---|---|---|
| `trajectories/fancy/heart.json` | none (pre-change runner) | 1 | completed (x2, identical) |
| `trajectories/fancy/heart.json` | none (new runner) | 1 | completed; CSV byte-identical to the pre-change run |
| `trajectories/fancy/heart.json` | `DEMO_EXTRA=--gravity-model …` | 1 | completed, but without the model (the wrapper drops it) |
| `trajectories/fancy/heart.json` | `--gravity-model` (in-process test) | 1 | completed, no guard fault |
| `trajectories/gravity_survey.json` | none | 1 | completed, no guard fault; pre-flight 49 601 cycles accepted, keep-out margin ≥ 35 mm (file sha256 `473b03a6…`) |
| `trajectories/gravity_survey.json` | none | 2 | completed, no guard fault; pre-flight 98 201 cycles accepted, keep-out margin ≥ 35 mm |

---

## 2026-10-29 (second change): `demo_move.py` 2026-10-29.2 — `--friction jN=X`

For [docs/motion/J4_TUNING.md](../../../docs/motion/J4_TUNING.md) §3 (plan
[J4_TUNING_PLAN.md](../../../docs/motion/J4_TUNING_PLAN.md) item 2). Workstation only, as above: no arm, no CAN bus,
no register; the only runner invocations were `run-trajectory.sh --rehearse`, in-process `--simulate --guard-sim`
rehearsals and the unit tests.

- `before_j4_friction/` — `demo_move.py` / `test_demo_move.py` as they were before this edit (= the 2026-10-29.1 files
  above), copied first; the same copies are in `~/rebot_setup/backup_pre_j4_friction_2026-10-29/` (the byte-identity
  test loads them). Checksums of every runner file before: `SHA256SUMS.before_j4_friction`.
- top level — the files after this change; `SHA256SUMS` — every runner file now.

| File | sha256 before | sha256 after | What changed |
|---|---|---|---|
| `demo_move.py` | `e80f2026…a7e29a` | `53924489…acacee3` | 2026-10-29.1 → **2026-10-29.2**: opt-in `--friction jN=X[,jM=Y]` (below); module docstring |
| `test_demo_move.py` | `9a5b35e0…acfc665` | `f3c5f33d…219606fc0` | new `FrictionFlag` class, 6 tests (below) |
| `run-trajectory.sh` | `75866449…39f76d` | unchanged | – (not to be edited) |
| `gravity.py`, `fakebus.py`, `test_guarded_runs.py` | (see `SHA256SUMS.before_j4_friction`) | unchanged | – |
| `first_move_j1.py` | `5f66524a…48498` | unchanged | – (TUNED / FRICTION_MAX_NM read only) |
| `tuned_gains.yaml` | `d82f2d14…fbf8` | unchanged | – |

`config/robot/arm_guard.toml` (repo) unchanged (sha256 `fbf08261…5288b`).

### What `--friction` does

- `--friction jN=X[,jM=Y]`: this run's Coulomb friction feed-forward for those joints is X Nm instead of
  `first_move_j1.TUNED`'s (J4 0.5). **0 ≤ X ≤ `first_move_j1.FRICTION_MAX_NM` (3.0), joints j1-j6**; a bad spec is a
  usage error before the bus is opened.
- It updates `FRICTION`, the one table both `feed_forward` and `model_feed_forward` read, so it applies with and without
  `--gravity-model` (and to that option's Coulomb trim at the caps); the arm_guard pre-flight sees the commands with it.
  Every cap, gain, abort and the guard are unchanged. `FRICTION` is restored after the run, like `GAINS`/`GRAV_SCALE`.
- The summary gets `friction` (as used: a copy, so the value written is this run's, not the restored default) and
  `friction_override` (`{}` without the flag); the log a line `Coulomb friction feed-forward for this run: j4 0.18 Nm
  (tuned 0.5)`.

### Evidence that the default is unchanged

- **Rehearsal diff:** `run-trajectory.sh --rehearse trajectories/fancy/heart.json 1` before the edit and after the
  final edit: byte-identical CSVs, **md5 `29beb50f31108975c2421428fb0e3d8c` both** (the same as before the gravity-model
  change). The summaries differ in `version`, timestamps, and the new `friction_override: {}`.
- **Test:** `test_without_the_flag_the_command_stream_is_byte_identical_to_both_pre_change_runners` compares every
  `enable`, `send_mit(pos, vel, kp, kd, tau)` and `disable` of a guarded smoke run between the new script, the copy in
  `backup_pre_j4_friction_2026-10-29/` and the copy in `backup_pre_gravity_model_2026-10-29/` (> 10 000 calls): equal.
- `DEMO_EXTRA="--friction j4=0.18" run-trajectory.sh --rehearse …` rehearses **without** the flag (identical CSV,
  `friction_override: {}`): the wrapper's rehearse branch does not pass `DEMO_EXTRA`, as found for `--gravity-model`.

### Tests (`FrictionFlag`, 6 new)

parse and bounds (accepted 0 and 3.0; refused 3.01, −0.1, j7, j9, nan, inf, malformed); a bad value is a usage error and
the fake bus sees no call; the feed-forward moves by exactly (X − F) × tanh with and without `--gravity-model` and no
other joint changes; a guarded heart rehearsal with `--friction j4=0.18` logs the line, records `friction` /
`friction_override`, its logged J4 feed-forward minus 2.5 × URDF gravity peaks at 0.18 Nm, and the next run is back to
0.5; the byte-identity test above; **the j4_tune rehearsals with the A/B overrides** (`--friction j4=0.18 --gains
j4=40/3`, `--friction j4=0.18 --gains j4=50/3`, `--gains j4=50/3`): completed, guard pre-flight clean, no guard fault.
Runner suite: **59 tests, all pass** (53 before + 6), none skipped:
`cd ~/rebot_setup && env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python -m unittest test_guarded_runs test_demo_move`.

### Rehearsals run for this change (simulation, real guard config)

| Trajectory | Flags | Slow | Outcome |
|---|---|---|---|
| `trajectories/fancy/heart.json` | none (pre-change runner) | 1 | completed, md5 `29beb50f…` |
| `trajectories/fancy/heart.json` | none (new runner) | 1 | completed; CSV byte-identical |
| `trajectories/fancy/heart.json` | `DEMO_EXTRA=--friction j4=0.18` | 1 | completed, but without the flag (the wrapper drops it) |
| `trajectories/fancy/heart.json` | `--friction j4=0.18` (in-process test) | 1 | completed, no guard fault |
| `trajectories/j4_tune.json` (file sha256 `d3af3b86…1fda23`) | none | 1 | completed, no guard fault; pre-flight 39 121 cycles accepted, keep-out margin ≥ 41 mm |
| `trajectories/j4_tune.json` | none | 2 | completed, no guard fault; pre-flight 77 241 cycles accepted, keep-out margin ≥ 41 mm |
| `trajectories/j4_tune.json` | `--friction j4=0.18 --gains j4=40/3`; `--friction j4=0.18 --gains j4=50/3`; `--gains j4=50/3` (in-process test) | 1 | all completed, no guard fault |
| `trajectories/j4_tune.json` | `--friction j4=0.3` (in-process, the what-if check in J4_TUNING §2) | 1 | completed, no guard fault |

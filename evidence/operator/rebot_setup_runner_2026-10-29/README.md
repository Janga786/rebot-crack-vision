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

## 2026-10-29, later: `run-trajectory.sh` rehearse branch passes `DEMO_EXTRA`

One-line change (readiness pass before the first physical run): the `--rehearse` branch now appends `${DEMO_EXTRA:-}`
to the `demo_move.py --simulate --guard-sim …` command, exactly as the `--check` and real-run branches already did, so a
rehearsal exercises the same per-run overrides (`--gains`, `--ff-bias`, `--gravity-model`, `--friction`) the operator
will run. Pre-change copy: `~/rebot_setup/backup_pre_rehearse_extra_2026-10-29/run-trajectory.sh` (sha256 `75866449…`);
after: `run-trajectory.sh` here (sha256 `083bdcda…`). Verified: `DEMO_EXTRA="--friction j4=0.18 --gains j4=40/3"
run-trajectory.sh --rehearse trajectories/j4_tune.json 2` completed with `friction_override: {"j4": 0.18}` and
`gains_override: {"j4": [40, 3]}` in the summary; without `DEMO_EXTRA` the heart rehearsal CSV is unchanged
(md5 `29beb50f…`). Nothing else in the wrapper changed; real runs are unaffected.

## 2026-10-29, audit 1 fixes (readiness audit before the first physical crack run)

Pre-change copies: `~/rebot_setup/backup_pre_audit1_2026-10-29/` and `before_audit1/` here (`SHA256SUMS.before_audit1`).
After: the files here (`SHA256SUMS`). Nothing was run on the arm; only unit tests and `run-trajectory.sh --rehearse`.

| File | sha256 before | sha256 after |
|---|---|---|
| `run-trajectory.sh` | `083bdcda566e02bdd7f09d1575bd7827aac5004dee99ea3682f646ef0fc27644` | `052619cd35d3f710f0fc6c981b7868259fba5409078b0c43e0681832082107fb` |
| `demo_move.py` (2026-10-29.2 → 2026-10-29.3) | `5392448974569b76474e0173333d3d37597abce3f93f0637e04fe3e90acacee3` | `43104d80b28f8fed52e68ae9b6a56bf362b57b47c8bfe6b3ce8a0e310b13aa16` |
| `test_demo_move.py` | `f3c5f33dc77d80832770ba3f1e4c80f7b1956b5a920a6853d307ac6219606fc0` | `11be1019a72198063cfb7dec61786e7fd7aacf422d2fc5ab35c432666a1c148b` |

What changed:
- **run-trajectory.sh, hold prompt (audit 1, BLOCKER):** only an empty line writes the go file; `stop` = SIGINT (park
  + off), `off` = write `DEMO_OFF` (switch off at the fold now), anything else prints "type Enter to run, stop, or off"
  and keeps waiting. Before, any text (`yes`, `off`) started the trajectory. Header text updated.
- **run-trajectory.sh, stale input (audit 2, BLOCKER):** pending terminal input is drained (`while read -r -t 0.05 _`)
  right before the ">>> Holding at zero" prompt, so an Enter typed while the arm moved to zero no longer starts the run.
  (`first_run.py` also `tcflush`es the terminal before handing it to the runner.)
- **demo_move.py:** run summary `path_kinds` = (kind, duration) of every segment (a zero-length approach/return is
  dropped, so the trajectory can be segment 0; `tools/analyze_run_gains.py` now picks it by kind). `park()` and the
  guard hold clamp a feed-forward that would trip the cap to ±(cap − 0.05 Nm) instead of stepping it to 0. The guard
  pre-flight also evaluates the feed-forward gravity at the reference ±0.05 rad on J2-J4 (`gravity_sag_check`, every
  0.05 s; summary `guard_preflight.gravity_margin_nm`) and refuses a run where that passes a cap, naming the joint and
  the margin (with and without `--gravity-model`). `--gravity-model`: the Coulomb trim also applies when gravity alone
  is within the 0.05 Nm margin. The guard hold target is clamped to the joint limits like `park()`, and a
  `PermissionError` from a DemoMotor cap there is logged instead of escaping (which disabled the motors). No default
  gain, friction, cap or arm_guard.toml value changed; every existing cap and abort is kept.
- **test_demo_move.py:** `AuditFixes` (6 tests): segment kinds + a fold-to-fold file as segment 0; the sag check's
  message/margin; the crack demo trace with the repo model refused up front; the model trim inside the margin; park
  and guard hold clamp (not zero) the feed-forward; guard hold clamps the target and survives a refused command.

Verification: runner suite **65 tests OK** (59 + 6). Default command stream unchanged: heart rehearsal CSV md5
`29beb50f31108975c2421428fb0e3d8c` (before and after), and both byte-identity tests pass. Rehearsals (`--rehearse`,
real guard config), all completed with no guard fault unless noted: heart x1; search.json, scan.json and
demo_20261029T013007Z_trace.json x2 with `DEMO_EXTRA="--gains j2=100/4,j3=80/4"`; gravity_survey.json x2;
j4_tune.json x2; heart x1 and gravity_survey.json x2 with `--gravity-model`. The crack demo trace with
`--gravity-model` (x1 and x2) is now **refused at pre-flight**: "j2 feed-forward gravity reaches 10.42 Nm with j2/j3/j4
−0.05 rad of sag, margin −0.42 Nm" (the correct, safe outcome: J2 sits 0.05-0.10 Nm from the cap at the reference).
Fake-backend reproductions of audit items 1 and 2 (pty-driven `run-trajectory.sh` with a stub `demo_move.py`): a stray
Enter during zeroing, `yes`, or `off` no longer start the run (`off` switches off; an empty line after the prompt does
start it).

## 2026-10-29, audit 2 fixes (readiness audit pass 2: six minor findings)

Pre-change copies: `~/rebot_setup/backup_pre_audit2_2026-10-29/` and `before_audit2/` here (`SHA256SUMS.before_audit2`).
After: the files here (`SHA256SUMS`). Nothing was run on the arm and the CAN bus was never opened: only unit tests,
`run-trajectory.sh --rehearse`, and the wrapper driven on a pty against a stub `demo_move.py` (fake runner).

| File | sha256 before | sha256 after |
|---|---|---|
| `run-trajectory.sh` | `052619cd35d3f710f0fc6c981b7868259fba5409078b0c43e0681832082107fb` | `9c1333f82c6ffde77d8a3fd89d19b6e7852028de8d8ff2d642d0474b459288db` |
| `demo_move.py` (2026-10-29.3 → 2026-10-29.4) | `43104d80b28f8fed52e68ae9b6a56bf362b57b47c8bfe6b3ce8a0e310b13aa16` | `5352950a792d4681845beba5b61bf823cda1bfd74609511c207bb8894b7195f0` |
| `test_demo_move.py` | `11be1019a72198063cfb7dec61786e7fd7aacf422d2fc5ab35c432666a1c148b` | `e9d5deae53d91bc705380bf6b49dd8605d61c522fb891529a26e26398aba9958` |

Also changed in the repo (not copied here): `scripts/crack_pipeline.sh` (item 5) and `tests/test_first_run.py` (its test).

What changed:
1. **run-trajectory.sh, early input:** the drain before the hold prompt no longer throws everything away. A line `stop`
   typed while the arm moves to zero = SIGINT as soon as it holds at zero (park to the fold + off; the hold prompt is
   skipped, the run-phase stop/off loop runs); `off` = writes `DEMO_OFF` (off at the fold); an empty line or anything
   else is still dropped (a stray Enter never starts the run). Header text updated.
2. **demo_move.py, Ctrl-C before enabling:** `run()` installs a SIGINT handler on its first line (the wrapper starts
   it in the background, where SIGINT was ignored until `control()` installed its handlers, so a Ctrl-C during the
   pre-check was lost and the run went on to enable). If it fired, the run refuses after the bus pre-check, before the
   check-only/confirmation step, or at the top of `control()` before the first enable: "REFUSED (nothing was
   enabled): Ctrl-C / stop received before enabling", outcome `refused`, exit 3. `control()` still installs its own
   handlers (and restores this one); the caller's handler is restored at the end. The wrapper's trap message now
   says what a Ctrl-C does in each phase.
3. **demo_move.py, feed-forward fade on abort:** `park()` and `guard_recover()` take `fade` (default 1.0 for direct
   callers); `control()` passes the fade of the last command (`min(1, t / SETTLE_S)`, 0 at enable), recorded as
   `stats.ff_fade_at_abort`. An abort in the first second after enable no longer steps J3 from 1x to 2.5x URDF
   gravity (the 2026-10-17 kick); the park log says "feed-forward fade-in kept at …" when it is below 1.
4. **demo_move.py, stream gap in the post-fault hold:** a `stream` abort (> 50 ms between cycles) while holding after
   a guard fault logs a WARNING (`stats.hold_stream_gaps`), restarts the gap clock and re-sends the hold at once; it
   no longer disables (the motors hold their last command through a gap, CAN timeout 0). stale/status/bus/guard there
   still disable as before.
5. **crack_pipeline.sh search/scan (repo):** exit `$(( rrc ? rrc : crc ))`: a failed arm run fails the step even when
   the capture succeeded; "next:" only when both exit 0. `first_run.py` session needs no change (any rc != 0 =
   STOPPED, the runner summary outcome is checked too).
6. **run-trajectory.sh, J1 "type yes":** the INT trap sets a flag; the question reads in 1 s slices and a Ctrl-C ends
   it: "Ctrl-C: not confirmed; nothing was enabled.", exit 4 (before, `read` kept waiting).

Tests: `AuditFixes2` (6 new): Ctrl-C (real SIGINT) during the bus pre-check refuses with no enable/send_mit and the
handler restored; a flag left before `control()` refuses there; `park(fade=0.2)` sends the faded J3 feed-forward
(within 1e-6; > 0.5 Nm below the fade-1 value); a simulated run stopped 0.2 s after enable parks with
`ff_fade_at_abort` in [0.2, 0.21) and the first J3 park command on the fake bus = the faded feed-forward within 1e-6;
a stream gap in the guard hold warns, re-sends at once and keeps holding; stale/status/bus/guard there still disable.
Repo: `test_search_and_scan_exit_with_the_arm_runs_code_even_when_the_capture_succeeded` (10 cases; the old script
returned 0 with the arm run at exit 3).

Verification: runner suite **71 tests OK** (65 + 6); repo suite **750 passed, 1 skipped** (740 + 10). Default command stream unchanged: heart rehearsal CSV md5
`29beb50f31108975c2421428fb0e3d8c` (unchanged), both byte-identity tests pass. Rehearsals (`--rehearse`, real guard
config), all completed, no guard fault: heart x1; search.json, scan.json, demo_20261029T013007Z_trace.json x2 with
`DEMO_EXTRA="--gains j2=100/4,j3=80/4"` (search/scan with `--state-file`); gravity_survey.json x2; j4_tune.json x2.
pty (stub runner): `stop` typed at 0.8 s during zeroing → parked (exit 3); `off` typed then → off at the hold; a stray
Enter at 0.8 s → dropped (no go); `yes` during zeroing then Enter at the prompt → runs; Enter / stop / off / Ctrl-C at
the prompt and stop / off during the run unchanged; J1 question: Ctrl-C (also after typing `ye`) → exit 4 within 1 s,
`no` / EOF → exit 4, `yes` then Enter → runs.

## 2026-10-29, audit pass 3: one line in `run-trajectory.sh`

After an early `stop` or `off` (typed while the arm moved to zero) the wrapper now drains any further buffered lines, so a
second `stop` typed early cannot reach the run loop as a second Ctrl-C (which means "switch off now"). Backup:
`~/rebot_setup/backup_pre_audit3_2026-10-29/run-trajectory.sh`. Heart rehearsal unchanged (md5 29beb50f…).

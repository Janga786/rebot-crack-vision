# EXECUTION.md — operator runbook for `execute_trajectory` (mock → dry → real)

**Status:** operator-facing. Normative source for behaviour: `docs/adr/016-commissioning-gated-execution.md`
(ADR-016) and `docs/INTERFACES.md` §11. This document explains how to run the executor and read its
output; it adds nothing to, and never relaxes, those two sources. If anything here disagrees with them,
they win.

**As shipped, `--mode real` is refused, full stop.** Three independent `real`-only gates fail on a clean
checkout: `config/scene/scene.yaml` carries `nominal` entries (`G-SCENE`), `config/robot/end_effector.yaml`
carries `nominal` entries (`G-EE`), and `config/robot/commissioning.yaml` ships `commissioned: false`
(`G-COMMISSIONING`), with its e-stop record unverified (`G-ESTOP`). No flag, config edit or environment
variable in this repo changes that — each is fixed only by the card that owns the underlying evidence
(GEOM-05/GEOM-07 for the scene/end-effector configs, MOT-09 for commissioning). This document does not
claim any hardware readiness; it describes a gate that is designed to refuse until that evidence exists.

## 1. Who this is for

MOT-09 (arm bring-up commissioning) and, later, OPS-01 (operational procedures) run `execute_trajectory`
without reading its source. This runbook is the contract they rely on. Everything below cites the exact
commands, files and exit codes the executor actually uses.

## 2. Prerequisites, by mode

| Mode | Needs | Produces/consumes |
|---|---|---|
| `mock` | The overlay built (`scripts/ros/build_ws.sh`). A mock driver running: either the `moveit_mock` headless stack (`ros2 launch crackvision_motion mock_planning.launch.py`) or `vendor_mock` (`ros2 run rebot_motion mock_driver`, from `~/rebot_ws`'s install). Production scene applied (`ros2 run crackvision_motion apply_scene`). | A `crackvision.joint_trajectory/1` file (§11.2, from MOT-07). Nothing hardware-facing. |
| `dry` | Everything `mock` needs, **or** the live vendor driver (`reBotArmController`) up and reachable, per §4 below. No motion is ever sent in this mode regardless of which driver is live. | Same trajectory file. Optionally a commissioning record and approval file, to preview what `real` would do (`real_preview[]`). |
| `real` | Everything `dry` needs against the **live vendor driver specifically** (profile `vendor`), plus: `config/robot/commissioning.yaml` → `commissioned: true` with a matching limits hash (MOT-09); `config/robot/end_effector.yaml` and `config/scene/scene.yaml` fully `measured` (GEOM-05/GEOM-07); a `crackvision.execution_approval/1` file naming the trajectory's sha256, fresh within `approval_max_age_s` (MOT-08); env var `CRACKVISION_ARM_REAL=1`; a controlling `/dev/tty` to type the confirmation phrase on. | A real `FollowJointTrajectory` goal, under the monitoring in §7. |

## 3. Producing the records the gates read

- **Commissioning record** (`config/robot/commissioning.yaml`, schema `crackvision.commissioning/1`,
  ADR-016 §11.4): ships `commissioned: false`. Only MOT-09 may flip it to `true`, from operator-witnessed
  bring-up evidence on the physical unit — never a placeholder edit from this repo. It must also carry
  `limits_file_sha256` equal to the current `config/robot/b601_dm_limits.yaml` sha256 (`G-COMMISSIONING`),
  and `estop.kind: hardware` with `estop.verified: true` once the physical e-stop circuit has been tested
  by hand (`G-ESTOP`).
- **Execution approval** (schema `crackvision.execution_approval/1`, §11.5): produced by MOT-08, one file
  per trajectory, binding `trajectory_sha256` to `approved_by`/`approved_utc`/`preview_artifacts[]`.
  `G-APPROVAL` fails if no approval names the current trajectory's sha, or if it is older than
  `execution.yaml.approval_max_age_s` (ships 3600 s). Once both of those pass, `G-APPROVAL` also verifies
  the preview binding (§18.3): `preview_artifacts` is non-empty, each entry's sha256 matches its file on
  disk, exactly one entry is a `crackvision.trajectory_preview/1` manifest bound to this trajectory's sha,
  and that manifest's `check_report.all_pass == true` — produced by `trajectory_preview`/`approve_trajectory`
  (MOT-08.5, §18).
- **Trajectory file** (schema `crackvision.joint_trajectory/1`, §11.2): produced by MOT-07, planned and
  retimed against the current `config/robot/b601_dm_limits.yaml`, carrying the sha256 of that limits file
  and of the end-effector/scene configs it was planned against, so the gates can detect drift
  (`G-LIMITS-HASH`, `G-STALE-CONFIG`).

## 4. Running it

```bash
source scripts/ros/env_ros.sh                   # scrubbed ROS Humble + ~/rebot_ws underlay (ADR-007)
set +u; source ros2_ws/install/setup.bash        # the crackvision overlay
ros2 run crackvision_motion execute_trajectory --trajectory PATH --mode dry \
    [--profile moveit_mock|vendor_mock|vendor] [--speed-scale S] [--approval PATH] \
    [--execution-config PATH] [--commissioning PATH] [--limits PATH] [--scene-config PATH] \
    [--end-effector-config PATH] [--service-timeout-s SEC] [--record-dir DIR]
```

`--mode` defaults to `dry` (the safest non-refusing choice, ADR-016 §1): forgetting the flag never gives a
silent `mock` "success" that looks like a real rehearsal. `--mode real --profile moveit_mock` (or any
mode/profile pair outside the §11.1 table) is a usage error, exit 2, before any gate runs.

**Mock progression:**
```bash
bash scripts/ros/build_ws.sh                                     # build the overlay once
# Terminal A: bring up the mock stack (moveit_mock)
./scripts/ros/env_ros.sh ros2 launch crackvision_motion mock_planning.launch.py
# Terminal B, after sourcing env_ros.sh + the overlay, with the production scene applied:
ros2 run crackvision_motion apply_scene --config config/scene/scene.yaml
ros2 run crackvision_motion execute_trajectory --trajectory logs/plans/my_traj.json --mode mock
```
`bash scripts/ros/test_execution.sh` runs this whole progression (plus `vendor_mock` and refusal cases)
headlessly, with no hardware and no real vendor driver, and is the quickest way to see every mode/profile
combination exercised end to end.

**Dry rehearsal against the live vendor driver** (motion-free; see MOT-11/`VENDOR_DRIVER.md` before doing
this — as of the 2026-10-10 survey the vendor driver has outstanding FAILs and must not be launched yet):
```bash
./env.sh python scripts/motion/check_vendor_driver.py --rebot-ws "$HOME/rebot_ws" \
    --facts evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml   # exit 0 required first
# operator's own real-driver bring-up (§5), then:
ros2 run crackvision_motion execute_trajectory --trajectory PATH --mode dry --profile vendor
```
`dry` constructs no `FollowJointTrajectory` `ActionClient` in any profile, so it cannot move the arm even
against the live driver. It still energises/holds the arm if the driver is already up (§5) — that is a
property of launching the driver, not of this executor.

**Real** (will refuse on a clean checkout — see the top of this document):
```bash
CRACKVISION_ARM_REAL=1 ros2 run crackvision_motion execute_trajectory --trajectory PATH \
    --mode real --profile vendor --approval APPROVAL_PATH
```
Typing `EXECUTE <first 8 hex chars of the trajectory's sha256>` at the prompt, on a real controlling tty,
is the only thing that satisfies `G-CONFIRM`; no flag, env var or piped stdin substitutes for it.

**GEOM-05/GEOM-07/MOT-09 ordering note (ADR-016 §3, quoted):**
> The resolution is that **GEOM-05 and GEOM-07's calibration motion is not this executor's job.** Both
> cards move the arm by **vendor hand-guided gravity compensation** (`/rebotarm/gravity_compensation/start`,
> a service `execute_trajectory` never calls — Decision §8)... MOT-09 is explicitly scoped to
> **readback-only dry rehearsals**: `execute_trajectory --mode dry` against the live vendor driver, which
> runs every gate except the four real-only ones (§11.7) and moves nothing... Only once
> `commissioning.yaml` says `commissioned: true` and `end_effector.yaml` and `scene.yaml` are fully
> `measured`... does `--mode real` become reachable at all.
>
> **Escalated alternative (not adopted, needs technical-lead sign-off before anyone implements it):** a
> "commissioning purpose tier"... is explicitly **not** decided here... Until that happens, GEOM-05/07 use
> hand-guided gravity compensation, not `execute_trajectory`, full stop.

Calibration motion therefore never goes through this executor, by design, unless a technical lead
approves and writes an ADR amendment for the escalated alternative above. Do not route GEOM-05/GEOM-07
motion through `execute_trajectory` under time pressure.

## 5. Real-driver bring-up (operator's own commands, per ADR-016 and `VENDOR_DRIVER.md`)

The executor never starts, enables or disables the vendor driver — §8 of ADR-016 lists every vendor
surface it touches (`follow_joint_trajectory`, read-only `joint_states`/`arm_status`) and every one it
never calls (`enable`, `disable`, `safe_home`, `park`, `move_to_pose`, `gravity_compensation/*`,
`set_zero`, `set_mode`, the gripper, and the low-level `cmd/*` passthrough topics). Bring-up and enabling
are the operator's own documented procedure, run once per session, before any `real` (or vendor-driver
`dry`) invocation:

1. Re-run `check_vendor_driver.py` (§4 above) and confirm exit 0 (no FAIL). As of the 2026-10-10 survey
   this fails (`V-SAFE-PARK`, `V-SDK-API`, `V-PYDEPS`) — see `docs/motion/VENDOR_DRIVER.md`. Fix every
   FAIL vendor-side first; this repo never patches `~/rebot_ws`.
2. Disable `safe_park` (or fix `safe_park_q`) before launch — the shipped `safe_park.yaml` parks joint2/
   joint3 above their URDF upper limit, into the folded contact.
3. Launch the driver from an `env_ros.sh` shell, same `ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY` as the executor
   — the vendor deploy kit's own scripts default to `ROS_DOMAIN_ID=42` with a separate localhost DDS
   profile, which an executor in this repo's environment would not see.
4. **Launching `reBotArmController` energises the arm** (connect() runs the SDK's POS_VEL mode switch and
   `enable_all()`, and `init_gripper()` enables the gripper too). There is no separate `/enable` call to
   make; by the time the driver is up, the arm is already energised and holding. A `dry` rehearsal against
   it therefore means a live, holding arm even though the executor itself sends nothing.
5. The executor's own `G-START-STATE` check (vendor profile) requires the latched `/rebotarm/arm_status`
   to already report `enabled: true`, `state_machine: "IDLE"` and no error codes — it never calls `enable`
   itself and refuses informatively if the arm is not already in that state.
6. **After the session**, bring the arm to rest and power-cycle it: the vendor driver's own shutdown runs
   safe_home/park before disable. After any vendor-driver session, power-cycle the arm so the flash-stored
   MIT mode returns — several bring-up scripts refuse to run otherwise.
7. Use a `/dev/tty*` device path for `channel` (e.g. `/dev/ttyACM0`, or HOST-04's `/dev/ttyREBOT_ARM`),
   never `/dev/rebot_arm`, which switches the SDK to SocketCAN.

**Why the executor does not use any vendor motion service:** `execute_trajectory` sends goals on exactly
one interface, `/{ns}/follow_joint_trajectory`, and never calls `enable`, `disable`, `safe_home`, `park`,
`move_to_pose`/`move_to_pose_ik`, `set_zero`/`set_mode`, `gravity_compensation/*` or any gripper surface
(ADR-016 §8 table). Each of those either arms/de-energises the arm, drives it through an un-gated path, or
is simply out of scope for this card's decomposition — routing any of them through this executor would
bypass the gate catalogue in §11.7 that is the entire point of this ADR.

## 6. Gate catalogue

Every row is evaluated and reported every run for its applicable modes (ADR-016 §11.7); the executor never
stops at the first failure. `skip` = not evaluated in that mode. `refuse` = a violation is `fail`/`error`
(exit 3, nothing sent to any driver). `warn` = reported only, never refuses, in `mock`/`dry`.

| Gate id | What it checks | Applies to | Satisfied by | Remedy if failing |
|---|---|---|---|---|
| `G-ARM` | env `CRACKVISION_ARM_REAL=1` literally set | real only (previewed in dry) | operator session env | `export CRACKVISION_ARM_REAL=1` before the real invocation |
| `G-TRAJ` | file matches `crackvision.joint_trajectory/1` (§11.2); `purpose: "test"` refuses only in real | all | MOT-07 (trajectory producer) | replan/regenerate the file; use `purpose: "crack_task"` for a real run |
| `G-LIMITS-HASH` | trajectory's `limits_file.sha256` matches current `b601_dm_limits.yaml`; `null` warns in mock/dry, refuses in real | all | MOT-07 (binds the sha at plan time) | replan against the current limits file |
| `G-LIMITS` | unscaled file within `b601_dm_limits.yaml` positions/velocities/accelerations + §11.6 margin rule; scaled trajectory within `scale`/`scale²` of those limits | all | MOT-07/MOT-01 (planning limits, retiming) | replan within limits, or fix `--speed-scale` |
| `G-SPEED` | effective speed scale in `(0, cap]`; mock/dry cap = `execution.yaml.speed_scale.cap`, real cap = `commissioning.yaml.speed_scale_cap` | all | operator's `--speed-scale`, MOT-09 (raises the real cap with evidence) | lower `--speed-scale`, or get the cap raised via MOT-09 |
| `G-SCENE` | `scene_core.load_config` succeeds; `assert_commissioning_ready` (no `nominal` entries) warns in mock/dry, refuses in real | all | MOT-03/GEOM-05/GEOM-07 (populates `measured` scene entries) | measure and record the scene objects, or fix the config |
| `G-EE` | `end_effector.load_config` succeeds; `assert_commissioning_ready` (no `nominal` entries) warns in mock/dry, refuses in real | all | GEOM-05/GEOM-07 (hand-eye, tool pivot) | complete GEOM-05/GEOM-07, write `measured` values |
| `G-COMMISSIONING` | `commissioning.yaml` loads, `commissioned == true`, `limits_file_sha256` matches current limits | real only (previewed in dry) | MOT-09 (bring-up commissioning) | complete MOT-09 bring-up and set `commissioned: true` |
| `G-ESTOP` | `commissioning.yaml.estop.kind == "hardware"` and `estop.verified == true` | real only (previewed in dry) | MOT-09 (hands-on e-stop test) | MOT-09 verifies the physical e-stop circuit and records it |
| `G-ELIGIBLE` (`crack_task`/`test`) | referenced paths3d file exists, sha matches, `execution_eligible == true`; integrity (missing/mismatched file) refuses in every mode, ineligibility warns in mock/dry, refuses in real | all | GEOM-08 §10.6 (paths3d eligibility) | regenerate/fix the paths3d file so `execution_eligible: true` |
| `G-ELIGIBLE` (`view_move`, §17.2) | referenced `view_plan` file exists, sha matches, loads via `load_view_plan` with non-null `chosen`/`transit_leg`, and the trajectory's endpoints match `transit_leg`'s within `1e-9` rad — integrity, refuses in every mode; then the view plan's recorded `limits`/`end_effector`/`scene` shas match the trajectory's own — eligibility, warns in mock/dry, refuses in real | all | MOT-07.10 (`plan_view`/`view_plan` producer) | regenerate/fix the view plan so it loads cleanly with a matching `chosen`/`transit_leg`, or replan the trajectory against the current view plan/configs |
| `G-APPROVAL` | an approval file names the trajectory's sha256, within `approval_max_age_s` — then, only once both of those pass (§18.3), the preview binding: `preview_artifacts` is non-empty; every entry's current sha256 on disk matches its recorded `sha256`; exactly one entry parses as a `crackvision.trajectory_preview/1` manifest; that manifest's `trajectory.sha256` matches this trajectory; and its `check_report.all_pass == true`; each warns in mock/dry, refuses in real | all | MOT-08 (execution approval), MOT-08.5 (preview-artifact binding, §18.3) | get a fresh approval naming this exact trajectory sha; if the sha/age already match but the preview binding fails, re-run `trajectory_preview`/`approve_trajectory` so the approval lists exactly one bound, passing `crackvision.trajectory_preview/1` manifest whose `trajectory.sha256` matches this trajectory |
| `G-STALE-CONFIG` | non-null `end_effector_config_sha256`/`scene_config_sha256` match the current files (refuse in every mode if mismatched); `null` warns in mock/dry, refuses in real | all | MOT-07 (binds the shas at plan time) | replan against the current end-effector/scene config |
| `G-GRAPH` | driver-profile node-identity discrimination (§11.1/ADR-016 §2): mock profiles refuse if `reBotArmController` is visible; `vendor` refuses unless it hosts the server and no mock driver / passthrough publisher is visible | all (online) | correct driver for the chosen profile, no stray vendor-mock process | stop the wrong driver, use the matching `--profile` |
| `G-START-STATE` | live `joint_states` fresh and within `tolerances.start_state_rad` of point 0; `vendor` profile also needs `arm_status.enabled/state_machine/error_codes` healthy | all (online) | operator positions the arm at the trajectory's start; §5 bring-up for `vendor` | move the arm to the trajectory's first point, or complete bring-up |
| `G-COLLISION` | every densified waypoint valid against the applied production scene via the MOT-02 mock stack's `/check_state_validity` | all (online) | MOT-02 mock stack up, MOT-03 scene applied | fix the trajectory/scene, or apply the production scene first |
| `G-CONFIRM` | typed `EXECUTE <sha8>` on a controlling `/dev/tty`, asked only after every other gate passed | real only | the operator, interactively | type the exact phrase on a real terminal; no scripted substitute exists |

## 7. E-stop

**The physical, hardware e-stop is the safety function this project relies on. `/crackvision/estop` and
`/rebot_motion/estop` are secondary, software-only conveniences** that depend on the ROS graph being alive
(ADR-016 §7). If anything looks wrong, **press the hardware e-stop first** — do not wait on a software
topic.

The software topics exist for fast, scriptable stops during development/rehearsal:
```bash
ros2 topic pub /crackvision/estop std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /rebot_motion/estop std_msgs/msg/Bool "{data: true}" --once
```
Either topic, a `SIGINT`/`SIGTERM` to the executor, stale `joint_states`, a tracking-error excursion past
`tolerances.tracking_rad`, or a goal running past `timeouts.goal_s` all trigger the identical response
(ADR-016 §7):

1. Cancel the in-flight `follow_joint_trajectory` goal. The vendor driver holds the measured position with
   torque on.
2. Watch `joint_states` until the arm has stopped (settles within `timeouts.cancel_settle_s`).
3. If the cancel is not acknowledged, the arm does not settle, or `joint_states` go stale, the executor
   prints **"PRESS THE HARDWARE E-STOP"** — software cannot do more safely at that point.
4. Write the execution record (`outcome: estopped`, or `aborted` for tracking/timeout/driver loss) and
   exit 1.

**The executor never calls `/{ns}/disable` or any other vendor service, on e-stop or at teardown.** On
exit the arm is left energised and holding. Bringing it to rest afterward is the operator's job: the
vendor's own sequence (safe_home or park, *then* disable), or the hardware e-stop. That stays the procedure
until MOT-09 confirms on the physical unit that `disable` brings the arm to a safe rest, not an
uncontrolled fall under gravity.

## 8. Reading the execution record

Every invocation except `--dry-run` writes `crackvision.execution_record/1` to
`logs/execution/<stamp>_<mode>_<sha8>.json` (`<sha8>` = first 8 hex chars of the trajectory sha256;
override the directory with `--record-dir`), including on refusal. Key fields:

- `outcome`: `refused | rehearsed | aborted | estopped | completed`.
- `gate_report[]`: one entry per applicable §11.7 gate, with `outcome ∈ {pass, fail, warn, skip, error}`
  and a human-readable `detail`.
- `real_preview[]` (`dry` only): `G-ARM`, `G-COMMISSIONING`, `G-ESTOP` evaluated with `real`'s rules.
- `would_refuse_in_real[]`: every gate id that is only `warn` here but would refuse under `real`.
- `input_shas`, `effective_speed_scale`, `joint_trace[]`: provenance and the sampled commanded/actual
  trajectory, for post-run review.

`--dry-run` (distinct from `--mode dry`, §0.3) writes no record at all — it is an offline-only, in-process
report to stdout, exit 0 regardless of outcome.

## 9. Failure recovery, by outcome/exit code

| Outcome / exit code | Meaning | Next safe action |
|---|---|---|
| `refused` (exit 3) | At least one gate reported `fail`/`error`; nothing was sent to any driver | Read `gate_report[]` for every `fail`/`error` id, apply the §6 remedy for each, re-run. No motion occurred, no recovery needed on the arm itself |
| Usage/config error (exit 2) | Bad mode/profile pair, malformed `execution.yaml`, unreadable paths | Fix the CLI invocation or config file; no ROS/motion was attempted |
| `G-START-STATE` fail (start mismatch, exit 3) | The arm is not at the trajectory's first point (or, `vendor` profile, not `enabled`/`IDLE`) | Move the arm to the trajectory's start (plan a short move there first), or complete bring-up (§5), then re-run |
| `G-COLLISION` fail (exit 3) | The densified trajectory is invalid against the applied scene | Fix the trajectory or the scene; nothing was sent, the arm did not move |
| `estopped` (exit 1) | An e-stop trigger fired (topic, signal, staleness, tracking excursion, goal timeout); the goal was cancelled and the arm is holding | Confirm the arm has actually settled (check live `joint_states`); if the record notes "PRESS THE HARDWARE E-STOP" and settlement wasn't confirmed, press it now. Otherwise, diagnose the trigger cause, then bring the arm to rest with the vendor's own safe_home/park + disable sequence before any further run |
| `aborted` (exit 1) | Runtime failure distinct from a clean e-stop (driver loss, unexpected exception, tracking/timeout abort) | Same as `estopped`: verify the arm is holding/stopped via live `joint_states`; press the hardware e-stop if in doubt; investigate the driver/log before re-running |
| `completed` (exit 0, `mock`/`real`) | Goal completed and arrival verified | Normal completion; review `joint_trace[]` if anything looked off |
| `rehearsed` (exit 0, `dry`) | No gate refused in `dry`; nothing was sent to any driver regardless of `would_refuse_in_real` | Review `would_refuse_in_real[]` before attempting `real` with the same trajectory |

In every runtime-failure case (`estopped`/`aborted`), the arm is left energised and holding by design
(§7) — never call `disable` from a script or assume it is safe to walk away; a human brings the arm to
rest using the vendor's documented sequence, or the hardware e-stop.

## 10. See also

- `docs/adr/016-commissioning-gated-execution.md` — normative source for every rule above.
- `docs/INTERFACES.md` §11 — the frozen contract (schemas, CLI, exit codes, numeric defaults).
- `docs/motion/VENDOR_DRIVER.md` (MOT-11) — current vendor-driver readiness findings; re-run its checker
  before every vendor-driver session.
- `scripts/ros/test_execution.sh` — headless mock verification of this whole progression (mock, dry,
  refusals, collision, e-stop injection, vendor-interface mock); no hardware involved.

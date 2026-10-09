# ADR-016: Commissioning-gated execution — modes, driver profiles and the real-motion gate catalogue

**Status:** accepted · **Date:** 2026-10-08, amended 2026-10-09 (MOT-05.1.R1) · **Supersedes:** — · **Superseded by:** —

## Context

REQ-MOT-1 (MoveIt integration with the real B601-DM model) and REQ-INT-2 (separately gated hardware
commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced
evidence) converge on one piece of software: whatever module ultimately turns a planned trajectory into
`FollowJointTrajectory` goals on a physical arm. MOT-05 is that module (`execute_trajectory`). Getting its
safety design wrong is categorically worse than getting any planning card wrong, because a planning bug
produces a bad plan that never moves metal; an execution bug moves metal.

Two commissioning gates already exist and already refuse by default:
`crackvision_motion.scene_core.assert_commissioning_ready` (MOT-03, raises `SceneNotCommissionedError`
while any `config/scene/scene.yaml` object/ACM entry is `nominal`) and
`crackvision_description.end_effector.assert_commissioning_ready` (ADR-014, raises
`EndEffectorNotCalibratedError` while `tool`, `wrist_camera` or any collision block in
`config/robot/end_effector.yaml` is `nominal`). Both are unconditional refusals today — correct, but MOT-05
is the first card that actually has to call them from inside a real-execution code path and survive the
operational fact that GEOM-05 (camera hand-eye) and GEOM-07 (tool pivot) themselves need the arm moving
before they can promote those files to `measured`.

Survey of the vendor stack this executor must sit in front of (`~/rebot_ws`, read-only from this repo),
done 2026-10-08, extended 2026-10-09. Paths below are relative to
`~/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/` unless stated otherwise.

- The real driver is node `reBotArmController` (`rebotarm_controller.py:18-20`), namespace parameter
  `arm_namespace` defaulting to `rebotarm` (`rebotarm_controller.py:30,46`). It builds its ROS surface from
  four objects (`rebotarm_controller.py:81-94`) and exposes:
  - **Actions.**
    - `control_msgs/action/FollowJointTrajectory` at `/{ns}/follow_joint_trajectory`
      (`ros_actions.py:27-35`), the only interface this ADR's executor is allowed to drive motion through.
      It rejects a goal whose `joint_names` isn't exactly `hardware.joint_names` (`ros_actions.py:118-124`)
      and plans each segment as a time-ratio linear interpolation honouring each point's `time_from_start`
      (`ros_actions.py:147-184`).
    - `rebotarm_msgs/action/MoveToPose` at `/{ns}/move_to_pose` (`ros_actions.py:18-26`): a Cartesian
      end-pose motion action. **Motion-producing.**
    - `control_msgs/action/GripperCommand` at `/{ns}/gripper/command` (`ros_actions.py:36-44`).
  - **Services** (Trigger/typed) at `/{ns}/`: `enable`, `disable`, `safe_home`, `park`,
    `gravity_compensation/{start,stop}`, `set_zero`, `set_mode`, `move_to_pose_ik`, `gripper/{set,open,close}`
    (`ros_services.py:20-97`). Every one of these moves, arms or de-energises the arm.
  - **Low-level passthrough command topics.** For every arm joint, the node subscribes
    `/{ns}/joints/<joint>/cmd/mit` (`JointMitCmd`), `/{ns}/joints/<joint>/cmd/pos_vel` (`JointPosVelCmd`)
    and `/{ns}/joints/<joint>/cmd/vel` (`JointVelCmd`) (`motor_passthrough.py:19-46`, subscribed at
    `motor_passthrough.py:71-82`), plus the same three under `/{ns}/gripper/cmd/*` (`motor_passthrough.py:47-69,83-90`).
    Each message drives a motor directly. While a trajectory runs (`state_machine == "TRAJ_RUNNING"`), a
    joint command is rejected under the default `cmd_arbitration: reject` (`rebotarm_controller.py:31`).
    Under `cmd_arbitration: preempt` it **preempts the running trajectory** (`motor_passthrough.py:139-157`).
    **Motion-producing.**
  - **Readback / status topics.**
    - `sensor_msgs/msg/JointState` on `/{ns}/joint_states` (`ros_publishers.py:17-27`), the project's
      joint readback.
    - Per-joint `JointMotorState` on `/{ns}/joints/<joint>/state` (`ros_publishers.py:28-36`).
    - `rebotarm_msgs/msg/ArmStatus` on `/{ns}/arm_status`, latched (`TRANSIENT_LOCAL`,
      `ros_publishers.py:37-47`). It carries `mode`, `enabled`, `control_loop_active`, `state_machine`,
      `per_joint_status_code` and `error_codes` (`ros_publishers.py:109-119`). It is republished after
      every trajectory (`ros_actions.py:192-194`) and every passthrough command (`motor_passthrough.py:117`).
  - **Cancel holds position.** When a `follow_joint_trajectory` goal is cancelled, the driver calls
    `hold_current_position()` and reports the goal cancelled (`ros_actions.py:175-180`). It does the same when
    execution throws (`ros_actions.py:186-188`). `hold_current_position` sets the end-position controller's
    target to the *measured* joint positions (`hardware_manager.py:237-241`). The arm stays energised and
    servoes to where it is.
  - **`disable` drops torque.** `/rebotarm/disable` → `HardwareManager.disable()` → `RobotArm.disable()` →
    per-vendor `ControllerManager.disable_all()` → `JointGroup.disable()` (`hardware_manager.py:280-288`,
    `~/rebot_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:243-252`). This
    bottoms out in a compiled motor-bridge FFI call (`motorbridge.core.Controller.disable_all`, outside
    `~/rebot_ws`, not inspectable from source). `disable()` also stops the end-position control loop
    (`hardware_manager.py:284-286`). Nothing in `~/rebot_ws` mentions a mechanical brake or holding clutch on
    any joint. The driver's own shutdown runs `safe_park` or `safe_home` *before* `disable`
    (`rebotarm_controller.py:104-109` → `hardware_manager.py:199-216`; params `safe_home_on_shutdown` and
    `disable_after_safe_home` both default `True`, `rebotarm_controller.py:34-35`). It never cuts power in
    place. **This ADR treats `disable` as torque-off with a possible uncontrolled fall under gravity** (the
    conservative assumption) until MOT-09's hands-on commissioning confirms otherwise on the physical unit.
    Nothing here may assume a brake exists.
- `rebot_motion.mock_driver` (node `mock_rebotarm_driver`, `~/rebot_ws/src/rebot_motion/rebot_motion/mock_driver.py:36-38`)
  serves `/{ns}/follow_joint_trajectory`, `/{ns}/gripper/command`, `/{ns}/joint_states` and the Trigger
  services `enable`, `disable`, `safe_home`, `park`, `gravity_compensation/{start,stop}`
  (`mock_driver.py:53-65`). It has **no** `move_to_pose`, no passthrough topics and no `arm_status`.
- `~/rebot_ws/deploy/run_real_demo.sh` and `preflight.sh` already implement the shape this ADR adopts:
  two independent axes (driver mock|real × plan dry|go, `run_real_demo.sh:29-33`), a REAL execute refused
  without both `REAL=1` and a typed `yes` on `/dev/tty` (`run_real_demo.sh:170-192`), a `trap` teardown on
  any exit that e-stops, parks, disables and kills children (`run_real_demo.sh:46-70`), and the existing
  `/rebot_motion/estop` convention. `motion_runner.py`'s e-stop handler cancels the active goal **then** calls
  `/{ns}/disable` (`~/rebot_ws/src/rebot_motion/rebot_motion/motion_runner.py:124-134`). This ADR
  deliberately does **not** copy that second step (Decision §7).

None of this vendor evidence says anything about a *hardware* e-stop circuit (a physical button wired to
motor power, independent of any ROS node). The project does not control that circuit and must not pretend
software can substitute for it.

## Decision

### 1. Modes and the CLI default

Three modes: **`mock`** (goals only ever reach a mock driver), **`dry`** (every offline gate plus every
online read-only gate runs; no `ActionClient` for `follow_joint_trajectory` is ever constructed, so it is
safe to point at the live vendor driver as a rehearsal; it reports what `real` would additionally refuse)
and **`real`**. `execute_trajectory`'s `--mode` default is **`dry`** — the safest non-refusing choice, so
an operator who forgets the flag gets a full report and no motion, never a silent `mock` success that looks
like a real rehearsal.

Which gate failures refuse in which mode is fixed by §11.7's refusal rule. In short: integrity failures
(malformed file, limits violated, a non-null sha binding that mismatches, unsafe graph, start-state
mismatch, collision) refuse in **every** mode. Authorization and commissioning failures (no approval,
ineligible paths3d, nominal calibration, unbound shas) refuse only in `real`. In `mock`/`dry` they are
reported with outcome `warn`, never `pass`. `dry` exits 0 when nothing refuses in `dry`, even if it reports
refusals that only `real` would make.

`--dry-run` (§0.3, universal) is a different, stronger thing: offline-only gate report, no ROS node, no
file writes, exit 0 regardless of gate outcome. `--mode dry` runs the online gates too (it needs ROS) and
always writes the §11.9 execution record, including on refusal. A CLI invocation must not conflate the two;
`--dry-run --mode real` is accepted (and still only runs the offline gates).

The executor is an `rclpy` node. It is therefore invoked as the `crackvision_motion` ROS entry point
(`ros2 run crackvision_motion execute_trajectory`, after sourcing `scripts/ros/env_ros.sh` and the overlay)
under `/usr/bin/python3`, not through `./env.sh python`. This is the project's standing rclpy convention
(ADR-007) and is consistent with MOT-05.4 (§11.10).

### 2. Driver profiles (`config/motion/execution.yaml`, §11.1)

| Profile | Action | `joint_states` | Expected server node | Valid modes |
|---|---|---|---|---|
| `moveit_mock` | `/rebotarm_controller/follow_joint_trajectory` | `/joint_states` | ros2_control mock hardware (MOT-02) | mock, dry |
| `vendor_mock` | `/rebotarm/follow_joint_trajectory` | `/rebotarm/joint_states` | `mock_rebotarm_driver` | mock, dry |
| `vendor` | `/rebotarm/follow_joint_trajectory` | `/rebotarm/joint_states` | `reBotArmController` | dry, real |

A mode/profile pair outside this table is a usage error (exit 2), detected before any gate runs.

Graph discrimination (gate `G-GRAPH`, read-only `rclpy` graph introspection — `get_action_server_names_and_types`/
node-name lookups/publisher counts, never an `ActionClient.send_goal`): a `moveit_mock`/`vendor_mock` run
refuses if a node named `reBotArmController` is visible anywhere on the graph. A `vendor` run refuses unless
the action server backing `/rebotarm/follow_joint_trajectory` is hosted by a node named exactly
`reBotArmController` **and** no node named `mock_rebotarm_driver` is visible. For `vendor`, `G-GRAPH` also
refuses if any publisher exists on a `/rebotarm/joints/*/cmd/*` or `/rebotarm/gripper/cmd/*` topic. Such a
publisher is a second motion source, and under `cmd_arbitration: preempt` it would preempt the trajectory
(Context). This is a structural proof, re-checked at every run, not a configuration-file promise: a mock run
physically cannot address the real driver (its profile's action name and expected-node check point at the
mock), and a real run physically cannot be satisfied by a mock driver masquerading on the same names (it
would fail `G-GRAPH` on the node-name mismatch or the mock-node exclusion). `vendor_mock` exists because
`rebot_motion.mock_driver` is interface-identical to the real driver on the `/rebotarm/follow_joint_trajectory`
and `/rebotarm/joint_states` names. That lets a rehearsal exercise the *exact* names and node-identity checks
`real` will use, not a second, different mock namespace.

### 3. The real-mode commissioning calls

`execute_trajectory` in `real` mode calls both `crackvision_motion.scene_core.assert_commissioning_ready`
and `crackvision_description.end_effector.assert_commissioning_ready` on the loaded scene/end-effector
configs, with **no except clause that swallows either exception** — a `SceneNotCommissionedError` or
`EndEffectorNotCalibratedError` becomes a `G-SCENE`/`G-EE` gate failure (§11.7) and the process exits
3. There is no code path, flag, env var or config value that lets `real` mode proceed past either call
raising.

**The ordering problem is real and is not solved by weakening this gate.** GEOM-05 (camera hand-eye) and
GEOM-07 (tool pivot) are the cards that *produce* the `measured` values these gates require, and both need
the arm physically moved (through a sequence of hand-eye or pivot poses) before they have anything to
write. MOT-09 (arm bring-up commissioning) needs the arm moved at low speed to verify joint readback and
limits before `config/robot/commissioning.yaml` can ever say `commissioned: true`. None of these can be
unblocked by `execute_trajectory`'s gate, by construction — that is what the gate is for.

The resolution is that **GEOM-05 and GEOM-07's calibration motion is not this executor's job.** Both cards
move the arm by **vendor hand-guided gravity compensation** (`/rebotarm/gravity_compensation/start`, a
service `execute_trajectory` never calls — Decision §8), an operator physically guiding the arm by hand
while the vendor controller only cancels gravity torque, logged as its own evidenced procedure outside this
executor. `execute_trajectory` is not involved and its gates never run during that work, because no
`crackvision.joint_trajectory/1` file is being executed. MOT-09 is explicitly scoped to **readback-only dry
rehearsals**: `execute_trajectory --mode dry` against the live vendor driver, which runs every gate except
the four real-only ones (§11.7) and moves nothing, giving the operator the joint-readback and limits
evidence MOT-09 needs to populate `commissioning.yaml` without ever constructing a `FollowJointTrajectory`
goal. Only once `commissioning.yaml` says `commissioned: true` and `end_effector.yaml` and `scene.yaml` are
fully `measured` (all three are separate `real` gates, §11.7) does `--mode real` become reachable at all. At
that point GEOM-05/07 are already satisfied, and the ordering problem has resolved itself in the only
direction that doesn't bypass anything.

**Escalated alternative (not adopted, needs technical-lead sign-off before anyone implements it):** a
"commissioning purpose tier" that lets `real` mode run a narrow, hand-guided-only *class* of trajectories
(e.g. tiny amplitude, `purpose: calibration_probe`) against an end-effector/scene config that is still
partially `nominal`, on the theory that GEOM-05/07's own motion is low-risk. This is explicitly **not**
decided here: it would require its own ADR amendment, a second, independently-reasoned gate (what makes a
"calibration probe" trajectory safe when the file format already distinguishes `crack_task`/`test` and
nothing else — §11.2), and sign-off from a technical lead before any code implements it. Until that
happens, GEOM-05/07 use hand-guided gravity compensation, not `execute_trajectory`, full stop.

### 4. Speed policy

**Real-mode default and cap.** In `real` (and in `dry`, which previews exactly what `real` would send), the
`speed_scale` used when `--speed-scale` is omitted is `execution.yaml`'s `speed_scale.real_default`, which
ships **`0.10`**. The real-mode cap is `commissioning.yaml`'s `speed_scale_cap`, which ships **`0.10`**
(§11.4). An operator raises the cap only by editing the commissioning record with evidence (a logged
low-speed run, per MOT-09). `execution.yaml` can never raise the real cap. `mock` uses
`speed_scale.default` and the cap `speed_scale.cap` (§11.1), independent of the commissioning record, since
no real torque is at stake there.

**Refuse, never clamp.** The effective scale is `--speed-scale` if given, else the mode's default. It is
never silently reduced. `G-SPEED` (§11.7) refuses (exit 3) when the effective scale is outside
`(0, cap]` for the mode's cap. In `dry` it also reports `warn` when the scale exceeds the commissioning cap
that `real` would apply. An operator who asks for 0.5 on a 0.10-capped arm gets a refusal naming both
numbers, not a quietly different motion.

**Retiming is uniform time scaling**: applying `scale ∈ (0, 1]` multiplies every point's velocities by
`scale` and accelerations by `scale²`, leaves positions and point ordering unchanged and stretches every
`t_s` by `1/scale`. This is the standard retiming identity (`v(t) → v(t·scale)·scale`,
`a(t) → a(t·scale)·scale²` for `t → t/scale`). It is cheap and exact precisely because it never touches
positions, so it cannot introduce a new position or collision violation.

**The 10%-of-limits claim is a checked property, not an assumption.** `G-LIMITS` checks two things in
every mode, and both must hold:

1. the **unscaled** trajectory (scale 1.0, as written in the file) is within `b601_dm_limits.yaml`
   positions, velocities and accelerations; and
2. the **scaled** trajectory sent to the driver has every velocity `≤ scale · velocity_limit` and every
   acceleration `≤ scale² · acceleration_limit`.

(2) follows from (1) for exact uniform retiming, and it is evaluated anyway as a cross-check on the retiming
code. Together they guarantee that a `real` run at the shipped cap moves at no more than 10% of the
velocity limits and 1% of the acceleration limits. A file that is 10× over the limits is refused at (1),
whatever the scale. It cannot be "retimed into compliance".

### 5. Limits-hash policy

`config/robot/b601_dm_limits.yaml` is this project's single planning-limits source (MOT-01). Two
independent sha256 checks exist, for two independent purposes:

- `G-LIMITS-HASH`: the **trajectory's own** `limits_file.sha256` (§11.2), checked against the *current*
  `b601_dm_limits.yaml` in **every** mode whenever it is non-`null` — a trajectory produced against a
  different limits file is suspect even in `mock`, and catching that early is free. It may be `null` only
  in `mock`/`dry` (e.g. a hand-written `purpose: test` fixture with no MOT-07 provenance; reported `warn`);
  `real` refuses a `null` sha unconditionally, because real execution must always be traceable to the
  limits file it was planned against.
- `G-COMMISSIONING` (`real` only): the **commissioning record's** `limits_file_sha256` (§11.4) must equal
  the current `b601_dm_limits.yaml` — this proves the commissioning survey (speed cap, e-stop verification,
  joint-range checks) was performed against the limits file currently in force, not a stale one from before
  a since-revised limit.

These can disagree (a trajectory planned last week against limits that haven't changed since, versus a
commissioning record from before a limits edit) and both must independently hold for `real`.

### 6. Position-margin policy (handles the all-zero home)

The SRDF's all-zero `home` sits with `joint2`/`joint3` **exactly on their upper limit** (`0.0`,
`config/robot/b601_dm_limits.yaml`). Any policy that subtracts a safety margin from the hard bounds before
checking positions would make the vendor's own canonical rest pose fail the gate — clearly wrong. So:

- **The hard pass/fail bound is `[lower, upper]` inclusive**, exactly `b601_dm_limits.yaml`'s values, for
  every point's position in every mode. It is never narrowed inward. `home` passes.
- `position_margin_rad` (§11.12) is **not** subtracted from the bound. A point is `near_upper` when
  `upper − q ≤ position_margin_rad` and `near_lower` when `q − lower ≤ position_margin_rad`. Both flags are
  reported in the gate detail.
- **Approach rule (sign-correct for both bounds).** Define the *toward-bound speed* of a joint at a point:
  `w = +v` for the upper bound and `w = −v` for the lower bound. A near-limit point is a margin violation
  only if it is moving toward its bound (`w > 0`) **and not decelerating to it**: the next point's
  toward-bound speed is larger (`w_next > w`). The last point is a violation if its explicit velocity is
  given and `w > 0`; with no explicit velocity it ends at rest. Moving away from the bound (`w ≤ 0`) is
  always fine, and so is arriving at it at constant or falling speed. The rule is applied once to explicit
  velocities when present and once to forward finite differences (the per-segment velocity the vendor's
  linear interpolation actually commands, `ros_actions.py:147-184`).
- **Home-approach example.** `joint2` goes `−0.50 → … → −0.04, −0.02, −0.01, 0.00` at a constant
  0.10 rad/s per segment, with margin 0.03. The points at −0.02 and −0.01 are `near_upper` with `w = 0.10`,
  and the next segment's speed is also 0.10, not larger. The last point, 0.00, is on the inclusive bound,
  with explicit velocity 0 or none. **Pass.** A trajectory that *passes through* 0.00 (`−0.20, −0.01, 0.00,
  −0.01, −0.20`) also passes: at −0.01 the next speed is ≤ the current, and at 0.00 `w < 0` (moving away).
  Fail case: `−0.03 → −0.02` at 0.10 rad/s, then `−0.02 → 0.00` at 0.20 rad/s accelerates into the bound,
  so the point at −0.02 is a `position_margin` violation, though every position is within bounds.

### 7. Vendor evidence → e-stop behaviour decision: cancel and hold, never auto-disable

From Context: cancelling a `follow_joint_trajectory` goal makes the real driver hold the measured position
with torque on (`ros_actions.py:175-180` → `hardware_manager.py:237-241`). `disable` drops torque, and with
no brake evidence anywhere it is assumed to let the arm fall under gravity. The vendor's own shutdown
safe-homes or safe-parks *before* disabling (`hardware_manager.py:199-216`).

**Decision: on any software e-stop trigger (§11.8), the executor cancels the in-flight goal and leaves the
arm holding. It never calls `/{ns}/disable`, or any other vendor service.** After the cancel it monitors
`joint_states` until the arm has stopped (max per-joint `|Δq|` below `tolerances.start_state_rad` across a
`timeouts.joint_state_s` window, reached within `timeouts.cancel_settle_s`; §11.8). It records the stop in the execution record and exits 1. If the cancel is not
acknowledged, or the arm does not settle, or `joint_states` are stale (the driver may be dead), software can
do nothing more safely. The executor prints a **"PRESS THE HARDWARE E-STOP"** instruction and still exits 1.

Disabling torque is left to the operator, using the vendor's own sequence (safe_home/park, *then* disable),
or to the hardware e-stop. That stays so until MOT-09 confirms on the physical unit that `disable` brings the
arm to a safe rest. Rationale: an automatic torque drop at an arbitrary pose is exactly what the
possible-fall assumption forbids. Several triggers (a tracking-error spike, a `joint_states` hiccup) fire
while the tool may be 1 cm from the specimen (§8.2 trace clearance). Dropping torque there trades a stopped,
holding arm for an uncontrolled fall onto the specimen or the operator. `motion_runner.py:124-134`'s
cancel-then-disable is the vendor precedent, and it is **not** followed, for this reason. No trigger
disables.

**The hardware e-stop is the safety function. The `/crackvision/estop` and `/rebot_motion/estop` topics are
secondary, software-only conveniences** that this executor subscribes to for a fast, scriptable stop during
development and rehearsal — they depend on the ROS graph being alive, which a true emergency does not get
to assume. `commissioning.yaml.estop.kind` is fixed to `"hardware"` (§11.4): the record is making a claim
about the physical e-stop circuit MOT-09 tests by hand, not about either topic. `G-ESTOP` checks that
claim, not topic wiring.

### 8. Vendor motion surfaces this executor never touches, and who does

`execute_trajectory` sends goals on exactly one vendor interface, `/{ns}/follow_joint_trajectory`. It
**reads** `/{ns}/joint_states` and, for the `vendor` profile, the latched `/{ns}/arm_status`. It calls no
vendor service, sends no goal to any other vendor action and publishes on no vendor topic. In particular it
never calls `disable` (§7). The MOT-05.4 grep check enforces the motion-producing names.

| Vendor surface (Context) | Executor | Owner |
|---|---|---|
| `/{ns}/follow_joint_trajectory` (`ros_actions.py:27-35`) | sends the gated goal; cancels on e-stop | this executor |
| `/{ns}/joint_states` (`ros_publishers.py:17-27`) | reads (start state, tracking, staleness) | — |
| `/{ns}/arm_status` (`ros_publishers.py:37-47`) | reads (`vendor` profile, `G-START-STATE`: `enabled`, `state_machine`, `error_codes`) | — |
| `/{ns}/joints/<joint>/state` (`ros_publishers.py:28-36`) | not used | — |
| `enable` | never | MOT-09 operator procedure (bring-up), run once per session before any `real` execution. `execute_trajectory` requires the arm already enabled (`arm_status.enabled`, `G-START-STATE`) and refuses informatively if it is not, rather than enabling it itself |
| `disable` | never (§7) | operator, after safe_home/park, or the hardware e-stop; until MOT-09 confirms a safe rest |
| `safe_home`, `park` | never | operator procedure / session teardown tooling (future MOT-xx), never inside a gated execution run |
| `/{ns}/move_to_pose` action (`ros_actions.py:18-26`), `move_to_pose_ik` service | never | out of scope: this executor only follows pre-planned, pre-approved joint trajectories (§11.2) and never sends an ad hoc Cartesian/IK target |
| `/{ns}/joints/<joint>/cmd/{mit,pos_vel,vel}` (`motor_passthrough.py:19-46,71-82`) | never published; for `vendor`, any publisher on them refuses `G-GRAPH` (§2) | vendor low-level tooling only. No crackvision card publishes on them; one that needs to gets its own ADR |
| `gravity_compensation/{start,stop}` | never | GEOM-05 / GEOM-07 hand-guided calibration procedures (§3) |
| `set_zero`, `set_mode` | never | operator/commissioning tooling (MOT-09), one-time hardware setup, never per-execution |
| `gripper/{set,open,close}`, `/{ns}/gripper/command` action, `/{ns}/gripper/cmd/*` (`motor_passthrough.py:47-69,83-90`) | never | out of scope for MOT-05–.6 entirely: no card in this decomposition drives the gripper, and a future task that does gets its own gate |

### 9. Confirmation

`real` execution requires **both** the env var `CRACKVISION_ARM_REAL=1` (`G-ARM`) **and** a typed phrase,
`EXECUTE <first 8 hex chars of the trajectory's sha256>`, read from a controlling `/dev/tty`, exact match
after `strip()` (`G-CONFIRM`). The e-stop topics/location and the effective `speed_scale` are printed
immediately before the prompt. There is no flag, env var or stdin redirection that substitutes for either —
`G-ARM` checks the literal env var (not a generic "non-interactive override"), and `G-CONFIRM` opens
`/dev/tty` directly (not `sys.stdin`), refusing with exit 3 if no controlling terminal exists, exactly
mirroring `run_real_demo.sh:186-189`'s existing `[ ! -r /dev/tty ]` refusal. Binding the phrase to the
trajectory's own sha (not a fixed word) means a copy-pasted confirmation from a previous run's terminal
history cannot accidentally confirm a different trajectory.

### 10. Approval, eligibility and staleness

`crackvision.execution_approval/1` (§11.5) binds `approved_by`/`approved_utc`/preview artefacts to one
`trajectory_sha256`. `G-APPROVAL` fails if no approval file names the trajectory's sha, or if
`now - approved_utc` exceeds `execution.yaml`'s `approval_max_age_s` (§11.12). It refuses in `real`. In
`mock`/`dry` it is reported `warn`.

`G-ELIGIBLE` enforces the §10.6 rule **from the paths3d file itself**. The `execution_eligible` copy inside
the trajectory is provenance only and is never trusted. In `real`:
- a `purpose: crack_task` trajectory with `source.paths3d: null` is refused. A crack task must be traceable
  to its paths3d file. (`purpose: test` is already refused in `real` by `G-TRAJ`.)
- the referenced file is opened, and its sha256 must equal `source.paths3d.sha256`. Only then are
  `execution_eligible` and `ineligible_reasons` read from it, and `execution_eligible` must be `true`. The
  reasons are copied into the gate detail.
- a trajectory copy of `execution_eligible` that disagrees with the file also fails, because it shows the
  trajectory was produced from a different eligibility state.

So a trajectory generated from an eligible paths3d file that was later regenerated, or found newly
ineligible, cannot execute on the strength of a stale snapshot. A missing file or a sha mismatch refuses
in every mode (integrity). An ineligible file, a disagreeing copy or a null binding on a crack task is
`warn` in `mock`/`dry`. Every paths3d produced today is ineligible (calibration is nominal), and mock smoke
tests must still run.

`G-STALE-CONFIG` binds the trajectory to the configs it was planned against:
- **`real` refuses a `null` `end_effector_config_sha256` or `scene_config_sha256`.** An unbound
  trajectory cannot be checked for staleness, so it is not allowed near the real arm.
- **In every mode, a non-null sha that differs from the current file refuses** (exit 3). A trajectory
  planned against yesterday's end-effector or scene file must not run against today's, not even on a mock.
  The fix is to replan.
- **In `mock`/`dry`, a `null` sha is reported `warn`.** This is what lets unbound test fixtures (MOT-05.2)
  run.

### 11. Real-mode collision-validity topology

`real` mode's `G-COLLISION` runs the MOT-02 headless mock-planning stack (`mock_planning.launch.py`) purely
as a **validity oracle**: the production `config/scene/scene.yaml` is applied to it (`apply_scene`), and
every densified waypoint (§11.12) is checked via that stack's `/check_state_validity`, called with an
**explicit `RobotState`** for each query. In `real` mode the oracle's own mock controller
(`/rebotarm_controller/follow_joint_trajectory`) is never addressed — `G-GRAPH` (§2) only ever resolves
`/rebotarm/follow_joint_trajectory` for the `vendor` profile, and nothing in `execute_trajectory` holds an
`ActionClient` for the oracle's controller name. It does not matter that the oracle's own *monitored*
planning-scene state (what it would use for an implicit-state validity query) is its mock robot's state,
because every query this executor sends carries the real-driver's live joint state explicitly — the oracle
is being used only for its collision/self-collision geometry evaluation, not for any notion of "current
state" of its own. This is the stated caveat: **a real run's collision checking is only as good as the
explicit state it supplies**, which `G-START-STATE`/`G-COLLISION` source directly from `/rebotarm/joint_states`
and the densified trajectory points, never from the oracle's internal state.

## Alternatives

- **One combined mode flag instead of two independent axes (driver × plan).** Rejected: `~/rebot_ws`'s own
  `run_real_demo.sh` already uses two independent axes (`DRIVER=mock|real`, `PLAN=dry|go`) and that split is
  exactly right here too — `dry` must be usable against *either* a mock or the live vendor driver, which a
  single `mock|dry|real` enum cannot express as cleanly as a driver profile (§2) orthogonal to a gate-depth
  mode (§1). This ADR keeps the vendor shell script's two-axis shape but folds it into one `--mode` plus a
  `--profile` (still a separate selector; `--mode real --profile moveit_mock` is a usage error under the
  profile table in §2, which preserves the orthogonality without a combinatorial flag).
- **Trust the trajectory file's own `limits_file.sha256` alone, skip recomputing `G-LIMITS` numerically.**
  Rejected: a sha match only proves the trajectory was *planned* against the right file; it proves nothing
  about whether the point data, retiming (§4) or hand-edited test fixtures (`purpose: test`) actually
  respect those numbers. `G-LIMITS` always recomputes from the point data, unscaled and scaled.
- **Check limits only after speed scaling.** Rejected (MOT-05.1 review): an over-limit file retimed at
  0.10 can land exactly on the limits and pass. The real cap would then bound nothing relative to the
  limits (§4).
- **Clamp the requested speed scale to the cap.** Rejected: a silent clamp makes the executed motion differ
  from what the operator asked for and confirmed. Refusing with both numbers is explicit (§4).
- **Cancel then disable on e-stop (vendor `motion_runner.py` precedent).** Rejected: under the
  possible-fall assumption it turns a recoverable stop into an uncontrolled fall at an arbitrary pose (§7).
- **"Commissioning purpose tier" to unblock GEOM-05/GEOM-07/MOT-09 motion through this executor.** Not
  adopted — written up as an escalated alternative in Decision §3, explicitly requiring technical-lead
  approval before implementation, specifically so it is never the default path a future card reaches for
  under time pressure.
- **Confirmation via a `--yes`/`--force` flag for scripted real runs.** Rejected outright: REQ-INT-2 asks
  for operator-sourced evidence under controlled conditions, which a scriptable flag defeats by
  construction; the phrase must come from a human at a keyboard, on every single real run.

## Consequences

**Makes easy:**
- MOT-05.2–.6 implement against one frozen contract (§11) instead of re-deriving modes, schemas or gate ids
  per sub-card.
- `dry` mode gives operators a free, motion-free rehearsal against the *actual* vendor driver before MOT-09
  ever risks real motion. It previews the real speed scale and lists everything `real` would refuse.
- Mock smoke tests run with unbound fixtures, no approval and ineligible paths3d inputs (all `warn`), while
  integrity failures still stop them.
- GEOM-05/07/MOT-09 have an explicit, non-bypassing path forward despite the chicken-and-egg commissioning
  problem.

**Costs:**
- Three driver profiles, a per-mode refusal rule (`fail` vs `warn`) and two sha-hash policies
  (trajectory-level and commissioning-level) to keep straight. MOT-07/MOT-08/MOT-09 must each emit exactly
  the fields §11 defines, or the gates they feed will misreport.
- MOT-07 must emit trajectories that are within the limits at scale 1.0, bound to the limits, end-effector,
  scene and paths3d files, or `real` refuses them.
- After a software e-stop the arm is left energised and holding. A human must then bring it to rest (vendor
  safe_home/park, then disable) or press the hardware e-stop. That is deliberate (§7), and MOT-05.6 must
  document it as the recovery step.
- The `motorbridge` FFI boundary means this ADR cannot cite source for the last step of what `disable`
  actually does electrically — only for the ROS-visible call chain down to that boundary.

## Revisit when

- MOT-09 measures the real `disable` behaviour (confirms or refutes torque-off/gravity-fall). If `disable`
  provably brings the arm to a safe rest, an amendment may add an operator-confirmed disable step after the
  §7 hold.
- A "commissioning purpose tier" (§3) gets technical-lead approval and its own ADR amendment.
- A real hardware e-stop circuit is wired into something this project can read, making `G-ESTOP`
  verifiable in software rather than solely operator-attested.
- The vendor driver changes its cancel semantics (`ros_actions.py:175-180`) or its passthrough arbitration
  default (`rebotarm_controller.py:31`).

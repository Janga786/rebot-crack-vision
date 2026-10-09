# ADR-016: Commissioning-gated execution — modes, driver profiles and the real-motion gate catalogue

**Status:** accepted · **Date:** 2026-10-08 · **Supersedes:** — · **Superseded by:** —

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
done 2026-10-08:

- The real driver is node `reBotArmController`
  (`~/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:18-20`), namespace
  parameter `arm_namespace` defaulting to `rebotarm`
  (`rebotarm_controller.py:30,46`). It exposes:
  - `control_msgs/action/FollowJointTrajectory` at `/{ns}/follow_joint_trajectory`
    (`ros_actions.py:27-35`), the only interface this ADR's executor is allowed to drive motion through.
    It rejects a goal whose `joint_names` isn't exactly `hardware.joint_names`
    (`ros_actions.py:118-124`) and plans each segment as a time-ratio linear interpolation honouring each
    point's `time_from_start` (`ros_actions.py:147-184`).
  - `sensor_msgs/msg/JointState` on `/{ns}/joint_states`, published by `JointStatePublisher`
    (`ros_publishers.py:17-27`), the project's status readback.
  - Trigger/typed services at `/{ns}/`: `enable`, `disable`, `safe_home`, `park`,
    `gravity_compensation/{start,stop}`, `set_zero`, `set_mode`, `move_to_pose_ik`,
    `gripper/{set,open,close}` and the `gripper/command` `control_msgs/action/GripperCommand` action
    (`ros_services.py:20-97`, `ros_actions.py:36-44`). **Every one of these moves or arms the arm**, and
    none of them is `follow_joint_trajectory` — see Decision §6.
  - `/rebotarm/disable` → `HardwareManager.disable()` → `RobotArm.disable()` → per-vendor
    `ControllerManager.disable_all()` → `JointGroup.disable()`
    (`hardware_manager.py:280-288`, `rebotarm.py:243-252`), which bottoms out in a compiled motor-bridge
    FFI call (`motorbridge.core.Controller.disable_all`, outside `~/rebot_ws`, not further inspectable from
    source). Nothing in `~/rebot_ws` mentions a mechanical brake or holding clutch on any joint, and the
    driver's own shutdown path explicitly runs `safe_home` *before* `disable`
    (`rebotarm_controller.py:102-107`, params `safe_home_on_shutdown`/`disable_after_safe_home` both
    default `True`) rather than just cutting power in place — consistent with disable meaning "drop
    torque", which a non-self-locking joint does not survive under gravity without controlled support.
    **This ADR treats `disable` as torque-off with a possible uncontrolled fall under gravity** (the
    conservative assumption) until MOT-09's hands-on commissioning procedure confirms otherwise on the
    physical unit; nothing here is allowed to assume a brake exists.
  - `~/rebot_ws/deploy/run_real_demo.sh` and `preflight.sh` already implement the shape this ADR adopts:
    two independent axes (driver mock|real × plan dry|go, `run_real_demo.sh:29-33`), a REAL execute
    refused without both `REAL=1` and a typed `yes` on `/dev/tty` (`run_real_demo.sh:170-192`), a `trap`
    teardown on any exit that e-stops, parks, disables and kills children
    (`run_real_demo.sh:46-70`), and the existing `/rebot_motion/estop` convention. `motion_runner.py`'s own
    e-stop handler cancels the active goal **then** calls `/{ns}/disable`
    (`motion_runner.py:124-134`) — i.e. the vendor-stack precedent for "what does software e-stop do" is
    cancel-then-disable, not hold-in-place. This ADR's executor follows the same order (Decision §7).

None of this vendor evidence says anything about a *hardware* e-stop circuit (a physical button wired to
motor power, independent of any ROS node). The project does not control that circuit and must not pretend
software can substitute for it.

## Decision

### 1. Modes and the CLI default

Three modes: **`mock`** (goals only ever reach a mock driver), **`dry`** (every offline gate plus every
online read-only gate runs; no `ActionClient` for `follow_joint_trajectory` is ever constructed, so it is
safe to point at the live vendor driver as a rehearsal; it prints what `real` would additionally refuse)
and **`real`**. `execute_trajectory`'s `--mode` default is **`dry`** — the safest non-refusing choice, so
an operator who forgets the flag gets a full report and no motion, never a silent `mock` success that looks
like a real rehearsal.

`--dry-run` (§0.3, universal) is a different, stronger thing: offline-only gate report, no ROS node, no
file writes, exit 0 regardless of gate outcome. `--mode dry` runs the online gates too (it needs ROS) and
always writes the §11.9 execution record, including on refusal. A CLI invocation must not conflate the two;
`--dry-run --mode real` is accepted (and still only runs the offline gates).

### 2. Driver profiles (`config/motion/execution.yaml`, §11.1)

| Profile | Action | `joint_states` | Expected server node | Valid modes |
|---|---|---|---|---|
| `moveit_mock` | `/rebotarm_controller/follow_joint_trajectory` | `/joint_states` | ros2_control mock hardware (MOT-02) | mock, dry |
| `vendor_mock` | `/rebotarm/follow_joint_trajectory` | `/rebotarm/joint_states` | `mock_rebotarm_driver` | mock, dry |
| `vendor` | `/rebotarm/follow_joint_trajectory` | `/rebotarm/joint_states` | `reBotArmController` | dry, real |

Graph discrimination (gate `G-GRAPH`, read-only `rclpy` graph introspection — `get_action_server_names_and_types`/
node-name lookups, never an `ActionClient.send_goal`): a `mock`/`vendor_mock` run refuses if a node named
`reBotArmController` is visible anywhere on the graph; a `real`/`vendor` run refuses unless the action server
backing `/rebotarm/follow_joint_trajectory` is hosted by a node named exactly `reBotArmController` **and**
no node named `mock_rebotarm_driver` is visible. This is a structural proof, re-checked at every run, not a
configuration-file promise: a mock run physically cannot address the real driver (its profile's action name
and expected-node check point at the mock), and a real run physically cannot be satisfied by a mock driver
masquerading on the same names (it would fail `G-GRAPH` on the node-name mismatch or the mock-node
exclusion). `vendor_mock` exists because `rebot_motion.mock_driver` (`~/rebot_ws` install, node
`mock_rebotarm_driver`, `~/rebot_ws/src/rebot_motion/rebot_motion/mock_driver.py:36-65`) is
interface-identical to the real driver on the `/rebotarm/...` names, which lets a rehearsal exercise the
*exact* names and node-identity checks `real` will use, not a second, different mock namespace.

### 3. The real-mode commissioning calls

`execute_trajectory` in `real` mode calls both `crackvision_motion.scene_core.assert_commissioning_ready`
and `crackvision_description.end_effector.assert_commissioning_ready` on the loaded scene/end-effector
configs, with **no except clause that swallows either exception** — a `SceneNotCommissionedError` or
`EndEffectorNotCalibratedError` propagates as a `G-SCENE`/`G-EE` gate failure (§11.7) and the process exits
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
service `execute_trajectory` never calls — Decision §6), an operator physically guiding the arm by hand
while the vendor controller only cancels gravity torque, logged as its own evidenced procedure outside this
executor. `execute_trajectory` is not involved and its gates never run during that work, because no
`crackvision.joint_trajectory/1` file is being executed. MOT-09 is explicitly scoped to **readback-only dry
rehearsals**: `execute_trajectory --mode dry` against the live vendor driver, which runs every gate except
the four real-only ones (§11.7) and moves nothing, giving the operator the joint-readback and limits
evidence MOT-09 needs to populate `commissioning.yaml` without ever constructing a `FollowJointTrajectory`
goal. Only once `commissioning.yaml` says `commissioned: true` (which itself requires `end_effector.yaml`
and `scene.yaml` to already be fully `measured` — §11.4) does `--mode real` become reachable at all, at
which point GEOM-05/07 are already satisfied and the ordering problem has resolved itself in the only
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

`real` mode's effective `speed_scale` is `min(requested_scale, commissioning.speed_scale_cap)`, and
`commissioning.yaml` ships with `speed_scale_cap: 0.10` (§11.4) — i.e. **real execution is capped at 10% of
the `b601_dm_limits.yaml` velocity/acceleration limits until an operator raises the cap in the commissioning
record with evidence** (a logged low-speed run, per MOT-09). `mock`/`dry` use
`execution.yaml`'s `default_speed_scale`/`speed_scale_cap` (§11.1), independent of the commissioning record,
since no real torque is ever at stake there. `G-SPEED` (§11.7) fails if the requested scale exceeds the
mode-appropriate cap, in every mode.

**Retiming is uniform time scaling**: given a trajectory already respecting the limits at `scale = 1.0`,
applying `scale ∈ (0, 1]` multiplies every point's velocities by `scale` and accelerations by `scale²`,
leaving positions and point ordering unchanged and stretching `time_from_start` by `1/scale`. This is the
standard trapezoidal/TOTG retiming identity (`v(t) → v(t·scale)·scale`, `a(t) → a(t·scale)·scale²` for
`t → t/scale`) and is cheap and exact precisely because it never touches positions — it cannot introduce a
new limit or collision violation that wasn't already present at `scale = 1.0`. `G-LIMITS` is evaluated
**after** this scaling is applied, never before.

### 5. Limits-hash policy

`config/robot/b601_dm_limits.yaml` is this project's single planning-limits source (MOT-01). Two
independent sha256 checks exist, for two independent purposes:

- `G-LIMITS-HASH`: the **trajectory's own** `limits_file.sha256` (§11.2), checked against the *current*
  `b601_dm_limits.yaml` in **every** mode whenever it is non-`null` — a trajectory produced against a
  different limits file is suspect even in `mock`, and catching that early is free. It may be `null` only
  in `mock`/`dry` (e.g. a hand-written `purpose: test` fixture with no MOT-07 provenance); `real` refuses a
  `null` sha unconditionally, because real execution must always be traceable to the limits file it was
  planned against.
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

- **Hard pass/fail bound is `[lower, upper]` inclusive**, exactly `b601_dm_limits.yaml`'s values, for every
  point's position in every mode — no inward narrowing, ever. `home` passes trivially.
- `position_margin_m` (really `_rad` for revolute joints; §11.12) is **not** subtracted from the bound. It
  is used only to (a) flag a point as `near_limit: true` in the gate detail when it is within
  `position_margin_rad` of either bound — informational, never changes pass/fail, so an operator reviewing
  a `dry` report sees that a waypoint is riding a limit — and (b) require that any point flagged
  `near_limit` have a velocity in that joint that is `<= 0` moving toward the limit it is near (not
  accelerating further past it). `home`'s trajectory points, as start/end states, have zero velocity in
  that joint and pass (b) trivially; a hypothetical mid-trajectory point approaching the limit with nonzero
  velocity *into* it fails (b) even though it still passes the hard bound in (a).

### 7. Vendor evidence → e-stop behaviour decision

From Context: the real driver's `disable` is assumed torque-off-with-possible-fall (no brake evidence
anywhere in `~/rebot_ws`), and the vendor stack's own precedent for a software e-stop
(`motion_runner.py:124-134`) is **cancel the active goal, then call `/{ns}/disable`** — not "hold in place."
This executor adopts the same order for consistency with the only prior art in this codebase: on any e-stop
trigger (§11.8), it cancels the in-flight `follow_joint_trajectory` goal and then calls `/{ns}/disable`.
Holding in place is not chosen because nothing in the vendor stack demonstrates a hold-in-place primitive
independent of the trajectory executing normally (cancelling a `FollowJointTrajectory` goal simply stops
commanding further interpolation — see `ros_actions.py:175-180`, which calls `hold_current_position()` on
cancel, so "cancel" already *is* "hold," and the subsequent `disable` is the deliberate, logged step that
drops torque rather than leaving the arm silently enabled and stationary with no supervisor attached).

**The hardware e-stop is the safety function. The `/crackvision/estop` and `/rebot_motion/estop` topics are
secondary, software-only conveniences** that this executor subscribes to for a fast, scriptable stop during
development and rehearsal — they depend on the ROS graph being alive, which a true emergency does not get
to assume. `commissioning.yaml.estop.kind` is fixed to `"hardware"` (§11.4): the record is making a claim
about the physical e-stop circuit MOT-09 tests by hand, not about either topic. `G-ESTOP` checks that
claim, not topic wiring.

### 8. Vendor motion-producing services this executor never calls

`enable`, `safe_home`, `park`, `move_to_pose_ik`, `gravity_compensation/{start,stop}`, `set_zero`,
`set_mode`, `gripper/{set,open,close,command}` — every vendor service/action in Context's bullet list except
`follow_joint_trajectory` and the readback-only `joint_states`. `execute_trajectory` calls `disable`
(e-stop/teardown only, §7) and nothing else that arms or moves the arm outside `follow_joint_trajectory`.
Ownership of the excluded operations:

| Operation | Owner |
|---|---|
| `enable` | MOT-09 operator procedure (bring-up), run once per session before any `real` execution; `execute_trajectory` requires the arm already enabled (`G-COLLISION`/`G-START-STATE` need live, moving joint state) and fails informatively if it is not, rather than enabling it itself |
| `safe_home`, `park` | Operator procedure / session teardown tooling (future MOT-xx), never inside a gated execution run |
| `move_to_pose_ik` | Out of scope — this executor only ever follows pre-planned, pre-approved joint trajectories (§11.2), never computes or sends an ad hoc IK pose |
| `gravity_compensation/{start,stop}` | GEOM-05 / GEOM-07 hand-guided calibration procedures (§3) |
| `set_zero`, `set_mode` | Operator/commissioning tooling (MOT-09), one-time hardware setup, never per-execution |
| `gripper/*` | Out of scope for MOT-05–.6 entirely — no card in this decomposition drives the gripper; a future task that does gets its own gate |

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

### 10. Approval and staleness

`crackvision.execution_approval/1` (§11.5) binds `approved_by`/`approved_utc`/preview artefacts to one
`trajectory_sha256`. `G-APPROVAL` fails if no approval file names the trajectory's sha, or if
`now - approved_utc` exceeds `execution.yaml`'s `approval_max_age_s` (§11.12). `G-ELIGIBLE` re-checks the
§10.6 `paths3d` rule directly: when the trajectory's `source.paths3d` is present, its recorded
`execution_eligible` must be `true` and its `sha256` must match the paths3d file's current sha256 — a
trajectory generated from an eligible paths3d file that was later regenerated (or found newly ineligible)
must not execute on the strength of a stale snapshot. `G-STALE-CONFIG` independently refuses `real` if the
trajectory's `end_effector_config_sha256`/`scene_config_sha256` (when non-null) don't match the *current*
`config/robot/end_effector.yaml`/`config/scene/scene.yaml` sha256 — a trajectory planned against yesterday's
end-effector or scene file must not run against today's, even if everything else passes.

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
  `--driver-profile` (driver profile is still a separate selector; `--mode real` with `--driver-profile
  moveit_mock` is rejected by the profile table in §2, which is the orthogonality preserved without a
  combinatorial flag).
- **Trust the trajectory file's own `limits_file.sha256` alone, skip recomputing `G-LIMITS` numerically.**
  Rejected: a sha match only proves the trajectory was *planned* against the right file; it proves nothing
  about whether retiming (§4) or hand-edited test fixtures (`purpose: test`) actually respect those numbers.
  `G-LIMITS` always recomputes from the point data.
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
  ever risks real motion.
- GEOM-05/07/MOT-09 have an explicit, non-bypassing path forward despite the chicken-and-egg commissioning
  problem.

**Costs:**
- Three driver profiles and two sha-hash policies (trajectory-level and commissioning-level) to keep
  straight; MOT-07/MOT-08/MOT-09 must each emit exactly the fields §11 defines or the gates they feed will
  misreport.
- The torque-off-on-disable assumption (§7) is conservative pending physical confirmation; if MOT-09 finds a
  real brake, the e-stop ordering in §7 can be loosened in a future ADR amendment but should not be
  loosened before that evidence exists.
- The `motorbridge` FFI boundary means this ADR cannot cite source for the last step of what `disable`
  actually does electrically — only for the ROS-visible call chain down to that boundary.

## Revisit when

- MOT-09 measures the real `disable` behaviour (confirms or refutes torque-off/gravity-fall) and finds it
  disagrees with §7's conservative assumption.
- A "commissioning purpose tier" (§3) gets technical-lead approval and its own ADR amendment.
- A real hardware e-stop circuit is wired into something this project can read, making `G-ESTOP`
  verifiable in software rather than solely operator-attested.
- The vendor driver gains a documented hold-in-place or braking primitive, which would let §7's e-stop
  ordering change from cancel-then-disable to something gentler.

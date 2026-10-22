# Safety case (mock/dry, no hardware)

## Scope

This document is the index for the project's mock/dry failure-injection safety case: it
ties each of six fault classes to the mechanism that refuses it, the evidence that this
refusal was exercised, and the single command an operator runs to reproduce the whole
suite.

**Scope boundary**: everything in this document runs headless, against synthetic data and
mock stacks (MoveIt mock planning, `rebot_motion`'s mock vendor driver), and **never**
drives real hardware. `CRACKVISION_ARM_REAL` is never set by any script this document
references. This satisfies the mock/dry half of **REQ-INT-2** (separately gated hardware
commissioning) and the failure-recovery half of **REQ-OPS-1** (operator workflow with
failure recovery) for these six fault classes.

REQ-INT-2 also requires a *separately gated* hardware-commissioning path — verified
limits, collision checks, e-stop, under controlled conditions, with operator-sourced
evidence. That path is **not** exercised here: it is the responsibility of the operator
cards **GEOM-05**, **GEOM-07**, **INT-04** and **INT-05**, which run on real hardware under
the commissioning gate and produce their own operator-witnessed evidence.

## Reproduce the whole suite

```
bash scripts/safety/run_safety_case.sh
```

This builds `ros2_ws/install` first if it is missing (`scripts/ros/build_ws.sh`), then runs
the five stages below in order, stopping and reporting the failing stage on the first
non-zero exit, and otherwise prints a one-line PASS/FAIL summary per fault class and exits
0.

## Fault classes

| Fault class | Injection method | Refusal mechanism | Exit-code / outcome contract | Evidence |
|---|---|---|---|---|
| Empty mask (no crack detected) | `scripts/perc/test_safety_empty_mask.sh` overwrites the stand-in prediction mask with an all-zero PNG of the correct `(height, width)` and runs `crackvision.skeleton` → `crackvision.paths` → `crackvision.path3d` on it. | No gate refusal: a crack-free prediction is a legitimate outcome, so all three CLIs take their own designed `EXIT_OK` path and report an honest, schema-valid empty result (`skeleton_pixels=0`, `paths.json components=[]`, `paths3d.json components=[]`, `execution_eligible=false`, `no_valid_points` in `ineligible_reasons`). | All three CLIs: exit 0 (`EXIT_OK`). | [docs/integration/safety/EMPTY_MASK.md](safety/EMPTY_MASK.md) |
| Invalid depth (no-return / zero depth) | `scripts/perc/test_safety_invalid_depth.sh` overwrites a real synthetic case's depth PNG with the device's uint16 zero no-return sentinel (100% invalid, and a partial 24-row band) and runs `crackvision.path3d` on each. | `path3d`'s own per-point depth validity check (never fabricates a 3D point from invalid depth): each invalid pixel is reported `valid=false` with a populated `reason` and null `p_optical_m`/`p_base_m`/`n_base`; the §10.3 gap-interpolation policy only fills small bounded gaps and still marks those points `valid=true` with a non-null, finite coordinate. | `crackvision.path3d` (both runs): exit 0 (`EXIT_OK`) — a per-point invalidity is not a batch failure. | [docs/integration/safety/INVALID_DEPTH.md](safety/INVALID_DEPTH.md) |
| Unreachable path | `scripts/ros/test_safety_unreachable.sh` translates the synthetic specimen/crack +5.0 m along base_link's X axis, far outside `docs/motion/REACHABILITY.md`'s documented envelope, then runs `plan_view` and `plan_crack_path` against it on the headless mock MoveIt stack. | `cli_common.py`'s `PreconditionError` → `EXIT_PRECONDITION` mapping: no IK candidate/trace waypoint can be found, so both CLIs refuse cleanly instead of fabricating or silently truncating a plan; no `view_plan.json`/`cartesian_plan.json` is written. | `plan_view`, `plan_crack_path`: exit 3 (`EXIT_PRECONDITION`). | [docs/integration/safety/UNREACHABLE_PATH.md](safety/UNREACHABLE_PATH.md) |
| Stale calibration/config | `scripts/ros/test_execution.sh`'s offline-mode phases exercise `execute_trajectory` with end-effector/scene config sha256 mismatches and nulls, per the driver-profile matrix. | `G-STALE-CONFIG` (`ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py`, `docs/INTERFACES.md` §11.7): a non-null config sha256 mismatch refuses in every mode; a null sha256 warns in mock/dry and refuses in real. Evaluated alongside the other offline gates (`G-ARM`, `G-TRAJ`, `G-LIMITS-HASH`, `G-SCENE`, `G-EE`, `G-COMMISSIONING`, `G-ESTOP`, `G-APPROVAL`) every run, independent of ordering (§11.7: an earlier gate's refusal never skips a later one). | Offline refusal: exit 3, `outcome=refused`, refusal id set includes `G-STALE-CONFIG` as documented. | [docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md](safety/STALE_CALIBRATION_AND_LIMITS.md) |
| Limit violations | `scripts/ros/test_execution.sh` checks a trajectory's positions, velocities and accelerations twice (unscaled and scaled), re-deriving limits independently of the offline trajectory checker. | `G-LIMITS` (`execution_gate.py`, `docs/INTERFACES.md` §11.7/§11.6 margin policy): any out-of-limit trajectory is refused, independent of and cross-checked against the offline trajectory-checker's own limit faults (MOT-07.8's 7 injected faults). | Offline refusal: exit 3, `outcome=refused`, refusal id set `{G-LIMITS}` (or superset), no ROS contact attempted. | [docs/integration/safety/STALE_CALIBRATION_AND_LIMITS.md](safety/STALE_CALIBRATION_AND_LIMITS.md) |
| Any bad motor command (garbage IK, branch/wrist flip, out-of-range target, overspeed, excessive gains/feed-forward, keep-out intrusion, stale/faulted motor, an encoder a whole turn off at startup), from any caller | `scripts/safety/test_arm_guard.sh`: fuzzes 180 000 random/adversarial commands, runs closed-loop hostile-planner simulations through the patched `motorbridge` classes, replays all 34 real logged runs, checks the startup check against every start logged 2026-10-14..21 and the offline pre-flight against the live guard, and exercises the fail-closed start-up hook. | `arm_guard` ([docs/motion/ARM_GUARD.md](../motion/ARM_GUARD.md)): the last-line filter installed in every env that can reach the motors; a violating command is never sent, the joint is held and the guard latches. | Violations: hold command sent, `GuardFault` raised, latched until `reset()`; setup/permission violations: `GuardRefused`, nothing sent. Script exit 0 when all 201 tests pass and the hook is active in `rebot`/`lerobot`. | [docs/motion/ARM_GUARD.md](../motion/ARM_GUARD.md#verification) |

## Traceability

- **REQ-INT-2** (mock/dry half): all five rows above are exercised entirely in simulation/mock
  (synthetic perception data, headless MoveIt mock stack, `rebot_motion`'s mock vendor
  driver) with no real hardware ever addressed. The real-hardware half of REQ-INT-2
  (verified limits, collision checks, e-stop, controlled conditions, operator-sourced
  evidence) is out of scope here and is carried by GEOM-05, GEOM-07, INT-04 and INT-05.
- **REQ-OPS-1** (operator workflow with failure recovery): each row documents a clean,
  predictable failure mode (a specific exit code and refusal/outcome contract, never a
  crash, hang or silent wrong answer) for its stage of the capture → plan → preview →
  execute workflow, so an operator can recognize and recover from each fault class.

## Scripts referenced

- `scripts/perc/test_safety_empty_mask.sh`
- `scripts/perc/test_safety_invalid_depth.sh`
- `scripts/ros/test_safety_unreachable.sh`
- `scripts/safety/test_arm_guard.sh`
- `scripts/ros/test_execution.sh`
- `scripts/safety/run_safety_case.sh` (orchestrates the four above, in this order)

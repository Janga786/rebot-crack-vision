# arm_guard — the last-line motor-command filter

`packages/arm_guard` sits directly in front of the motor library (`motorbridge`) in every Python process that can
reach the B601-DM's motors. Every command, whoever produced it (MoveIt/TRAC-IK, the vendor SDK's own IK, the
vendor ROS driver, LeRobot, an operator script, a bug), is checked against one approved set of limits before it
reaches the CAN bus. A command that breaks a rule is never sent: the guard **holds the arm** and latches.

Built 2026-10-21. Status: workstation-only. Verified offline against a fake bus, the real `motorbridge` classes
(no bus opened), every real run logged on the arm, and full rehearsals of the operator's controller
(`~/rebot_setup/demo_move.py`) running trajectories through it; **not yet run on the powered arm**. To run the
first trajectory on the arm, follow [FIRST_GUARDED_RUN.md](FIRST_GUARDED_RUN.md). The open items are in
[`TODO.md`](../../TODO.md).

## What it guarantees, and what it doesn't

**Guarantee.** For every command that goes through `motorbridge.Motor` in a process where the guard is installed,
and for **any** input whatsoever, the command that reaches the bus:

- has finite values only;
- targets a position inside the configured joint range, or holds a rest pose that already sits past it (≤ 0.05 rad,
  J2 ≤ 0.50 rad drooped) or moves inward from it, never further out;
- is at most `max_step` (0.15 rad, gripper 0.3) from the joint's **measured** position, so an IK branch flip,
  wrist flip or garbage solution cannot make a joint jump;
- asks for a speed the joint can still stop from before its limit (`v ≤ min(vel_cap, √(2·a·distance))`);
- uses MIT gains and feed-forward within their caps (kp, |τ_ff|; kd above 5 is lowered to 5, which the MIT
  encoder does anyway);
- keeps the arm's keep-out margins (table plane, base column, upper arm), or at least does not make an existing
  violation worse;

and only while the motor's measured state is healthy (no fault code, position within range, speed and torque
below their abort levels, feedback fresher than 0.25 s), and only after the startup check passed: at the first
enable of a session, every joint read inside its window around the fold (J1 unwrapped by whole turns). This is enforced by `core.py`'s pure functions and proven
by the tests below, including closed-loop fuzzing with a hostile planner (no joint ever left its range, no speed
cap was exceeded, keep-out margins never fell below −10 mm).

**It does not guarantee physical safety by itself.** That still depends on:

1. **The limits being right.** The joint ranges are the vendor URDF's and the keep-out geometry is a coarse
   nominal proxy: neither is measured on this arm yet (MOT-09, MOT-10; [`TODO.md`](../../TODO.md) item 2).
2. **The encoders being right.** The startup check (S1) catches a joint that reads a whole turn off or was re-zeroed
   somewhere else: it must read within its window (±0.4 rad around the fold; J1 unwrapped by whole turns). A
   reading that is wrong by less than its window would still pass.
3. **The host being alive.** If the process dies, the motors keep their last (accepted) command forever because
   the motors' CAN timeout is off (`TIMEOUT = 0`): [`TODO.md`](../../TODO.md) item 1.
4. **Nobody going around it.** Remaining bypasses: calling motorbridge's C ABI directly
   (`get_abi().lib.motor_handle_send_*`), the Rust tools `ws_gateway`/`motor_cli` (never load Python), and an
   interpreter without the hook (see *Install*). The 24 V switch remains the only hardware e-stop.

## How it fits with the other safety layers

| Layer | What | Role |
|---|---|---|
| Planner | MoveIt + TRAC-IK, mesh collision, `b601_dm_limits.yaml` (1.0 rad/s, 0.03 rad margin) | makes good plans; convenience, not a guarantee |
| Validator | `trajectory_check.py`, `execution_gate.py` (G-LIMITS, G-COLLISION, …), typed approval | rejects a bad plan before execution |
| **Command filter** | **`arm_guard` (this)**, every command, every process | **catches whatever got past the layers above, or never went through them** |
| Motor firmware | over-current / over-temperature / under-voltage → motor disables; CAN timeout (to be set) | survives a crashed host |

The guard's limits sit slightly *outside* the planner's (e.g. 1.2 rad/s vs 1.0) so that normal planned motion never
trips it. This follows the pattern industrial controllers use (UR, FANUC DCS, ABB SafeMove): an independent checker
of both command and measured state, a checksummed configuration, latched faults reset only by a person.

## Rules

| | Rule | On violation |
|---|---|---|
| R1 | every number finite | fault |
| R2 | target in `[lower, upper]`; a rest pose past a limit (≤ the joint's `rest_tolerance`: 0.05, J2 0.50) may be held or left inward, never pushed further. The allowance past the limit is kept for the session and shrinks with the accepted targets, never with the measured position (which may lead the reference into the range). It never shrinks below the fold rest band (global 0.05), so a joint can always return to its fold rest reading, but never back into a deep droop | fault |
| R3 | `|target − measured| ≤ max_step` | fault |
| R4 | speed ≤ `min(vel_cap, max(0.05, √(2·accel_stop·d)))`, `d` = distance to the limit ahead | MIT/VEL: fault. POS_VEL/FORCE_POS: `vlim` lowered |
| R5 | MIT `kp ≤ kp_max`, `|τ_ff| ≤ tau_ff_max`; `kd ≤ kd_max` | kp/τ: fault. kd: lowered |
| R6 | keep-out: `gripper_link` z ≥ 0.12 m (the 2026-10-17 demo's rule), `link4–6` z ≥ 0.06–0.08 m, wrist/gripper outside a 0.09 m column around the base below 0.13 m, and ≥ 0.06 m from the upper arm | fault |
| R7 | measured state: status ENABLED/DISABLED, position within range ± rest tolerance, `|vel| ≤ vel_abort`, `|torque| ≤ torque_abort`, feedback age ≤ 0.25 s (identical frames count as fresh only if a register read succeeds: the guard's own, or the caller's) | fault |
| S1 | startup check, before a session's first enable (all motors on the bus disabled): every joint inside its `[startup.window]` (fold ± 0.4 rad; J2 −0.4…+0.5, J7 −0.6…+0.1); J1 (`turn_ambiguous`) unwrapped by whole turns, and every later check uses that offset (`raw = physical + 2πk`) | enable refused |
| — | VEL mode (`ensure_mode(VEL)`, `send_vel`) | refused (keeps spinning if commands stop) |
| — | `enable()`: needs no latch, a live link, PMAX/VMAX/TMAX matching the model (else MIT commands are mis-scaled), a non-VEL mode, and R7 | refused |
| — | `set_zero_position`, `store_parameters`, `set_can_timeout_ms`, register writes | refused unless inside `with arm_guard.config_writes("reason"):` with that motor DISABLED; the vendor SDK's loop-gain writes (RID 25–28, bounded) are allowed |
| — | any other motorbridge method not on the allowlist (RobStride/other vendors, unknown future methods) | refused |
| — | a second process opening the same bus | refused (exclusive lock in `/tmp/arm_guard_locks`) |

Only two adjustments are ever made, both toward slower/softer: `vlim` and `kd` lowered. A position target is never
moved: clamping would bend the path into one nobody planned.

**On a fault** the offending joint immediately gets a *hold* command at its measured position (MIT: the joint's
tuned hold gains, zero velocity, feed-forward capped; POS_VEL: `vlim` 0.2), then `GuardFault` is raised. The guard
latches: every later motion command on every joint is turned into a hold at the position latched at the fault, and
`enable()` is refused, until the process restarts or an operator calls `arm_guard.reset("reason")`. The guard never
disables a motor itself: the arm is gravity-loaded and drops when disabled (ADR-016 §7). Every fault, reset,
adjustment and configuration write goes to `~/.local/state/arm_guard/events.jsonl`.

The exceptions derive from `PermissionError`, not motorbridge's `CallError`, so the vendor SDK's
`except CallError: pass` cannot swallow them.

## Configuration and approval

- Limits: [`config/robot/arm_guard.toml`](../../config/robot/arm_guard.toml) (sources for every value in its header).
- Approval: `config/robot/arm_guard.approved.json` holds the SHA-256 of that file and of the URDF. If either
  changes, the guard refuses **all** motion until a person re-approves:
  ```
  cd packages/arm_guard/src && ../../../env.sh python -m arm_guard check       # what is enforced now
  cd packages/arm_guard/src && ../../../env.sh python -m arm_guard approve --by <name> --note "<why>"
  ```
- Widening a joint range beyond the URDF needs an explicit `beyond_urdf_reason` in the config (e.g. the "show"
  demo's J1 at −3.76 rad, which the guard currently refuses).
- A config that is missing, malformed, inconsistent or unapproved means: every guarded call raises
  `GuardConfigError`, so nothing moves. `disable()` and reads still work.

## Install

The guard loads at interpreter start through a one-line `arm_guard.pth` in the env's `site-packages`. If the
guard cannot load, `import motorbridge` fails instead of running unguarded.

```
cd packages/arm_guard/src
../../../env.sh python -m arm_guard install-hook          # rebot, lerobot (+ crackvision if present)
../../../env.sh python -m arm_guard hook-status           # verifies each env: hook present, classes patched, config ok
../../../env.sh python -m arm_guard uninstall-hook        # removes it
```

Installed 2026-10-21 in `~/miniconda3/envs/rebot` (motorbridge 0.5.6) and `~/miniconda3/envs/lerobot` (0.5.5).
The vendor ROS driver also imports it explicitly and refuses to start without it (patch to
`~/rebot_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`, local commit in `~/rebot_ws`, copy in
[`packages/arm_guard/vendor_patches/`](../../packages/arm_guard/vendor_patches)). The guard runs on Python 3.10
(ROS Humble's interpreter, via `tomli`) through 3.12, with the standard library only.

Operator API: `arm_guard.status()` (includes J1's whole-turn offset), `arm_guard.reset("reason")`,
`arm_guard.config_writes("reason")`, `arm_guard.preflight.check_plan(cfg, readings, ticks)` (would the guard accept
this command stream?), `python -m arm_guard events`. Cost: about 0.55 ms per 7-motor command cycle (the keep-out
FK runs on every arm-joint command).

## Compatibility with existing code

| Caller | Under the guard |
|---|---|
| Operator tuning scripts (`first_move_j1.py`, `demo_move.py`) | all 33 logged runs replay clean (gains ≤ 100, |τ_ff| ≤ 10, steps ≤ 0.09 rad). The **"show" demo is refused**: it uses J1 at −3.76 rad, beyond the URDF ±2.8, and at 2× J5 measured 2.62 rad/s, above the 2.5 abort |
| `demo_move.py` 2026-10-21.1 / `run-trajectory.sh` | guard-aware: refuses a real run without the guard; checks every planned command offline first (`arm_guard.preflight`) and refuses before enabling; on a guard fault, holds and waits for the operator (`stop` = reset + slow park, `off` = switch off), never parks past the guard or disables on its own. `--trajectory` runs `crackvision.joint_trajectory/1` files. [FIRST_GUARDED_RUN.md](FIRST_GUARDED_RUN.md) |
| Vendor SDK, POS_VEL | `vlim` 5.0/3.0 lowered to 1.2/2.0 rad/s; `move_to_pose`'s single jump to an IK solution is refused by R3 (the motor would have run there at 5 rad/s) |
| Vendor SDK, MIT | kd 8 → 5 (the encoder did this silently before) |
| Vendor ROS driver | additionally refuses `safe_park_q` (joint3 +0.136, past its limit: VENDOR_DRIVER.md V-SAFE-PARK). The driver still cannot start for its own pre-existing reasons (V-SDK-API, V-PYDEPS) |
| LeRobot follower | kd 12 → 5; `calibrate()`'s `set_zero_position` refused outside `config_writes()`; its soft limits (−200°…1°) are wider than the guard's, so the guard's apply |

## Verification

`bash scripts/safety/test_arm_guard.sh` (stage 5 of `scripts/safety/run_safety_case.sh`) runs 201 tests:

- `test_core.py`: every rule, edges included (52).
- `test_runtime_patch.py`: the guard installed on a fake motorbridge with simulated joints: allowlist, enable gate,
  latch + hold, keep-out trip, stale feedback, fault codes, config writes, bus lock, no-config behaviour (34).
- `test_fuzz.py`: 180 000 random/adversarial commands, each accepted one re-checked against every rule
  independently; closed-loop runs (MIT and POS_VEL, 500 Hz) where a hostile "IK" sends random branches, dives,
  NaN, absurd gains and overspeed to every joint; a smooth planner heading for goals inside the table/arm.
- `test_replay.py`: all 34 real runs from the 2026-10-14/17 tuning sessions (33 accepted, the "show" demo refused).
- `test_config.py`: validation and the approval gate. `test_autoload.py`: the start-up hook (including fail-closed
  paths) and the real installed motorbridge in `rebot`/`lerobot` fully classified.
- `test_startup.py`: S1 against every start logged 2026-10-14…21, J1 whole turns (refused when its range would leave
  the MIT position mapping), J2 droop, the session allowance (no outward ratchet, the 2026-10-21 rehearsal's fold
  return where the measured position leads the reference, float rounding at an exact limit), caller register reads
  as liveness. `test_preflight.py`: the offline mirror, including that whatever it passes the live guard passes.

The operator's controller has its own suite under the guard: `~/rebot_setup/test_guarded_runs.py` (trajectory
files, the monotone cubic, pre-flight refusals, a latch mid-trajectory held until the operator's stop, a fault during
a park, a persistent fault ending in the operator's `off`, J1 one turn off, J2 drooped, the real-run guard
requirement).

**Rehearsal (2026-10-21):** a full simulated run with the real config caught a fold-return refusal: a min-jerk ending
on exactly 0.0 produced a J2 target of +1e-16 against a limit of 0. That is fixed (R2 above, `EPS_RAD`) and is now a
test.

**Mutation check (2026-10-21):** each rule was disabled in turn (position window, step limit, keep-out, stopping
speed, vlim lowering) and the fuzz suite failed every time; with the rules intact it passes.

## Design sources

Patterns adapted (not copied verbatim): UR / FANUC DCS / ABB SafeMove / KUKA SafeOperation (independent checker,
command + measured state, stopping distance in the limits, checksummed config, latched faults); libfranka
`rate_limiting.h` (speed bound shrinking near a limit; Apache-2.0); ros2_control `joint_limits` soft limiter
(Apache-2.0); LeRobot `ensure_safe_goal_position` (step limit against the measured position; Apache-2.0); MoveIt
Servo (halt on violation). Damiao motor facts: Seeed DM-J4310 manual (protections, fault codes) and wiki (PMAX/
VMAX/TMAX are mapping ranges, TIMEOUT in 50 µs units).

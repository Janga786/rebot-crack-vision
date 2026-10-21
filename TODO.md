# TODO — where the arm-safety work stands and what comes next

**Read this first if you are picking up the "make IK/arm motion safe" work.** Last updated 2026-10-21.

## Where we are

`packages/arm_guard` is built, tested and installed: it is the last-line filter in front of every motor command
(the full design, guarantee and limits are in [`docs/motion/ARM_GUARD.md`](docs/motion/ARM_GUARD.md)).

- Done (2026-10-21, workstation-only, no hardware touched):
  - the guard: R1–R7 rules, hold-and-latch on any fault, a method allowlist on `motorbridge`, an exclusive bus
    lock, a checksummed and approved config (`config/robot/arm_guard.toml` + `.approved.json`);
  - startup hook installed in `~/miniconda3/envs/rebot` and `~/miniconda3/envs/lerobot` (`python -m arm_guard
    hook-status` → both OK);
  - the vendor ROS driver patched to refuse to start without the guard (local commit in `~/rebot_ws`, copy in
    `packages/arm_guard/vendor_patches/`);
  - 155 tests (`bash scripts/safety/test_arm_guard.sh`, also stage 5 of `scripts/safety/run_safety_case.sh`):
    closed-loop fuzz against a hostile planner, a replay of all 34 real runs, mutation-checked.
- **Not yet true:** the guard has never run against the powered arm, and its limits are the vendor URDF's, not
  measured ones.

The three items below are next, in this order. **All three need the physical arm, so every powered or bus step
needs the operator's explicit OK for that step.** Don't run the arm, open the bus, or write motor registers on your
own initiative. Workstation code, tests and docs are fine to do first.

---

## 1. Motor CAN watchdog (firmware `TIMEOUT` register)

**Why.** `TIMEOUT` (RID 9) is `0` on all 7 motors (read 2026-10-10, `~/rebot_setup/SETUP_LOG.md`). The motor
firmware is the only layer that survives a dead host. Right now, if the control process crashes or the USB cable
comes out, each motor keeps executing its last command forever (for example, an MIT setpoint at kp = 100).

**The decision the operator must make first.** When the watchdog trips, the motor *disables*. A disabled motor goes
limp, so **J2/J3 drop from any raised pose**, the same as cutting the 24 V switch. That contradicts ADR-016 §7
("cancel and hold, never auto-disable"). The two options:

- (a) keep `TIMEOUT = 0`: a dead host leaves the arm holding its last command, unattended;
- (b) set a timeout: a dead host makes the arm fall, from wherever it is.

Ask the operator. If they choose (b), record the decision as an ADR-016 amendment.

**How (once approved).**
- Value: about 200 ms (`set_can_timeout_ms(200)` → register value 4000, in 50 µs units). That is 20–100× the
  2–10 ms loop period. Check first that no caller pauses its command stream that long: the vendor SDK's 500 Hz
  loop, LeRobot's ~30 Hz loop (**33 ms, fine**), and the tuning scripts (200 Hz).
- Confirm the watchdog behaviour on a single **unloaded** joint first (J1 or the gripper), then decide which joints
  get it. The behaviour to confirm: what resets it (control frames only, or feedback requests too, which is
  unverified), how long until it trips, and what the motor does after the trip.
- All writes go through the guard's config path, motors DISABLED:
  ```python
  import arm_guard, motorbridge
  with arm_guard.config_writes("TIMEOUT=200ms, operator <name> approved <date>"):
      m.set_can_timeout_ms(200)
      m.store_parameters()          # persist to flash; without it the value is lost on power-off
  ```
- Read it back (`get_register_u32(9)`) on every motor and file the readings as operator evidence.
- Make the guard check it. Add `can_timeout_ms` to `[guard]` in `arm_guard.toml`, read RID 9 in
  `Guard.check_enable` (`packages/arm_guard/src/arm_guard/runtime.py`) and refuse to enable a motor whose
  `TIMEOUT` differs. Add a test, re-approve the config, and update ARM_GUARD.md ("does not guarantee" item 3).
- Optional: a keep-alive thread in long-running callers, so a slow planner can't trip the watchdog.

## 2. Measure the real joint limits (MOT-09 step E1, then a limits change)

**Why.** Every range the guard enforces (`config/robot/arm_guard.toml`) is the vendor URDF's
(`config/robot/b601_dm_limits.yaml`). Nobody has checked them on this arm. Known oddities:
- J2 rests at +0.0235 rad, past its upper limit of 0.
- J3's `safe_park_q` is +0.136.
- The "show" demo turned J1 to −3.76 rad, past ±2.8.
- The gripper's full-open angle is unknown (the widest seen is −1.776 rad).

**How.** Follow MOT-09's E1 ([`plan/cards/MOT-09.md`](plan/cards/MOT-09.md)): a **readback-only** survey.
1. All motors DISABLED, the operator moving each joint by hand while the arm is supported. No powered motion.
2. Record the min/max angle per joint and any physical contact found inside the URDF range (cable harness,
   self-contact, base plate), plus the gripper's closed/open angles. Readback goes through the guard; reads are
   pass-through, so nothing is refused.
3. File the CSV under `evidence/operator/MOT-09/joint_range_*.csv` (`claude-auto evidence add MOT-09 <file>`).
4. If a contact sits inside a URDF range, MOT-09 opens a separate limits card for `b601_dm_limits.yaml`, which must
   never be edited inside MOT-09 itself. Whatever that card decides:
   - narrow `arm_guard.toml` to the measured range minus a margin (and never widen it beyond the URDF without
     `beyond_urdf_reason`);
   - update the gripper's `j7` range;
   - re-approve with `python -m arm_guard approve --by <operator>`;
   - re-run `bash scripts/safety/test_arm_guard.sh`. Note that `test_replay.py` uses the real logs, so a narrower
     range may legitimately refuse an old run: update its expectations with a written reason.
5. While the arm is there: measure the table plane and the specimen (MOT-10 / MOT-10.4) and replace the nominal
   keep-out numbers in `[keepout]` with measured ones.

## 3. Startup position check (encoder plausibility before the first enable)

**Why.** The guard compares every command with the *measured* position, so a wrong reading defeats it. The
reading can be wrong after a power cycle with a joint turned. CONTROLLER_TUNING §12 says: "Multi-turn position
after a power cycle with J1 turned has not been verified — return J1 near 0 before powering off". Today the guard
only refuses to enable a joint that reads *outside* its range. A reading that is wrong but still in range passes.

**How.**
- Add a startup check to the guard (copy ABB SafeMove's "synchronisation" idea). At the first `enable()` after the
  process starts, every joint on the bus must read within a configured window of a known pose: the fold, each joint
  within 0.10 rad of 0, the gripper within 0.4 rad of −0.2. These are the tolerances `demo_move.py` already
  enforces. Until that holds, refuse to enable, or allow a slow-jog-only mode if the operator wants one.
- Config:
  ```toml
  [startup]
  pose = { j1 = 0.0, j2 = 0.0, ... }
  tolerance_rad = 0.10
  per_joint_tolerance = { j7 = 0.4 }
  ```
  Implement it in `Guard.check_enable` / `Controller.enable_all` (`runtime.py`, `patch.py`).
- Allow another known pose only through an explicit operator confirmation (`arm_guard.confirm_start_pose("reason")`),
  logged as an event.
- Investigate with the arm (operator OK required): power-cycle with J1 at +1.0 rad and at −3.0 rad, and see what
  position the motor reports afterwards. That tells us whether multi-turn wrap is real for the 4340P/4310 and
  whether the check needs more than a window, for example a second source such as the camera or a gravity-torque
  signature.
- Tests: wrong-but-in-range readings refused; the fold accepted; the confirmation path logged.

---

## Also open (smaller, or someone else's)

- **The vendor ROS driver still can't start**, for pre-existing reasons: `hardware_manager.py` imports `RobotArm`,
  but the SDK at `0324b73` exports `RebotArm` (VENDOR_DRIVER.md V-SDK-API), and motorbridge/pinocchio are missing
  from `/usr/bin/python3` (V-PYDEPS). Porting it touches many SDK internals. Do it in its own card, with the guard
  hook installed in whatever interpreter ends up running the driver.
- **Remaining guard bypasses** (by design, documented): direct C-ABI calls and the Rust tools
  `ws_gateway`/`motor_cli`. Closing them means moving the checks into motorbridge's Rust `send_raw`.
- **The "show" demo** (J1 at −3.76 rad) is refused by the guard. If the operator wants it back, add
  `beyond_urdf_reason` to `[joints.j1]` with the cable-harness caveat, then re-approve. Don't do it silently.
- **The claude-auto orchestrator** (`claude-auto status`) is paused (`operator_stop`). This work was done outside it,
  so no plan card records it. If it should be tracked there, add a card that points at this file.

## Quick commands

```
bash scripts/safety/test_arm_guard.sh                                  # everything, offline
cd packages/arm_guard/src && ../../../env.sh python -m arm_guard check  # enforced limits + approval
cd packages/arm_guard/src && ../../../env.sh python -m arm_guard hook-status
cd packages/arm_guard/src && ../../../env.sh python -m arm_guard events # fault/reset/config-write log
```

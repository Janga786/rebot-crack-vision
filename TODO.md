# TODO — where the arm-safety and camera work stand and what comes next

**Read this first if you are picking up the arm-safety work (items 0–3) or the D405 camera pipeline (C).** Last updated 2026-10-22.

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
- Done (2026-10-21 evening, workstation-only), on the operator's "get it 100% ready to run a trajectory on the arm":
  - the startup check (item 3 below, mostly), J1 whole-turn unwrapping, J2 droop tolerance, a session allowance
    past a limit that cannot ratchet, the offline pre-flight (`arm_guard.preflight`); 201 tests;
  - the operator's controller made guard-aware and able to run trajectory files: `~/rebot_setup/demo_move.py`
    2026-10-21.1, `run-trajectory.sh`, `trajectories/`, `test_guarded_runs.py`. Rehearsed in simulation with the
    real guard config: both trajectories complete with zero guard faults;
  - the config re-approved for offline testing only, marked *operator review pending*.
- **Not yet true:** the guard has never run against the powered arm, and its limits are the vendor URDF's, not
  measured ones.

**Next is the operator's first guarded run** (item 0). Items 1 and 2 follow. **Every powered or bus step needs the
operator's explicit OK for that step.** Don't run the arm, open the bus, or write motor registers on your own
initiative. Workstation code, tests and docs are fine to do first.

---

## 0. The first guarded trajectory run — DONE 2026-10-22

`smoke_raise.json --slow 2` completed on the powered arm: tracking ≤ 0.023 rad, no guard fault
([evidence](evidence/operator/first-guarded-run-2026-10-22/README.md)).

The operator saw a wobble: J2/J3 rang at 3.3 Hz with kd 1. An A/B run with J2/J3 kd 4 cut it by 61 % / 48 %.

Next, each on the operator's go:
1. **Decide** whether kd 4 becomes the J2/J3 default (`first_move_j1.TUNED`, `tuned_gains.yaml`,
   CONTROLLER_TUNING). Today it is `demo_move.py --gains j2=100/4,j3=80/4`.
2. Run the same trajectory at its own speed.
3. Run `first_guarded_run.json`.
4. Give sparse-waypoint trajectories a smoother interpolation: velocity kinks at the middle point.

Summary of the whole session: [docs/SESSION_2026-10-21_22.md](docs/SESSION_2026-10-21_22.md). What follows is the original plan, kept for
reference.

### Original plan

Follow [`docs/motion/FIRST_GUARDED_RUN.md`](docs/motion/FIRST_GUARDED_RUN.md):
1. Review and re-approve the guard config under the operator's own name.
2. Run the read-only `--check`.
3. Run `./run-trajectory.sh trajectories/smoke_raise.json 2`.
4. Then full speed, then `first_guarded_run.json`.

After it, look at the run's summary (`guard` events, max tracking error, `max_gap_s`) and the guard's event log.
Then update ARM_GUARD.md's status line ("not yet run on the powered arm") with what actually happened. If the guard
tripped, the CSV shows what it saw.

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

## 3. Startup position check — mostly DONE (2026-10-21)

Built: `[startup]` in `arm_guard.toml`, `core.startup_check`, `Guard._startup`.
- Before a session's first enable (every motor on the bus disabled), every joint must read inside its window
  around the fold (±0.4 rad; J2 −0.4…+0.5; J7 −0.6…+0.1). The windows cover every start logged 2026-10-14…21.
- J1 is unwrapped by whole turns: it read +6.335, +6.278 and +6.272 facing forward after power cycles on
  2026-10-18; every other forward reading was within 0.08 of 0. The guard then checks J1 with that offset.
- J1 is refused if its range would leave the MIT position mapping (two turns off).
- Tests: `tests/test_startup.py`.

Still open:
- `arm_guard.confirm_start_pose("reason")`: starting from another known pose by explicit operator confirmation
  (e.g. the show demo's "folded, facing the audience" start). Not built. Today the operator turns the base to
  forward.
- With the arm: power-cycle with J1 at +1.0 rad and at −3.0 rad. That confirms the reading is off by exactly
  whole turns (2π), which the data supports but no controlled test has shown. Also check whether any other joint
  can come back a turn off (none has).

The original analysis follows.

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

## C. Camera (D405) pipeline — connected and working 2026-10-22

Done:
- **Software (2026-10-22 morning):** `scripts/camera.py` with `check`, `record`, `snap` and `live`; the live
  `CaptureFrame` source (`crackvision.d405_live`); the in-memory network (`crackvision.live_predictor`, identical to
  the batch path with TTA); [`docs/camera/D405_SETUP.md`](docs/camera/D405_SETUP.md).
- **The real camera (2026-10-22, after the operator plugged it in):**
  - D405 serial 260522273107 on USB 3.2.
  - Capture at 30 fps with no dropped frames.
  - `check`: validator PASS, verdict WARN for the firmware only.
  - Video test: 10 s at 1280×720, lossless and validated, plus an MP4.
  - `snap` and `live` run the model on the camera. Live is 30 fps video with the model at 17 passes a second
    (848×480) or 8 (HD); it was 4.7 fps before the latency fix the same day.
  - Full report: [`docs/camera/D405_FIRST_CONNECTION.md`](docs/camera/D405_FIRST_CONNECTION.md); evidence in
    `evidence/camera/`.
  - Fixed along the way: recordings dropped frames because PNGs were written inside the capture loop. They now
    capture to memory first and write on 8 threads.

Next:
1. **A positive test on real cracks.** Point the wrist at a cracked surface (or a printed crack photo) at
   10–40 cm. Watch `./env.sh python scripts/camera.py live` (http://127.0.0.1:8090), then run `snap`. Light the
   surface: the D405 has no projector. Stop the arm before `snap`.
2. **Firmware.** The camera runs 5.15.1.55, which predates 5.17.0.9 (the D405 auto-exposure fixes). Updating to
   5.17.0.10 is the operator's decision (D405_SETUP.md §5), never a tool's. Colour looked fine in room light.
3. **File CAM-01 in the orchestrator:**
   - `claude-auto evidence add CAM-01 evidence/camera/d405_discovery.json`
   - `claude-auto hw declare d405_connected`

   The doc it wants (`docs/camera/D405_FIRST_CONNECTION.md`) exists.
4. CAM-04: `record` at two or more distances in 10–40 cm over a test surface. Then the PERC-05 dataset and the
   PERC-06/08 zero-shot fitness decision.
5. On the arm: route the cable with slack at J4–J6.
6. Later: GEOM-04.1 (an OpenCV build with `calibrateHandEye`), then hand-eye calibration, then captures tied to
   joint states (INTERFACES §9) so cracks land in robot coordinates.

## Also open (smaller, or someone else's)

- **The vendor ROS driver still can't start**, for pre-existing reasons: `hardware_manager.py` imports `RobotArm`,
  but the SDK at `0324b73` exports `RebotArm` (VENDOR_DRIVER.md V-SDK-API), and motorbridge/pinocchio are missing
  from `/usr/bin/python3` (V-PYDEPS). Porting it touches many SDK internals. Do it in its own card, with the guard
  hook installed in whatever interpreter ends up running the driver.
- **Remaining guard bypasses** (by design, documented): direct C-ABI calls and the Rust tools
  `ws_gateway`/`motor_cli`. Closing them means moving the checks into motorbridge's Rust `send_raw`.
- **The "show" demo** (`basic-demo.sh`) is refused by the guard's pre-flight before anything moves, for two
  reasons. J1 at −3.76 rad is beyond the URDF; and at 2×, J5 measured up to 2.62 rad/s (2026-10-18…21), above the
  guard's 2.5 rad/s abort. To bring it back:
  - add `lower = -3.9` and a `beyond_urdf_reason` to `[joints.j1]` (with the cable-harness caveat);
  - raise the 4310 `vel_cap`/`vel_abort` (about 2.6/3.2), or run the show at 1.5× at most;
  - re-approve.

  This is the operator's decision. Don't do it silently.
- **`~/rebot_setup` is not under version control.** The runner (`demo_move.py`, `run-trajectory.sh`,
  `trajectories/`) lives there. The pre-guard versions are in `~/rebot_setup/backup_pre_guard_2026-10-21/`.
- **Guard overhead** is about 0.55 ms per 7-motor cycle, almost all of it the keep-out FK run on every arm-joint
  command. That is fine at 200 Hz. If the vendor SDK's 500 Hz loop ever runs under the guard, compute the FK once
  per cycle instead.
- **The claude-auto orchestrator** (`claude-auto status`) is paused (`operator_stop`). This work was done outside it,
  so no plan card records it. If it should be tracked there, add a card that points at this file.

## Quick commands

```
bash scripts/safety/test_arm_guard.sh                                  # everything, offline
cd ~/rebot_setup && ./run-trajectory.sh --rehearse trajectories/smoke_raise.json   # full run in simulation
cd packages/arm_guard/src && ../../../env.sh python -m arm_guard check  # enforced limits + approval
cd packages/arm_guard/src && ../../../env.sh python -m arm_guard hook-status
cd packages/arm_guard/src && ../../../env.sh python -m arm_guard events # fault/reset/config-write log
```

# First guarded trajectory run — operator runbook

**Status (2026-10-21):** everything below has been rehearsed offline: a simulation using the real guard config,
the operator's real controller and the real trajectory files. **Nothing has run on the powered arm yet.** Every
step at the arm is the operator's own action.

## What runs

```
~/rebot_setup/run-trajectory.sh ─► demo_move.py --trajectory ─► motorbridge ─► USB2CAN ─► 7 Damiao motors
                                   (the 2026-10-17 controller)    └── arm_guard sits in front of every command
```

- **The controller** is the one behind every powered run since 2026-10-17 (`basic-demo.sh`):
  - tuned MIT gains;
  - gravity and friction feed-forward;
  - 200 Hz;
  - aborts on tracking error (0.10 rad), torque, velocity, a stalled loop, stale feedback or a motor fault code;
  - a controlled slow park back to the fold.

  The fold (all joints 0) is the start and end pose of every run.
- **New in `demo_move.py`** (version 2026-10-21.1):
  - **`--trajectory FILE`** plays a `crackvision.joint_trajectory/1` file (docs/INTERFACES.md §11.2): first a
    min-jerk approach from the fold, then the trajectory itself, then a min-jerk return to the fold. The
    trajectory is a monotone cubic through the file's points, so it never overshoots between points and starts
    and ends at rest. The gripper holds at 0.
  - **The guard pre-flight.** Before anything is enabled, every command of the run is checked against arm_guard's
    rules. A run the guard would stop half-way is refused up front.
  - **Guard-fault handling.** If the guard stops the arm anyway, the script never parks past the guard and never
    switches off on its own. It keeps the arm holding and waits for you.
  - **The guard is mandatory.** A real run refuses to start unless arm_guard is active with an approved config.
- **The guard** (`packages/arm_guard`, [ARM_GUARD.md](ARM_GUARD.md)) includes the startup check added
  2026-10-21:
  - Before the first enable, every joint must read inside its window around the fold.
  - J1 may read whole turns off after a power cycle; the guard unwraps it.
  - J2 may start drooped below the fold, up to +0.5 rad; the guard only allows it to be lifted from there.

### Trajectories ready to run (`~/rebot_setup/trajectories/`)

| File | What | Length | Planned peak speed |
|---|---|---|---|
| `smoke_raise.json` | The repo's MOT-05.2 fixture, unchanged (sha256 `5deef216…`): fold → raised pose P = [0.1, −0.6, −0.9, 0.1, 0.1, 0.1] | 2 s approach + 3 s + 3.1 s return | J3 0.54 rad/s |
| `first_guarded_run.json` | `make_trajectory.py`: raise, look left/right (base + wrist), nod, roll, back to raised | 2 + 18.5 + 3.5 s | J6 0.56, J3 0.54 rad/s |

For comparison, the show demo at 2× peaks at 1.12 rad/s on J1–J3 and 2.34 rad/s on J5. Both files pass:
- the repo's independent checker (`crackvision.trajectory_check`: schema, C-LIMITS, C-MARGIN);
- the demo's own path check (URDF limits, speed caps, gripper ≥ 0.12 m above the base plane);
- the guard pre-flight (keep-out margin ≥ 41 mm).

Any other `crackvision.joint_trajectory/1` file works the same way, for example the planner's output.

## 1. At the desk, before powering on (about 5 minutes, no hardware)

```bash
cd ~/Projects/rebot_crack_vision/packages/arm_guard/src
../../../env.sh python -m arm_guard check          # the limits the guard enforces; read them
../../../env.sh python -m arm_guard approve --by Janga786 --note "reviewed for the first guarded run"
../../../env.sh python -m arm_guard hook-status    # rebot: OK
bash ~/Projects/rebot_crack_vision/scripts/safety/test_arm_guard.sh        # guard tests, all pass
cd ~/rebot_setup
env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python -m unittest test_guarded_runs test_demo_move   # ~3 min
./run-trajectory.sh --rehearse trajectories/smoke_raise.json 2             # outcome: completed
```

The config is currently approved for offline testing only, marked *operator review pending*. Approving
under your own name replaces that record. Any edit to `arm_guard.toml` makes the guard refuse all motion until it is
approved again.

## 2. At the arm

**Preconditions:**
- Base clamped, area clear, your hand on the 24 V switch.
- The arm folded, J1 facing forward.
- Nothing else using the arm: no `basic-demo.sh`, LeRobot, vendor driver or motorbridge CLI. The guard and the
  script both refuse a second process.

1. Switch on the 24 V. Plug in the USB2CAN; `/dev/ttyACM0` should appear.
2. **Read-only check**:
   ```bash
   cd ~/rebot_setup && ./run-trajectory.sh --check trajectories/smoke_raise.json 2
   ```
   This opens the bus, reads registers and positions, checks the path and runs the guard pre-flight. It enables
   nothing. Expected:
   - `pre-checks OK; path OK: …`
   - `arm_guard pre-flight OK: …`
   - if J1 reads a turn off, a `whole-turn offsets {'j1': 1}` line.

   Exit 4 means the J1 reference is unknown; the run in step 3 then asks you to turn the base to forward.
3. **First powered trajectory, at half speed:**
   ```bash
   ./run-trajectory.sh trajectories/smoke_raise.json 2
   ```
   1. All seven motors enable, the arm moves to zero over at least 3 s and holds (this also lifts a drooped J2).
   2. Press Enter. The arm raises to P, comes to rest there, returns to the fold, holds 1 s, and every motor is
      switched off.
   3. `outcome: completed` and `all motors DISABLED (confirmed)` close the run.
4. Then the same file at full speed (`… smoke_raise.json`, no factor), then `first_guarded_run.json 2`, then
   `first_guarded_run.json`.

## 3. What can happen, and what to do

| You see | What it means | What to do |
|---|---|---|
| `REFUSED (… nothing was enabled)` | A pre-check, the path check, the guard pre-flight or the guard's startup check failed. Nothing moved. | Read the reason. "not starting from the folded rest" or "startup check failed": fold the arm by hand. J1 not forward: turn the base. `startup: j1 reads +2.77`: the base was turned while the arm was off. |
| `ABORT: [tracking/torque/velocity/signal …]` then `PARK` | The script's own abort (as with `basic-demo.sh`). The guard is not involved. | Nothing: it parks slowly to the fold and switches off. |
| `GUARD FAULT, HOLDING: <reason>` | The guard refused a command. It sent a hold and latched, so the arm holds where it is. **Nothing is switched off automatically.** | Look at the arm. Then type `stop` + Enter (or Ctrl-C): the guard is reset (logged with the reason) and the arm parks slowly to the fold, then switches off. If that park faults again, it holds again. |
| It keeps faulting on every park (e.g. a stuck sensor) | The fault persists. | Support the arm by hand, then type `off` + Enter: everything is switched off at once and **the arm drops** into your hands. |
| `uncontrollable fault` (bus error, stale feedback, motor fault code) | The script can no longer hold reliably. | It holds 0.5 s, then switches off: **a raised arm drops**. This is unchanged from before; the CAN watchdog is TODO item 1. |

During the run, `stop` + Enter parks and switches off, and `off` + Enter switches off **immediately** (the arm
drops). The **24 V switch is the e-stop at all times: a raised arm drops.**

## 4. After the run

- Each run writes `~/rebot_setup/runs/traj_<file>_<UTC>.csv` (200 Hz: reference, position, velocity, torque and
  feed-forward per joint) and a `.json` summary. The summary includes:
  - `guard`: the config hash and every guard event during the run (startup, faults, resets, adjustments);
  - `guard_preflight`: the pre-flight result;
  - `trajectory`: the file's path and sha256.
- `cd ~/Projects/rebot_crack_vision/packages/arm_guard/src && ../../../env.sh python -m arm_guard events` lists the
  guard's own log.
- To file a run as evidence: `claude-auto evidence add <card> <file>`.

## 5. The show demo (`basic-demo.sh`) is refused now

The guard pre-flight refuses it before anything is enabled, for two reasons:
- it turns J1 to −3.76 rad, beyond the URDF's ±2.8;
- at 2×, J5 measured up to 2.62 rad/s in the nine runs of 2026-10-18…21, above the guard's 2.5 rad/s abort for the
  wrist motors.

To bring it back:
1. Give `[joints.j1]` `lower = -3.9` and a `beyond_urdf_reason`.
2. Raise the 4310 `vel_cap` and `vel_abort` (to about 2.6 and 3.2), or run the show at 1.5× at most.
3. Re-approve the config.

Both changes are the operator's decision, recorded in the config.

## Known limits

- **Unmeasured limits.** The guard's joint ranges are the vendor URDF's, and the keep-out geometry is nominal
  (TODO.md item 2).
- **No CAN watchdog.** `TIMEOUT = 0`: if the process dies, the motors keep their last command (TODO.md item 1).
- **Simulated plant.** The rehearsal's arm is crude: independent joints, gravity fixed at the rest pose. It proves
  the logic and the limits, not the dynamics. The real dynamics are the ones the operator has already run at up to
  2× (show) on this controller.
- **Overhead.** The guard adds about 0.55 ms of computation to each 5 ms control cycle (measured on the
  workstation).

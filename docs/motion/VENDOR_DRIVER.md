# VENDOR_DRIVER.md — vendor-driver readiness note (MOT-11)

**Status:** operator-facing. Read before any session that launches the real `reBotArmController`
(MOT-09 step D1). A PASS from the checker below does **not** commission anything and is **not**
evidence for `config/robot/commissioning.yaml` — commissioning is MOT-09's hands-on bring-up, not
a static file check.

## Re-run before every vendor-driver session

```
./env.sh python scripts/motion/check_vendor_driver.py --rebot-ws "$HOME/rebot_ws" \
    --facts evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml --format text
```

Exit 0 = no FAIL (WARNs may remain). Exit 3 = at least one FAIL — the vendor stack is not ready;
fix every FAIL's remedy (below) before launching `bringup.launch.py`/`driver.launch.py`. Exit 2 =
a required input file is missing. `scripts/motion/check_vendor_driver.py --help` lists every flag;
`--format json`/`--json-out PATH` give a machine-readable report. The tool is read-only: it never
writes under `~/rebot_ws`, only `logs/` (docs/INTERFACES.md §0.4) and `--json-out`.

- **After pinning the SDK.** `V-SDK-API`'s own remedy is to pin the vendored SDK, an operator
  action outside this repo: `git -C ~/rebot_ws/third_party/reBotArm_control_py checkout 8427281`.
  The same re-run command above then evaluates the pinned layout: up to and including `8427281`
  the SDK's arm module is `actuator/arm.py` (not `actuator/rebotarm.py`), and `V-BAUD` reads the
  baud and cites it from whichever of the two files is present.
- **Out-of-tree driver dependencies.** If a dependency `V-PYDEPS` checks for was installed outside
  the standard interpreter search dirs (for example `pip install --target DIR`), pass
  `--driver-pythonpath DIR` so the checker finds it — and launch the real driver with that same
  `DIR` on `PYTHONPATH`, or the checker's PASS will not match what actually runs.
- **Config fixes apply to the vendor default files.** The checker reads the default config files
  under `--rebot-ws` (`arm.yaml`, `gripper.yaml`, `driver_params.yaml`, `safe_park.yaml`), not any
  launch-time override (`arm_config:=`, `safe_park_config:=`). A config remedy above must be
  applied to those default files for a re-run to see it.

## What launching the vendor driver actually does

- **At connect:** `HardwareManager.connect()` starts the end-position controller, seeds its
  target to `safe_park_q` (`hardware_manager.py:243-245`), writes the POS_VEL control mode and
  gain registers (25-28) to every arm motor, then `enable_all()`s the arm and, in
  `init_gripper()`, the gripper too (`hardware_manager.py:190-197,572-600`). **Launching the
  driver energises every motor**, arm and gripper, before any trajectory is sent.
- **At shutdown:** if `safe_park_enabled` (it is, by default), the driver drives every joint to
  `safe_park_q` before disabling (`hardware_manager.py:199-216`, `rebotarm_controller.py:104-109`).

## Current findings against `~/rebot_ws@2ebc4b8` / SDK `0324b73` (2026-10-10)

Every citation below is computed by the checker from the vendor file's own content (never
hard-coded), so it stays correct as the vendor tree changes — re-run the command above to refresh
this table.

| id | status | citation | remedy |
|---|---|---|---|
| V-PORT | PASS | `arm.yaml:4`, `gripper.yaml:3`, `driver_params.yaml:6` | — |
| V-BAUD | PASS | `rebotarm.py:486` | — |
| V-IDS | PASS | `arm.yaml:9`, `gripper.yaml:5` vs facts | — |
| V-ORDER | PASS | `arm.yaml:9` joints | — |
| V-DIRECTION | PASS | no sign/direction/offset/invert keys; every facts sign check `match` | — |
| V-ZERO | PASS | facts `zero.consistent_with_urdf_home` | — |
| V-GRIPPER-SIGN | PASS | `ros_services.py:16,17`, `ros_publishers.py:7,8` | — |
| V-GRIPPER-RANGE | WARN | facts `full_open_rad` is `null`; largest observed opening -1.776 rad vs the -4.9/-5.0 rad constants (`ros_services.py:16`, `ros_publishers.py:7`) | measure the true full-open motor angle before trusting these constants |
| V-GRIPPER-WIDTH | WARN | `ros_publishers.py:9` (0.045 m) vs URDF gripper upper (0.0715 m) | reconcile or document the discrepancy |
| V-GRIPPER-UNITS | WARN | `ros_actions.py:213` passes `goal.position` (metres) straight to `set_gripper_target` (`hardware_manager.py:610`, motor radians) | add a metres→motor-radians conversion in the `GripperCommand` handler |
| V-SAFE-PARK | **FAIL** | `safe_park.yaml:5` `safe_park_q` puts joint2 (+0.011253) and joint3 (+0.135614) above their URDF upper limit 0.0; `hardware_manager.py:190,194` seeds this target at connect, `hardware_manager.py:199,211` parks to it on shutdown | fix `safe_park_q` so every joint is within `b601_dm_limits.yaml`, or disable `safe_park_enabled` until it is |
| V-STARTUP | WARN | `hardware_manager.py:190,572` connect()/init_gripper(), `hardware_manager.py:193` SDK `ArmEndPos.start()` | confirm with the operator that energising every motor at launch is intended |
| V-GAINS | WARN | `arm.yaml:16,19` POS_VEL `vlim` 5.0/3.0 rad/s vs the 1.0 rad/s planning limit; MIT kp/kd 120/10 vs the bring-up caps (kp≤10, kd≤1) | lower `arm.yaml`'s POS_VEL `vlim` and MIT `kp`/`kd` before any real run |
| V-SDK-API | **FAIL** | `hardware_manager.py:38,39,40,41,436,573` imports `RobotArm`/`ArmEndPos`/`actuator.gripper.load_cfg`; the vendored SDK (`0324b73`) exports only `RebotArm`/`RebotArmEndPose` | pin the SDK submodule to a commit that still exports `RobotArm`/`ArmEndPos` (last known: `8427281`), or update `hardware_manager.py`'s imports to the SDK's current API |
| V-PYDEPS | **FAIL** | `motorbridge`/`pinocchio` not found under `/usr/bin/python3`'s standard dist-packages/site-packages | install both into the interpreter the driver actually runs under |
| V-QOS | PASS (since MOT-05.7) | `execute_trajectory.py:569` subscribes `joint_states` with `qos_profile_sensor_data` | — |
| V-PROFILE | PASS | `rebotarm_controller.py:18,30`, `ros_actions.py:30`, `ros_publishers.py:24,44` match `execution.yaml` `driver_profiles.vendor` | — |
| V-ARBITRATION | PASS | `driver_params.yaml:5` `cmd_arbitration: reject` | — |
| V-URDF | WARN | `bringup.launch.py:26` publishes the gripper-less `reBot-DevArm_fixend.urdf` (`docs/motion/ROBOT_MODEL.md` §3) | repoint at the gripper URDF if a real session needs the gripper in TF |

**Three FAILs as of 2026-10-10: the vendor driver cannot be launched against this `~/rebot_ws` and
SDK state.** `V-SDK-API` means `HardwareManager.__init__` cannot even import the SDK; `V-PYDEPS`
means `motorbridge`/`pinocchio` aren't on the driver interpreter's path either way; `V-SAFE-PARK`
means that even if the driver started, its default shutdown/connect target drives joint2/joint3
into the folded contact. All three must be fixed vendor-side (never patched from this repo) before
any dry rehearsal against the live driver.

This note never edits `~/rebot_ws` — every remedy above is applied by the operator, vendor-side.

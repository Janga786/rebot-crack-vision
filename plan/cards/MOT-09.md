# MOT-09 — Arm bring-up commissioning (operator)

```json card
{
  "kind": "hardware",
  "depends_on": [
    "MOT-05",
    "HOST-04",
    "MOT-11"
  ],
  "requirements": [
    "REQ-INT-2"
  ],
  "spec_state": "ready",
  "scope": {
    "write": [
      "config/robot/commissioning.yaml"
    ]
  },
  "inputs": [
    "evidence/operator/MOT-09/bringup_2026-10-10_summary.md",
    "evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/INTERFACES.md#11",
    "docs/motion/VENDOR_DRIVER.md",
    "docs/motion/EXECUTION.md",
    "docs/host/DEVICE_ACCESS.md",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "real-preview-commissioning",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'set +u; source ros2_ws/install/setup.bash; unset CRACKVISION_ARM_REAL; ros2 run crackvision_motion execute_trajectory --dry-run --mode real --profile vendor --trajectory ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json --root \"$PWD\" </dev/null 2>/dev/null | python3 -c \"import json,sys; g={x[\\\"gate\\\"]: x[\\\"outcome\\\"] for x in json.load(sys.stdin)[\\\"gate_report\\\"]}; print(g); sys.exit(0 if g[\\\"G-COMMISSIONING\\\"] == \\\"pass\\\" and g[\\\"G-ESTOP\\\"] == \\\"pass\\\" else 1)\"'",
        "timeout_s": 180,
        "expect_exit": 0
      },
      {
        "id": "commissioning-evidence",
        "cmd": "/usr/bin/python3 -sE -c \"import os,sys,yaml; c=yaml.safe_load(open('config/robot/commissioning.yaml')); e=c['estop']; paths=list(e['evidence'])+list(c['evidence']); bad=[p for p in paths if not os.path.isfile(p)]; print('missing:', bad); sys.exit(0 if (c['commissioned'] is True and c['joint_ranges_verified'] is True and e['operator'] and e['verified_utc'] and e['evidence'] and not bad and abs(float(c['speed_scale_cap'])-0.10) < 1e-12) else 1)\"",
        "timeout_s": 60,
        "expect_exit": 0
      },
      {
        "id": "evidence-set",
        "cmd": "bash -c 'd=evidence/operator/MOT-09; for g in \"power_on_scan_*.txt\" \"rest_check_*.json\" \"estop_test_*.md\" \"first_move_j1_*.csv\" \"first_move_j1_*.json\" \"vendor_check_*.json\" \"dry_rehearsal_*.json\" \"joint_range_*.csv\"; do ls $d/$g >/dev/null 2>&1 || { echo \"missing $d/$g\"; exit 1; }; done'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "dry-rehearsal-record",
        "cmd": "/usr/bin/python3 -sE -c \"import glob,json,sys; fs=sorted(glob.glob('evidence/operator/MOT-09/dry_rehearsal_*.json')); fs or sys.exit('missing evidence/operator/MOT-09/dry_rehearsal_*.json'); r=json.load(open(fs[-1])); g={x['gate']: x['outcome'] for x in r['gate_report']}; print(r['mode'], r['driver_profile'], r['outcome'], g.get('G-GRAPH'), g.get('G-START-STATE')); sys.exit(0 if (r['schema']=='crackvision.execution_record/1' and r['mode']=='dry' and r['driver_profile']=='vendor' and r['outcome']=='rehearsed' and g.get('G-GRAPH')=='pass' and g.get('G-START-STATE') in ('pass','warn')) else 1)\"",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Every evidence file under evidence/operator/MOT-09/ comes from the physical B601-DM in an attended session, added with `claude-auto evidence add MOT-09 <file>`. Nothing is simulated or hand-edited after capture. Simulation never passes a physical step.",
      "Order is respected and evidenced. The hardware e-stop was wired and tested twice: unloaded (B5), then with joint1 enabled and holding (C3). Both tests happened before any vendor-driver launch (D3), and no motion was attempted without the e-stop within the operator's reach.",
      "The first powered motion was joint1 only, using ~/rebot_setup/first_move_j1.py as reviewed by the operator: kp<=10, kd<=1, zero feed-forward torque, only motor 1 enabled, a ramp of +0.10 rad over >=2 s and back. Its CSV/JSON show command streaming at >=100 Hz, max |target-position| below 0.10 rad, the torque below the script's justified bound, and a clean disable. An aborted run is recorded with its trigger and is not silently retried.",
      "The vendor-driver dry rehearsal ran only after scripts/motion/check_vendor_driver.py (MOT-11) reported no FAIL; its JSON is filed as vendor_check_*.json. It used `execute_trajectory --mode dry --profile vendor` against the live reBotArmController. The record has outcome rehearsed and no goal was ever sent, because dry never constructs the ActionClient. The driver was launched with safe_park disabled (VENDOR_DRIVER.md). joint_states before and after are unchanged apart from the driver's own start-up hold.",
      "The joint-range survey (E1) was readback-only, with every motor DISABLED and the arm supported by hand. It records the min/max reached per joint against b601_dm_limits.yaml and any contact found inside a URDF range. joint_ranges_verified is true only if no such contact was found. A contact opens a separate limits card; b601_dm_limits.yaml is never edited here.",
      "config/robot/commissioning.yaml is filled only from that evidence in one operator commit. limits_file_sha256 is the sha256 of the b601_dm_limits.yaml surveyed against. estop.verified is true with verified_utc, operator and estop.evidence paths. joint_ranges_verified is set per E1. evidence[] lists every MOT-09 file. speed_scale_cap stays 0.10, because no logged --mode real low-speed run exists yet. commissioned is true only when all of these hold. No other config file is changed.",
      "Nothing in this card runs --mode real, starts gravity compensation, calls set_zero/set_mode, writes motor registers or IDs, or changes the motor-side TIMEOUT. The vendor driver's own start-up mode switch is the one documented exception, made by the vendor code (VENDOR_DRIVER.md), and the mode is restored by power-cycling (D5)."
    ]
  },
  "hardware": [
    "arm_usb_connected",
    "arm_powered_estop_verified",
    "workspace_clear"
  ],
  "motion": true,
  "track": "physical",
  "id": "MOT-09",
  "title": "Arm bring-up commissioning (operator)",
  "parent": "L-MOTION",
  "outcome": "Arm powered on and scanned. Hardware e-stop wired and tested, unloaded and with joint1 holding. First low-gain joint1 move logged. Vendor-driver readiness verified (MOT-11), and an `execute_trajectory --mode dry --profile vendor` rehearsal recorded against the live driver. Joint ranges surveyed readback-only. config/robot/commissioning.yaml filled from that evidence. `--mode real` stays unreachable until GEOM-05/GEOM-07/MOT-10 measure the end-effector and scene (ADR-016 §3)."
}
```

Refined from draft 2026-10-10 by the overnight Claude session, on the operator's instruction, using the
operator's 2026-10-10 bring-up evidence (`evidence/operator/MOT-09/`). Operator procedure: no agent ever performs
any step of it (ADR-012/ADR-016). The overnight morning report is `~/rebot_setup/NEXT_SESSION.md`, outside the
repo; it carries the copy-pasteable checklist for the next session.

## Already established (2026-10-10, see `bringup_2026-10-10_summary.md`)
- Damiao USB2CAN `2e88:4603` on `/dev/ttyACM0`, dm-serial at 921600 baud, CAN 1 Mbps, 24 V. motorbridge 0.5.6 in
  conda env `rebot`; always run with `env -u PYTHONPATH`.
- Motor N is at CAN `0x0N`, feedback `0x1N`. M1–M3 are 4340P, M4–M7 are 4310. All in CTRL_MODE 1 (MIT). TIMEOUT=0
  on all, so no motor-side watchdog.
- Zero is Seeed's folded "sit-down" pose, equal to the URDF all-zero home (joint2/joint3 resting on upper = 0.0).
- Signs match the URDF for joint1–joint6. Gripper (M7) opening reads a **negative** motor angle.
- At rest, every joint was within 0.05 rad of zero.

**Not established:** any e-stop, any powered motion, `disable` behaviour, joint ranges, the vendor driver, or
commissioning. This card produces those.

## How this card stays inside ADR-016
The executor is used here only in `--mode dry` (Phase D), which is exactly how ADR-016 §3 scopes MOT-09. The
powered steps are operator bring-up actions outside `execute_trajectory`:

- C1 is the joint1 low-gain move.
- B5 and C3 are the e-stop tests.

ADR-016 already assigns this kind of work to MOT-09: §3 says MOT-09 "needs the arm moved at low speed", and §8
makes `enable`, `set_zero` and `set_mode` MOT-09 operator tooling. `first_move_j1.py` talks to motor 1 directly
over motorbridge. It publishes on no ROS topic and builds no `crackvision.joint_trajectory/1`, so it neither uses
nor bypasses any executor gate. joint1 carries no gravity load, so dropping its torque cannot make the arm fall.

## Procedure

Stop at the first failed expectation, record it, and do not improvise around it.

### A — Before power (24 V off)
- **A1.** Read this card, `~/rebot_setup/NEXT_SESSION.md`, `docs/motion/VENDOR_DRIVER.md` (MOT-11) and
  `docs/motion/EXECUTION.md` (MOT-05.6).
- **A2.** Read `~/rebot_setup/first_move_j1.py` in full. Run its offline tests and `--simulate` (NEXT_SESSION.md
  has the commands).
- **A3.** Hardware e-stop. Install an inline latching, normally-closed e-stop in the 24 V supply line between the
  PSU and the arm. Its contacts must be DC-rated for at least 24 V and the PSU's current. Mount it within reach
  of the operator position.
  - Until it exists, the operator alone decides whether the 24 V supply switch is an acceptable interim stop for
    the joint1-only steps (B5, C1, C3). That decision goes into `estop_test_<date>.md`.
  - Commissioning (`estop.verified: true`) needs the inline e-stop, or a documented operator decision naming the
    physical circuit that is relied on.
- **A4.** Clear the workspace around the base. Then run
  `claude-auto hw declare workspace_clear --note "..."`.
- **A5.** Device access.
  - If HOST-04 is accepted, plug in the USB2CAN with 24 V still off and check that `/dev/ttyACM0` (and the
    `/dev/ttyREBOT_ARM` symlink) show `crw-rw---- root plugdev`.
  - Otherwise use the stop-gap `sudo chmod 666 /dev/ttyACM0` and note it.

### B — Power on, read-only
- **B1.** Nothing may hold the port: `fuser -v /dev/ttyACM0`, `pgrep -ax ws_gateway`, `pgrep -ax reBotArmControl`.
- **B2.** Hand on the e-stop. Switch on 24 V. Damiao motors power up DISABLED.
- **B3.** Run a read-only `motorbridge-cli scan` over 0x01–0x20. Expect 7/7 at 0x0N/0x1N and nothing else.
  Save it as `power_on_scan_<date>.txt`.
- **B4.** Run `p1_registers.py`, then `first_move_j1.py --check-only`. Expect all DISABLED, CTRL_MODE 1, model
  registers correct, and |q| < 0.05 rad on all 7. The JSON is `rest_check_<date>.json`.
- **B5.** Unloaded e-stop test. Press the e-stop: the scan finds 0 motors. Release it: the scan finds 7/7, all
  DISABLED, positions unchanged. Write `estop_test_<date>.md` (who, when, which device, what was observed).

### C — First powered motion: joint1 only
- **C1.** Run `first_move_j1.py` and type `ENABLE J1` on the terminal. Expect joint1 to swing about 5.7° to the
  robot's **left** and back, about 7 s in all. Expect exit 0 and a CSV+JSON under `~/rebot_setup/runs/`.
  - On an abort, read the printed trigger, record it, and stop.
- **C2.** File the CSV and JSON as evidence (`first_move_j1_*.csv/json`).
- **C3.** Powered e-stop test. Run `first_move_j1.py --estop-test`: joint1 enables and holds. Press the e-stop
  within 30 s. Expect "feedback lost — expected for --estop-test". Release the e-stop and repeat B3 and B4.
  Append the result to `estop_test_<date>.md`.
- **C4.** Only after B5 and C3 pass with the e-stop that commissioning will rely on, run
  `claude-auto hw declare arm_powered_estop_verified --note "..."`.

### D — Vendor driver plus a dry rehearsal of the executor
The executor sends nothing in dry mode, but **the vendor driver energises every motor at launch**
(VENDOR_DRIVER.md).

Preconditions:
- MOT-05 is accepted, including MOT-05.7's QoS fix.
- MOT-11 is accepted.
- The operator has applied the vendor-side fixes listed in VENDOR_DRIVER.md: runtime dependencies for
  `/usr/bin/python3`, the SDK API mismatch, and safe_park disabled.

Steps:
- **D1.** Run `./env.sh python scripts/motion/check_vendor_driver.py --rebot-ws ~/rebot_ws --facts evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml --format json`.
  It must exit 0. Save the output as `vendor_check_<date>.json`.
- **D2.** Repeat B4: all joints at rest and CTRL_MODE 1.
- **D3.** Hand on the e-stop. In a `scripts/ros/env_ros.sh` shell, launch the driver as VENDOR_DRIVER.md
  specifies, with safe_park disabled. Expect the arm to energise in POS_VEL and hold all-zero, a move of at most
  about 0.05 rad.
- **D4.** In a second scrubbed shell, start the validity oracle and run
  `ros2 run crackvision_motion execute_trajectory --mode dry --profile vendor --trajectory ros2_ws/src/crackvision_motion/test/fixtures/trajectory_smoke.json --root "$PWD"`
  following EXECUTION.md. Expect exit 0 and outcome `rehearsed`. File the record from `logs/execution/` as
  `dry_rehearsal_<date>.json`.
- **D5.** Stop the driver as EXECUTION.md says. With safe_park disabled it safe-homes at most about 0.05 rad,
  then disables at the fold. Power-cycle the 24 V supply: the driver wrote CTRL_MODE=POS_VEL to RAM, and the
  stored MIT mode returns. Repeat B4.

### E — Joint-range survey (readback-only, every motor DISABLED)
- **E1.** Run the read-only logger (`p1_live_positions.py --duration 900`). Support the arm by hand and move one
  joint at a time towards each URDF limit, stopping at the first contact and never forcing. File the CSV as
  `joint_range_<date>.csv`, with a note of the reached ranges and any contacts.

### F — Commissioning record
- **F1.** Edit `config/robot/commissioning.yaml` from the evidence alone, in one commit (see the criteria).
  `speed_scale_cap` stays 0.10.
- **F2.** Run `claude-auto hw-done MOT-09 --base <sha before F1> --note "..."`. That runs the checks above, then
  an independent review.

## Known follow-ups (not part of this card)
- `disable` behaviour at a loaded pose. ADR-016 §7's possible-fall assumption stands until it is measured under
  controlled conditions. MOT-09 measures only joint1, which carries no load.
- Raising `speed_scale_cap`. That needs a logged low-speed `--mode real` run, which is reachable only after
  GEOM-05/07 and MOT-10.
- When F1 sets `commissioned: true`, MOT-05.3's accepted `shipped-uncommissioned` check (`^commissioned: *false`)
  will fail on its next re-review. The plan must decide how that check reads the shipped state, for example from
  its own reviewed commit.

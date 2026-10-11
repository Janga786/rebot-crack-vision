# MOT-12 — Vendor checker: evaluate the SDK layout its own V-SDK-API remedy produces (SDK pinned at 8427281, actuator/arm.py)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-11"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "scripts/motion/check_vendor_driver.py",
      "tests/test_check_vendor_driver.py",
      "tests/fixtures/vendor_driver/**",
      "docs/motion/VENDOR_DRIVER.md"
    ]
  },
  "inputs": [
    "plan/cards/MOT-11.md",
    "evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/INTERFACES.md#0",
    "docs/motion/ROBOT_MODEL.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "live-report",
        "cmd": "bash -c './env.sh python scripts/motion/check_vendor_driver.py --rebot-ws \"$HOME/rebot_ws\" --facts evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml --format json > /dev/null; rc=$?; echo rc=$rc; test $rc -eq 0 -o $rc -eq 3'",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "pinned-sdk-live",
        "cmd": "bash -c 't=$(mktemp -d); trap \"rm -rf $t\" EXIT; w=$t/ws; mkdir -p $w/src/rebotarm_bringup $w/src/rebotarmcontroller $w/third_party/reBotArm_control_py && cp -r \"$HOME/rebot_ws/src/rebotarm_bringup/config\" \"$HOME/rebot_ws/src/rebotarm_bringup/launch\" $w/src/rebotarm_bringup/ && cp -r \"$HOME/rebot_ws/src/rebotarmcontroller/rebotarmcontroller\" $w/src/rebotarmcontroller/ && git -c safe.directory=\"*\" -C \"$HOME/rebot_ws/third_party/reBotArm_control_py\" archive 8427281 reBotArm_control_py | tar -x -C $w/third_party/reBotArm_control_py || { echo \"could not build the 8427281 layout\"; exit 1; }; out=$(./env.sh python scripts/motion/check_vendor_driver.py --rebot-ws $w --format text 2>&1); rc=$?; echo \"$out\"; echo rc=$rc; test $rc -eq 0 -o $rc -eq 3 && echo \"$out\" | grep -qE \"^V-SDK-API +PASS\" && echo \"$out\" | grep -qE \"^V-BAUD +PASS\" && echo \"$out\" | grep -q \"/actuator/arm.py:\"'",
        "timeout_s": 180,
        "expect_exit": 0
      },
      {
        "id": "read-only",
        "cmd": "bash -c '! grep -nE \"import (motorbridge|serial|rclpy|subprocess)|from (motorbridge|serial|rclpy|subprocess) |os\\.system|Popen|rmtree|\\.unlink\\(|os\\.remove|shutil\\.(move|copy)\" scripts/motion/check_vendor_driver.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "When <sdk>/reBotArm_control_py/actuator/rebotarm.py is absent and actuator/arm.py is present (the SDK layout up to 8427281, which V-SDK-API's own remedy produces), the checker reads the dm-serial transport rule and the baud from arm.py and runs every check. V-BAUD then cites actuator/arm.py:<line> (the Controller.from_dm_serial call, arm.py:120 at 8427281). The checker exits 2 only when neither file exists. Today it exits 2 with 'required vendor file missing: .../actuator/rebotarm.py' against that layout.",
      "Unit tests add a fixture for the 8427281 layout: actuator/arm.py with `if self._channel.startswith(\"/dev/tty\"): return Controller.from_dm_serial(self._channel, 921600)`, actuator/__init__.py exporting RobotArm, controllers/__init__.py exporting ArmEndPos, and actuator/gripper.py defining load_cfg. The good variant gives exit 0. The variant without the ArmEndPos export gives exit 3 with V-SDK-API as the only FAIL. A variant with neither rebotarm.py nor arm.py gives exit 2. A unit test checks that a detached HEAD (.git/HEAD holding a raw commit hash, as `git checkout 8427281` leaves it) is reported as the SDK HEAD. The existing tests still pass, and the fixture tree's file hashes are unchanged after every run.",
      "docs/motion/VENDOR_DRIVER.md keeps the existing re-run command and adds three notes. (a) After the operator pins the SDK (`git -C ~/rebot_ws/third_party/reBotArm_control_py checkout 8427281`, an operator action outside this repo), the same command evaluates the pinned layout. (b) Driver dependencies installed outside the standard dirs (for example `pip install --target DIR`) are found with `--driver-pythonpath DIR`, and the driver must then run with the same DIR on PYTHONPATH. (c) The checker evaluates the vendor default config files under --rebot-ws, not launch-time overrides (`arm_config:=`, `safe_park_config:=`), so a config fix must be applied to those files for the checker to see it.",
      "The result pastes the output of the pinned-sdk-live check and the live report against the real ~/rebot_ws. Nothing is ever written under ~/rebot_ws: the pinned layout is built in a temporary directory with `git archive`, which only reads.",
      "All acceptance criteria of MOT-11 and MOT-11.R1 still hold at the new revision."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 1,
    "context": 3,
    "consequence": 4,
    "task_class": "python-pure-lib",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 90,
  "id": "MOT-12",
  "title": "Vendor checker: evaluate the SDK layout its own V-SDK-API remedy produces (SDK pinned at 8427281, actuator/arm.py)",
  "parent": "L-MOTION",
  "outcome": "check_vendor_driver.py reaches a verdict (exit 0 or 3, never 2) against ~/rebot_ws with the SDK pinned at 8427281. The operator can then verify the vendor fixes with the same command before MOT-09's dry rehearsal."
}
```

Operator-filed 2026-10-11 (overnight session, on the operator's instruction to verify the vendor driver before any real run).

## Why

MOT-11's checker is accepted, and against today's `~/rebot_ws` it correctly reports three FAILs: V-SAFE-PARK,
V-SDK-API and V-PYDEPS. The remedy it prints for V-SDK-API is to pin the vendored SDK to `8427281`, the last
commit that exports `RobotArm` and `ArmEndPos`. At that commit, though, the SDK's arm module is
`actuator/arm.py`; `actuator/rebotarm.py` only exists in later commits. The checker treats `rebotarm.py` as a
required file (`check_baud` reads it), so against the remedied tree it stops with exit 2, "required vendor file
missing". The operator can never get a verdict on the fix the checker asks for. MOT-09's step D1 (re-run the
checker, no FAIL) therefore cannot pass on the route the checker recommends.

## Overnight reproduction (read-only; nothing under `~/rebot_ws` was written)

The overnight session built a scratch copy outside the repo with the vendor `config/`, `launch/` and controller
sources, plus the SDK package from `git archive 8427281`:
- The accepted checker exits 2 against it.
- MOT-11's own check functions, with the single change that the baud check reads `actuator/arm.py` when
  `rebotarm.py` is absent, give V-SDK-API PASS and V-BAUD PASS. The `8427281` file selects dm-serial for
  `/dev/tty*` at 921600 (`arm.py:119-121`), the same rule as `rebotarm.py:485-487`. Every other check ran
  unchanged.
- The `pinned-sdk-live` acceptance check above fails today with `rc=2`. It is the regression test for this card.

## Scope notes

- Stay a pure, read-only static checker (MOT-11 criterion 1); the `read-only` grep is unchanged.
- Don't add launch-override flags here. The morning procedure applies config fixes to the vendor files themselves
  (the operator applies diffs outside this repo), so the default files are the ones that matter. Criterion (c)
  documents that limit.
- Don't change V-PYDEPS. `--driver-pythonpath` already covers out-of-tree dependencies; only document it.

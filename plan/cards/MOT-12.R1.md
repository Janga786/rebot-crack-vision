# MOT-12.R1 — Repair MOT-12: The detached-HEAD test depends on an untracked .git/HEAD fixture and f

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "docs/motion/VENDOR_DRIVER.md",
      "scripts/motion/check_vendor_driver.py",
      "tests/fixtures/vendor_driver/**",
      "tests/fixtures/vendor_driver/pinned_sdk",
      "tests/test_check_vendor_driver.py"
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
      "The test writes the detached HEAD at runtime, for example `(sdk_root/'.git').mkdir(exist_ok=True); (sdk_root/'.git'/'HEAD').write_text('8427281...\\n')` in _make_pinned_sdk_tree or in the test itself, instead of relying on a tracked .git path. `git archive HEAD | tar -x -C $t && cd $t && ./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider` must exit 0 with all tests passing.",
      "All acceptance criteria of MOT-12 still hold at the new revision"
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
  "priority": 95,
  "repairs": "MOT-12",
  "findings": [
    "04e7ad1b9a34d8b5"
  ],
  "id": "MOT-12.R1",
  "title": "Repair MOT-12: The detached-HEAD test depends on an untracked .git/HEAD fixture and f",
  "parent": "L-MOTION",
  "outcome": "Resolve the review findings on MOT-12 while every acceptance criterion of MOT-12 still holds."
}
```

Repair work generated deterministically from review attempt `00498-MOT-12-review` of `MOT-12`.
Original card: `plan/cards/MOT-12.md` — its acceptance criteria must still hold.

## Finding 1 [major] The detached-HEAD test depends on an untracked .git/HEAD fixture and fails in a clean checkout
- Evidence: The implementer lists tests/fixtures/vendor_driver/pinned_sdk/.git/HEAD in files_changed, but the commit's diffstat has only 7 files and `git ls-files tests/fixtures/vendor_driver/pinned_sdk` does not include it, because git refuses to track `.git` path components. `git archive edfbcc1 | tar -x -C $t; cd $t; ./env.sh pytest tests/test_check_vendor_driver.py -q` -> 'FAILED tests/test_check_vendor_driver.py::test_pinned_sdk_detached_head_reported_as_sdk_head / 1 failed, 18 passed'. The report there shows '(HEAD None)'.
- Consequence: The [unit] acceptance check passes only because of a stray untracked file in this working tree. Any fresh clone, CI run or later re-verification fails, and the detached-HEAD behaviour the card requires has no committed test.
- Affected: tests/test_check_vendor_driver.py, tests/fixtures/vendor_driver/pinned_sdk
- Acceptance condition: The test writes the detached HEAD at runtime, for example `(sdk_root/'.git').mkdir(exist_ok=True); (sdk_root/'.git'/'HEAD').write_text('8427281...\n')` in _make_pinned_sdk_tree or in the test itself, instead of relying on a tracked .git path. `git archive HEAD | tar -x -C $t && cd $t && ./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider` must exit 0 with all tests passing.

# MOT-11.R1 — Repair MOT-11: Unit tests depend on untracked empty fixture directories; 10/13 fail f

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
      "tests/fixtures/vendor_driver/good/deps",
      "tests/test_check_vendor_driver.py"
    ]
  },
  "inputs": [
    "evidence/operator/MOT-09/bringup_2026-10-10_facts.yaml",
    "evidence/operator/MOT-09/bringup_2026-10-10_summary.md",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/INTERFACES.md#0",
    "docs/motion/ROBOT_MODEL.md",
    "scripts/motion/compare_urdf.py"
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
        "id": "read-only",
        "cmd": "bash -c '! grep -nE \"import (motorbridge|serial|rclpy|subprocess)|from (motorbridge|serial|rclpy|subprocess) |os\\.system|Popen|rmtree|\\.unlink\\(|os\\.remove|shutil\\.(move|copy)\" scripts/motion/check_vendor_driver.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "From a clean checkout of the commit (git archive or git clone), `./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider` passes 13/13. For example, commit tracked placeholder files such as deps/motorbridge/__init__.py and deps/pinocchio/__init__.py, or create the deps tree in tmp_path inside the test.",
      "Every reported check carries a path:line (or path:start-end) citation into the vendor tree (or the repo file it checks), computed from the file content. On the live run, V-SAFE-PARK cites hardware_manager.py lines for _seed_endpos_target/connect and shutdown/safe_park; V-STARTUP cites connect/init_gripper lines and the SDK start; V-SDK-API cites the import lines; V-GRIPPER-UNITS cites ros_actions.py:213 and the set_gripper_target lines. A unit test asserts that each check's citation matches a ':<digits>' pattern.",
      "All acceptance criteria of MOT-11 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "python-pure-lib",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 99,
  "repairs": "MOT-11",
  "findings": [
    "7a2f5c671900ed31",
    "0c27cb7206b33376"
  ],
  "id": "MOT-11.R1",
  "title": "Repair MOT-11: Unit tests depend on untracked empty fixture directories; 10/13 fail f",
  "parent": "L-MOTION",
  "outcome": "Resolve the review findings on MOT-11 while every acceptance criterion of MOT-11 still holds."
}
```

Repair work generated deterministically from review attempt `00435-MOT-11-review` of `MOT-11`.
Original card: `plan/cards/MOT-11.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] Unit tests depend on untracked empty fixture directories; 10/13 fail from a clean checkout
- Evidence: tests/test_check_vendor_driver.py `_run` passes `--driver-pythonpath FIXTURES/deps`. Locally, tests/fixtures/vendor_driver/good/deps/{motorbridge,pinocchio} are empty dirs, and `git ls-files tests/fixtures/vendor_driver/good/deps` prints nothing. Running `git archive d52b2c5 | tar -x` then `./env.sh pytest tests/test_check_vendor_driver.py -q` gives '10 failed, 3 passed', including test_good_fixture_exits_zero_and_has_no_fail and every single-defect test, because V-PYDEPS adds an extra FAIL.
- Consequence: The accepted commit does not carry its own passing test suite. Any fresh clone or CI run fails, so the scheduler's PASS reflects a dirty working tree, not the reviewed revision.
- Affected: tests/fixtures/vendor_driver/good/deps, tests/test_check_vendor_driver.py
- Acceptance condition: From a clean checkout of the commit (git archive or git clone), `./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider` passes 13/13. For example, commit tracked placeholder files such as deps/motorbridge/__init__.py and deps/pinocchio/__init__.py, or create the deps tree in tmp_path inside the test.

## Finding 2 [major] Most checks lack file:line citations into ~/rebot_ws; card-specified line citations missing
- Evidence: Live text report: V-SAFE-PARK citation is '.../hardware_manager.py seeds this target at connect and parks to it on shutdown' with no :199-216,243-245. V-STARTUP is 'hardware_manager.py connect()/init_gripper()' with no :190-197,572-600 and no SDK ArmEndPos.start reference. V-SDK-API has no :38-39,573. V-GRIPPER-UNITS is 'ros_actions.py execute_gripper_command' with no :213 or :610-615. V-QOS PASS has an empty citation. V-PORT, V-IDS, V-ORDER, V-PROFILE and V-ARBITRATION give bare paths. The line numbers that do appear (ros_services.py:16-17, ros_publishers.py:7-8) are hard-coded strings, not located in the source.
- Consequence: The criterion explicitly requires a file:line citation for each check. After a vendor change, the operator cannot find the offending lines from the report, and the hard-coded line numbers go stale silently.
- Affected: scripts/motion/check_vendor_driver.py
- Acceptance condition: Every reported check carries a path:line (or path:start-end) citation into the vendor tree (or the repo file it checks), computed from the file content. On the live run, V-SAFE-PARK cites hardware_manager.py lines for _seed_endpos_target/connect and shutdown/safe_park; V-STARTUP cites connect/init_gripper lines and the SDK start; V-SDK-API cites the import lines; V-GRIPPER-UNITS cites ros_actions.py:213 and the set_gripper_target lines. A unit test asserts that each check's citation matches a ':<digits>' pattern.

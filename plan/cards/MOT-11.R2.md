# MOT-11.R2 — Repair MOT-11: pinned_sdk fixture's .git/HEAD still cannot be tracked by git; clean-c

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
      "tests/fixtures/vendor_driver/pinned_sdk/.git/HEAD",
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
      "From a clean checkout (e.g. `git archive HEAD | tar -x -C <tmp>` or a fresh `git clone`) of the commit, `./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider` passes all tests, including test_pinned_sdk_detached_head_reported_as_sdk_head, with the fixture's detached-HEAD state constructed at test run time (e.g. writing `.git/HEAD` into a tmp_path-based SDK tree inside `_make_pinned_sdk_tree`) rather than relying on a committed `.git` path.",
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
    "fe4e3d3bf2fbbcca"
  ],
  "id": "MOT-11.R2",
  "title": "Repair MOT-11: pinned_sdk fixture's .git/HEAD still cannot be tracked by git; clean-c",
  "parent": "L-MOTION",
  "outcome": "Resolve the review findings on MOT-11 while every acceptance criterion of MOT-11 still holds."
}
```

Repair work generated deterministically from review attempt `00500-MOT-11-rereview` of `MOT-11`.
Original card: `plan/cards/MOT-11.md` — its acceptance criteria must still hold.

## Finding 1 [blocker] pinned_sdk fixture's .git/HEAD still cannot be tracked by git; clean-checkout unit tests fail
- Evidence: `git ls-files tests/fixtures/vendor_driver/pinned_sdk` lists only the 4 .py files, never `.git/HEAD`, even though `tests/fixtures/vendor_driver/pinned_sdk/.git/HEAD` exists on disk with content `8427281aaaa...`. `git add -n` / `git check-ignore -v` on that path shows it is neither trackable nor matched by a .gitignore rule - git structurally refuses to index any path under a directory component literally named `.git`. Reproducing a clean checkout via `git archive HEAD | tar -x` and running `./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider` there gives `1 failed, 18 passed`, with the failure being `test_pinned_sdk_detached_head_reported_as_sdk_head`. This is exactly the previously-filed open finding (5cb420ec0159e286); only a repair *plan* (plan/cards/MOT-12.R1.md) exists for it at this revision, no actual fix.
- Consequence: Anyone doing a fresh git clone/CI checkout of this repo gets a failing required unit test for this card, blocking the acceptance check the scheduler itself runs; the scheduler's passing log was only captured because the implementer's local working tree still had the stray untracked file left over from a previous session.
- Affected: tests/fixtures/vendor_driver/pinned_sdk/.git/HEAD, tests/test_check_vendor_driver.py
- Acceptance condition: From a clean checkout (e.g. `git archive HEAD | tar -x -C <tmp>` or a fresh `git clone`) of the commit, `./env.sh pytest tests/test_check_vendor_driver.py -q -p no:cacheprovider` passes all tests, including test_pinned_sdk_detached_head_reported_as_sdk_head, with the fixture's detached-HEAD state constructed at test run time (e.g. writing `.git/HEAD` into a tmp_path-based SDK tree inside `_make_pinned_sdk_tree`) rather than relying on a committed `.git` path.

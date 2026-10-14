# OPS-01.3 — doctor stage: aggregate check_env/check_host/check_realsense into one operator report

**Accepted:** 2026-10-14 · **Work package:** Operator workflow & handoff (L-OPS) · **Track:** software · **Verified commit:** [`8bfd805`](https://github.com/Janga786/rebot-crack-vision/commit/8bfd8059df0b6d01dfb81268f88bcc7ec577b516)

## What was done

Added a single `operator_doctor.py` command that runs the existing environment, host, and RealSense camera diagnostics plus a ROS workspace build check, and prints one combined pass/warn/fail report with a JSON log. All 6 unit tests pass, and a live run on this machine showed the expected result (all green except the already-known no-GPU/no-camera warnings).

Files changed:
- [scripts/operator_doctor.py](https://github.com/Janga786/rebot-crack-vision/blob/8bfd8059df0b6d01dfb81268f88bcc7ec577b516/scripts/operator_doctor.py) (+216 / −0)
- [tests/test_operator_doctor.py](https://github.com/Janga786/rebot-crack-vision/blob/8bfd8059df0b6d01dfb81268f88bcc7ec577b516/tests/test_operator_doctor.py) (+150 / −0)

Commits: [`8bfd805`](https://github.com/Janga786/rebot-crack-vision/commit/8bfd8059df0b6d01dfb81268f88bcc7ec577b516)

## Why it was done

One command (`scripts/operator_doctor.py`) that runs the existing environment/host/camera diagnostics and the ROS overlay build check, and prints a single pass/warn/fail table with remediation pointers, per docs/INTERFACES.md §19's doctor definition.

- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Operator workflow & handoff: **4/12** tasks accepted; whole project: **136/170**.
- REQ-OPS-1: 23/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (unit ✔, help ✔); independent audit accepted it (criteria: 11 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “OPS-01.3 implementation is complete and correct. The scripts/operator_doctor.py command successfully aggregates the three existing check scripts (check_env.py, check_host.py, check_realsense.py) via subprocess with --json flags, parses their differing JSON schemas robustly, adds one new informational row for ros2_ws/install/setup.bash presence (WARN if missing), and outputs a combined human table plus JSON logs. Exit codes are correct (0 for success, 3 for failure, never 1 or 2 except usage errors). All 6 unit tests pass, properly stubbing subprocess calls without requiring
…[128 chars clipped]”

## What it unlocks next

- Closer: OPS-01.8 — operator_cli.sh dispatcher: doctor/capture/segment/paths/plan/preview/execute, resumable 'run', end-to-end headless test + OPERATOR_WORKFLOW.md (still needs OPS-01.4, OPS-01.5, OPS-01.6, OPS-01.7)

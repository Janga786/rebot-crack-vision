# HOST-01 — Host version manifest + drift/isolation checker

**Accepted:** 2026-09-28 · **Work package:** Host platform & reproducibility (L-HOST) · **Track:** software · **Verified commit:** [`6c4c62c`](https://github.com/Janga786/rebot-crack-vision/commit/6c4c62c83213f8f089f533edcf47ce075f2e4700)

## What was done

Added an automated host-health check that snapshots the machine's OS, driver, CUDA, ROS package, Gazebo, and crackvision-environment versions into a pinned manifest and re-verifies them on demand, catching silent drift before it breaks a demo. It also proves ROS and the perception environment stay properly isolated (no shared Python paths). All three required checks — the drift comparison, the manifest's JSON validity, and 14 unit tests on the comparison logic — pass with exit code 0.

Files changed:
- [config/host_versions.json](https://github.com/Janga786/rebot-crack-vision/blob/6c4c62c83213f8f089f533edcf47ce075f2e4700/config/host_versions.json) (+39 / −0)
- [docs/host/HOST_MANIFEST.md](https://github.com/Janga786/rebot-crack-vision/blob/6c4c62c83213f8f089f533edcf47ce075f2e4700/docs/host/HOST_MANIFEST.md) (+79 / −0)
- [scripts/check_host.py](https://github.com/Janga786/rebot-crack-vision/blob/6c4c62c83213f8f089f533edcf47ce075f2e4700/scripts/check_host.py) (+409 / −0)
- [tests/test_check_host.py](https://github.com/Janga786/rebot-crack-vision/blob/6c4c62c83213f8f089f533edcf47ce075f2e4700/tests/test_check_host.py) (+222 / −0)

Commits: [`6c4c62c`](https://github.com/Janga786/rebot-crack-vision/commit/6c4c62c83213f8f089f533edcf47ce075f2e4700)

## Why it was done

An automated check proves the host still matches a recorded, pinned software manifest and that ROS, perception and local inference stay isolated.

- Requirement **REQ-HOST-1**: Reproducible host setup with pinned, recorded versions and an automated drift check
- Requirement **REQ-HOST-2**: Isolation between ROS (system py3.10), perception (conda crackvision py3.11) and local inference, verified automatically

## How it moves the project forward

- Host platform & reproducibility: **4/9** tasks accepted; whole project: **15/69**.
- REQ-HOST-1: 1/4 contributing tasks done
- REQ-HOST-2: 1/2 contributing tasks done
- Verification: automated checks run by the pipeline itself (drift ✔, json ✔, unit ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “HOST-01 implements a stdlib-only drift/isolation checker that matches every acceptance check and criterion. All three scheduler commands re-run clean (exit 0/0/0). The nvidia_driver=null value was independently verified against a real nvidia-smi failure in this sandbox, confirming the implementer's claim rather than a bug. Isolation checks (no ROS on crackvision sys.path, ROS python imports rclpy/moveit_msgs, disjoint site-packages) all pass live. Tests are properly hermetic (fake inventories only, no real system calls). Scope is respected exactly. No blocking or major issues found.”

## What it unlocks next

- **Ready to start:** LLM-02 — Build pinned llama.cpp (CUDA, sm_86) in user space
- Closer: HOST-06 — Host reproducibility audit (fresh clone) (still needs HOST-02, LLM-02, TC-016, HOST-02, LLM-02)

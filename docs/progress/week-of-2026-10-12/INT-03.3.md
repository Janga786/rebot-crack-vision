# INT-03.3 — Unreachable-path failure-injection test (mock-stack planning)

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`5b963e7`](https://github.com/Janga786/rebot-crack-vision/commit/5b963e70f75ce8b5689fe333f6d3671de56f9c95)

## What was done

A new automated test now proves that when a crack is placed far outside the robot arm's reachable workspace, the planning software safely refuses to produce a plan rather than generating a bogus one: both the camera-view planner and the crack-path planner correctly exit with error code 3 in 1-2 seconds, with no plan file written and no crashes. Along the way the test surfaced a subtlety in the crack data format (two places a 3D position is stored) that had to be handled for the test itself to exercise the unreachable case correctly; the production planning code needed no changes.

Files changed:
- [docs/integration/safety/UNREACHABLE_PATH.md](https://github.com/Janga786/rebot-crack-vision/blob/5b963e70f75ce8b5689fe333f6d3671de56f9c95/docs/integration/safety/UNREACHABLE_PATH.md) (+120 / −0)
- [scripts/ros/test_safety_unreachable.sh](https://github.com/Janga786/rebot-crack-vision/blob/5b963e70f75ce8b5689fe333f6d3671de56f9c95/scripts/ros/test_safety_unreachable.sh) (+284 / −0)

Commits: [`5b963e7`](https://github.com/Janga786/rebot-crack-vision/commit/5b963e70f75ce8b5689fe333f6d3671de56f9c95)

## Why it was done

A new headless mock-stack script places a synthetic crack entirely outside the B601-DM reachability envelope and proves plan_view/plan_crack_path refuse cleanly (no committed plan, no hang, no crash) instead of producing a bogus plan, documented in docs/integration/safety/UNREACHABLE_PATH.md.

- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Integration & commissioning: **8/14** tasks accepted; whole project: **133/170**.
- REQ-INT-2: 19/26 contributing tasks done
- REQ-OPS-1: 21/33 contributing tasks done
- Verification: automated checks run by the pipeline itself (unreachable ✔); independent audit accepted it (criteria: 6 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “INT-03.3 adds a test script and a write-up. I checked them against the source, the scheduler log and the run artifacts, and every criterion holds, so I'm accepting. - **Exit code:** the code already defines exit code 3 for "no feasible candidate". In plan_view.py:438-442 the error (`PreconditionError`) is raised before `write_view_plan`. In plan_crack_path.py:1042-1047 it is raised before `write_plan`. - **Fault injection:** the injected crack is out of reach. In the artifact, all 38 waypoint positions have x between 5.26 and 5.32 m. - **Clean refusal:** neither plan file w
…[483 chars clipped]”

## What it unlocks next

- Closer: INT-03.5 — Assemble the safety case + unified failure-injection runner (still needs INT-03.4)

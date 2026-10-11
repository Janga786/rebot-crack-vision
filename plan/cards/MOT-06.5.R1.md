# MOT-06.5.R1 — Repair MOT-06.5: IK solver timeout equals the service-call wait timeout: infeasible IK 

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py",
      "ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "docs/motion/CARTESIAN_PROBE.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_sweep.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_apply.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py"
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
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "regression",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The IK solver timeout set in build_ik_request is strictly smaller than the wait timeout used for /compute_ik calls through MoveItClients. Either the request uses a smaller solver budget, or the IK call path adds a margin as reachability_sweep does. A unit test asserts that the IK request timeout plus a positive margin is at most the effective call timeout for the IK service (or equivalent).",
      "All acceptance criteria of MOT-06.5 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "ros-client",
    "local_ok": false
  },
  "track": "simulation",
  "priority": 95,
  "repairs": "MOT-06.5",
  "findings": [
    "5899f7273d3e73ea"
  ],
  "id": "MOT-06.5.R1",
  "title": "Repair MOT-06.5: IK solver timeout equals the service-call wait timeout: infeasible IK ",
  "parent": "MOT-06",
  "outcome": "Resolve the review findings on MOT-06.5 while every acceptance criterion of MOT-06.5 still holds."
}
```

Repair work generated deterministically from review attempt `00520-MOT-06.5-review` of `MOT-06.5`.
Original card: `plan/cards/MOT-06.5.md` — its acceptance criteria must still hold.

## Finding 1 [major] IK solver timeout equals the service-call wait timeout: infeasible IK can surface as a runtime timeout (exit 1) instead of ik_failed
- Evidence: cartesian_moveit.py build_ik_request: `timeout_s = float(cfg["service_timeout_s"])` ... `req.ik_request.timeout.sec = sec`, and MoveItClients.call defaults to `effective_timeout_s = self.timeout_s` (the natural construction is MoveItClients(node, cfg['service_timeout_s']) = 2.0 s). The existing reachability_sweep.py:402 guards this exact race: `_call_service(node, clients['ik'], req, cfg['ik']['timeout_s'] + _IK_CALL_TIMEOUT_MARGIN_S)` with `_IK_CALL_TIMEOUT_MARGIN_S = 2.0`. kinematics.yaml uses TRAC-IK, which runs for the full requested timeout when no solution exists; CARTESIAN_PROBE.md records NO_IK_SOLUTION for roll=0, which is the first candidate in roll_offsets_deg.
- Consequence: For an unreachable or singular candidate pose, MoveIt returns NO_IK_SOLUTION only after ~2.0 s, while the client stops waiting at 2.0 s. call() then raises ServiceCallTimeoutError, which escapes as a runtime failure (exit 1) instead of the candidate failing with `ik_failed` and the search moving on (§13.5 step 1). Nothing in this layer exposes or prevents the race, and no test covers the interaction between the IK request's timeout and the call timeout.
- Affected: ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_moveit.py, ros2_ws/src/crackvision_motion/test/test_cartesian_moveit.py
- Acceptance condition: The IK solver timeout set in build_ik_request is strictly smaller than the wait timeout used for /compute_ik calls through MoveItClients. Either the request uses a smaller solver budget, or the IK call path adds a margin as reachability_sweep does. A unit test asserts that the IK request timeout plus a positive margin is at most the effective call timeout for the IK service (or equivalent).

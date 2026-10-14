# INT-03.3 — Unreachable-path failure-injection test (mock-stack planning)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-06"
  ],
  "requirements": [
    "REQ-INT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/ros/test_safety_unreachable.sh",
      "docs/integration/safety/UNREACHABLE_PATH.md",
      "ros2_ws/src/crackvision_motion/crackvision_motion/plan_crack_path.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/plan_view.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#13",
    "docs/motion/CARTESIAN_PLANNING.md",
    "docs/motion/REACHABILITY.md",
    "scripts/ros/run_integration_e2e.sh",
    "scripts/ros/run_cartesian_synthetic.sh",
    "config/motion/cartesian_planning.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unreachable",
        "cmd": "bash scripts/ros/build_ws.sh && bash scripts/ros/test_safety_unreachable.sh",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The script follows the existing synthetic-case + moveit_mock-profile pattern (run_integration_e2e.sh / run_cartesian_synthetic.sh) but places the specimen/crack far enough outside the documented reachability envelope (docs/motion/REACHABILITY.md) that no IK candidate can succeed for any segment.",
      "ros2 run crackvision_motion plan_view and plan_crack_path are both run against it; the exit code asserted is the one the code's own contract already defines for \"no feasible candidate found\" (verified by reading the source, not assumed), and no cartesian_plan.json/view_plan.json is written in a form a downstream consumer could mistake for a valid, committable plan.",
      "The run is bounded by the planner's own documented --service-timeout-s / candidate budget -- the test asserts it terminates within that bound, not an arbitrary wrapper timeout, and produces no unhandled exception/traceback.",
      "If a genuine gap was found (a committable-looking plan file, a hang past the documented bound, or a crash), the fix in plan_crack_path.py/plan_view.py reuses the existing CartesianConfigError/PreconditionError/exit-code idiom; docs/INTERFACES.md itself is never edited by this card -- if the fix would require changing it, the card stops and reports instead.",
      "docs/integration/safety/UNREACHABLE_PATH.md documents the injection method, exact commands, observed exit codes/log output, and the safe-refusal evidence, citable by INT-03.5."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4,
    "task_class": "integration-safety-test",
    "local_ok": false
  },
  "track": "simulation",
  "id": "INT-03.3",
  "title": "Unreachable-path failure-injection test (mock-stack planning)",
  "parent": "INT-03",
  "outcome": "A new headless mock-stack script places a synthetic crack entirely outside the B601-DM reachability envelope and proves plan_view/plan_crack_path refuse cleanly (no committed plan, no hang, no crash) instead of producing a bogus plan, documented in docs/integration/safety/UNREACHABLE_PATH.md."
}
```

Write scripts/ros/test_safety_unreachable.sh reusing the moveit_mock bring-up pattern from scripts/ros/run_integration_e2e.sh / run_cartesian_synthetic.sh (build a scratch scene.yaml, copy end_effector.yaml/limits into the scratch root, bring up the mock MoveIt stack). Generate a synthetic case normally, then translate the specimen/crack pose (in the scratch paths3d.json and matching scratch scene.yaml) by several metres so every point sits well outside docs/motion/REACHABILITY.md's documented envelope -- keep paths3d.json otherwise schema-valid. Run ros2 run crackvision_motion plan_view and then plan_crack_path against it. Before writing assertions, read plan_crack_path.py/plan_view.py to find the exit code and file-write behaviour their own code already implements for the case where no candidate succeeds for any segment -- do not assume a particular exit code. Assert that behaviour holds: the documented exit code, no plan file written that a consumer could mistake for valid/committable, termination within the planner's own --service-timeout-s/candidate budget (not an arbitrary external timeout), and no unhandled exception. If this surfaces a genuine safety gap (a plan file that looks committable, a hang past the documented bound, or a crash), fix the minimal root cause in plan_crack_path.py/plan_view.py using their existing CartesianConfigError/PreconditionError/exit-code idiom; never edit docs/INTERFACES.md to make a fix fit -- stop and report instead if that would be required. Tear down the mock stack cleanly on exit (success or failure), following the existing scripts' process-group-kill pattern. Write docs/integration/safety/UNREACHABLE_PATH.md with the injection method, exact commands, observed exit codes/log excerpts, and the safe-refusal evidence, for INT-03.5 to cite.

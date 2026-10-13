# MOT-08.2 — Preview core: bind trajectory/plan/retime/check inputs, FK tool path, summary metrics and trajectory_preview/1 manifest (pure, no plotting)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.1"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/preview_core.py",
      "tests/test_preview_core.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#17",
    "src/crackvision/trajectory_check.py",
    "src/crackvision/kinematics.py",
    "config/robot/b601_dm_limits.yaml",
    "config/motion/execution.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_preview_core.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "independence",
        "cmd": "bash -c 'test -f src/crackvision/preview_core.py && ! grep -nE \"^\\s*(from|import)\\s+(crackvision_motion|rclpy|matplotlib)\" src/crackvision/preview_core.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "checker-regression",
        "cmd": "./env.sh pytest tests/test_trajectory_check.py tests/test_trajectory_check_path.py tests/test_trajectory_check_view.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Every §18 preview refusal rule raises a typed refusal exception carrying the reason. Tests cover a retime-report sha mismatch, a check-report sha mismatch, an invalid trajectory and a wrong plan schema.",
      "Summary metrics match §18 exactly: finite-difference v/a per §11.7, limits read from the limits file, headroom, and leg ranges taken from retime_report point_range under the shared-boundary convention. A hand-computed test case checks the numbers.",
      "validate_manifest is strict: unknown or missing keys and bad sha strings are reported. build_manifest followed by validate_manifest round-trips cleanly, and every path in it is repo-relative.",
      "Both a cartesian_plan/1 (crack_task) and a view_plan/1 (view_move) input are accepted.",
      "No matplotlib, crackvision_motion or rclpy import."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4,
    "task_class": "python-lib",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.2",
  "title": "Preview core: bind trajectory/plan/retime/check inputs, FK tool path, summary metrics and trajectory_preview/1 manifest (pure, no plotting)",
  "parent": "MOT-08",
  "outcome": "crackvision.preview_core validates and sha-binds the preview inputs, computes the §18 summary metrics and tool-tip path with crackvision.kinematics, and builds, validates and loads crackvision.trajectory_preview/1 manifests, with unit tests."
}
```

Implement src/crackvision/preview_core.py against INTERFACES §18 (conda side, Python 3.11, numpy + yaml only).

Public API (adjust names only if §18 dictates otherwise):
- `class PreviewRefusal(Exception)` (→ exit 3 in the CLIs) and `class PreviewUsageError(Exception)` (→ exit 2: unreadable or unparseable files).
- `bind_inputs(trajectory_path, plan_path, retime_report_path, check_report_path, *, root) -> dict`
  - Loads JSON and validates the trajectory with `crackvision.trajectory_check.validate_trajectory_schema`.
  - Validates the plan with `validate_cartesian_plan_schema` or `validate_view_plan_schema`, dispatched on the plan's `schema`. Reuse these validators; do not reimplement them.
  - Computes every sha256 and enforces the §18 sha-binding refusals.
  - Derives `check_report.all_pass` and `failed_checks`.
  - Returns the repo-relative paths, which requires every input to resolve under --root; otherwise raise PreviewUsageError.
- `tool_tip_path(traj, chain, end_effector) -> np.ndarray (N,3)`: same FK composition as `trajectory_check._tool_tip_positions` (`chain.fk(q) @ end_effector.T_gripper_link_tool_tip`).
- `compute_summary(traj, limits, tool_xyz, retime_report, preview_speed_scale) -> dict`, per §18. Durations scale as `t / s`. Tool speed scales by `s`.
- `build_manifest(bound, summary, artifacts, *, created_utc, producer) -> dict`, `validate_manifest(doc) -> list[str]` (empty = valid) and `load_manifest(path) -> dict` (raises on invalid).
- `file_sha256(path)`.

The limits file format is config/robot/b601_dm_limits.yaml. Read it with yaml and mirror how trajectory_check reads it. `preview_speed_scale` comes from execution.yaml `speed_scale.real_default`.

Tests (tests/test_preview_core.py) build tiny synthetic trajectories, plans, retime reports and check reports in tmp_path, the way tests/test_trajectory_check_path.py does: hand-built dicts with sha256 computed on the written files. Cover:
- every refusal;
- a hand-checked summary on a 3-point trajectory;
- shared-boundary leg ranges;
- the manifest round trip, plus rejection of an unknown key, a missing key and a non-hex sha;
- a failing check report giving `all_pass: false` without raising.
Use the real URDF and config/robot/end_effector.yaml for one FK sanity test: at the all-zero home, tool_tip agrees with `crackvision.kinematics` directly.

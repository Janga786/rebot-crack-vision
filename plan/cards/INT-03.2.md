# INT-03.2 — Invalid-depth failure-injection test (perception+geometry chain)

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-INT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/perc/test_safety_invalid_depth.sh",
      "docs/integration/safety/INVALID_DEPTH.md",
      "src/crackvision/path3d.py",
      "src/crackvision/lift3d.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0.2",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "src/crackvision/geometry.py#classify_depth",
    "docs/geometry/PATH3D_VERIFICATION.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "invalid-depth",
        "cmd": "bash scripts/perc/test_safety_invalid_depth.sh",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The script builds a normal synthetic case (real crack) through skeleton/paths as in run_synthetic_perception.sh, then before running path3d overwrites the capture record's depth PNG (depth.file, same dimensions) with the device's uint16 zero \"no-return\" sentinel for (a) 100% of pixels and (b) a partial subset, each run separately.",
      "For the 100%-invalid run: path3d's exit code is checked against what the code's own contract defines for zero valid points (read, not assumed); the paths3d.json's per-point entries are valid=false with a populated reason (never a silent point at the camera origin, per #9/#10); ineligible_reasons includes no_valid_points; execution_eligible=false.",
      "For the partial run: behaviour is checked against the #10.3 interpolation/split policy already documented in docs/geometry/PATH3D_VERIFICATION.md, with no crash and no silently-fabricated point for an invalid pixel.",
      "No unhandled exception in path3d's stderr in either run.",
      "If a genuine gap was found (crash, a fabricated point, or execution_eligible left true when it should be false), the fix in path3d.py/lift3d.py reuses the existing error-handling idiom; if none was found, neither file is touched.",
      "docs/integration/safety/INVALID_DEPTH.md documents the injection method, exact commands, observed output, and the safe-refusal evidence, citable by INT-03.5."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "integration-safety-test",
    "local_ok": false
  },
  "track": "simulation",
  "id": "INT-03.2",
  "title": "Invalid-depth failure-injection test (perception+geometry chain)",
  "parent": "INT-03",
  "outcome": "A new headless script corrupts a synthetic capture record's depth image (device-zero sentinel, full and partial) and proves crackvision.path3d never fabricates a 3D point from invalid depth and correctly marks the result ineligible, documented in docs/integration/safety/INVALID_DEPTH.md."
}
```

Write scripts/perc/test_safety_invalid_depth.sh: generate a normal synthetic case with a real crack (tools/synth_scene3d.py, same pattern as run_synthetic_perception.sh) and run it through skeleton/paths normally so a valid skeleton/paths.json exists. Before running path3d, read the capture record to find depth.file, then overwrite that PNG (same uint16 dtype/dimensions) with every pixel set to 0 -- geometry.classify_depth's documented 'device 0 no-return sentinel' -- and run ./env.sh python -m crackvision.path3d. Repeat with only a subset of pixels zeroed (e.g. a band or random ~50%) to exercise the mixed valid/invalid case. For the 100%-invalid run, read path3d.py's own contract for zero valid points before asserting an exit code (do not assume); load the written paths3d.json and assert every point has valid=false with a non-empty reason, no NaN leaking through as a fake coordinate, ineligible_reasons contains no_valid_points, and execution_eligible=false. For the partial run, cross-check the observed interpolation/split behaviour against the #10.3 policy already documented in docs/geometry/PATH3D_VERIFICATION.md and assert no crash and no fabricated point at an invalid pixel. Grep stderr in both runs for a traceback and fail loudly if found. If a genuine defect surfaces, fix the minimal root cause in path3d.py/lift3d.py using their existing error-handling idiom; otherwise leave them untouched. Write docs/integration/safety/INVALID_DEPTH.md with the injection method, exact commands, observed output, and why this is a safe refusal, for INT-03.5 to cite.

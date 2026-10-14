# INT-03.1 — Empty-mask failure-injection test (perception chain)

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-INT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/perc/test_safety_empty_mask.sh",
      "docs/integration/safety/EMPTY_MASK.md",
      "src/crackvision/skeleton.py",
      "src/crackvision/paths.py",
      "src/crackvision/path3d.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0.2",
    "docs/INTERFACES.md#10",
    "scripts/perc/run_synthetic_perception.sh",
    "src/crackvision/naming.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "empty-mask",
        "cmd": "bash scripts/perc/test_safety_empty_mask.sh",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The script reuses tools/synth_scene3d.py / the run_synthetic_perception.sh pattern to produce a synthetic case, then overwrites the stand-in prediction mask (naming.prediction_path) with an all-zero mask of the same dimensions before running skeleton/paths/path3d via ./env.sh python -m crackvision.<module>.",
      "Every step's exit code is examined against docs/INTERFACES.md #0.2 and asserted explicitly (a crack-free mask is a legitimate result, not a runtime error, unless the code's own contract says otherwise -- the reviewer checks the assertion matches what the code actually does, not an assumption).",
      "skeleton_pixels=0, paths.json component_count=0, and the resulting paths3d.json validates against docs/INTERFACES.md #10's schema with components=[] -- no fabricated point, no silent success claim.",
      "No Python traceback or unhandled exception appears in any step's stderr; the script fails loudly (non-zero exit, printed diagnosis) if one does.",
      "If a genuine gap was found (crash, malformed output, or a falsely-eligible result), the fix in skeleton.py/paths.py/path3d.py reuses that module's existing EXIT_RUNTIME/EXIT_PRECONDITION idiom rather than inventing a new exit-code category; if no gap was found, none of those three files are touched.",
      "docs/integration/safety/EMPTY_MASK.md states the injection method, exact commands, observed exit codes/outputs, and why this is a safe outcome, in enough detail for INT-03.5 to cite without re-deriving it."
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
  "id": "INT-03.1",
  "title": "Empty-mask failure-injection test (perception chain)",
  "parent": "INT-03",
  "outcome": "A new headless script feeds an all-zero (no-crack) mask through crackvision.skeleton -> crackvision.paths -> crackvision.path3d and proves the chain reaches a clean, schema-valid empty result with no crash and no fabricated crack, documented in docs/integration/safety/EMPTY_MASK.md."
}
```

Write scripts/perc/test_safety_empty_mask.sh following the established pattern of scripts/perc/run_synthetic_perception.sh (INT-01.1): generate a synthetic case with tools/synth_scene3d.py and complete the case_map.json entry the same way that script does, then, before the skeleton step, overwrite the stand-in prediction mask at naming.prediction_path with an all-zero PNG of the same (height, width) -- a legitimate 'no crack detected' input, not a malformed file. Run ./env.sh python -m crackvision.skeleton, crackvision.paths and crackvision.path3d against it, exactly as an operator would invoke them, under a fresh logs/ scratch root. Read each module's own exit-code contract (src/crackvision/skeleton.py, paths.py, path3d.py, docs/INTERFACES.md #0.2) before asserting an expected exit code -- do not assume EXIT_OK; assert whatever the code's own contract actually specifies for zero-crack input. Parse the written skeleton/paths/paths3d JSON files and assert: skeleton_pixels=0; paths.json component_count=0; paths3d.json is schema-valid per docs/INTERFACES.md #10 with components=[] and makes no eligibility claim it cannot back up. Grep every step's captured stderr for a traceback/unhandled exception and fail loudly if found. If this surfaces a genuine defect (a crash, a malformed/fabricated output, or a falsely-eligible result), fix the minimal root cause in the offending module using its own existing EXIT_RUNTIME/EXIT_PRECONDITION idiom -- do not touch any file you didn't need to fix. Finish by writing docs/integration/safety/EMPTY_MASK.md: the fault class, the exact injection method and commands, the observed exit codes and output, and why the result is a safe refusal/non-claim, written so INT-03.5 can cite it directly without re-deriving any of it.

# GEOM-08.8.R2 — Repair GEOM-08.8: Evidence doc's 'Specimen centre' narrative misstates the current state

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2",
    "REQ-INT-1"
  ],
  "scope": {
    "write": [
      "docs/COMPLETION_LOG.md",
      "docs/geometry/PATH3D_VERIFICATION.md",
      "tests/test_path3d_synthetic.py",
      "tools/synth_scene3d.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#3.13",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "src/crackvision/path3d.py",
    "src/crackvision/paths.py",
    "src/crackvision/kinematics.py",
    "src/crackvision/geometry.py",
    "src/crackvision/capture_record.py",
    "config/robot/end_effector.yaml",
    "config/motion/specimen_placement.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "synthetic-e2e",
        "cmd": "./env.sh pytest tests/test_path3d_synthetic.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      },
      {
        "id": "evidence-doc",
        "cmd": "test -s docs/geometry/PATH3D_VERIFICATION.md",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "docs/geometry/PATH3D_VERIFICATION.md's 'Specimen centre' section is updated to (a) accurately describe the current config/motion/specimen_placement.yaml content (feasible: true, centre (0.26, 0.0, 0.0)) rather than the stale 'feasible: false, placement: null' description, and (b) explain the actual reason the generator still uses the ADR-014 fallback in every run this card exercises -- that `specimen_center()` resolves the path against the generator's `--out`/scaffolded root, which never contains a copy of the live `config/motion/specimen_placement.yaml` -- rather than attributing it to the file being infeasible.",
      "All acceptance criteria of GEOM-08.8 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 4,
    "consequence": 4,
    "task_class": "verification-synthetic",
    "local_ok": false
  },
  "track": "calibration",
  "priority": 95,
  "repairs": "GEOM-08.8",
  "findings": [
    "8800fd10616a35de"
  ],
  "id": "GEOM-08.8.R2",
  "title": "Repair GEOM-08.8: Evidence doc's 'Specimen centre' narrative misstates the current state",
  "parent": "GEOM-08",
  "outcome": "Resolve the review findings on GEOM-08.8 while every acceptance criterion of GEOM-08.8 still holds."
}
```

Repair work generated deterministically from review attempt `00313-GEOM-08.8-rereview` of `GEOM-08.8`.
Original card: `plan/cards/GEOM-08.8.md` — its acceptance criteria must still hold.

## Finding 1 [major] Evidence doc's 'Specimen centre' narrative misstates the current state of config/motion/specimen_placement.yaml
- Evidence: docs/geometry/PATH3D_VERIFICATION.md:36-39 reads: "As of this card that file is a nominal 'no feasible placement found' result (`value_status: nominal`, `feasible: false`, `placement: null` ... not yet re-run since an earlier upstream change), so every run in this card instead falls back to the ADR-014 §3 nominal placement (0.29, 0)". But the current config/motion/specimen_placement.yaml (set by MOT-04.5, commit 9f2951b) has `feasible: true` and `placement: {center_xy_m: [0.26, 0.0], surface_z_m: 0.0, ...}`. I confirmed `tools/synth_scene3d.specimen_center(Path('.'))` (repo root) now returns `(array([0.26, 0., 0.]), 'config/motion/specimen_placement.yaml: placement.center_xy_m/surface_z_m')` -- i.e. the file is no longer infeasible, contradicting the doc's stated reason. Separately, I verified by direct reproduction (standalone script calling `generate_case`/`build_case_paths3d` exactly as the test does) that every exercised invocation (pytest's `tmp_path_factory` root, and the doc's own `--out /tmp/demo` examples) still lands on the ADR-014 fallback `center=[0.29, 0., -0.01]` and reproduces the exact table numbers (0.184mm/0.539mm etc.) -- because `_ensure_project_scaffold` only copies `config/project.yaml` into the generator's `--out` root, never `config/motion/specimen_placement.yaml`, so `specimen_center(root)` always hits its `OSError` fallback branch there regardless of the live repo file's content.
- Consequence: A reader of this evidence doc (e.g. GEOM-09 consuming these numbers for its uncertainty budget, or a future card deciding whether to re-run this verification after an upstream placement change) is told the fallback is used because no feasible placement exists yet, and would reasonably conclude that re-running MOT-04.3's recommend_placement would make this card pick up the real centre. That is false: under every documented invocation of synth_scene3d.py, the live repo's specimen_placement.yaml is structurally unreachable (it's never copied into the generator's scaffolded `--out` root), so the ADR-014 fallback centre (0.29, 0, -0.01) will keep being used even though the repo now has a different, real placement centre (0.26, 0, 0.0) available.
- Affected: docs/geometry/PATH3D_VERIFICATION.md, tools/synth_scene3d.py
- Acceptance condition: docs/geometry/PATH3D_VERIFICATION.md's 'Specimen centre' section is updated to (a) accurately describe the current config/motion/specimen_placement.yaml content (feasible: true, centre (0.26, 0.0, 0.0)) rather than the stale 'feasible: false, placement: null' description, and (b) explain the actual reason the generator still uses the ADR-014 fallback in every run this card exercises -- that `specimen_center()` resolves the path against the generator's `--out`/scaffolded root, which never contains a copy of the live `config/motion/specimen_placement.yaml` -- rather than attributing it to the file being infeasible.

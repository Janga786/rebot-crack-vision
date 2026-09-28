# CAM-03 — Recording validator with actionable diagnostics

```json card
{
  "kind": "impl",
  "depends_on": [
    "CAM-02"
  ],
  "requirements": [
    "REQ-CAM-1",
    "REQ-CAM-2"
  ],
  "scope": {
    "write": [
      "scripts/validate_recording.py",
      "tests/test_validate_recording.py"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_validate_recording.py -q"
      }
    ],
    "criteria": [
      "Checks: metadata completeness, depth scale present and plausible (D405 ≈ 1e-4 m/unit), timestamp monotonicity, colour/depth delta distribution, dropped frames, USB 2 fallback resolution, invalid-depth fraction per frame",
      "Each failure prints a cause and a concrete fix; exit codes per INTERFACES §0.2"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 2,
    "task_class": "script"
  },
  "track": "camera",
  "priority": 45,
  "id": "CAM-03",
  "title": "Recording validator with actionable diagnostics",
  "parent": "L-CAM",
  "outcome": "One command tells whether a recording is usable and exactly what is wrong if not."
}
```



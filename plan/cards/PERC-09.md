# PERC-09 — Path accuracy metrics on synthetic ground truth

```json card
{
  "kind": "impl",
  "depends_on": [
    "PERC-03"
  ],
  "requirements": [
    "REQ-PERC-2"
  ],
  "scope": {
    "write": [
      "tools/synth_cracks.py",
      "tests/test_path_accuracy.py",
      "docs/perception/PATH_ACCURACY.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_path_accuracy.py -q"
      }
    ],
    "criteria": [
      "Synthetic generator draws cracks with known centerlines (width, curvature, branches, noise)",
      "Metrics: symmetric Hausdorff and mean distance (px), coverage, ordering errors; thresholds justified and enforced"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3
  },
  "priority": 40,
  "id": "PERC-09",
  "title": "Path accuracy metrics on synthetic ground truth",
  "parent": "L-PERC",
  "outcome": "Path extraction accuracy is measured against known centerlines, with regression thresholds."
}
```



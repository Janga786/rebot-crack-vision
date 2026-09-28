# GEOM-06 — TCP (pivot) calibration solver + boresight procedure

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-01"
  ],
  "requirements": [
    "REQ-GEOM-3"
  ],
  "scope": {
    "write": [
      "src/crackvision/calibration/tcp.py",
      "src/crackvision/calibration/__init__.py",
      "tests/test_tcp.py",
      "docs/calibration/TCP_BORESIGHT_PROCEDURE.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_tcp.py -q"
      }
    ],
    "criteria": [
      "Least-squares pivot calibration with residuals; synthetic tests recover a known TCP within 0.1 mm under noise",
      "Procedure states fixtures, poses, acceptance thresholds and what is operator-measured vs computed"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4
  },
  "track": "calibration",
  "priority": 40,
  "id": "GEOM-06",
  "title": "TCP (pivot) calibration solver + boresight procedure",
  "parent": "L-GEOM",
  "outcome": "Software to calibrate the tool-centre point and a written procedure for physical boresight verification."
}
```



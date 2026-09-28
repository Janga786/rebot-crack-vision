# PERC-04 — Path visualization + pipeline stage

```json card
{
  "kind": "impl",
  "depends_on": [
    "PERC-03",
    "TC-011"
  ],
  "requirements": [
    "REQ-PERC-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/visualize_paths.py",
      "tests/test_visualize_paths.py",
      "run_test.sh"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_visualize_paths.py -q"
      },
      {
        "id": "syntax",
        "cmd": "bash -n run_test.sh"
      }
    ],
    "criteria": [
      "Overlay shows path order, direction arrows, branch colouring; never alters data artefacts",
      "run_test.sh gains an opt-in --paths stage"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 2,
    "consequence": 2,
    "task_class": "script"
  },
  "priority": 40,
  "id": "PERC-04",
  "title": "Path visualization + pipeline stage",
  "parent": "L-PERC",
  "outcome": "Operators see ordered paths (direction, order, branches) over the image, produced by the one-command pipeline."
}
```



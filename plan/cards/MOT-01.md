# MOT-01 — Robot model reconciliation (vendor vs presentation URDF) + limits file

```json card
{
  "kind": "research",
  "requirements": [
    "REQ-MOT-1"
  ],
  "scope": {
    "write": [
      "docs/motion/ROBOT_MODEL.md",
      "config/robot/b601_dm_limits.yaml",
      "scripts/motion/compare_urdf.py",
      "tests/test_compare_urdf.py"
    ]
  },
  "inputs": [
    "presentation/sim/reBot_B601_DM_with_gripper.urdf"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "limits",
        "cmd": "./env.sh python -c \"import yaml;d=yaml.safe_load(open('config/robot/b601_dm_limits.yaml'));J=d['joints'];assert len(J)>=6;assert all(all(k in J[j] for k in ('lower','upper','velocity','acceleration','source')) for j in J)\""
      },
      {
        "id": "pytest",
        "cmd": "./env.sh pytest tests/test_compare_urdf.py -q"
      }
    ],
    "criteria": [
      "Reads ~/rebot_ws read-only; records its git commit AND that moveit_config has 2 uncommitted files (which versions were used)",
      "compare_urdf.py diffs joints (axes, origins, limits) and links between the vendor model and presentation/sim URDF",
      "Every limit value cites its source; acceleration limits missing from URDF are taken from MoveIt joint_limits.yaml or flagged UNKNOWN (never invented)",
      "Canonical source chosen and justified; TCP/gripper frames identified"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4
  },
  "track": "simulation",
  "priority": 55,
  "id": "MOT-01",
  "title": "Robot model reconciliation (vendor vs presentation URDF) + limits file",
  "parent": "L-MOTION",
  "outcome": "One canonical B601-DM model and a sourced limits file (position, velocity, acceleration, effort) for all planning."
}
```



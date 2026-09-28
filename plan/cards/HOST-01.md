# HOST-01 — Host version manifest + drift/isolation checker

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-HOST-1",
    "REQ-HOST-2"
  ],
  "scope": {
    "write": [
      "scripts/check_host.py",
      "config/host_versions.json",
      "docs/host/HOST_MANIFEST.md",
      "tests/test_check_host.py"
    ]
  },
  "inputs": [
    "docs/MACHINE_STATE.md",
    "env.sh"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "drift",
        "cmd": "/usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json",
        "timeout_s": 600
      },
      {
        "id": "json",
        "cmd": "python3 -m json.tool config/host_versions.json >/dev/null"
      },
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_check_host.py -q"
      }
    ],
    "criteria": [
      "Manifest records: OS release, kernel (warn-only), NVIDIA driver, CUDA toolkits present, ROS distro and exact versions of ros-humble-{desktop,moveit,ros2-control,realsense2-camera,librealsense2}, gz-sim, crackvision env python/torch/nnunetv2/pyrealsense2/numpy, claude CLI (warn-only), bwrap",
      "Default mode compares live vs manifest: exit 0 on match, 1 on drift with a readable table; --record rewrites the manifest; --json output",
      "Isolation assertions: crackvision interpreter has no /opt/ros path on sys.path; ROS python (/usr/bin/python3 after sourcing Humble) imports rclpy and moveit_msgs; the two never share site-packages",
      "Uses only the stdlib under /usr/bin/python3 -sE; no sudo; never modifies anything"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "script"
  },
  "priority": 55,
  "id": "HOST-01",
  "title": "Host version manifest + drift/isolation checker",
  "parent": "L-HOST",
  "outcome": "An automated check proves the host still matches a recorded, pinned software manifest and that ROS, perception and local inference stay isolated."
}
```

Record today's live versions with `--record` (dpkg-query -W for ROS/gz packages, nvidia-smi, `./env.sh python -c` for the conda env,
`claude --version`). docs/host/HOST_MANIFEST.md explains each pin and how to update it deliberately. Unit-test the comparison logic
with fake inventories (no real system calls in tests).

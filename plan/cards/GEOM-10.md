# GEOM-10 — End-of-arm model: tool_tip + wrist D405 (nominal CAD prior) in the planning model

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-02",
    "GEOM-01"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-GEOM-2",
    "REQ-GEOM-3"
  ],
  "scope": {
    "write": [
      "config/robot/end_effector.yaml",
      "ros2_ws/src/crackvision_description/**",
      "ros2_ws/src/crackvision_motion/crackvision_motion/check_end_effector.py",
      "ros2_ws/src/crackvision_motion/launch/mock_planning.launch.py",
      "ros2_ws/src/crackvision_motion/package.xml",
      "ros2_ws/src/crackvision_motion/setup.py",
      "scripts/ros/test_end_effector.sh",
      "docs/adr/014-end-effector-frames-and-task-phases.md",
      "docs/adr/012-frames-and-conventions.md",
      "docs/INTERFACES.md",
      "docs/motion/ROBOT_MODEL.md",
      "docs/motion/ROS_WORKSPACE.md",
      "docs/TECHNICAL_APPROACH.md",
      "README.md"
    ]
  },
  "inputs": [
    "docs/adr/012-frames-and-conventions.md",
    "docs/motion/ROBOT_MODEL.md",
    "docs/calibration/PLAN.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "end-effector",
        "cmd": "bash scripts/ros/test_end_effector.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "mock-plan-regression",
        "cmd": "bash scripts/ros/test_mock_plan.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "sweep-regression",
        "cmd": "bash scripts/ros/test_reachability.sh",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "vendor-untouched",
        "cmd": "bash -c 'test -z \"$(cd ~/rebot_ws && git status --porcelain -- src/rebotarm_bringup/description src/rebotarm_moveit_config/config/rebotarm.urdf.xacro src/rebotarm_moveit_config/config/rebotarm.srdf)\"'",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Every number in config/robot/end_effector.yaml carries value_status nominal plus a provenance/source that a reviewer can re-derive: tool_tip = gripper_link origin (closed-finger tip from the canonical URDF finger meshes); camera_link = Seeed stock D405_305_Mount (pinned commit + sha256) placed as in the bowenszhu study and mapped by Rx(pi); housing box from realsense-ros 4.57.7 _d405.urdf.xacro; mount box from the mount mesh bounds. No value is presented as measured.",
      "The mount-side assumption (camera on the gripper face pointing up at the all-zero pose, gripper_link -Z) is stated as an operator-confirmable assumption, and the live check verifies the config encodes it (camera above gripper at q=0).",
      "The overlay xacro/SRDF only ADD frames, padded boxes and rigid-pair ACM entries to the unmodified vendor rebotarm.urdf.xacro/rebotarm.srdf; the SRDF arm chain tip stays gripper_tcp; no camera proxy is exempted from collision with link1..link6, base_link, the table or the specimen.",
      "Nominal realsense optical frames are only emitted with nominal_camera_frames:=true (mock), never by default, so a real realsense2_camera driver never gets a competing TF publisher.",
      "mock_planning.launch.py validates the config before MoveItConfigsBuilder (which only warns on a missing description) and fails loudly; check_end_effector proves URDF origins == config, FK reproduces T_gripper_link_{tool_tip,camera_link}, the home state is collision-free, and /compute_ik accepts tool_tip and camera_link as ik_link_name with FK agreement <= 1 mm / 0.5 deg.",
      "end_effector.assert_commissioning_ready refuses while any block is nominal (tested), for MOT-05 to call.",
      "ADR-014 and the appended INTERFACES §8 match the implementation; ADR-012 / ROBOT_MODEL / TECHNICAL_APPROACH carry amendment notes instead of silently rewritten history; ~/rebot_ws is not modified."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4
  },
  "track": "simulation",
  "priority": 70,
  "id": "GEOM-10",
  "title": "End-of-arm model: tool_tip + wrist D405 (nominal CAD prior) in the planning model",
  "parent": "L-GEOM",
  "outcome": "config/robot/end_effector.yaml is the single, provenance-tagged source of the tool tip and the eye-in-hand D405 pose on gripper_link; the mock MoveIt stack plans with those frames and padded camera/mount collision proxies; ADR-014 records why."
}
```

Implemented interactively by the technical-lead recovery session (2026-09-30) and submitted for
independent review with `claude-auto hw-done` (acceptance checks run by the scheduler machinery first).

Operator facts: the D405 is eye-in-hand on Seeed's stock D405_305_Mount (~15 deg pitch) on the B601-DM;
the installed extrinsic is NOT measured; CAD is a prior only; hand-eye calibration (GEOM-04/05) supersedes it.

Review focus: (1) re-derive the tool_tip claim from the canonical URDF finger meshes
(~/rebot_ws/src/rebotarm_bringup/description/meshes_b601_gripper/gripper_left.STL placed by gripper_joint1 at
q=0 -> max x in gripper_link); (2) re-derive the camera prior chain documented in the YAML `source` and ADR-014
§1 (bowenszhu simulation/simulate_fov.py constants; Rx(pi) mapping from the RS gripper_end / new DM end_link
frames); (3) confirm no collision checking against the environment or the arm links was weakened; (4) run the
acceptance checks.

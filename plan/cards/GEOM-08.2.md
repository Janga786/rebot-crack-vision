# GEOM-08.2 — Pure-numpy B601-DM forward kinematics + end-effector transforms (crackvision.kinematics)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-08.1",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/kinematics.py",
      "tests/test_kinematics.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#6.2",
    "docs/INTERFACES.md#8",
    "docs/INTERFACES.md#9",
    "docs/INTERFACES.md#10",
    "docs/motion/ROBOT_MODEL.md",
    "presentation/sim/reBot_B601_DM_with_gripper.urdf",
    "config/robot/end_effector.yaml",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_kinematics.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "no-ros-import",
        "cmd": "! grep -nE '^\\s*(import|from) (rclpy|moveit|tf2|geometry_msgs)' src/crackvision/kinematics.py",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The URDF fixed-axis rpy convention is R = Rz(yaw)·Ry(pitch)·Rx(roll), verified against scipy.",
      "The chain walk base_link→gripper_link uses the URDF's own joint origins and axes. Nothing is hand-copied.",
      "FK rejects wrong joint names or counts and positions beyond the limits by more than 1e-3 rad.",
      "The end_effector loader reproduces the yaml `derived.optical_axis_in_parent` [0.9659, 0, 0.2588] to 1e-4. This is an independent check of the camera prior's rpy.",
      "The loader raises when a measured block lacks the §10 `uncertainty` fields, and returns the §10 nominal priors for nominal blocks.",
      "Nominal optical transform: optical +z maps to camera_link +x, optical +x to −y, optical +y to −z."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 4,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-08.2",
  "parent": "GEOM-08",
  "title": "Pure-numpy B601-DM forward kinematics + end-effector transforms (crackvision.kinematics)",
  "outcome": "`crackvision.kinematics` computes T_base_link_gripper_link = FK(q) from the committed URDF copy, and loads T_gripper_link_camera_link / T_gripper_link_tool_tip, their value_status and §10 calibration sigmas, and the file sha256 from end_effector.yaml. It also exposes the nominal D405 camera_link→optical transform. No ROS import."
}
```

Create src/crackvision/kinematics.py (numpy, scipy, yaml and xml.etree only; no ROS).

API:
- `DEFAULT_URDF` = <root>/presentation/sim/reBot_B601_DM_with_gripper.urdf (MOT-01: kinematics identical to the vendor canonical file). `DEFAULT_END_EFFECTOR` = <root>/config/robot/end_effector.yaml.
- `file_sha256(path) -> str`.
- `transform_from_xyz_rpy(xyz, rpy) -> (4,4)`; `transform_from_xyz_quat(xyz, quat_xyzw)`; `transform_to_xyz_quat(T)`.
- `class KinematicsError(Exception)`.
- `load_chain(urdf_path=DEFAULT_URDF, base='base_link', tip='gripper_link') -> KinematicChain`.
  - Walk parent links from tip to base and keep the ordered joints: name, type, origin, axis, limits.
  - Fixed joints are included; revolute and continuous rotate about the axis; prismatic is supported but none is expected.
  - Records `urdf_sha256`.
- `KinematicChain.joint_names` must equal ['joint1'..'joint6'] for this model (assert).
- `KinematicChain.fk(positions)`: accepts a sequence of 6 floats in joint_names order, or a mapping. Returns T_base_link_gripper_link (4×4).
  - Raises KinematicsError on a missing or extra name, a non-finite value, or a position outside [lower − 1e-3, upper + 1e-3].
- `@dataclass EndEffector`, built by `load_end_effector(path=DEFAULT_END_EFFECTOR)`. Fields:
  - T_gripper_link_camera_link, T_gripper_link_tool_tip;
  - wrist_camera_value_status, tool_value_status, sha256;
  - camera_sigma_pos_m, camera_sigma_rot_rad, tool_sigma_pos_m, sigma_source.
  - Only the fields needed are read; full schema validation stays in ros2_ws/crackvision_description. Do not import it.
  - Nominal blocks get the §10 priors (0.010 m, 5° in rad, 0.005 m). Measured blocks must carry `uncertainty: {position_sigma_m, rotation_sigma_rad, source}` (the tool may omit rotation), else KinematicsError.
  - Also check that `parent_link == gripper_link`.
- `NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME`: xyz 0, rpy (−π/2, 0, −π/2).

Tests (tests/test_kinematics.py):
1. rpy convention against scipy `Rotation.from_euler('xyz', rpy)` (extrinsic xyz == URDF).
2. FK at q=0 equals the product of the joint origin transforms parsed independently in the test with ElementTree.
3. Rotating only joint1 by θ rotates the gripper_link position about joint1's axis through joint1's origin.
4. Limit, name and count rejection.
5. When ~/rebot_ws's vendor URDF exists, FK from it equals FK from the copy for 10 seeded q (skip otherwise).
6. `load_end_effector` on the real yaml:
   - the camera_link +X expressed in gripper_link equals yaml `derived.optical_axis_in_parent` (atol 1e-4);
   - the distance from the camera origin to tool_tip ≈ `derived.optical_centre_to_tool_tip_m` (atol 1e-3) — only if that derived value is defined from camera_link's origin; if not, test only what the yaml states;
   - statuses are nominal and the priors are returned.
7. A tmp yaml with a measured block and no uncertainty → KinematicsError; with uncertainty → values returned.
8. The nominal optical axis mapping.

Do not modify geometry.py, tcp.py or any ros2_ws file.

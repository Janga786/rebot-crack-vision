# CAM-05.2 — Static optical-frame TF capture + joint-state capture bridge (crackvision_camera ROS 2 package)

```json card
{
  "kind": "impl",
  "depends_on": [
    "CAM-05.1",
    "MOT-02"
  ],
  "requirements": [
    "REQ-CAM-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_camera/package.xml",
      "ros2_ws/src/crackvision_camera/setup.py",
      "ros2_ws/src/crackvision_camera/resource/crackvision_camera",
      "ros2_ws/src/crackvision_camera/crackvision_camera/__init__.py",
      "ros2_ws/src/crackvision_camera/crackvision_camera/capture_optical_tf.py",
      "ros2_ws/src/crackvision_camera/crackvision_camera/capture_joint_state.py",
      "ros2_ws/src/crackvision_camera/test/test_capture_optical_tf.py",
      "ros2_ws/src/crackvision_camera/test/test_capture_joint_state.py"
    ]
  },
  "inputs": [
    "docs/adr/015-camera-ros-integration-scope.md",
    "docs/INTERFACES.md#8.4",
    "docs/adr/012-frames-and-conventions.md",
    "plan/cards/GEOM-08.4.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/check_end_effector.py",
    "ros2_ws/src/crackvision_motion/package.xml"
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
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_camera/test -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "cli-help-tf",
        "cmd": "bash scripts/ros/env_ros.sh env PYTHONPATH=ros2_ws/src/crackvision_camera:ros2_ws/src/crackvision_motion python3 -m crackvision_camera.capture_optical_tf --help",
        "timeout_s": 60,
        "expect_exit": 0
      },
      {
        "id": "cli-help-js",
        "cmd": "bash scripts/ros/env_ros.sh env PYTHONPATH=ros2_ws/src/crackvision_camera:ros2_ws/src/crackvision_motion python3 -m crackvision_camera.capture_joint_state --help",
        "timeout_s": 60,
        "expect_exit": 0
      },
      {
        "id": "no-motion-code",
        "cmd": "bash -c '! grep -nE \"moveit_msgs|FollowJointTrajectory|ExecuteTrajectory|ActionClient\" ros2_ws/src/crackvision_camera/crackvision_camera/capture_optical_tf.py ros2_ws/src/crackvision_camera/crackvision_camera/capture_joint_state.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Both tools reuse crackvision_motion.cli_common.run_cli/add_common_args/PreconditionError/find_root (exec_depend crackvision_motion added to package.xml) rather than re-deriving §0 exit-code/logging/--dry-run plumbing — mirrors how scene_apply.py already imports crackvision_description.",
      "capture_optical_tf looks up tf2 with target_frame='camera_link', source_frame='camera_color_optical_frame' (a tf2_ros Buffer + TransformListener, bounded wait via --timeout-s, default 10.0 s), and on success writes TF.json = {'xyz_m': [x,y,z], 'quat_xyzw': [x,y,z,w]} (plus optional target_frame/source_frame/captured_at_utc metadata keys) to --output (default data/calibration/camera_optical_tf.json). This is the exact T_a_b direction ADR-012 defines: p_camera_link = T * p_optical.",
      "capture_optical_tf raises PreconditionError (exit 3) on timeout with an actionable message naming realsense2_camera as the expected TF publisher (e.g. 'is realsense2_camera running? ros2 launch realsense2_camera rs_launch.py ...'); --dry-run performs the lookup but writes no file.",
      "capture_joint_state subscribes /joint_states (sensor_msgs/msg/JointState), waits up to --timeout-s (default 10.0 s) for a message containing all six ARM_JOINTS (imported from crackvision_motion.check_end_effector.ARM_JOINTS, not redefined), reorders into canonical joint1..joint6 order regardless of incoming order, ignores extra names (e.g. gripper_joint1/2), and writes JS.json = {'joint_names': [...], 'positions_rad': [...], 'stamp_ns': int, 'stamp_source': str} to --output (default data/calibration/joint_state_<UTC>.json).",
      "stamp_ns comes from the message header.stamp when nonzero (stamp_source='joint_states_header'); otherwise from the node's clock at receipt (stamp_source='wall_clock_at_receipt'). Missing any of the six required joints after the timeout is a PreconditionError (exit 3), not a partial write.",
      "Tests run without a colcon build (same pattern as ros2_ws/src/crackvision_motion/test/test_cli_common.py: sys.path.insert for both ros2_ws/src/crackvision_camera and ros2_ws/src/crackvision_motion). test_capture_optical_tf.py publishes a known transform with tf2_ros.StaticTransformBroadcaster in-process and asserts the written TF.json matches within 1e-6; a no-publisher run asserts exit 3. test_capture_joint_state.py publishes a JointState with extra/out-of-order names and asserts correct reordering/filtering and both stamp_source paths; a no-publisher run asserts exit 3.",
      "console_scripts entry points capture_optical_tf and capture_joint_state are registered in setup.py, mirroring crackvision_motion's setup.py shape exactly (find_packages, ament resource marker, package.xml share data_files).",
      "Nothing under src/crackvision, docs/INTERFACES.md, or ros2_ws/src/crackvision_motion is modified by this card. source.kind: 'recording' assembly (replay-based capture records) is explicitly out of scope here, per GEOM-08.4's own note reserving it for CAM-05/INT-02 later."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "ros-node",
    "local_ok": false
  },
  "track": "camera",
  "id": "CAM-05.2",
  "parent": "CAM-05",
  "title": "Static optical-frame TF capture + joint-state capture bridge (crackvision_camera ROS 2 package)",
  "outcome": "A new ros2_ws/src/crackvision_camera package provides two small rclpy tools: capture_optical_tf (reads camera_link -> camera_color_optical_frame from realsense2_camera's published TF and writes TF.json) and capture_joint_state (reads /joint_states and writes JS.json), in exactly the formats GEOM-08.4's crackvision.capture_record CLI already specifies for its --optical-tf / --joint-state inputs. Both are fully unit-testable without a camera or arm connected."
}
```

Create a new ament_python ROS 2 package `ros2_ws/src/crackvision_camera`, scaffolded exactly like `ros2_ws/src/crackvision_motion` (same `package.xml` format-3 shape, same `setup.py` pattern with `find_packages`/ament resource index/`package.xml` share data). `package.xml` exec_depends: `rclpy`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`, `crackvision_motion` (for `cli_common`). Maintainer email `jangarabliss@gmail.com`.

**`capture_optical_tf.py`** — console_script `capture_optical_tf`. Reads the static transform `camera_link -> camera_color_optical_frame` that `realsense2_camera` publishes (ADR-012 §6.2: this is the only way this transform may be obtained; never hand-build it from `extrinsics_depth_to_color` metadata). Use `tf2_ros.Buffer` + `tf2_ros.TransformListener`, `buffer.lookup_transform('camera_link', 'camera_color_optical_frame', rclpy.time.Time(), timeout=Duration(seconds=args.timeout_s))`. On success write `--output` (default `data/calibration/camera_optical_tf.json`) as `{'xyz_m': [t.x, t.y, t.z], 'quat_xyzw': [q.x, q.y, q.z, q.w]}` plus a couple of informational extras (`target_frame`, `source_frame`, `captured_at_utc`). On timeout, raise `PreconditionError` naming what's missing. Extra flags: `--timeout-s` (float, default 10.0), `--output` (Path). `main_fn(args, log, summary)` does the rclpy init/spin/lookup/write; call `rclpy.shutdown()` in a `finally`.

**`capture_joint_state.py`** — console_script `capture_joint_state`. Subscribes `sensor_msgs/msg/JointState` on `/joint_states`, spins with `rclpy.spin_once` in a loop bounded by `--timeout-s` (default 10.0) until a message arrives whose `name` list is a superset of `crackvision_motion.check_end_effector.ARM_JOINTS` (`joint1..joint6`). Build `positions_rad` by indexing that message's `position` list per `ARM_JOINTS` order (do not assume the publisher's own order). `stamp_ns`: if `msg.header.stamp.sec or msg.header.stamp.nanosec`, use `stamp.sec * 1_000_000_000 + stamp.nanosec` and `stamp_source='joint_states_header'`; else use `node.get_clock().now().nanoseconds` and `stamp_source='wall_clock_at_receipt'`. Write `--output` (default `data/calibration/joint_state_<UTC>.json`, `crackvision_motion.cli_common.utc_stamp()` for the stamp) as `{'joint_names': list(ARM_JOINTS), 'positions_rad': [...], 'stamp_ns': int, 'stamp_source': str}`. Missing joints after timeout -> `PreconditionError`.

Both `main(argv)` functions follow the `check_end_effector.py` shape exactly: `run_cli(TOOL, _run, argv, extra_args=_extra_args)`.

Tests: build fixture nodes in the test process. For the TF test, start your own single-shot node, spin a `StaticTransformBroadcaster` publishing a known `TransformStamped` (`camera_link` -> `camera_color_optical_frame`, e.g. xyz `(0.01, -0.02, 0.03)`, a non-identity quaternion), then call the module's internal lookup function directly (factor the lookup out of `main_fn` into a small `_lookup(node, timeout_s) -> TransformStamped`-style helper so tests don't have to shell out) and assert round-trip values. For the joint-state test, publish a `JointState` with names in a scrambled order plus `gripper_joint1`/`gripper_joint2`, and assert the written JSON reorders/filters correctly. Cover the timeout path for both (no publisher running) and assert `main([...]) == 3`. Follow `test_cli_common.py`'s no-colcon-build technique: `sys.path.insert(0, str(Path(__file__).resolve().parents[1]))` for this package, plus `sys.path.insert(0, <repo_root>/ros2_ws/src/crackvision_motion)` to resolve `crackvision_motion.cli_common`/`check_end_effector`.

Do not add a launch file and do not add a continuously-publishing node — both tools are one-shot: look up/subscribe once, write a file, exit. That is the whole of CAM-05.1's ADR-015 decision made concrete.

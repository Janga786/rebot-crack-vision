# ROS_WORKSPACE.md — ROS 2 overlay workspace + headless MoveIt mock planning (MOT-02)

**Status:** NORMATIVE for motion-stack work in this repo.

## 0. What this is

This repo carries a small ROS 2 **overlay** workspace, `ros2_ws/`, that sits on top of
`~/rebot_ws/install` (the robot's own ROS 2 workspace, built and maintained outside this
project). The overlay never modifies `~/rebot_ws` — it is consumed strictly read-only,
exactly like a vendored dependency. `ros2_ws/build`, `ros2_ws/install` and `ros2_ws/log`
are colcon's own output directories; they are git-ignored (`.gitignore`) and regenerated
by `scripts/ros/build_ws.sh`.

The overlay currently contains one package, `crackvision_motion` (ament_python), which
provides a headless (no RViz) launch of MoveIt + `ros2_control` mock hardware for the
canonical B601-DM model (see `docs/motion/ROBOT_MODEL.md`, MOT-01), plus a small script
that sends it a plan-only joint-space goal — the smoke test that proves this repo can
plan against the real robot model without touching hardware.

## 1. `scripts/ros/env_ros.sh`

Mirrors `env.sh` (TC-002 / adr/007) for the ROS side: a shell with the crackvision conda
env active leaks a conda `PYTHONPATH`/`PATH` that shadows ROS Humble's system `python3`,
so rclpy nodes must be launched from a scrubbed shell.

`env_ros.sh`:
- unsets `PYTHONPATH`, `PYTHONHOME`, `CONDA_PREFIX`, `CONDA_DEFAULT_ENV`, `CONDA_SHLVL`,
  `CONDA_PYTHON_EXE`, and strips any `miniconda3`/`envs/crackvision` entries off `PATH`;
- relaxes `set -u` (`set +u`) around `source /opt/ros/humble/setup.bash` and
  `source ~/rebot_ws/install/setup.bash` — both reference variables that are unset in a
  fresh shell — then restores the caller's original `set -u` state;
- sets `ROS_LOCALHOST_ONLY=1` (leaves `ROS_DOMAIN_ID` as already set in the environment
  per project convention);
- points `ROS_LOG_DIR` at `ros2_ws/log/ros_home` inside this repo, since `~/.ros` is
  read-only under this project's sandbox;
- appends `$HOME/opt/nlopt/lib` to `LD_LIBRARY_PATH` if present, because
  `trac_ik_kinematics_plugin` (built into the `~/rebot_ws` underlay) `dlopen`s
  `libnlopt.so.0`, which is not on the default system loader path on this workstation.

Usage:
```bash
source scripts/ros/env_ros.sh              # activate in the current shell
./scripts/ros/env_ros.sh ros2 launch ...   # run one command in the scrubbed env
```

## 2. `ros2_ws/src/crackvision_motion`

An `ament_python` package:
- `launch/mock_planning.launch.py` — the same node graph as
  `rebotarm_moveit_config/launch/demo.launch.py` (`robot_state_publisher`,
  `ros2_control_node` with `mock_components/GenericSystem`, the
  `joint_state_broadcaster` / `rebotarm_controller` / `gripper_controller` spawners,
  `move_group`) minus RViz and the GUI dependency, so it runs headless. Exiting
  `move_group` (e.g. via SIGINT to the whole process group) cascades to
  `ros2_control_node` exiting and then to a full `LaunchService` shutdown.
- `crackvision_motion/plan_joint_goal.py` (console script `plan_joint_goal`) — sends one
  plan-only `moveit_msgs/action/MoveGroup` goal for the `arm` group (a joint-space target
  within `config/robot/b601_dm_limits.yaml` bounds, collision-free against the default
  scene) and exits 0 iff `MoveItErrorCodes.SUCCESS` comes back, 1 otherwise. It never sets
  `plan_only=False`, so no trajectory is ever executed — nothing here can move the mock
  (let alone real) arm.

## 3. `scripts/ros/build_ws.sh`

`source env_ros.sh`, `cd ros2_ws`, `colcon build --symlink-install`. Exit 0 on a clean
build.

## 4. `scripts/ros/test_mock_plan.sh`

1. Sources `env_ros.sh`, then `ros2_ws/install/setup.bash` (fails with a clear message if
   the overlay hasn't been built yet).
2. `set -m` (job control) so the backgrounded `ros2 launch crackvision_motion
   mock_planning.launch.py` gets its **own** process group, separate from the script's —
   without this, signalling "the launch's process group" during cleanup would also signal
   the test script itself.
3. Backgrounds the launch (output to `ros2_ws/log/mock_plan_launch.log`), records its PID
   and PGID, and checks it didn't die on startup.
4. Runs `ros2 run crackvision_motion plan_joint_goal`, which does its own
   `wait_for_server` (up to 90 s) via rclpy's graph discovery before sending the goal —
   deliberately not a separate `ros2 action list` polling loop from the shell, since that
   goes through the `ros2` CLI daemon and was observed to lag behind the action actually
   being available.
5. On exit (success, failure, or signal) a trap sends `SIGINT` to the launch's process
   group, waits up to 10 s for it to disappear, and falls back to `SIGKILL` — so no
   `move_group` / `ros2_control_node` / spawner / `robot_state_publisher` /
   `static_transform_publisher` process is ever left running.
6. Exits with `plan_joint_goal`'s exit code.

Verified locally (three consecutive runs from a clean `ros2_ws/build|install|log`):
```
$ bash scripts/ros/build_ws.sh          # exit 0
$ bash scripts/ros/test_mock_plan.sh
test_mock_plan.sh: launch pid=3402 pgid=3402 up, running plan_joint_goal (plan-only, no execution)
[INFO] [...] [plan_joint_goal]: plan succeeded for group 'arm' (error_code=1)
test_mock_plan.sh: plan_joint_goal exit=0, shutting down mock stack
(exit 0)
```
After each run, `ps aux` shows no leftover `move_group` / `ros2_control_node` /
`robot_state_publisher` / `static_transform_publisher` / spawner processes, and
`~/rebot_ws`'s `git status` is unchanged from the baseline recorded in
`docs/motion/ROBOT_MODEL.md` (the same 2 pre-existing uncommitted files, untouched).

## 5. Known limitation carried over from MOT-01

`trac_ik_kinematics_plugin` (the `arm` group's configured IK solver — see
`docs/motion/ROBOT_MODEL.md` §0) needs `libnlopt.so.0`, which `env_ros.sh` locates at
`$HOME/opt/nlopt/lib` on this workstation. The mock-planning smoke test only exercises
joint-space goals, which do not require IK, so this does not block MOT-02; a Cartesian
goal for the `arm` group would need that library on the loader path (or a rebuilt/repackaged
`trac_ik_kinematics_plugin`) to succeed.

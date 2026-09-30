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

## 5. Reachability sweep (MOT-04)

`crackvision_motion/reachability_sweep.py` (console script `reachability_sweep`) queries
move_group's `/compute_ik` for every `reachability_core.enumerate_targets()` x
`reachability_core.orientations()` sample of a `crackvision.reachability_config/1` config
(`docs/INTERFACES.md` §7.1), re-checks every accepted solution with an independent
`/compute_fk` call (position error ≤ 1 mm, boresight-axis error ≤ 0.5°), and writes a
`crackvision.reachability_map/1` JSON (`docs/INTERFACES.md` §7.2). It also adds a static
surface-collision slab per `surface_z` layer via `/get_planning_scene` /
`/apply_planning_scene` (extending the allowed-collision matrix, then restoring it and removing
the slab in a `finally` block, even on error) so IK's own collision checking accounts for the
inspected surface.

**Services/topics used, and nothing else:** `/compute_ik`, `/compute_fk`,
`/get_planning_scene`, `/apply_planning_scene` (all four services), plus the
`/robot_description` and `/robot_description_semantic` topics (URDF for
`reachability_core.reach_bound_from_urdf`'s prefilter bound, SRDF for the `home` group-state IK
seed fallback). There is no action client anywhere in the module and it never mentions
`MoveGroup`, `/move_action`, `ExecuteTrajectory` or `FollowJointTrajectory` — IK/FK are pure
kinematics queries against move_group's own solver, so nothing this node does can move the mock
(let alone real) arm, exactly like `plan_joint_goal` above but with an even smaller surface
(queries only, no plan/execute action at all).

### Commands

```bash
scripts/ros/env_ros.sh ros2 run crackvision_motion reachability_sweep --dry-run   # config only, no ROS needed
scripts/ros/run_reachability.sh --reachability-config CFG --out OUT [--require-all-reachable] [--max-duration-s N]
scripts/ros/test_reachability.sh   # smoke test against the fixture below
```

`run_reachability.sh` sources `env_ros.sh` and the built overlay itself (exit 3 with a hint if
`ros2_ws/install` doesn't exist yet), starts the same headless mock stack as
`test_mock_plan.sh` (via the shared `scripts/ros/_mock_stack.sh` helper, factored out of
`test_mock_plan.sh`'s launch/pgid/cleanup pattern), runs `reachability_sweep`, tears the stack
down, and exits with the sweep's exit code.

One footgun worth recording: `source env_ros.sh` with **no** explicit arguments inherits the
*caller's* positional parameters as env_ros.sh's own `"$@"`, which env_ros.sh's own tail
(`[ "$#" -gt 0 ] && exec "$@"`) then treats as "run this one command in the scrubbed env" and
tries to `exec` the caller's own `--flags`. `run_reachability.sh` hits this directly (it forwards
CLI flags to `reachability_sweep`), so it saves `"$@"` to an array, clears the positional
parameters (`set --`) before sourcing `env_ros.sh`, and restores them after.

### Fixture probe (`reachability_smoke.yaml`)

`ros2_ws/src/crackvision_motion/test/fixtures/reachability_smoke.yaml` is a 6-target config
(x ∈ {0.20, 0.95} m, y ∈ {−0.02, 0.0, 0.02} m, surface_z = 0.12 m, standoff = 0.06 m,
surface-collision **on**). The near/far split and the standoff were chosen from a live throwaway
probe against `/compute_ik` with the surface-collision slab applied and
`avoid_collisions: true` (matching production):

```
$ scripts/ros/env_ros.sh python3 <throwaway probe: reachability_core.orientations() @ (0.20,0,0.12)
  and (0.95,0,0.12), standoffs 0.01/0.04/0.06/0.08/0.10/0.12/0.15, slab added exactly as
  reachability_sweep.py would (margin_m=0.05, thickness_m=0.02, allowed_links=[base_link,link1])>
standoff 0.01 -> reachable 0/8   (every roll: NO_IK_SOLUTION)
standoff 0.04 -> reachable 0/8
standoff 0.06 -> reachable 7-8/8   <- used in the fixture
standoff 0.08 -> reachable 7/8
standoff 0.10 -> reachable 7/8
standoff 0.12 -> reachable 0/8
x=0.95, any y, standoff 0.06 -> reachable 0/8   (used as the "far" target)
```

The 0.01/0.04 m standoffs from the production `config/motion/reachability.yaml` fail on every
roll *with the slab active*: an FK sweep of every link at a known-good (no-slab) solution showed
`gripper_link` sitting **below** `gripper_tcp` by the same ~44.3 mm `docs/motion/ROBOT_MODEL.md`
§4 records for the fixed `gripper_tcp` joint (boresight-down inverts which end is "forward"), so
the ~4-5 cm gripper body straddles the 2 cm-thick slab for any standoff below ~0.06 m and IK
correctly reports no collision-free solution. This is a real geometric interaction with the
canonical model, not a bug — it means a real inspection run needs a standoff clearing the
gripper's own body, not just the fingertip. `x = 0.95` sits inside
`reach_bound_from_urdf`'s conservative sphere (≈1.007 m from the mock stack's own
`/robot_description`), so it reaches a real `/compute_ik` call and comes back genuinely
`unreachable` (not `prefiltered`) — this exercises the real IK-failure path, not just the
prefilter shortcut.

### Verified locally

```
$ bash scripts/ros/build_ws.sh                    # exit 0
$ bash scripts/ros/test_reachability.sh
...
test_reachability.sh: map assertions OK -- counts={'reachable': 3, 'unreachable': 3}
test_reachability.sh: no stray processes in launch pgid <pgid>
test_reachability.sh: OK
(exit 0)
```

The sweep itself (6 targets x 8 orientations, one surface_z layer) ran `complete: true` in
about 2.5 s; the whole `test_reachability.sh` run (launch bring-up + sweep + teardown) takes
about 30-40 s, dominated by move_group's own startup. `bash scripts/ros/test_mock_plan.sh`
(MOT-02) and the existing `pytest` suite under
`ros2_ws/src/crackvision_motion/test/` (92 tests) were re-run after this card's changes and
still pass unchanged. `--require-all-reachable` against the fixture (3/6 unreachable) exits 1;
`--max-duration-s` set below the sweep's natural runtime produces `complete: false` and a §0.4
`status: partial` (exit 1); with the mock stack not running, `--service-timeout-s 3` exits 3
within the timeout and writes no map. After every run, `ps aux` shows no leftover `move_group` /
`ros2_control_node` / `robot_state_publisher` / spawner process, matching MOT-02's own teardown
guarantee.

## 6. Known limitation carried over from MOT-01

`trac_ik_kinematics_plugin` (the `arm` group's configured IK solver — see
`docs/motion/ROBOT_MODEL.md` §0) needs `libnlopt.so.0`, which `env_ros.sh` locates at
`$HOME/opt/nlopt/lib` on this workstation. The mock-planning smoke test only exercises
joint-space goals, which do not require IK, so this does not block MOT-02; a Cartesian
goal for the `arm` group would need that library on the loader path (or a rebuilt/repackaged
`trac_ik_kinematics_plugin`) to succeed.

## 7. End-of-arm overlay and task-frame reachability (ADR-014)

The mock stack now plans against the vendor model **plus** `crackvision_description`: `tool_tip`,
the wrist D405 `camera_link`, and padded camera/mount collision proxies, all placed from
`config/robot/end_effector.yaml` (every value nominal until GEOM-05/07). `~/rebot_ws` is still only
read. `mock_planning.launch.py` validates that file first and refuses to start on an invalid one.
It also adds realsense's nominal optical frames, because no camera driver runs in the mock.

```
$ bash scripts/ros/test_end_effector.sh              # 21 offline tests + live check_end_effector
  PASS: URDF joint origin of 'tool_tip' / 'camera_link' / proxies equal the config
  PASS: FK T_gripper_link_tool_tip / T_gripper_link_camera_link match (|dp| < 1e-16 m)
  PASS: all-zero (SRDF home) state is collision-free with the camera proxies
  PASS: camera_link is above gripper_link at the zero pose (dz=+0.0640 m)
  PASS: compute_ik with ik_link_name='tool_tip' / 'camera_link' ... FK agrees (0.000 mm, 0.000 deg)
$ bash scripts/ros/test_reachability_task_frames.sh  # tool_tip @0.01/0.04 m + camera view @0.25 m
  test_reachability_task_frames.sh: tool counts={'reachable': 6, 'unreachable': 6}; camera view reachable (tilt 0.0 deg)
```

MoveIt's `/compute_ik` accepts any frame rigidly attached to the SRDF chain tip as `ik_link_name`, so
the `arm` chain stays `base_link → gripper_tcp`. The sweep's `specimen_block` proxy and `environment`
table (INTERFACES §8.3) replace the old grid-wide thin slab. That slab ran under the base and
collided with link2 at surface_z ≥ ≈0.10 m. The legacy `slab` model is still the default, so
`test_reachability.sh` above is unchanged and still passes.

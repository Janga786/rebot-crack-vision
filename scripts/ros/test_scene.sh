#!/usr/bin/env bash
# scripts/ros/test_scene.sh — MOT-03 scene-stack smoke test: unit tests, baseline plan, apply the
# production scene (table + specimen; no world camera object, ADR-014), assert the objects are
# live, re-plan, then apply an oversized collider fixture on top and assert the same goal is
# rejected. Mirrors scripts/ros/test_reachability.sh's launch/pgid/cleanup pattern via
# scripts/ros/_mock_stack.sh.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

source "$SCRIPT_DIR/env_ros.sh" || exit 1

echo "== unit tests: scene_core (pure python, no ROS) =="
python3 -m pytest "$REPO_ROOT/ros2_ws/src/crackvision_motion/test/test_scene_core.py" -q -p no:cacheprovider || exit 1

[ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ] || { echo "run scripts/ros/build_ws.sh first" >&2; exit 1; }
set +u
source "$REPO_ROOT/ros2_ws/install/setup.bash"
set -u

set -m
source "$SCRIPT_DIR/_mock_stack.sh"
mkdir -p "$REPO_ROOT/ros2_ws/log"
start_mock_stack "$REPO_ROOT/ros2_ws/log/test_scene_launch.log" || exit 1

echo "== baseline: default goal with no crackvision scene applied =="
ros2 run crackvision_motion plan_joint_goal || { echo "baseline plan failed" >&2; exit 1; }

echo "== apply config/scene/scene.yaml =="
ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" --config "$REPO_ROOT/config/scene/scene.yaml" || exit 1

echo "== assert table/specimen appear in the planning scene =="
ros2 run crackvision_motion assert_scene_objects --object-id table --object-id specimen || exit 1

echo "== default goal still plans with the nominal production scene applied =="
ros2 run crackvision_motion plan_joint_goal || { echo "plan failed after nominal scene" >&2; exit 1; }

echo "== apply the oversized collider fixture on top =="
ros2 run crackvision_motion apply_scene --root "$REPO_ROOT" --config "$REPO_ROOT/ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml" || exit 1

echo "== same goal must now be rejected =="
ros2 run crackvision_motion plan_joint_goal
if [ $? -eq 0 ]; then
    echo "expected rejection, got success" >&2
    exit 1
fi
echo "test_scene.sh: colliding goal correctly rejected"

echo "test_scene.sh: OK"
exit 0

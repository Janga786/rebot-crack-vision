"""assert_scene_objects — confirm collision-object ids are present in the live planning scene (MOT-03).

console_script: `ros2 run crackvision_motion assert_scene_objects --object-id ID [--object-id ID ...]`.
Only ever calls `/get_planning_scene`. Exits 0 if every `--object-id` is present in
`world.collision_objects`, 1 otherwise (missing ids listed on stderr), so `scripts/ros/test_scene.sh`
can prove "scene objects appear in the planning scene" without hand-parsing `ros2 service call`
output.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node

SERVICE_WAIT_TIMEOUT_S = 90.0
CALL_TIMEOUT_S = 10.0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="assert_scene_objects")
    parser.add_argument("--object-id", action="append", dest="object_ids", required=True)
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = Node("assert_scene_objects")
    try:
        client = node.create_client(GetPlanningScene, "/get_planning_scene")
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_S):
            print("assert_scene_objects: '/get_planning_scene' did not become available", file=sys.stderr)
            return 1

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT_S)
        if not future.done():
            print("assert_scene_objects: get_planning_scene call timed out", file=sys.stderr)
            return 1
        resp = future.result()
        if resp is None:
            print("assert_scene_objects: get_planning_scene call failed", file=sys.stderr)
            return 1

        present = {obj.id for obj in resp.scene.world.collision_objects}
        missing = [oid for oid in args.object_ids if oid not in present]
        if missing:
            print(
                f"assert_scene_objects: missing object id(s) {missing}; present={sorted(present)}",
                file=sys.stderr,
            )
            return 1
        node.get_logger().info(f"all requested object ids present in the planning scene: {args.object_ids}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

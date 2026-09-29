"""scene_apply — build and apply MoveIt collision objects + ACM entries from a scene config (MOT-03).

console_script: `ros2 run crackvision_motion apply_scene`. Loads a `crackvision.scene_config/1`
YAML (docs/motion/SCENE.md), builds one box CollisionObject per configured object and extends the
*current* AllowedCollisionMatrix with the configured allowed_collisions pairs (so calling this
more than once layers scenes rather than clobbering them), then applies both as a single
PlanningScene diff via `/apply_planning_scene`. Only ever calls `/get_planning_scene` and
`/apply_planning_scene` -- no action client anywhere in this file, so nothing here can move the
mock (let alone real) arm.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .scene_core import SceneError, load_config

DEFAULT_CONFIG = "config/scene/scene.yaml"
SERVICE_WAIT_TIMEOUT_S = 90.0
SCENE_CALL_TIMEOUT_S = 10.0


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def build_collision_objects(config: Dict[str, Any], operation: int = CollisionObject.ADD) -> List[CollisionObject]:
    """One CollisionObject per configured object. `operation=CollisionObject.REMOVE` builds
    remove-ops (id + operation only, no geometry needed)."""
    objects: List[CollisionObject] = []
    for obj in config["objects"]:
        co = CollisionObject()
        co.header.frame_id = obj["pose"]["frame"]
        co.id = obj["id"]
        co.operation = operation
        if operation == CollisionObject.ADD:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = list(obj["dimensions_m"])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = obj["pose"]["position_m"]
            qx, qy, qz, qw = _quat_from_rpy(*obj["pose"]["rpy_rad"])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
            co.primitives = [prim]
            co.primitive_poses = [pose]
        objects.append(co)
    return objects


def extend_acm(acm: AllowedCollisionMatrix, config: Dict[str, Any]) -> AllowedCollisionMatrix:
    """Symmetric NxN -> (N+k)x(N+k) matrix, adding one row/col per configured object id not
    already present, `enabled=True` only for the pairs listed in config['allowed_collisions']
    (in either order), `False` (i.e. collision-checked) for every other pair -- including against
    every pre-existing robot link."""
    old_names = list(acm.entry_names)
    old_rows = [list(v.enabled) for v in acm.entry_values]
    new_ids = [obj["id"] for obj in config["objects"] if obj["id"] not in old_names]

    allowed_pairs = set()
    for entry in config["allowed_collisions"]:
        allowed_pairs.add((entry["link_a"], entry["link_b"]))
        allowed_pairs.add((entry["link_b"], entry["link_a"]))

    def is_allowed(a: str, b: str) -> bool:
        return a != b and (a, b) in allowed_pairs

    all_names = old_names + new_ids
    new_rows: List[List[bool]] = []
    for i, name_i in enumerate(all_names):
        if i < len(old_names):
            new_rows.append(old_rows[i] + [is_allowed(name_i, nid) for nid in new_ids])
        else:
            new_rows.append([is_allowed(name_i, nid) for nid in all_names])

    new_acm = AllowedCollisionMatrix()
    new_acm.entry_names = all_names
    new_acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in new_rows]
    new_acm.default_entry_names = list(acm.default_entry_names)
    new_acm.default_entry_values = list(acm.default_entry_values)
    return new_acm


def _call_service(node: Node, client, request, timeout_s: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        future.cancel()
        return None
    return future.result()


def apply_scene(node: Node, get_scene_client, apply_scene_client, config: Dict[str, Any]) -> None:
    """Fetch the current ACM, extend it per `config`, and apply the object adds + ACM diff in one
    ApplyPlanningScene call. Raises RuntimeError on any service failure."""
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    resp = _call_service(node, get_scene_client, req, SCENE_CALL_TIMEOUT_S)
    if resp is None:
        raise RuntimeError("get_planning_scene call failed while reading the ACM")

    new_acm = extend_acm(resp.scene.allowed_collision_matrix, config)
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = build_collision_objects(config)
    scene.allowed_collision_matrix = new_acm

    apply_req = ApplyPlanningScene.Request()
    apply_req.scene = scene
    apply_resp = _call_service(node, apply_scene_client, apply_req, SCENE_CALL_TIMEOUT_S)
    if apply_resp is None or not apply_resp.success:
        raise RuntimeError("apply_planning_scene call failed")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="apply_scene")
    parser.add_argument("--config", type=Path, default=Path(DEFAULT_CONFIG), help="scene config YAML path")
    parser.add_argument("--root", type=Path, default=None, help="project root (default: cwd)")
    args = parser.parse_args(argv)

    root = args.root.expanduser().resolve() if args.root is not None else Path.cwd()
    config_path = args.config if args.config.is_absolute() else (root / args.config)

    try:
        config = load_config(config_path)
    except (SceneError, OSError) as exc:
        print(f"apply_scene: invalid scene config {config_path}: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = Node("apply_scene")
    try:
        get_scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        apply_scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        for client, name in (
            (get_scene_client, "/get_planning_scene"),
            (apply_scene_client, "/apply_planning_scene"),
        ):
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_S):
                print(f"apply_scene: service '{name}' did not become available", file=sys.stderr)
                return 3
        apply_scene(node, get_scene_client, apply_scene_client, config)
        ids = ", ".join(obj["id"] for obj in config["objects"])
        node.get_logger().info(f"applied scene from {config_path}: objects=[{ids}]")
        return 0
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

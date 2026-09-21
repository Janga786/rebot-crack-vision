#!/usr/bin/env python
"""Diagnostic (not a deliverable): at q=0, find which LOCAL axis of gripper_link
the actual finger meshes extend along, by transforming their world-space AABB
corners into gripper_link's own frame (frame taken from arm_kinematics.fk(0),
already verified to match world -Z on the local +Z axis)."""
import sys, time, os
sys.path.insert(0, "/home/boosterk1/Projects/rebot_crack_vision/presentation/demo")
import arm_kinematics as ak
import numpy as np

t0 = time.time()
def log(m): print(f"[t={time.time()-t0:6.1f}s] {m}", flush=True)

log("launching SimulationApp")
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
log("constructed")

import omni.usd, omni.kit.commands
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from pxr import UsdGeom, Usd

URDF_PATH = "/home/boosterk1/Projects/rebot_crack_vision/presentation/sim/reBot_B601_DM_with_gripper.urdf"

stage_utils = omni.usd.get_context()
stage_utils.new_stage()
stage = stage_utils.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()
_, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False
cfg.fix_base = True
cfg.create_physics_scene = True
cfg.make_default_prim = False
success, robot_path = omni.kit.commands.execute(
    "URDFParseAndImportFile", urdf_path=URDF_PATH, import_config=cfg, get_articulation_root=True)
log(f"import success={success} path={robot_path}")
simulation_app.update()

world = World()
world.reset()
art = SingleArticulation(prim_path=robot_path)
art.initialize()
# hold at all-zero pose
names = list(art.dof_names)
targets = np.zeros(len(names))
for i, n in enumerate(names):
    if n in ("gripper_joint1", "gripper_joint2"):
        targets[i] = 0.02
art.set_joint_positions(targets)
for _ in range(10):
    world.step(render=False)
log(f"dof positions after hold: {dict(zip(names, art.get_joint_positions()))}")

# find prim paths for gripper_link / gripper_left / gripper_right under robot_path
def find_prims(stage, root, needles):
    found = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        name = prim.GetName()
        for needle in needles:
            if name == needle:
                found[needle] = prim.GetPath().pathString
    return found

robot_root = "/" + robot_path.strip("/").split("/")[0]
log(f"robot_root={robot_root}")
all_names = [p.GetName() for p in Usd.PrimRange(stage.GetPrimAtPath(robot_root))]
log(f"all prim names under root: {all_names}")
paths = find_prims(stage, robot_root, ["gripper_link", "gripper_left", "gripper_right", "link6"])
log(f"found prims: {paths}")

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False)

def world_aabb(path):
    prim = stage.GetPrimAtPath(path)
    box = cache.ComputeWorldBound(prim)
    rng = box.ComputeAlignedBox()
    return np.array(rng.GetMin()), np.array(rng.GetMax())

T_tool = ak.fk(np.zeros(6))
R_tool = T_tool[:3, :3]
p_tool = T_tool[:3, 3]
log(f"fk(0) tool position={np.round(p_tool,4)}  local+Z(world)={np.round(R_tool[:,2],4)}")

for name in ["link6", "gripper_link", "gripper_left", "gripper_right"]:
    if name not in paths:
        continue
    lo, hi = world_aabb(paths[name])
    center_world = (lo + hi) / 2.0
    center_local = R_tool.T @ (center_world - p_tool)
    extent_world = hi - lo
    log(f"{name:14s} world_center={np.round(center_world,4)}  "
        f"LOCAL(gripper_link frame) center={np.round(center_local,4)}  "
        f"world_extent={np.round(extent_world,4)}")

log("closing")
simulation_app.close()

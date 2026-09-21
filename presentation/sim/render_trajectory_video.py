#!/usr/bin/env python
"""Animate the solved inspection trajectory in Isaac Sim and encode an MP4.

    source ~/miniconda3/etc/profile.d/conda.sh && conda activate isaaclab
    python -u presentation/sim/render_trajectory_video.py [--frames N] [--fps N] [--speed X]

Reads:
    presentation/demo/joint_trajectory.json    (20 solved poses, q(t))
    presentation/demo/velocity_trajectory.json (per-segment speed, for the on-frame HUD)
    presentation/sim/reBot_B601_DM_with_gripper.urdf (already-rewritten mesh paths)
    presentation/assets/synthetic_crack_input.png

Writes:
    presentation/sim/_video_frames/frame_%05d.png   (deleted after encoding)
    presentation/renders/trajectory.mp4

This duplicates the scene-construction helpers from build_scene.py (materials,
environment, coupon, path visualization, URDF import, apply_joint_pose,
look_at_quat_usd, shoot) rather than importing that script, so a change here
can never destabilize the already-verified still-image pipeline. Anything
fixed in one should be checked in the other.

WHAT'S REAL AND WHAT'S A PRESENTATION CHOICE
---------------------------------------------
The joint angles at each of the 20 keyframes are the actual IK solution from
path_to_joint_trajectory.py (max tool position error 0.089 mm, boresight held
within a fraction of a degree of straight down at every keyframe -- see
joint_trajectory.json's own per-waypoint axis_err_deg). What is NOT real:
motion BETWEEN keyframes is linear interpolation in joint space, not a
re-solved IK path -- an honest, standard approximation for a smooth-looking
video between validated poses, not a claim that the tool stays exactly on
the 45 mm standoff surface at every intermediate instant. The video is also
played back at --speed 2x real time (default) so a presentation audience
isn't watching 30 seconds of a robot moving 2 cm/s; this is stated on-screen
via the HUD, not hidden.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
import traceback

_t0 = time.time()


def log(msg):
    print(f"[t={time.time()-_t0:7.1f}s] {msg}", flush=True)


p = argparse.ArgumentParser()
p.add_argument("--fps", type=int, default=20)
p.add_argument("--speed", type=float, default=2.0, help="playback speed multiplier vs real time")
p.add_argument("--settle-steps", type=int, default=6)
p.add_argument("--rt-subframes", type=int, default=6)
p.add_argument("--max-frames", type=int, default=0, help="0 = full video; N = quick timing test")
args = p.parse_args()

log("launching SimulationApp (headless)")
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 1600, "height": 900})
log("SimulationApp constructed")

import numpy as np
import omni.usd
import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.sensors.camera import Camera
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, UsdPhysics
from PIL import Image, ImageDraw, ImageFont

log("imports done")

# ---------------------------------------------------------------------------
# paths / config (mirrors build_scene.py exactly -- keep in sync by hand)
# ---------------------------------------------------------------------------
PRES_ROOT = "/home/boosterk1/Projects/rebot_crack_vision/presentation"
SIM_DIR = os.path.join(PRES_ROOT, "sim")
RENDER_DIR = os.path.join(PRES_ROOT, "renders")
FRAMES_DIR = os.path.join(SIM_DIR, "_video_frames")
os.makedirs(RENDER_DIR, exist_ok=True)
os.makedirs(FRAMES_DIR, exist_ok=True)

URDF_PATH = os.path.join(SIM_DIR, "reBot_B601_DM_with_gripper.urdf")
SOURCE_CRACK_IMG = os.path.join(PRES_ROOT, "assets", "synthetic_crack_input.png")
BAKED_TEXTURE_PATH = os.path.join(SIM_DIR, "coupon_crack_texture.png")
TRAJ_JSON = os.path.join(PRES_ROOT, "demo", "joint_trajectory.json")
VEL_JSON = os.path.join(PRES_ROOT, "demo", "velocity_trajectory.json")
OUT_MP4 = os.path.join(RENDER_DIR, "trajectory.mp4")

COUPON_CENTER = (0.22, 0.0)  # see crack_to_path.py -- chosen for IK reachability, verified by a sweep
COUPON_SIZE = 0.30
COUPON_TOP_Z = 0.12
SLAB_THICK = 0.05
SUPPORT_SIZE = 0.20
FLOOR_Z = -0.45
TABLE_SIZE = 0.26

xmin = COUPON_CENTER[0] - COUPON_SIZE / 2.0
xmax = COUPON_CENTER[0] + COUPON_SIZE / 2.0
ymin = COUPON_CENTER[1] - COUPON_SIZE / 2.0
ymax = COUPON_CENTER[1] + COUPON_SIZE / 2.0

SPHERE_RADIUS = 0.0045
TUBE_RADIUS = 0.002
ACCENT_HEX = (0xEB, 0x68, 0x34)
BLUE_HEX = (0x2A, 0x78, 0xD6)
ACCENT_RGB = tuple(c / 255.0 for c in ACCENT_HEX)
BLUE_RGB = tuple(c / 255.0 for c in BLUE_HEX)


# ---------------------------------------------------------------------------
# helpers (copied verbatim from build_scene.py -- see that file's comments)
# ---------------------------------------------------------------------------
def make_preview_material(stage, path, diffuse, emissive=None, roughness=0.6, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if emissive is not None:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def make_textured_material(stage, path, texture_file):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    tex = UsdShade.Shader.Define(stage, path + "/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_file)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    st_reader = UsdShade.Shader.Define(stage, path + "/STReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def add_box(stage, path, center_xyz, size_xyz, material):
    cx, cy, cz = center_xyz
    sx, sy, sz = size_xyz
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(cube).SetTranslate((cx, cy, cz))
    UsdGeom.XformCommonAPI(cube).SetScale((sx, sy, sz))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    return cube


def bake_coupon_texture():
    src = np.array(Image.open(SOURCE_CRACK_IMG).convert("RGB"))
    oriented = np.rot90(src, k=1)
    Image.fromarray(oriented).save(BAKED_TEXTURE_PATH)
    log(f"baked coupon texture -> {BAKED_TEXTURE_PATH}")


def quat_from_matrix(R):
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m[2, 1] - m[1, 2]) / S
        y = (m[0, 2] - m[2, 0]) / S
        z = (m[1, 0] - m[0, 1]) / S
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / S
        x = 0.25 * S
        y = (m[0, 1] + m[1, 0]) / S
        z = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / S
        x = (m[0, 1] + m[1, 0]) / S
        y = 0.25 * S
        z = (m[1, 2] + m[2, 1]) / S
    else:
        S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / S
        x = (m[0, 2] + m[2, 0]) / S
        y = (m[1, 2] + m[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z], dtype=float)


def look_at_quat_usd(eye, target, up_hint=(0.0, 0.0, 1.0)):
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    fwd = target - eye
    fwd = fwd / np.linalg.norm(fwd)
    up_hint = np.asarray(up_hint, dtype=float)
    if abs(np.dot(fwd, up_hint)) > 0.995:
        up_hint = np.array([1.0, 0.0, 0.0])
    right = np.cross(fwd, up_hint)
    right = right / np.linalg.norm(right)
    true_up = np.cross(right, fwd)
    R = np.column_stack([right, true_up, -fwd])
    return quat_from_matrix(R)


def _quat_to_euler_zyx_deg(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def add_path_visualization(stage, waypoints, parent_path, sphere_radius, tube_radius):
    UsdGeom.Xform.Define(stage, parent_path)
    sphere_mat = make_preview_material(
        stage, f"{parent_path}/SphereMat", ACCENT_RGB,
        emissive=tuple(c * 0.5 for c in ACCENT_RGB), roughness=0.35,
    )
    tube_mat = make_preview_material(
        stage, f"{parent_path}/TubeMat", BLUE_RGB,
        emissive=tuple(c * 0.6 for c in BLUE_RGB), roughness=0.25,
    )
    for i, (x, y, z) in enumerate(waypoints):
        sph = UsdGeom.Sphere.Define(stage, f"{parent_path}/wp_{i:02d}")
        sph.CreateRadiusAttr(sphere_radius)
        UsdGeom.XformCommonAPI(sph).SetTranslate((x, y, z))
        UsdShade.MaterialBindingAPI.Apply(sph.GetPrim()).Bind(sphere_mat)
    for i in range(len(waypoints) - 1):
        p0 = np.array(waypoints[i])
        p1 = np.array(waypoints[i + 1])
        seg = p1 - p0
        length = float(np.linalg.norm(seg))
        if length < 1e-6:
            continue
        mid = (p0 + p1) / 2.0
        d = seg / length
        z_axis = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(d, z_axis)) > 0.9999:
            R = np.eye(3) if np.dot(d, z_axis) > 0 else np.diag([1.0, -1.0, -1.0])
        else:
            axis = np.cross(z_axis, d)
            axis = axis / np.linalg.norm(axis)
            angle = math.acos(float(np.clip(np.dot(z_axis, d), -1.0, 1.0)))
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0],
            ])
            R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
        q = quat_from_matrix(R)
        cyl = UsdGeom.Cylinder.Define(stage, f"{parent_path}/seg_{i:02d}")
        cyl.CreateRadiusAttr(tube_radius)
        cyl.CreateHeightAttr(length)
        xform_api = UsdGeom.XformCommonAPI(cyl)
        xform_api.SetTranslate(tuple(mid))
        xform_api.SetRotate(tuple(_quat_to_euler_zyx_deg(q)))
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(tube_mat)


def build_environment(stage):
    UsdGeom.Xform.Define(stage, "/World")
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(450.0)
    dome.CreateColorAttr(Gf.Vec3f(0.75, 0.8, 0.9))
    key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(22000.0)
    key.CreateWidthAttr(0.6)
    key.CreateHeightAttr(0.6)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.82, 0.55))
    key_xform = UsdGeom.XformCommonAPI(key)
    key_xform.SetTranslate((0.1, -0.9, 1.1))
    key_xform.SetRotate((-40.0, 15.0, 0.0))
    fill = UsdLux.RectLight.Define(stage, "/World/FillLight")
    fill.CreateIntensityAttr(3000.0)
    fill.CreateWidthAttr(0.8)
    fill.CreateHeightAttr(0.8)
    fill.CreateColorAttr(Gf.Vec3f(0.75, 0.82, 1.0))
    fill_xform = UsdGeom.XformCommonAPI(fill)
    fill_xform.SetTranslate((0.8, 0.9, 0.9))
    fill_xform.SetRotate((-35.0, -140.0, 0.0))
    floor_mat = make_preview_material(stage, "/World/Materials/FloorMat", (0.24, 0.25, 0.28), roughness=0.9)
    floor = UsdGeom.Mesh.Define(stage, "/World/Floor")
    s = 3.0
    floor.CreatePointsAttr([
        Gf.Vec3f(-s, -s, FLOOR_Z), Gf.Vec3f(s, -s, FLOOR_Z),
        Gf.Vec3f(s, s, FLOOR_Z), Gf.Vec3f(-s, s, FLOOR_Z),
    ])
    floor.CreateFaceVertexCountsAttr([4])
    floor.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    floor.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    floor.CreateExtentAttr([Gf.Vec3f(-s, -s, FLOOR_Z), Gf.Vec3f(s, s, FLOOR_Z)])
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(floor_mat)
    table_mat = make_preview_material(stage, "/World/Materials/TableMat", (0.42, 0.45, 0.5), roughness=0.5, metallic=0.15)
    add_box(stage, "/World/RobotPlinth", (0.0, 0.0, FLOOR_Z / 2.0), (TABLE_SIZE, TABLE_SIZE, -FLOOR_Z), table_mat)


def build_coupon(stage):
    concrete_mat = make_preview_material(stage, "/World/Materials/ConcreteMat", (0.60, 0.585, 0.55), roughness=0.9)
    support_top = COUPON_TOP_Z - SLAB_THICK
    add_box(
        stage, "/World/Coupon/Support",
        (COUPON_CENTER[0], COUPON_CENTER[1], FLOOR_Z / 2.0 + support_top / 2.0),
        (SUPPORT_SIZE, SUPPORT_SIZE, support_top - FLOOR_Z),
        concrete_mat,
    )
    add_box(
        stage, "/World/Coupon/Slab",
        (COUPON_CENTER[0], COUPON_CENTER[1], support_top + SLAB_THICK / 2.0),
        (COUPON_SIZE, COUPON_SIZE, SLAB_THICK),
        concrete_mat,
    )
    crack_mat = make_textured_material(stage, "/World/Materials/CrackMat", BAKED_TEXTURE_PATH)
    top_z = COUPON_TOP_Z + 0.0004
    mesh = UsdGeom.Mesh.Define(stage, "/World/Coupon/TopFace")
    pts = [
        Gf.Vec3f(xmin, ymin, top_z), Gf.Vec3f(xmax, ymin, top_z),
        Gf.Vec3f(xmax, ymax, top_z), Gf.Vec3f(xmin, ymax, top_z),
    ]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.CreateExtentAttr([Gf.Vec3f(xmin, ymin, top_z), Gf.Vec3f(xmax, ymax, top_z)])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(crack_mat)


def import_robot(stage):
    enable_extension("isaacsim.asset.importer.urdf")
    simulation_app.update()
    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.collision_from_visuals = False
    import_config.make_default_prim = False
    import_config.create_physics_scene = True
    import_config.distance_scale = 1.0
    success, robot_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=URDF_PATH, import_config=import_config,
        get_articulation_root=True,
    )
    log(f"URDF import success={success} articulation_root={robot_prim_path}")
    return robot_prim_path


def apply_joint_pose(articulation, joint_angles_rad, gripper_opening_m=0.02):
    names = list(articulation.dof_names)
    targets = np.array(articulation.get_joint_positions()).reshape(-1).copy()
    name_to_val = {f"joint{i+1}": joint_angles_rad[i] for i in range(6)}
    name_to_val["gripper_joint1"] = gripper_opening_m
    name_to_val["gripper_joint2"] = gripper_opening_m
    for i, n in enumerate(names):
        if n in name_to_val:
            targets[i] = name_to_val[n]
    articulation.set_joint_positions(targets)


# ---------------------------------------------------------------------------
# trajectory interpolation + HUD
# ---------------------------------------------------------------------------
def build_interpolator(traj_doc):
    wps = traj_doc["waypoints"]
    ts = np.array([w["t_s"] for w in wps])
    qs = np.array([w["q_rad"] for w in wps])
    labels = [w["label"] for w in wps]

    def q_at(t):
        t = min(max(t, ts[0]), ts[-1])
        i = int(np.searchsorted(ts, t, side="right") - 1)
        i = min(max(i, 0), len(ts) - 2)
        t0, t1 = ts[i], ts[i + 1]
        frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        q = qs[i] * (1 - frac) + qs[i + 1] * frac
        return q, labels[i], labels[i + 1], frac

    return q_at, float(ts[0]), float(ts[-1])


_FONT = None


def _font(size=26):
    global _FONT
    if _FONT is None:
        try:
            _FONT = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size)
        except Exception:
            _FONT = ImageFont.load_default()
    return _FONT


def draw_hud(rgb_array, t_real, t_total, speed_cm_s, playback_speed):
    img = Image.fromarray(rgb_array[..., :3] if rgb_array.shape[-1] == 4 else rgb_array)
    draw = ImageDraw.Draw(img, "RGBA")
    pad = 18
    box_w, box_h = 430, 92
    draw.rounded_rectangle([pad, pad, pad + box_w, pad + box_h], radius=10,
                            fill=(11, 11, 11, 165))
    f1 = _font(24)
    f2 = _font(18)
    draw.text((pad + 16, pad + 12), f"t = {t_real:5.1f} s  (of {t_total:.1f} s)", font=f1, fill=(255, 255, 255, 255))
    draw.text((pad + 16, pad + 44), f"tool speed ≈ {speed_cm_s:4.1f} cm/s   ·   playback {playback_speed:.0f}×",
              font=f2, fill=(220, 220, 215, 255))
    draw.text((pad + 16, pad + 66), "boresight held straight down (IK-constrained)",
              font=f2, fill=(190, 210, 245, 255))
    return np.array(img)


def main():
    bake_coupon_texture()

    stage_utils = omni.usd.get_context()
    stage_utils.new_stage()
    stage = stage_utils.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    log("new stage created")

    build_environment(stage)
    build_coupon(stage)

    traj_doc = json.load(open(TRAJ_JSON))
    vel_doc = json.load(open(VEL_JSON)) if os.path.exists(VEL_JSON) else None
    waypoints_xyz = [w["xyz_fk_m"] for w in traj_doc["waypoints"]]
    add_path_visualization(stage, waypoints_xyz, "/World/InspectionPath", SPHERE_RADIUS, TUBE_RADIUS)
    log("environment + coupon + path built")

    robot_prim_path = import_robot(stage)
    simulation_app.update()

    world = World()
    world.reset()
    art = SingleArticulation(prim_path=robot_prim_path)
    art.initialize()
    log(f"articulation initialized, dof_names={list(art.dof_names)}")

    camera = Camera(prim_path="/World/ShotCam", resolution=(1600, 900))
    camera.initialize()
    camera.set_lens_aperture(0.0)
    for _ in range(5):
        simulation_app.update()

    q_at, t0, t1 = build_interpolator(traj_doc)
    duration_video_s = (t1 - t0) / args.speed
    n_frames = max(2, int(round(duration_video_s * args.fps)))
    if args.max_frames:
        n_frames = min(n_frames, args.max_frames)
    log(f"trajectory spans {t1-t0:.2f}s real -> {duration_video_s:.2f}s video "
        f"at {args.fps}fps ({n_frames} frames), speed={args.speed}x")

    # Same camera composition as build_scene.py's scene_overview shot, re-centred
    # on the coupon's corrected (closer, more reachable) placement -- see that
    # file's comment for the offset-preserving derivation.
    eye = (-1.20, -0.95, 0.83)
    target = (0.14, 0.0, 0.16)
    q_cam = look_at_quat_usd(eye, target)
    camera.set_world_pose(position=np.array(eye), orientation=q_cam, camera_axes="usd")
    camera.set_projection_mode("perspective")
    camera.set_focal_length(2.2)
    camera.set_clipping_range(0.01, 20.0)

    # segment-average speed lookup (nearest segment by t_real), for the HUD only
    def speed_at(t_real):
        if vel_doc is None:
            return 2.0
        segs = vel_doc["segments"]
        best = min(segs, key=lambda s: abs(s["t_mid_s"] - t_real))
        return best["speed_m_s"] * 100.0

    t_frame0 = time.time()
    for fi in range(n_frames):
        frac = fi / (n_frames - 1) if n_frames > 1 else 0.0
        t_real = t0 + frac * (t1 - t0)
        q, lbl_from, lbl_to, seg_frac = q_at(t_real)
        apply_joint_pose(art, q, gripper_opening_m=0.02)
        world.step(render=False)

        rgb = None
        for _ in range(args.settle_steps):
            rep.orchestrator.step(rt_subframes=args.rt_subframes, pause_timeline=False)
            rgb = camera.get_rgb()
        if rgb is None or rgb.size == 0:
            log(f"frame {fi}: FAILED, no pixel data -- aborting")
            break
        framed = draw_hud(rgb, t_real, t1 - t0, speed_at(t_real), args.speed)
        Image.fromarray(framed).save(os.path.join(FRAMES_DIR, f"frame_{fi:05d}.png"))
        if fi % 25 == 0 or fi == n_frames - 1:
            elapsed = time.time() - t_frame0
            rate = (fi + 1) / elapsed if elapsed > 0 else 0
            eta = (n_frames - fi - 1) / rate if rate > 0 else float("nan")
            log(f"frame {fi+1}/{n_frames}  t_real={t_real:5.2f}s  "
                f"({lbl_from}->{lbl_to} {seg_frac:.2f})  {rate:.2f} fps  eta={eta:5.0f}s")

    n_written = len([f for f in os.listdir(FRAMES_DIR) if f.endswith(".png")])
    log(f"rendered {n_written}/{n_frames} frames in {time.time()-t_frame0:.1f}s")

    stage.GetRootLayer().Export(os.path.join(SIM_DIR, "crack_inspection_trajectory.usd"))
    log("saved animated-scene stage snapshot (final pose)")

    if n_written >= 2:
        cmd = [
            "ffmpeg", "-y", "-framerate", str(args.fps),
            "-i", os.path.join(FRAMES_DIR, "frame_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
            OUT_MP4,
        ]
        log("encoding: " + " ".join(cmd))
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            log("ffmpeg FAILED:\n" + result.stderr[-2000:])
        else:
            log(f"wrote {OUT_MP4}")
    else:
        log("too few frames written -- skipping ffmpeg encode")


try:
    main()
except Exception:
    log("EXCEPTION in main():")
    traceback.print_exc()
finally:
    log("closing SimulationApp")
    simulation_app.close()
    log("closed")

#!/usr/bin/env python
"""Build the reBot B601-DM crack-inspection capstone scene in Isaac Sim and
render presentation stills, fully headless.

Run (from anywhere, with the isaaclab conda env active):

    source ~/miniconda3/etc/profile.d/conda.sh && conda activate isaaclab
    python -u /home/boosterk1/Projects/rebot_crack_vision/presentation/sim/build_scene.py

Writes:
    presentation/sim/crack_inspection.usd         (the built stage)
    presentation/sim/coupon_crack_texture.png     (baked/oriented texture, see note below)
    presentation/renders/scene_overview.png
    presentation/renders/topdown_path.png
    presentation/renders/closeup_coupon.png

TEXTURE ORIENTATION -- WHY THE ROTATION EXISTS
-----------------------------------------------
`presentation/demo/crack_to_path.py` maps a source-image pixel (row, col) to a
robot-frame world point with:
    image +row -> robot -X ,  image +col -> robot -Y
The coupon top face here is a single quad with the "obvious" UV assignment
(s = (x-xmin)/size along X, t = (y-ymin)/size along Y). Whether that lines the
texture file up correctly depends on the renderer's texture-sample convention,
which is not documented anywhere we could see -- so it was measured, not
assumed: `_calib_probe.py` painted the coupon with a 4-quadrant color texture,
dropped a distinctly-colored marker at each of the 4 image corners (using the
exact same pixel->world formula the real path uses), rendered top-down, and
read back which color ended up under which marker. The result showed a
consistent 90-degree-CCW misalignment (`np.rot90(img, k=1)` mapped the observed
result exactly), so that rotation is applied once, here, to the real crack
image before it is used as the coupon texture. If you ever change the UV
assignment below, re-run `_calib_probe.py` and re-derive this.
"""
import json
import math
import os
import time
import traceback

_t0 = time.time()


def log(msg):
    print(f"[t={time.time()-_t0:7.1f}s] {msg}", flush=True)


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
from PIL import Image

log("imports done")

# ---------------------------------------------------------------------------
# paths / config
# ---------------------------------------------------------------------------
PRES_ROOT = "/home/boosterk1/Projects/rebot_crack_vision/presentation"
SIM_DIR = os.path.join(PRES_ROOT, "sim")
RENDER_DIR = os.path.join(PRES_ROOT, "renders")
os.makedirs(RENDER_DIR, exist_ok=True)

URDF_PATH = os.path.join(SIM_DIR, "reBot_B601_DM_with_gripper.urdf")
SOURCE_CRACK_IMG = os.path.join(PRES_ROOT, "assets", "synthetic_crack_input.png")
BAKED_TEXTURE_PATH = os.path.join(SIM_DIR, "coupon_crack_texture.png")
PATH_JSON = os.path.join(PRES_ROOT, "demo", "crack_path_3d.json")
USD_OUT_PATH = os.path.join(SIM_DIR, "crack_inspection.usd")

# scene geometry (metres)
COUPON_CENTER = (0.42, 0.0)
COUPON_SIZE = 0.30
COUPON_TOP_Z = 0.12
SLAB_THICK = 0.05
SUPPORT_SIZE = 0.20
FLOOR_Z = -0.45
TABLE_SIZE = 0.26  # kept clear of the tight topdown frame -- see notes near the topdown shot

xmin = COUPON_CENTER[0] - COUPON_SIZE / 2.0
xmax = COUPON_CENTER[0] + COUPON_SIZE / 2.0
ymin = COUPON_CENTER[1] - COUPON_SIZE / 2.0
ymax = COUPON_CENTER[1] + COUPON_SIZE / 2.0

# path visuals (floating standoff path, shown in the overview/closeup shots)
SPHERE_RADIUS = 0.0045
TUBE_RADIUS = 0.002
# flat, zero-standoff path glued to the coupon surface -- used ONLY for the
# topdown shot, so that shot has zero parallax between the path and the crack
# no matter how the camera is aimed (see topdown-camera notes in main()).
FLAT_SPHERE_RADIUS = 0.0035
FLAT_TUBE_RADIUS = 0.0015
FLAT_PATH_Z = 0.1215  # clear above the textured top face (0.1204) -- avoid sinking into the slab

ACCENT_HEX = (0xEB, 0x68, 0x34)   # warm accent, spheres
BLUE_HEX = (0x2A, 0x78, 0xD6)     # path tube

ACCENT_RGB = tuple(c / 255.0 for c in ACCENT_HEX)
BLUE_RGB = tuple(c / 255.0 for c in BLUE_HEX)

# static "inspecting" arm pose (joint1..joint6, radians) -- solved offline with a
# closed-form FK + L-BFGS-B fit so the gripper hovers, pointing straight down,
# right at waypoint #9 of the path (see report / commit notes for the solver).
STATIC_ARM_POSE_RAD = [0.41839, -1.90421, -0.57086, -1.33942, 1.15000, 0.00044]
STATIC_GRIPPER_OPEN_M = 0.02

# retracted/home pose used only for the topdown shot, so the arm doesn't sweep
# across the coupon and occlude the crack -- verified clear of the topdown
# frame (x < 0.113 m or y outside +-0.165 m) with an offline FK check.
RETRACTED_ARM_POSE_RAD = [-1.6, 0.0, 0.0, 0.0, 0.0, 0.0]
RETRACTED_GRIPPER_OPEN_M = 0.0


# ---------------------------------------------------------------------------
# helpers
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
    cube.CreateSizeAttr(1.0)  # edge length 1.0 at scale=1, so scale == desired size directly
    UsdGeom.XformCommonAPI(cube).SetTranslate((cx, cy, cz))
    UsdGeom.XformCommonAPI(cube).SetScale((sx, sy, sz))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    return cube


def bake_coupon_texture():
    """Rotate the real crack image 90deg CCW (empirically calibrated, see module
    docstring) and save it as the texture actually applied to the coupon quad."""
    src = np.array(Image.open(SOURCE_CRACK_IMG).convert("RGB"))
    oriented = np.rot90(src, k=1)
    Image.fromarray(oriented).save(BAKED_TEXTURE_PATH)
    log(f"baked coupon texture -> {BAKED_TEXTURE_PATH} (rot90 k=1 of {SOURCE_CRACK_IMG})")


def pixel_to_world(r, c, img_shape=(512, 512)):
    """Mirrors presentation/demo/crack_to_path.py's pixel_to_world exactly
    (same constants, same formula) -- used only to place diagnostic corner
    fiducials, never to move the real waypoints."""
    h, w = img_shape
    m_per_px = COUPON_SIZE / w
    x = COUPON_CENTER[0] + (h / 2.0 - r) * m_per_px
    y = COUPON_CENTER[1] + (w / 2.0 - c) * m_per_px
    return (x, y)


def build_corner_fiducials(stage):
    """Diagnostic-only markers at the 4 SOURCE-IMAGE corners (0,0)/(0,511)/
    (511,0)/(511,511), run through the exact same pixel_to_world formula the
    real path uses. If the texture placement is correct, each marker should
    sit exactly on the coupon's geometric corner (by construction) AND on
    that corner's crack-image content. Hidden by default; shown only for the
    topdown diagnostic check."""
    colors = {
        "corner_00": (1.0, 1.0, 1.0),      # white  -> image top-left
        "corner_0W": (0.05, 0.05, 0.05),   # black  -> image top-right
        "corner_H0": (0.0, 1.0, 1.0),      # cyan   -> image bottom-left
        "corner_HW": (1.0, 0.0, 1.0),      # magenta-> image bottom-right
    }
    pixels = {
        "corner_00": (0, 0), "corner_0W": (0, 511),
        "corner_H0": (511, 0), "corner_HW": (511, 511),
    }
    UsdGeom.Xform.Define(stage, "/World/CornerFiducials")
    for name, (r, c) in pixels.items():
        x, y = pixel_to_world(r, c)
        sph = UsdGeom.Sphere.Define(stage, f"/World/CornerFiducials/{name}")
        sph.CreateRadiusAttr(0.006)
        UsdGeom.XformCommonAPI(sph).SetTranslate((x, y, FLAT_PATH_Z + 0.001))
        mat = make_preview_material(
            stage, f"/World/CornerFiducials/{name}_mat", colors[name],
            emissive=tuple(v * 0.6 for v in colors[name]), roughness=0.3,
        )
        UsdShade.MaterialBindingAPI.Apply(sph.GetPrim()).Bind(mat)
        log(f"corner fiducial {name}: pixel=({r},{c}) -> world=({x:.4f},{y:.4f})")


def quat_from_matrix(R):
    """3x3 rotation matrix -> (w, x, y, z) quaternion."""
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
    """Return a (w,x,y,z) quaternion in raw USD camera convention
    (+Y up, -Z forward) that points the camera at `target` from `eye`."""
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
    R = np.column_stack([right, true_up, -fwd])  # local X, Y, Z axes expressed in world
    return quat_from_matrix(R)


def add_path_visualization(stage, waypoints, parent_path, sphere_radius, tube_radius, z_override=None):
    """Build waypoint spheres + connecting tube under `parent_path`. If
    `z_override` is given, every point is dropped to that Z (used for the
    flat, zero-standoff copy of the path used in the topdown shot)."""
    if z_override is not None:
        waypoints = [(x, y, z_override) for (x, y, _z) in waypoints]

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


def set_visibility(stage, prim_path, visible):
    imageable = UsdGeom.Imageable(stage.GetPrimAtPath(prim_path))
    if visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()


def _quat_to_euler_zyx_deg(q):
    """(w,x,y,z) -> (rx,ry,rz) degrees, XYZ order, matching UsdGeom.XformCommonAPI.SetRotate default."""
    w, x, y, z = q
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def build_environment(stage):
    UsdGeom.Xform.Define(stage, "/World")

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(450.0)
    dome.CreateColorAttr(Gf.Vec3f(0.75, 0.8, 0.9))

    key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(22000.0)
    key.CreateWidthAttr(0.6)
    key.CreateHeightAttr(0.6)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.82, 0.55))  # warm key for separation/contrast
    key_xform = UsdGeom.XformCommonAPI(key)
    key_xform.SetTranslate((0.1, -0.9, 1.1))
    key_xform.SetRotate((-40.0, 15.0, 0.0))

    fill = UsdLux.RectLight.Define(stage, "/World/FillLight")
    fill.CreateIntensityAttr(3000.0)
    fill.CreateWidthAttr(0.8)
    fill.CreateHeightAttr(0.8)
    fill.CreateColorAttr(Gf.Vec3f(0.75, 0.82, 1.0))  # cool fill, contrasts the warm key
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
    top_z = COUPON_TOP_Z + 0.0004  # tiny epsilon to avoid z-fighting with the slab top face
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
    # s=(x-xmin)/size, t=(y-ymin)/size at the 4 corners above; the texture file
    # itself is pre-rotated (bake_coupon_texture) to compensate for the
    # renderer's sampling convention -- see module docstring.
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
        "URDFParseAndImportFile",
        urdf_path=URDF_PATH,
        import_config=import_config,
        get_articulation_root=True,
    )
    log(f"URDF import success={success} articulation_root={robot_prim_path}")
    return robot_prim_path


def apply_joint_pose(articulation: SingleArticulation, joint_angles_rad, gripper_opening_m=0.02):
    """Set the 6 arm joints + 2 gripper prismatic joints by NAME (order-independent),
    then the caller should step the world so PhysX propagates the pose to the
    visible link transforms. Reusable as-is for future trajectory playback:
    call once per animation frame with a new `joint_angles_rad`."""
    names = list(articulation.dof_names)
    targets = np.array(articulation.get_joint_positions()).reshape(-1).copy()
    name_to_val = {f"joint{i+1}": joint_angles_rad[i] for i in range(6)}
    name_to_val["gripper_joint1"] = gripper_opening_m
    name_to_val["gripper_joint2"] = gripper_opening_m
    for i, n in enumerate(names):
        if n in name_to_val:
            targets[i] = name_to_val[n]
    articulation.set_joint_positions(targets)


def shoot(camera: Camera, position, target, projection, resolution, out_path,
          horizontal_aperture=None, vertical_aperture=None, focal_length=None,
          up_hint=(0.0, 0.0, 1.0), settle_steps=24, rt_subframes=24):
    q = look_at_quat_usd(position, target, up_hint=up_hint)
    camera.set_world_pose(position=np.array(position), orientation=q, camera_axes="usd")
    camera.set_projection_mode(projection)
    if projection == "orthographic" and vertical_aperture is not None:
        # resolution is wider than tall (16:9); constrain by the SHORTER (vertical)
        # dimension so the full coupon height stays in frame, let horizontal
        # auto-widen (maintain_square_pixels) to show extra context left/right.
        camera.set_vertical_aperture(vertical_aperture, maintain_square_pixels=True)
    elif projection == "orthographic" and horizontal_aperture is not None:
        camera.set_horizontal_aperture(horizontal_aperture)
    if projection == "perspective" and focal_length is not None:
        camera.set_focal_length(focal_length)
    camera.set_clipping_range(0.01, 20.0)

    # Camera pose/projection changes need a few frames to propagate through the
    # render graph AND for the RTX accumulation/denoiser to settle on the new
    # view before a capture is both correct (not a stale previous frame) and
    # clean (not accumulation noise) -- always burn a fixed, generous number of
    # frames rather than an early-exit std check, which passed on noisy/stale
    # frames before. Cheap: the whole scene renders in ~20s regardless.
    rgb = None
    for i in range(settle_steps):
        rep.orchestrator.step(rt_subframes=rt_subframes, pause_timeline=False)
        rgb = camera.get_rgb()
    std = None if rgb is None else float(np.std(rgb))
    log(f"  {os.path.basename(out_path)}: settled after {settle_steps} steps, std={std}")
    if rgb is None or rgb.size == 0:
        log(f"  {os.path.basename(out_path)}: FAILED -- no pixel data")
        return False
    Image.fromarray(rgb[..., :3] if rgb.shape[-1] == 4 else rgb).save(out_path)
    log(f"  wrote {out_path}  shape={rgb.shape}")
    return True


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
    log("environment + coupon built")

    with open(PATH_JSON) as f:
        path_doc = json.load(f)
    waypoints = path_doc["world_waypoints_xyz_m"]
    # floating path (real standoff height, 45mm above the surface) -- used for
    # the overview/closeup "how the arm actually inspects it" shots.
    add_path_visualization(stage, waypoints, "/World/InspectionPath", SPHERE_RADIUS, TUBE_RADIUS)
    # flat path glued to the surface (zero standoff) -- used ONLY for the
    # topdown shot so there is zero parallax between path and painted crack,
    # regardless of camera perspective. This is the actual alignment proof.
    add_path_visualization(
        stage, waypoints, "/World/InspectionPathFlat",
        FLAT_SPHERE_RADIUS, FLAT_TUBE_RADIUS, z_override=FLAT_PATH_Z,
    )
    log(f"path visualization built ({len(waypoints)} waypoints, floating + flat copies)")
    build_corner_fiducials(stage)
    set_visibility(stage, "/World/CornerFiducials", False)

    robot_prim_path = import_robot(stage)
    simulation_app.update()

    world = World()
    world.reset()
    log("world reset")

    art = SingleArticulation(prim_path=robot_prim_path)
    art.initialize()
    log(f"articulation initialized, dof_names={list(art.dof_names)}")

    camera = Camera(prim_path="/World/ShotCam", resolution=(1600, 900))
    camera.initialize()
    camera.set_lens_aperture(0.0)  # disable depth-of-field entirely (fStop=0)
    for _ in range(5):
        simulation_app.update()
    log(f"shot camera ready: default focal_length={camera.get_focal_length()} "
        f"h_aperture={camera.get_horizontal_aperture()} v_aperture={camera.get_vertical_aperture()}")

    results = {}

    # --- shots 1 & 2: arm in its "inspecting" pose, floating path visible ---
    apply_joint_pose(art, STATIC_ARM_POSE_RAD, STATIC_GRIPPER_OPEN_M)
    for _ in range(5):
        world.step(render=False)
    set_visibility(stage, "/World/InspectionPath", True)
    set_visibility(stage, "/World/InspectionPathFlat", False)
    log("static inspection pose applied, floating path shown")

    results["scene_overview.png"] = shoot(
        camera,
        position=(-1.0, -0.95, 0.85),
        target=(0.34, 0.0, 0.18),
        projection="perspective",
        resolution=(1600, 900),
        out_path=os.path.join(RENDER_DIR, "scene_overview.png"),
        focal_length=2.2,
    )
    results["closeup_coupon.png"] = shoot(
        camera,
        position=(0.60, -0.35, 0.30),
        target=(0.40, -0.02, COUPON_TOP_Z),
        projection="perspective",
        resolution=(1600, 900),
        out_path=os.path.join(RENDER_DIR, "closeup_coupon.png"),
        focal_length=3.0,
    )

    # --- shot 3: retracted arm pose (clear of the coupon), flat zero-parallax path ---
    apply_joint_pose(art, RETRACTED_ARM_POSE_RAD, RETRACTED_GRIPPER_OPEN_M)
    for _ in range(5):
        world.step(render=False)
    set_visibility(stage, "/World/InspectionPath", False)
    set_visibility(stage, "/World/InspectionPathFlat", True)
    log("retracted pose applied, flat path shown")

    results["topdown_path.png"] = shoot(
        camera,
        position=(COUPON_CENTER[0], COUPON_CENTER[1], 1.15),
        target=(COUPON_CENTER[0], COUPON_CENTER[1], 0.0),
        projection="orthographic",
        resolution=(1600, 900),
        out_path=os.path.join(RENDER_DIR, "topdown_path.png"),
        vertical_aperture=COUPON_SIZE * 1.05,
        up_hint=(1.0, 0.0, 0.0),
    )

    # diagnostic-only: corner fiducials at the 4 image corners, same topdown
    # camera pose already set by the shot above -- not one of the deliverables,
    # written to sim/ to check whether the texture's own corners coincide with
    # the coupon's geometric corners (rules out a crop/placement bug).
    set_visibility(stage, "/World/InspectionPathFlat", False)
    set_visibility(stage, "/World/CornerFiducials", True)
    shoot(
        camera,
        position=(COUPON_CENTER[0], COUPON_CENTER[1], 1.15),
        target=(COUPON_CENTER[0], COUPON_CENTER[1], 0.0),
        projection="orthographic",
        resolution=(1600, 900),
        out_path=os.path.join(SIM_DIR, "_fiducial_check.png"),
        vertical_aperture=COUPON_SIZE * 1.05,
        up_hint=(1.0, 0.0, 0.0),
    )
    set_visibility(stage, "/World/CornerFiducials", False)

    # restore the inspecting pose + floating path before saving the stage, so
    # reopening crack_inspection.usd shows the "hero" configuration by default.
    apply_joint_pose(art, STATIC_ARM_POSE_RAD, STATIC_GRIPPER_OPEN_M)
    for _ in range(3):
        world.step(render=False)
    set_visibility(stage, "/World/InspectionPath", True)
    set_visibility(stage, "/World/InspectionPathFlat", False)

    stage.GetRootLayer().Export(USD_OUT_PATH)
    log(f"saved stage -> {USD_OUT_PATH}")
    log(f"render results: {results}")


try:
    main()
except Exception:
    log("EXCEPTION in main():")
    traceback.print_exc()
finally:
    log("closing SimulationApp")
    simulation_app.close()
    log("closed")

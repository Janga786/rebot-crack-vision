#!/usr/bin/env python
"""Quick standalone probe (NOT a deliverable): determine the correct UV/texture
orientation for the coupon quad by rendering a 4-quadrant calibration texture
plus 4 marker spheres and reading the result back.

Run:
    source ~/miniconda3/etc/profile.d/conda.sh && conda activate isaaclab
    python -u presentation/sim/_calib_probe.py

Writes presentation/renders/_calib_probe.png for visual inspection.
"""
import os
import sys
import time
import traceback

_t0 = time.time()


def log(msg):
    print(f"[t={time.time()-_t0:7.1f}s] {msg}", flush=True)


log("launching SimulationApp")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 640, "height": 640})
log("SimulationApp constructed")

import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade
from isaacsim.sensors.camera import Camera
from PIL import Image

log("imports done")

ROOT = "/home/boosterk1/Projects/rebot_crack_vision/presentation"
SIM_DIR = os.path.join(ROOT, "sim")
RENDER_DIR = os.path.join(ROOT, "renders")
os.makedirs(RENDER_DIR, exist_ok=True)

COUPON_CENTER = (0.42, 0.0)
COUPON_SIZE = 0.30
COUPON_Z = 0.12
IMG_SHAPE = (512, 512)

xmin = COUPON_CENTER[0] - COUPON_SIZE / 2.0
xmax = COUPON_CENTER[0] + COUPON_SIZE / 2.0
ymin = COUPON_CENTER[1] - COUPON_SIZE / 2.0
ymax = COUPON_CENTER[1] + COUPON_SIZE / 2.0


def pixel_to_world(r, c):
    h, w = IMG_SHAPE
    m_per_px = COUPON_SIZE / w
    x = COUPON_CENTER[0] + (h / 2.0 - r) * m_per_px
    y = COUPON_CENTER[1] + (w / 2.0 - c) * m_per_px
    return (float(x), float(y), float(COUPON_Z + 0.006))


def main():
    # --- build calibration texture: TL(row<256,col<256)=RED, TR=GREEN, BL=BLUE, BR=YELLOW
    calib = np.zeros((512, 512, 3), dtype=np.uint8)
    calib[0:256, 0:256] = (230, 40, 40)       # RED    -> source r<256,c<256
    calib[0:256, 256:512] = (40, 200, 60)     # GREEN  -> source r<256,c>=256
    calib[256:512, 0:256] = (40, 90, 230)     # BLUE   -> source r>=256,c<256
    calib[256:512, 256:512] = (230, 210, 40)  # YELLOW -> source r>=256,c>=256
    calib_path = os.path.join(SIM_DIR, "_calib_quadrants.png")
    Image.fromarray(calib).save(calib_path)
    log(f"wrote calib texture {calib_path}")

    stage_utils = omni.usd.get_context()
    stage_utils.new_stage()
    stage = stage_utils.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    log("new stage created")

    UsdGeom.Xform.Define(stage, "/World")
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000.0)

    mesh = UsdGeom.Mesh.Define(stage, "/World/CouponTop")
    pts = [
        Gf.Vec3f(xmin, ymin, COUPON_Z),
        Gf.Vec3f(xmax, ymin, COUPON_Z),
        Gf.Vec3f(xmax, ymax, COUPON_Z),
        Gf.Vec3f(xmin, ymax, COUPON_Z),
    ]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.CreateExtentAttr([Gf.Vec3f(xmin, ymin, COUPON_Z), Gf.Vec3f(xmax, ymax, COUPON_Z)])
    texCoord = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying
    )
    texCoord.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])

    mat = UsdShade.Material.Define(stage, "/World/CouponTop/CalibMat")
    shader = UsdShade.Shader.Define(stage, "/World/CouponTop/CalibMat/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    tex = UsdShade.Shader.Define(stage, "/World/CouponTop/CalibMat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(calib_path)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    st_reader = UsdShade.Shader.Define(stage, "/World/CouponTop/CalibMat/STReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    log("coupon quad + calib material built")

    # markers at the 4 SOURCE-IMAGE CORNERS (deep inside each quadrant color), each
    # given a UNIQUE, visually distinct marker color so the corner-color correspondence
    # can be read directly off one image without needing to know the camera's axis
    # convention at all.
    markers = {
        "corner_00_expectRED": (pixel_to_world(0, 0), (1.0, 1.0, 1.0)),        # white
        "corner_0W_expectGREEN": (pixel_to_world(0, 511), (0.02, 0.02, 0.02)),  # near-black
        "corner_H0_expectBLUE": (pixel_to_world(511, 0), (0.0, 1.0, 1.0)),      # cyan
        "corner_HW_expectYELLOW": (pixel_to_world(511, 511), (1.0, 0.0, 1.0)),  # magenta
    }
    for name, ((x, y, z), color) in markers.items():
        sph = UsdGeom.Sphere.Define(stage, f"/World/Markers/{name}")
        sph.CreateRadiusAttr(0.014)
        UsdGeom.XformCommonAPI(sph).SetTranslate((x, y, z))
        mmat = UsdShade.Material.Define(stage, f"/World/Markers/{name}_mat")
        mshader = UsdShade.Shader.Define(stage, f"/World/Markers/{name}_mat/Shader")
        mshader.CreateIdAttr("UsdPreviewSurface")
        mshader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        mshader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[c * 0.7 for c in color]))
        mmat.CreateSurfaceOutput().ConnectToSource(mshader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(sph.GetPrim()).Bind(mmat)
        log(f"marker {name} color={color} -> world {(x, y, z)}")

    import math
    half = math.sqrt(0.5)
    log("constructing Camera object")
    cam = Camera(
        prim_path="/World/TopDownCam",
        resolution=(640, 640),
        position=np.array([COUPON_CENTER[0], COUPON_CENTER[1], 1.2]),
        orientation=np.array([half, 0.0, half, 0.0]),
    )
    log("Camera object constructed, calling initialize()")
    cam.initialize()
    log("camera initialized (render product created)")
    cam.set_world_pose(
        position=np.array([COUPON_CENTER[0], COUPON_CENTER[1], 1.2]),
        orientation=np.array([half, 0.0, half, 0.0]),
        camera_axes="world",
    )
    cam.set_projection_mode("orthographic")
    cam.set_horizontal_aperture(COUPON_SIZE)
    cam.set_clipping_range(0.01, 10.0)
    log("camera pose/projection set")

    for i in range(10):
        simulation_app.update()
    log("warmup updates done, stepping replicator orchestrator")

    rgb = None
    for i in range(15):
        rep.orchestrator.step(rt_subframes=8, pause_timeline=False)
        rgb = cam.get_rgb()
        std = None if rgb is None else float(np.std(rgb))
        log(f"step {i}: rgb={None if rgb is None else rgb.shape} std={std}")
        if rgb is not None and rgb.size > 0 and np.std(rgb) > 0.5:
            break

    log(f"final rgb = {None if rgb is None else rgb.shape}")
    if rgb is not None and rgb.size > 0:
        out_path = os.path.join(RENDER_DIR, "_calib_probe.png")
        Image.fromarray(rgb[..., :3] if rgb.shape[-1] == 4 else rgb).save(out_path)
        log(f"wrote {out_path}")
    else:
        log("rgb was empty/None -- nothing written")


try:
    main()
except Exception:
    log("EXCEPTION in main():")
    traceback.print_exc()
finally:
    log("closing SimulationApp")
    simulation_app.close()
    log("closed")

# Synthetic ground-truth verification of `crackvision.paths` -> `crackvision.path3d` (GEOM-08.8)

## Purpose

GEOM-08.7 built `crackvision.path3d`: pixel polylines + a capture record + a mask -> `base_link`
surface points, normals, segments and `tool_tip` waypoints (`docs/INTERFACES.md` §10). Nothing
until now ran that pipeline against a *known* 3D answer. This card adds:

- `tools/synth_scene3d.py` — a deterministic synthetic-scene generator. It renders a flat crack
  surface under the exact nominal eye-in-hand chain (`FK(q) . T_gripper_link_camera_link .
  T_camera_link_camera_color_optical_frame`, §9.2), by its own ray/plane intersection and forward
  projection — **never** calling `crackvision.geometry.deproject_pixels`, `crackvision.lift3d` or
  `crackvision.tool_waypoints`. It writes a full synthetic case (skeleton PNG, dilated mask,
  colour placeholder, aligned depth PNG, a §9 capture record, a `case_map.json` entry) plus a
  ground-truth JSON (`data/synthetic/{case}_gt.json`): the 3D curve samples, the plane normal, the
  solved joint state, and the camera pose.
- `tests/test_path3d_synthetic.py` — runs the real `crackvision.paths` CLI, then
  `crackvision.path3d.build_case_paths3d` directly, on a generated scene, and scores the result
  against the generator's ground truth.

**Scope of what this verifies:** software consistency of the pixel -> `base_link` lifting math
(projection algebra, annulus sampling, plane-fit normals, the §10.4 first-order covariance
propagation, the §10.3 gap policy, the §10.5 waypoint policy) under the *nominal* chain —
i.e. that the code computes what the geometry says it should, given a camera, surface and depth
sensor whose models are exactly as declared. It is **not** a measurement of physical accuracy:
the camera-to-robot extrinsic, the tool tip and the depth sensor's bias are not real
measurements here (GEOM-05, GEOM-07 and the stereo depth model's own calibration are what makes
those real), and the only "hardware" involved is the FK chain's arithmetic. End-to-end accuracy
on the real robot/camera is GEOM-05/07 and INT-04/05's job.

## Scene

- **Surface:** a flat plane in `base_link` through the specimen placement centre, with outward
  normal tilted about `base_link` `+Y` by the requested `--tilt-deg`.
- **Specimen centre:** `config/motion/specimen_placement.yaml`'s `placement.center_xy_m`/
  `surface_z_m`, when present and feasible. As of this card that file is a nominal
  "no feasible placement found" result (`value_status: nominal`, `feasible: false`,
  `placement: null` — MOT-04.3's `recommend_placement`, not yet re-run since an earlier
  upstream change), so every run in this card instead falls back to the ADR-014 §3 nominal
  placement **(0.29, 0)** on the table, with `table_z = -0.01 m` taken from
  `config/scene/scene.yaml`'s specimen collision box top face (`position_m.z=-0.015` +
  `dimensions_m.z/2=0.005`). `specimen_center()` logs/returns this fallback explicitly; see
  `center_source` in the `_gt.json` files and the measured table below.
- **View pose:** ADR-014 §2's view phase — `camera_link` **0.25 m** from the centre, optical axis
  within `--view-tilt-deg` of the surface's anti-normal, solved by 6-DOF numeric IK
  (`scipy.optimize.least_squares`, bounded by the URDF joint limits) over a grid of camera rolls
  (free per ADR-014 §2) and several seeds, keeping the globally best-converging solution.
- **`--tilt-deg` vs. `--view-tilt-deg` (two independent angles):** `--tilt-deg` tilts the
  **surface** itself about `base_link` `+Y` (`plane_normal`); `--view-tilt-deg` tilts the
  **camera** away from the surface anti-normal, independently of the surface's own tilt
  (`camera_target_pose`). The camera is placed on the cone of half-angle `--view-tilt-deg` around
  the surface normal, `VIEW_DISTANCE_M` from the centre, and pointed back at the centre, so the
  angle between the boresight (+Z) and the anti-normal is exactly `--view-tilt-deg` by
  construction — this is the quantity ADR-014 §2 bounds (≤ 15°), genuinely exercised here rather
  than approximated: `tests/test_path3d_synthetic.py::test_capture_distance_and_anti_normal_angle`
  measures `arccos(-dot(boresight, normal_base))` directly off the solved `T_base_optical` and
  asserts it equals the requested `--view-tilt-deg` to ±0.1°. (An earlier version of this card
  conflated the two angles — tilting only the surface and claiming that was "projectively
  equivalent" to an off-axis camera view; it is not, because a camera rigidly rotated together
  with the plane sees an identical image, so no oblique-view geometry was ever actually exercised.
  This has been fixed: `flat_scene`'s two parametrisations below are genuine 0° and 15° camera
  view tilts, surface left flat.)
- **Intrinsics:** 848x480, `fx = fy = 430.0`, `model: "none"` (no distortion), `depth_scale =
  1e-4 m/count` — matching the card body exactly.
- **Crack:** a sine arc (`±30 mm` of in-plane arc-length, `5 mm` amplitude, one period —
  ≈104-110 px at this geometry) is forward-projected to pixels and rasterised into a thin,
  8-connected 1-px skeleton via a two-pass arc-length-sampled Bresenham walk (see "Rasterisation"
  below), then dilated (5 px disks) into the crack mask. Pixels under the mask get **+3 mm**
  depth (farther from the camera — R-08's cavity model) when the depth image is rendered.
  Optional Gaussian depth noise (`geometry.depth_uncertainty_m`) and two invalid-depth holes can
  be injected (`--depth-noise`, `--holes`).

### Rasterisation note (a real bug found and fixed while building this generator)

The first implementation rounded a *densely oversampled* (row, col) curve independently per
axis. This reliably produced extra "thick corner" pixels (a point touching 3+ other skeleton
pixels) wherever the curve's local slope was not axis-aligned or exactly diagonal — verified by
convolution neighbour-counting (121/141 pixels at degree ≥ 3 for a first test run). `PERC-02`'s
graph builder correctly treats a degree-≥3 pixel as a junction, so the "single simple crack" was
coming out of `crackvision.paths` as a tree with spurious branches. The fix: estimate the
projected curve's total pixel length with a coarse pass, resample at roughly one anchor per pixel
of that length (`CHAIN_ANCHOR_PX_FACTOR = 0.5`), and connect consecutive rounded anchors with a
true integer Bresenham segment (error-accumulation on one axis, not independent per-axis
rounding). This produces a genuinely thin chain (0 pixels at degree ≥ 3, re-checked for both
tilts below) and is the reason this card's crack is deliberately short (~100-110 px, not longer):
the fix was verified empirically at this scale and is not claimed to generalize to arbitrarily
sharp curvature without re-checking.

## Results (seed 1 unless noted; `./env.sh pytest tests/test_path3d_synthetic.py -q`)

### Noise-free position/normal/waypoint accuracy (criterion: median ≤ 0.5 mm, max ≤ 1.5 mm,
normal ≤ 1°, trace-waypoint `+X . n_true ≤ -cos(1°)`, clearance error ≤ 0.5 mm)

`view tilt` is the angle between the camera boresight and the surface anti-normal (`--view-tilt-deg`,
measured by `test_capture_distance_and_anti_normal_angle`); the surface itself stays flat
(`tilt_deg=0`) in both rows below.

| view tilt | points (valid/total) | median error | max error | max normal angle | max `+X . n_true` | max clearance error | IK residual norm | roll used | measured view angle |
|---|---|---|---|---|---|---|---|---|---|
| 0° | 105 / 105 | 0.184 mm | 0.539 mm | 0.000° | -1.0000 | 8.7e-12 mm | 1.38e-17 | 270° | 0.0000° |
| 15° | 98 / 98 | 0.161 mm | 0.438 mm | 0.130° | -1.0000 | 0.257 mm | 4.50e-17 | 120° | 15.0000° |

All four thresholds pass with comfortable margin at both view tilts, including the genuine 15°
oblique-view case (camera boresight 15° off the surface anti-normal, verified against the
`±0.1°` tolerance). The residual position error (≈0.2-0.5 mm) is consistent with the ≤1 px
rounding of the forward-projected curve into integer pixel coordinates before it is handed to the
real pipeline as a skeleton raster (at `fx/z ≈ 1720 px/m` here, 1 px ≈ 0.58 mm) — i.e. it is
rasterisation quantisation, not a lifting-pipeline error. At 0° view tilt the normal angle is
exactly 0° because the annulus-sampled surface is noise-free, perfectly flat and viewed head-on,
so `geometry.fit_plane`'s SVD recovers the analytic plane normal to floating-point precision; at
15° the annulus samples now span a depth gradient across the oblique view and the plane fit still
recovers the normal to 0.130°, comfortably inside the 1° bound. The waypoint clearance error is
floating-point noise at 0° (§10.5 places a waypoint at exactly `surface_point + clearance_m *
n_out`, and `n_out` there equals the true normal to float precision) and grows to 0.257 mm at 15°
(still well under the 0.5 mm bound) because `n_out` there carries the same ≈0.13° plane-fit
residual as the normal-angle column above, which couples into the waypoint's standoff offset.

### Crack-cavity bias (criterion: the +3 mm cavity must not bias recovered surface points beyond
the noise-free bounds above)

Every valid point's `depth_m` (annulus-sampled surface depth) matches the exact noise-free
ray/plane depth at that pixel to within the same bound used above (≤ 1.5 mm; in practice every
point's `depth_m` was found exactly equal to the true surface depth at both view tilts — the annulus
sampler, by construction, excludes mask/cavity pixels, and GEOM-08.5's own test suite already
pins this behaviour at the unit level). Confirms §10.3's annulus method is immune to the cavity
by design, not by accident of this particular curve.

### Depth-noise / uncertainty consistency (criterion: ≥ 90% of valid points within `3 *
sigma_base` per axis, calibration term zeroed)

Fixture: a `measured`, zero-`uncertainty` copy of `config/robot/end_effector.yaml` (`position_sigma_m
= rotation_sigma_rad = 0` on both `wrist_camera` and `tool`), so §10.4's
`calib_term = cam_pos_sigma^2 + (|p_opt| * cam_rot_sigma)^2` is exactly 0 and `cov_base` carries
only the sensor terms (`pixel_noise_px`, `depth_uncertainty_stereo`).

Scene: tilt 0°, `seed=42`, `--depth-noise` (Gaussian, `sigma_z =
geometry.depth_uncertainty_m(z, fx)` per the GEOM-02 stereo model, `D405_NOMINAL_BASELINE_M` /
`D405_NOMINAL_DISPARITY_SIGMA_PX` defaults).

| axis | fraction of 105 valid, non-interpolated points within `3 sigma_base` |
|---|---|
| x | 1.000 |
| y | 1.000 |
| z | 1.000 |

All three axes clear the ≥ 0.90 bound with no failures (sample `sigma_base_m` ≈ `(0.59, 0.58,
0.65) mm` 1-sigma at this geometry) — the propagated covariance is, if anything, conservative
relative to the actual injected noise at this sample size, not under-covering it.

### Injected invalid-depth holes (criterion: §10.3 interpolation for a short gap, split for a
long gap)

Scene: tilt 0°, `seed=1`, `--holes`. One single-pixel zero-depth disk (radius tuned so its own
plus bleed-into-neighbours invalid run stays at or under `max_gap_px=5`) and one 10-pixel-wide
zero-depth run (radius `annulus_outer_px=8`, well past `max_gap_px`).

- Short hole: produced a 4-point run (`interpolated_idx = [82, 83, 84, 85]` for the paths.json
  order this run happened to land in — §10.3 numbers the chain from whichever end `crackvision.
  paths`' two-sweep-Dijkstra diameter search started from, so absolute indices are not
  reproducible run-to-run, only the *behaviour*). All 4 are `valid: true`, `interpolated: true`,
  `reason: null` (§10.2: a valid point's `reason` is always `null`, interpolated or not — the
  internal `"interpolated:<cause>"` tag lift3d tracks is bookkeeping, not part of the §10 schema).
- Long hole: 15 consecutive points stayed invalid (`reason` one of
  `insufficient_valid_fraction`/`no_valid_depth_in_annulus`, 5 and 10 points respectively) and the
  polyline was split into 2 `segments` (`(0,46)` and `(62,104)` for this run's numbering) —
  confirming the `max_gap_px` boundary is respected in both directions.

### IK / joint-limit self-check

Every generated scene asserts (`tools/synth_scene3d.py:generate_case`, re-checked by
`tests/test_path3d_synthetic.py::test_ik_converges_within_tolerance_and_joint_limits`) that the
best-of-grid IK solution's residual norm is `< 1e-6` (observed: `1.38e-17` at 0° view tilt,
`4.50e-17` at 15° view tilt) and that `chain.fk(q)` does not raise (i.e. every joint stays within
its URDF limits).

### Refusal rules (docs/INTERFACES.md §9.4/§10.6)

- A capture record with its `robot` block removed: `crackvision.path3d` exits **3**
  (`EXIT_PRECONDITION`), reason `capture_record_invalid` — verified directly against the CLI
  (`crackvision.path3d.main`), not just the library function.
- A nominal, synthetic capture (the default for every scene this generator produces, since
  `config/robot/end_effector.yaml` is still nominal and `synthetic: true` is always set):
  `execution_eligible: false`, with `wrist_camera_nominal`, `tool_nominal`,
  `optical_frames_nominal` and `synthetic_capture` all present in `ineligible_reasons`.

## Seeds and commands

```
./env.sh python tools/synth_scene3d.py --out /tmp/demo --seed 1 --view-tilt-deg 0
./env.sh python tools/synth_scene3d.py --out /tmp/demo --seed 1 --view-tilt-deg 15
./env.sh python tools/synth_scene3d.py --out /tmp/demo --seed 1 --view-tilt-deg 0 --holes
./env.sh python tools/synth_scene3d.py --out /tmp/demo --seed 42 --view-tilt-deg 0 --depth-noise

./env.sh pytest tests/test_path3d_synthetic.py -q -p no:cacheprovider
./env.sh pytest tests/ -q -p no:cacheprovider
```

`tests/test_path3d_synthetic.py` fixes `SEED = 1` for every noise-free/holes scene and
`NOISE_SEED = 42` for the depth-noise scene; both are deterministic given those seeds (the only
randomness in the generator is `numpy.random.default_rng(seed)` for the optional depth noise).

## Limitations (read before using these numbers for anything beyond this card)

- **Nominal chain only.** `FK(q)`, `T_gripper_link_camera_link` and
  `T_camera_link_camera_color_optical_frame` are all exactly as declared in
  `config/robot/end_effector.yaml`/the URDF/the §9.1 `nominal_d405` convention — there is no
  encoder error, no mounting error, no thermal drift. This is precisely what
  `uncertainty_model.unmodelled` already lists (`joint_encoder_and_fk`, `depth_bias`,
  `distortion_jacobian`, `capture_time_skew_motion`, `thermal_drift`); this card does not touch
  any of them.
- **Pinhole-only intrinsics.** `model: "none"` — no distortion model is exercised here; GEOM-02's
  `pyrealsense2` parity tests (`tests/test_geometry.py`) are what cover the Brown-Conrady path.
- **No real depth-sensor bias.** The injected noise is textbook Gaussian at the GEOM-02 stereo
  model's nominal sigma; the real D405's bias/non-Gaussian error modes are not modelled.
- **Synthetic, perfectly flat surfaces only.** One geometry (flat plane, two view tilts, one sine
  crack) is checked. It is not a claim that every curvature/crack-shape combination rasterises
  this cleanly — see the "Rasterisation note" above.
- **This is not GEOM-05/07 or INT-04/05.** It verifies that `crackvision.path3d`'s arithmetic
  matches the geometry it claims to implement, under a chain whose every transform is taken as
  exactly true. Real camera-to-robot calibration residuals (GEOM-05), real tool-tip calibration
  (GEOM-07), and end-to-end accuracy against the physical robot/camera/specimen (INT-04/05) are
  separate, not-yet-done verification steps that this card's numbers cannot stand in for.

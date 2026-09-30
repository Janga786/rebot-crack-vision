# GEOM-08.1 — Contracts: eye-in-hand capture record (§9) + robot-frame 3D path / tool-waypoint file with uncertainty and execution-eligibility policy (§10)

```json card
{
  "kind": "decision",
  "depends_on": [
    "GEOM-02",
    "GEOM-03",
    "PERC-03",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-GEOM-1",
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/COMPLETION_LOG.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#0.5",
    "docs/INTERFACES.md#3.10",
    "docs/INTERFACES.md#3.13",
    "docs/INTERFACES.md#6",
    "docs/INTERFACES.md#8",
    "docs/adr/012-frames-and-conventions.md",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "docs/calibration/PLAN.md#5",
    "config/robot/end_effector.yaml",
    "src/crackvision/geometry.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "sections",
        "cmd": "grep -q '^## 9\\. ' docs/INTERFACES.md && grep -q '^## 10\\. ' docs/INTERFACES.md && grep -q 'crackvision.capture_3d/1' docs/INTERFACES.md && grep -q 'crackvision.paths3d/1' docs/INTERFACES.md && grep -q 'execution_eligible' docs/INTERFACES.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "sections-0-8-unchanged",
        "cmd": "git diff --unified=0 HEAD~1 -- docs/INTERFACES.md | grep -E '^-[^-]' ; test $? -eq 1",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "contracts",
        "cmd": "./env.sh pytest tests/test_contracts.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§0–§8 are byte-identical. §9 and §10 are appended only.",
      "§9 fully specifies every field, type, unit and frame of the capture record, plus the refusal rules (missing joint state, missing or mismatched end_effector sha256, excessive clock skew, image-shape mismatch, downscaled case).",
      "§10 fully specifies the paths3d JSON (every field named, typed and framed), the annulus/gap policy, the covariance formula, the nominal calibration priors with their rationale, the waypoint and roll policy, and the eligibility reasons.",
      "The transform chain is written exactly as INTERFACES §8.4 / ADR-014 §4, using ADR-012 T_a_b notation and xyzw quaternions.",
      "The fit_plane sign pitfall and the joint6 driver-vs-gripper-model caveat are both documented.",
      "The uncertainty fields GEOM-05 and GEOM-07 must write (`uncertainty: {position_sigma_m, rotation_sigma_rad, source}`) are stated as a requirement on those cards."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "contract-spec",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-08.1",
  "parent": "GEOM-08",
  "title": "Contracts: eye-in-hand capture record (§9) + robot-frame 3D path / tool-waypoint file with uncertainty and execution-eligibility policy (§10)",
  "outcome": "docs/INTERFACES.md gains two appended normative sections: §9 the `crackvision.capture_3d/1` capture record (implements §8.4) and §10 the `crackvision.paths3d/1` file, its lifting/gap policy, first-order uncertainty model, tool-waypoint policy and execution-eligibility rules. Every later GEOM-08.x card implements against these sections."
}
```

Append two sections to docs/INTERFACES.md. Do not edit §0–§8 (§8.4 is implemented here, not changed). Keep the style of §3.13/§8: tables, a JSON example, then bullet rules. Also append one entry to docs/COMPLETION_LOG.md.

## §9 Eye-in-hand capture record (`crackvision.capture_3d/1`), implements §8.4
File: `data/captures/{case}_capture.json`, keyed by the §1 case_id, never committed. Fields:
- `schema`: "crackvision.capture_3d/1"; `case_id`; `synthetic`: bool (mandatory; true for any generated scene).
- `image`: {height, width}. Must equal the colour image, the aligned depth PNG, the mask and `paths.json` image_height/width (§0.5). A case_map entry with `downscaled: true` is refused.
- `color_intrinsics`: {width, height, fx, fy, ppx, ppy, model, coeffs[5]}, the §3.10 colour intrinsics. Consumed via `geometry.Intrinsics`.
- `depth`: {file (repo-relative uint16 z16 PNG, aligned to colour), depth_scale_m_per_unit (from the device, never defaulted), aligned_to: "color"}.
- `source`: {kind: d405_metadata|recording|synthetic, ref: repo-relative path, frame_index: int|null}.
- `capture_stamp_ns`: int, image capture time. `clock`: "utc_epoch_ns" (the only clock in v1).
- `robot`: {joint_names: [joint1..joint6] (exactly, in order), positions_rad: [6 floats], stamp_ns: int (same clock), stamp_source: str, kinematic_model: {path: repo-relative URDF, sha256}}.
- `end_effector`: {path: "config/robot/end_effector.yaml", sha256 (of the file bytes at capture), wrist_camera_value_status, tool_value_status}.
- `camera_optical`: {T_camera_link_camera_color_optical_frame: {xyz_m[3], quat_xyzw[4]}, source: driver_tf|nominal_d405}. `nominal_d405` = zero translation, rpy (−π/2, 0, −π/2): optical +z = camera_link +x, optical +x = −y, optical +y = −z. This is the realsense-ros nominal for the D405, whose colour stream is the left imager.

Refusal rules (consumers raise, and CLIs exit 3):
- the robot block is missing or incomplete, or a joint is outside the URDF limits by more than 1e-3 rad;
- `end_effector.sha256` is missing;
- |capture_stamp_ns − robot.stamp_ns| > max skew (default 0.1 s; the arm must be stationary at capture);
- `end_effector.sha256` differs from the current file, unless the consumer is given an explicit override. With the override, the output records both hashes and is ineligible for execution.

A capture without a robot block is valid for 2D only (§8.4).

Chain, stated exactly: p_base_link = FK(q) · T_gripper_link_camera_link · T_camera_link_camera_color_optical_frame · p_optical.
- FK uses the canonical gripper model (docs/motion/ROBOT_MODEL.md). The committed copy `presentation/sim/reBot_B601_DM_with_gripper.urdf` has kinematics identical to the vendor model.
- Caveat: the real driver's robot_description (`reBot-DevArm_fixend.urdf`) places joint6's origin 4.3 mm differently. Joint values are shared, and FK follows MoveIt's gripper model. This is an open item for MOT-09 / GEOM-09.

## §10 Robot-frame 3D crack paths and tool waypoints (`crackvision.paths3d/1`)
File: `data/paths3d/{case}_paths3d.json`. Top-level fields:
- `schema`, `case_id`, `frame`: "base_link", image_height/width;
- `sources`: {paths_json, capture_record, mask, end_effector: {sha256_at_capture, sha256_now}, urdf_sha256}, with a sha256 for each file;
- `parameters` (every value below);
- `calibration`: {wrist_camera: {value_status, position_sigma_m, rotation_sigma_rad, source}, tool: {value_status, position_sigma_m, source}};
- `uncertainty_model`: {terms[], unmodelled[]};
- `execution_eligible`: bool; `ineligible_reasons`: [str];
- `counts`: {points, valid, interpolated, invalid_by_reason{}}.

`components[]` mirrors paths.json: order_index, component_id, `polylines[]` with kind main|branch and branch_index (null for main). Each polyline has:
- `points[]`, one per DENSE pixel, in paths.json order: {row, col, u, v, valid, reason, interpolated, depth_m, fraction_valid, p_optical_m[3], p_base_m[3], n_base[3], normal_rms_m, sigma_normal_rad, cov_base_m2[9] (row-major 3×3), sigma_base_m[3]}. Invalid points carry null geometry, never a point at the camera origin.
- `segments[]`: {start_index, end_index (inclusive, into points), waypoints[]}.
- Each waypoint: {phase: approach|trace|retract, position_m[3], quat_xyzw[4] (pose of `tool_tip` in base_link), surface_point_m[3], normal_base[3], clearance_m, sigma_along_normal_m, sigma_max_m, interpolated, within_budget}.

Lifting policy:
- Surface depth per dense pixel: `geometry.sample_surface_depth_annulus` on the crack mask (R-08). Defaults: inner 3 px, outer 8 px, min_fraction_valid 0.3; valid band from `geometry.DEFAULT_VALID_DEPTH_RANGE_M`.
- The surface point is the crack pixel (u, v) deprojected at that surface depth (`geometry.deproject_pixels`).
- Normal: plane fit over the deprojected valid, non-mask annulus pixels (≥ 6 points, else reason `normal_fit_failed`). The OUTWARD normal is chosen explicitly by n·(−p_optical) > 0. NOTE: `geometry.fit_plane` canonicalises to optical n_z ≥ 0, which points away from the camera despite its comment, so never rely on its sign.
- Gaps: a run of ≤ max_gap_px (default 5) invalid points between valid ones is interpolated linearly in base_link: position, and a normalised lerp of normals. Those points get `interpolated: true` and the covariance of the larger-trace neighbour. Longer runs split the polyline into segments. Leading and trailing invalid runs are trimmed, never extrapolated. Segments with < 3 valid points are dropped, and the drop is counted.

Uncertainty (first order; GEOM-09 owns the full budget):
- σ_px default 1.0 px (skeleton-centreline prior). σ_z = `geometry.depth_uncertainty_m(z, fx)`.
- Σ_opt = J·diag(σ_px², σ_px², σ_z²)·Jᵀ with J = [[z/fx, 0, x/z], [0, z/fy, y/z], [0, 0, 1]] (pinhole linearisation; distortion Jacobian neglected, and listed as unmodelled).
- Σ_base = R·Σ_opt·Rᵀ + (σ_cam_pos² + (‖p_opt‖·σ_cam_rot)²)·I₃, where R is the rotation of T_base_link_camera_color_optical_frame.
- σ_normal_rad = normal_rms_m / (outer_radius_px·z/fx).
- Per waypoint: σ_along_normal = sqrt(nᵀ·Σ_base·n + σ_tool_pos²); within_budget ⇔ 3·σ_along_normal ≤ clearance_m.
- Calibration sigmas: for a `measured` block, read from its `uncertainty: {position_sigma_m, rotation_sigma_rad, source}`, which GEOM-05 and GEOM-07 MUST write, from the PLAN §4 layer-3 held-out residuals. A measured block without it → refuse. Nominal priors: wrist_camera 0.010 m / 5° (ADR-014 §5 prior-disagreement flag thresholds); tool 0.005 m (end_effector.yaml: "a few mm" seating error).
- unmodelled: joint_encoder_and_fk, depth_bias, distortion_jacobian, capture_time_skew_motion, thermal_drift.

Waypoint policy (ADR-014 §2):
- Resample each segment by 3D arc length at waypoint_spacing_m (default 0.002), both ends included. Normals are the normalised lerp of neighbours.
- tool_tip +X = −n_out (boresight into the surface). position = surface_point + clearance·n_out, with trace clearance 0.01 m.
- Roll is free (`roll_free: true` in parameters; MOT-06 may re-roll). The deterministic default is parallel transport. The first +Z is the capture-pose tool_tip Z (FK(q)·T_gripper_link_tool_tip) projected onto the plane ⟂ X; if its norm is < 1e-6, fall back to base Z, then base X. Each next Z = the previous Z projected and renormalised.
- One approach waypoint before and one retract waypoint after each segment, at 0.04 m along that end's n_out, with the same orientation.

Eligibility: execution_eligible = false, listing every reason, if any of:
- wrist_camera_nominal;
- tool_nominal;
- optical_frames_nominal (source nominal_d405);
- synthetic_capture;
- end_effector_changed_since_capture;
- uncertainty_exceeds_clearance (any waypoint with within_budget false);
- no_valid_points.

The file is still written for mock/sim planning. MOT-05 must refuse real execution of any path whose file says false.

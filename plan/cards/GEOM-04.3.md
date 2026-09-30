# GEOM-04.3 — Hand-eye AX=XB solve core: cross-method agreement, held-out residuals, CAD-prior flag, sigma_calib (crackvision.calibration.handeye)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-04.1",
    "GEOM-03",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/calibration/handeye.py",
      "tests/test_handeye.py"
    ]
  },
  "inputs": [
    "docs/calibration/PLAN.md#3",
    "docs/calibration/PLAN.md#4",
    "docs/calibration/PLAN.md#5",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "config/robot/end_effector.yaml",
    "src/crackvision/calibration/tcp.py",
    "ros2_ws/src/crackvision_description/crackvision_description/end_effector.py#rpy_to_matrix"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_handeye.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Uses the real OpenCV symbol names cv2.CALIB_HAND_EYE_TSAI/PARK/DANIILIDIS (PLAN §3's 'CALIBRATE_HAND_EYE_*' spelling is a wording slip, not a different decision — call this out in a comment so nobody 'fixes' it back).",
      "solve_hand_eye(poses, methods=('tsai','park','daniilidis')) raises below a MIN_POSES=3 algorithmic floor (cv2.calibrateHandEye's own minimum) and otherwise returns one HandEyeResult (rotation+translation, quat_xyzw + translation_m) per method, all expressing T_gripper_link_camera_link.",
      "cross_method_agreement computes every pairwise rotation geodesic angle (deg) and translation Euclidean distance (mm) across the solved methods and reports max/all pairs against PLAN §4 layer-2 thresholds (<0.3 deg, <1.5 mm) without averaging a failing pair away.",
      "held_out_residuals(X, held_out_poses, solve_poses) reconstructs the fixed target pose in base_link, T_base_target_i = T_base_gripper_i @ X @ T_camera_target_i, for every solve pose to form a reference (their mean pose) and for every held-out pose, and reports each held-out pose's position (m) and orientation (deg) deviation from that reference plus rms/max — this is the PLAN §4 layer-3 number, and the implementation must state this construction explicitly in a docstring (analogous to tcp.py's pivot-consistency check, generalised from a 3-DOF point to a 6-DOF pose).",
      "check_against_cad_prior(X) reads config/robot/end_effector.yaml's wrist_camera xyz_m/rpy_rad (rpy composed exactly as crackvision_description.end_effector.rpy_to_matrix does: R = Rz(yaw)@Ry(pitch)@Rx(roll), equivalently scipy Rotation.from_euler('XYZ', rpy_rad)) and flags (does not silently average away) any solve whose position delta exceeds 10 mm or whose rotation geodesic angle exceeds 5 deg from that prior, per ADR-014 §5.",
      "sigma_calib_from_held_out(held_out_result) returns (position_sigma_m, rotation_sigma_rad) as the rms held-out position/orientation residuals from held_out_residuals — named to match the position_sigma_m/rotation_sigma_rad fields a future measured end_effector.yaml wrist_camera.uncertainty block would carry (GEOM-05 writes that block; this function just produces the numbers).",
      "Synthetic tests cover: (a) noise-free recovery of a random T_gripper_camera to near machine precision; (b) recovery within PLAN §4's thresholds under injected Gaussian FK-pose noise and target-pose/detection noise at a plausible level, explicitly including a scenario built from the actual ADR-014 CAD-prior pose (15 deg pitch, config/robot/end_effector.yaml's wrist_camera values) and a second scenario using a deliberately mirrored/flipped mount-side pose; (c) check_against_cad_prior correctly passes scenario (b-first) and correctly flags scenario (b-second, wrong mount side); (d) cross_method_agreement/held_out_residuals visibly degrade on a deliberately low-orientation-diversity pose set versus a diverse one, mirroring tcp.py's degenerate-pose regression test."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 4,
    "consequence": 4,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-04.3",
  "parent": "GEOM-04",
  "title": "Hand-eye AX=XB solve core: cross-method agreement, held-out residuals, CAD-prior flag, sigma_calib (crackvision.calibration.handeye)",
  "outcome": "crackvision.calibration.handeye solves eye-in-hand AX=XB for T_gripper_link_camera_link with cv2.calibrateHandEye across TSAI/PARK/DANIILIDIS, checks the docs/calibration/PLAN.md §4 layer-2 cross-method agreement and layer-3 held-out residuals, flags disagreement with the config/robot/end_effector.yaml CAD prior beyond ADR-014's 10 mm/5 deg thresholds, and reports sigma_calib for the measured wrist_camera uncertainty GEOM-05 will later write."
}
```

Implement `src/crackvision/calibration/handeye.py`, mirroring `tcp.py`'s style (frozen dataclasses, explicit docstrings naming the exact equation each function solves, no cv2 black-box left unexplained).

- `HAND_EYE_METHODS = {'tsai': cv2.CALIB_HAND_EYE_TSAI, 'park': cv2.CALIB_HAND_EYE_PARK, 'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS}` (PLAN §3's mandated three; HORAUD/ANDREFF exist in OpenCV but are out of PLAN's scope, do not add them).
- `MIN_POSES = 3`.
- `@dataclass(frozen=True) HandEyePoseSample`: `quat_xyzw_gripper (4,)`, `translation_m_gripper (3,)` (both describing `T_base_link_gripper_link`, ADR-012 convention, mirroring `tcp.PoseSample`), `rotation_target2cam (3,3)`, `translation_target2cam_m (3,)` (from `charuco.DetectionResult`), plus pass-through `reprojection_rms_px`, `num_corners` for reporting.
- `@dataclass(frozen=True) HandEyeResult`: `method: str`, `rotation_gripper_camera (3,3)`, `translation_gripper_camera_m (3,)`, `quat_xyzw`, `translation_m` (the last two = `T_gripper_link_camera_link`, ready to write into `end_effector.yaml`'s `wrist_camera` block shape).
- `solve_hand_eye(poses, methods=HAND_EYE_METHODS) -> dict[str, HandEyeResult]`.
- `@dataclass(frozen=True) CrossMethodAgreement`: pairwise table (`{(m1, m2): {'rotation_deg': ..., 'translation_m': ...}}`), `max_rotation_diff_deg`, `max_translation_diff_m`, `passes: bool` against `LAYER2_ROTATION_THRESHOLD_DEG = 0.3`, `LAYER2_TRANSLATION_THRESHOLD_M = 0.0015`.
- `cross_method_agreement(results: dict[str, HandEyeResult]) -> CrossMethodAgreement`.
- `@dataclass(frozen=True) HeldOutResiduals`: per-held-out-pose position/orientation deviation, `rms_position_m`, `max_position_m`, `rms_orientation_rad`, `max_orientation_rad`, `passes: bool` against `LAYER3_POSITION_THRESHOLD_M = 0.002`, `LAYER3_ORIENTATION_THRESHOLD_RAD = math.radians(0.5)`.
- `held_out_residuals(x_result: HandEyeResult, solve_poses, held_out_poses) -> HeldOutResiduals`: build `T_base_target_i` for every solve pose (`T_base_gripper_i @ T_gripper_camera @ T_camera_target_i`), average to a reference pose (translation mean; rotation via `Rotation.mean()` or equivalent chordal mean), then the same construction for every held-out pose compared against that reference.
- `load_wrist_camera_prior(config_path=None) -> WristCameraPrior` (`xyz_m`, `rpy_rad`, `value_status`, `provenance`), a light yaml read exactly like `tcp.load_tool_tip_prior` but for the `wrist_camera` block.
- `@dataclass(frozen=True) CadPriorCheck`: `position_delta_m`, `position_delta_norm_m`, `rotation_delta_deg`, `flagged: bool` against `ADR014_POSITION_THRESHOLD_M = 0.010`, `ADR014_ROTATION_THRESHOLD_DEG = 5.0`.
- `check_against_cad_prior(x_result, prior: Optional[WristCameraPrior] = None) -> CadPriorCheck`.
- `sigma_calib_from_held_out(held_out: HeldOutResiduals) -> tuple[float, float]` = `(held_out.rms_position_m, held_out.rms_orientation_rad)`.

Build synthetic test poses directly as `HandEyePoseSample` (no need to render real images here — that realism is GEOM-04.4/04.5's job): pick a ground-truth `T_gripper_camera`, a fixed `T_base_target`, sample diverse `T_base_gripper_i` (orientation bands + azimuths per PLAN §2), derive the exact `T_camera_target_i = inv(T_gripper_camera) @ inv(T_base_gripper_i) @ T_base_target`, then perturb both sides with independent noise before solving.

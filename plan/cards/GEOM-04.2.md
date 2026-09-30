# GEOM-04.2 — ChArUco board target + detector wrapper with the layer-1 detection gate (crackvision.calibration.charuco)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-04.1",
    "GEOM-02",
    "GEOM-03"
  ],
  "requirements": [
    "REQ-GEOM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/calibration/charuco.py",
      "tests/test_charuco.py"
    ]
  },
  "inputs": [
    "docs/calibration/PLAN.md#1",
    "docs/calibration/PLAN.md#4",
    "docs/adr/012-frames-and-conventions.md",
    "src/crackvision/geometry.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_charuco.py -q -p no:cacheprovider",
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
      "BoardSpec matches PLAN §1 defaults exactly (DICT_5X5_250, squares_x=7, squares_y=5, nominal square_length_m=0.030, marker_length_m=0.022) but square_length_m/marker_length_m are required constructor args with no silent default used in a real solve — PLAN §1's mandatory caliper-measured print-scale check means the nominal value must never be the value actually passed into a solve; a docstring/comment says so plainly.",
      "detect(image, spec, intrinsics) returns: pixel corners, ids, num_corners, reprojection_rms_px, and a pass/fail 'ok' gated on num_corners >= 12 (of 24 interior corners, PLAN §2) AND reprojection_rms_px < LAYER1_REPROJECTION_RMS_PX = 0.5 (PLAN §4 layer 1) — never proceeds past a failing gate silently.",
      "Board pose in the camera optical frame (R_target2cam, t_target2cam) is computed via cv2.aruco.CharucoBoard.matchImagePoints + cv2.solvePnP (or the built-in board-pose estimator, whichever this OpenCV build actually exposes cleanly — verify which at implementation time) given a crackvision.geometry.Intrinsics-derived camera matrix/distortion vector, consistent with GEOM-02's distortion-coefficient layout.",
      "Unit tests render an actual board image with cv2.aruco.CharucoBoard.generateImage, warp it into a synthetic photo under a known camera pose/intrinsics (perspective warp, not just point projection), run detect() on the rendered pixels, and confirm the recovered pose matches the known one within a tight tolerance and the corner/RMS gate passes. A second test with the board mostly out of frame/occluded exercises the ok=False rejection path.",
      "The exposed synthetic-rendering helper (image + known pose given a BoardSpec/intrinsics) is a reusable function, not test-only code, so GEOM-04.4/04.5 can call it without duplicating the projection math."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-04.2",
  "parent": "GEOM-04",
  "title": "ChArUco board target + detector wrapper with the layer-1 detection gate (crackvision.calibration.charuco)",
  "outcome": "crackvision.calibration.charuco defines the docs/calibration/PLAN.md §1 ChArUco board, wraps cv2.aruco.CharucoDetector to return per-image corners/ids plus the board's pose in the camera optical frame and the §4 layer-1 reprojection-RMS gate, and includes a synthetic board-image renderer used by its own tests and reused by GEOM-04.4/04.5."
}
```

Implement `src/crackvision/calibration/charuco.py`.

- `@dataclass(frozen=True) BoardSpec`: `squares_x=7, squares_y=5, square_length_m, marker_length_m, dictionary_name='DICT_5X5_250'` (constructor requires square_length_m/marker_length_m explicitly — no default — per PLAN §1's mandatory measured-scale rule).
- `build_board(spec) -> cv2.aruco.CharucoBoard`.
- `LAYER1_REPROJECTION_RMS_PX = 0.5`; `MIN_CORNERS = 12` (of 24 interior corners for this board).
- `@dataclass(frozen=True) DetectionResult`: `corners (N,2) float`, `ids (N,) int`, `num_corners`, `reprojection_rms_px: Optional[float]`, `ok: bool`, `rotation_target2cam (3,3)`, `translation_target2cam_m (3,)` (both `None` when `ok` is False).
- `detect(image: np.ndarray, spec: BoardSpec, intrinsics: geometry.Intrinsics) -> DetectionResult`: run `cv2.aruco.CharucoDetector(build_board(spec)).detectBoard(image)`, gate on `MIN_CORNERS`, solve the board pose via PnP using `intrinsics`'s camera matrix/distortion (reuse `geometry.Intrinsics`'s existing fields — do not re-derive a second distortion-coefficient convention), compute reprojection RMS by re-projecting the solved corners and comparing to the detected pixels.
- `render_synthetic_view(spec, intrinsics, rotation_target2cam, translation_target2cam_m, image_size, *, noise_px=0.0, rng=None) -> np.ndarray`: generate the board image with `cv2.aruco.CharucoBoard.generateImage`, then warp/project it into a synthetic photo at the given target-in-camera pose (a full perspective warp of the printed board texture, not a bare point-cloud projection, so the returned image round-trips through the *real* detector in tests). Optionally add Gaussian pixel noise before returning.

Tests (`tests/test_charuco.py`): render at several known poses (including near the ADR-014 15° pitch's likely viewing geometry) and confirm `detect()` recovers each pose within a few mm / 1° and passes the layer-1 gate; a heavily-cropped/occluded render fails the gate (`ok=False`, corners below `MIN_CORNERS`); a known off-plane pose still solves (sanity against a degenerate planar-only PnP ambiguity).

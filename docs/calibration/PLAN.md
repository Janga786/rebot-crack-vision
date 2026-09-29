# PLAN.md — Camera-to-robot calibration plan (GEOM-03)

**Normative source for method rationale:** `docs/adr/013-calibration-method.md`. This document is
the operational checklist GEOM-04 (solver software) and GEOM-05 (operator hardware execution)
follow; it does not re-argue the decision, only lays out what to build/do and in what order.

## 0. Blocking status — read this first

**This plan is not authorized to run.** REQ-GEOM-2/3's calibration depends on one physical fact
that is not recorded anywhere in this repository or in `~/rebot_ws`: **where the D405 is mounted.**
Confirmed empty of any answer: the vendor/canonical URDF (`grep -rn "camera" *.urdf` → no hits),
every `plan/cards/GEOM-*.md` and `plan/cards/CAM-*.md`, `docs/ARCHITECTURE.md` ("No D405 is attached
to this machine right now"), and `docs/TECHNICAL_APPROACH.md` §2.1 ("there is no camera mounted on
this arm yet").

`docs/adr/013-calibration-method.md` recommends **eye-in-hand** (wrist-mounted) with supporting
reasoning, and this plan is written against that recommendation so it is ready to execute the
moment it is confirmed. But a recommendation is not a decision. **Before GEOM-04 is implemented
against this plan's specifics, or GEOM-05 executes any of it on hardware, the operator must confirm
in writing (a comment/commit note referencing this file, or an edit to this section) one of:**

- [ ] **Confirmed: eye-in-hand.** D405 rigidly mounted to `_____________` (arm link/bracket), fixed
      offset approximately `_____________`. → this plan applies as written.
- [ ] **Confirmed: eye-to-hand.** D405 fixed to `_____________` (stand/gantry/frame, described or
      referenced here). → §§2–5 switch to the robot-world branch noted inline below; the ChArUco
      spec, pose-diversity and residual-threshold content is unchanged.

Until one box above is checked (or an equivalent operator note exists), **GEOM-04/05 must not
proceed past synthetic-data testing**, and any status report on this card returns
`blocked` / `operator_decision` rather than `complete`.

## 1. Target: ChArUco board

| Parameter | Value | Why |
|---|---|---|
| Dictionary | `cv2.aruco.DICT_5X5_250` | Wide inter-marker Hamming distance; low false-positive rate at short range. |
| Board size | 7×5 squares (6×4 interior corners) | Matches the D405 colour stream's landscape aspect ratio at 10–40 cm standoff. |
| Nominal square length | 30 mm | Full board 210×150 mm — fits the frame at 15–30 cm, still resolvable at 40 cm. |
| Nominal marker length | 22 mm (`0.75 × square`) | OpenCV's standard marker-in-square ratio, leaves a white detection border. |
| Backing | Rigid flat plate (acrylic/aluminium), board bonded not just taped | `estimatePoseCharucoBoard` assumes planarity; a curled print reintroduces metric error the caliper check below can't catch after the fact. |

**Mandatory print-scale check** (do this before capturing any calibration pose):

1. Print the board at 100% scale (no "fit to page").
2. Measure at least 3 squares along each axis with digital calipers (not a ruler).
3. Compute `square_length_m = mean(measurements_mm) / 1000`.
4. Fail the check (reprint or re-measure) if any single measurement deviates from the nominal
   30 mm by more than 0.5% (0.15 mm).
5. Record the measured value in the calibration run's config/metadata — **never leave the nominal
   30 mm in the solver config uncorrected.**

## 2. Pose capture procedure

**Eye-in-hand branch (recommended, pending confirmation):** the board is fixed in the workspace
(e.g. taped flat on the bench/specimen fixture); the arm carries the camera through the pose set
below, capturing one still colour frame per pose (never during motion — matches this pipeline's
existing still-frame capture convention, `crackvision.realsense_capture`).

**Eye-to-hand branch (if confirmed instead):** the camera is fixed; the board is rigidly held by
the gripper (a printed board bonded to a fixture the gripper grips or bolts to `gripper_link`), and
the arm carries the *board* through an equivalent pose set relative to the fixed camera.

Either branch, the captured pose set must satisfy:

- **≥15 poses minimum, ≥20 recommended.**
- **Rotation diversity:** at least 3 tilt bands off boresight-down (≈±15°, ≈±30°, ≈±45°) **and**
  at least 4 azimuth/roll headings (≈0°, 90°, 180°, 270°) around the boresight, so collected
  rotation axes are not all near-parallel — a degenerate pose set (e.g. only wrist-yaw variation)
  leaves the hand-eye rotation solve under-determined regardless of pose count.
- **Translation diversity:** standoff varied across ≈15–40 cm, board not always centred in frame.
- **Full-board visibility:** ≥12 of 24 interior ChArUco corners detected per accepted pose; drop
  and re-capture any pose below that.
- **Reserve ≥4 poses as held-out** (captured the same way, excluded from the solve) for the layer-3
  verification residual in §4.

Record per pose: the robot's FK-reported `T_base_link_gripper_link` (or the fixed camera→world pose
in the eye-to-hand branch) at capture time, the raw colour frame, and the ChArUco detection result
(corner count, per-corner reprojection residual from `estimatePoseCharucoBoard`).

## 3. Algorithms

**Eye-in-hand branch:** `cv2.calibrateHandEye`, run with **all of**:
`CALIBRATE_HAND_EYE_TSAI`, `CALIBRATE_HAND_EYE_PARK`, and (once pose count comfortably exceeds 15)
`CALIBRATE_HAND_EYE_DANIILIDIS`. Report each method's `(R_cam2gripper, t_cam2gripper)` separately
before comparing.

**Eye-to-hand branch:** `cv2.calibrateRobotWorldHandEye`, run with both
`CALIBRATE_ROBOT_WORLD_HAND_EYE_SHAH` and `CALIBRATE_ROBOT_WORLD_HAND_EYE_LI`.

Neither branch accepts a single method's output on its own — see §4, layer 2.

## 4. Residual thresholds (all three layers must pass)

| Layer | What | Threshold |
|---|---|---|
| 1 — target detection (per pose, pre-solve gate) | ChArUco corner reprojection RMS | **< 0.5 px** |
| 2 — cross-method agreement (accept/reject the calibration) | Pairwise rotation difference between every pair of methods run | **< 0.3°** (geodesic) |
| 2 — cross-method agreement | Pairwise translation difference between every pair of methods run | **< 1.5 mm** (Euclidean) |
| 3 — held-out verification (final reported number) | Position residual, held-out poses | **< 2 mm** |
| 3 — held-out verification | Orientation residual, held-out poses | **< 0.5°** (matches `docs/motion/REACHABILITY.md`'s existing FK re-check tolerance) |

A pose or a full calibration run failing any layer is **re-captured/re-solved**, not silently
averaged past. GEOM-05's evidence record must include the actual numbers for all three layers, not
just pass/fail.

## 5. Uncertainty propagation

The final backprojected crack-path points (`p_base = T_base_camera · p_camera`, GEOM-02) carry two
independent error sources that must be combined, not reported separately as if only one existed:

1. **Depth/backprojection uncertainty** (REQ-GEOM-1, GEOM-02's own per-point model) — the D405's
   own per-pixel depth noise.
2. **Calibration uncertainty** — the §4 layer-3 held-out residual standard deviations, treated as a
   fixed (pose-independent) additional covariance term.

Combine per axis as `sigma_total ≈ sqrt(sigma_depth² + sigma_calib²)`. Every backprojected point
GEOM-02 (or later cards consuming it) reports must carry `sigma_total`, sourced from the actual
`sigma_calib` GEOM-05 measures — not a placeholder — once this calibration has been executed.

## 6. Evidence to produce (GEOM-04/GEOM-05)

- GEOM-04: synthetic-data test proving the solver recovers a known `T_gripper_camera` (or
  `T_base_world` in the eye-to-hand branch) within the §4 thresholds under injected pose/detection
  noise, before any hardware capture is attempted.
- GEOM-05: the real capture set (≥15 solve poses + ≥4 held-out), per-pose FK and detection
  residuals, the cross-method comparison table, and the held-out verification numbers — written as
  this card's evidence artefact once GEOM-05 is specified and run.

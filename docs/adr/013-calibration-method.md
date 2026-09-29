# ADR-013: Camera-to-robot calibration method + camera mounting

**Status:** proposed, blocked on operator confirmation · **Date:** 2026-09-29 · **Supersedes:** — · **Superseded by:** —

## Context
REQ-GEOM-2 (camera-to-robot calibration with explicit transform conventions and residuals) and
REQ-GEOM-3 (TCP calibration and physical boresight verification) need a concrete calibration
method before GEOM-04 (solver software) and GEOM-05 (operator execution) can be written. Two
questions have to be answered together, because the answer to the first constrains the algorithm
choice for the second:

1. **Where does the D405 physically mount?** Rigidly to the arm near the tool (*eye-in-hand*,
   `camera_link` a fixed child of some arm link, moving with every joint change) or fixed in the
   world, watching the arm/target from outside (*eye-to-hand*, `camera_link` a fixed child of
   `base_link` or an independent tripod/stand frame)?
2. **Which hand-eye solve fits that mounting?** `cv2.calibrateHandEye` (classic eye-in-hand/
   eye-to-hand formulation, `AX = XB`) or `cv2.calibrateRobotWorldHandEye` (robot-world formulation,
   needed when the target — not the camera — is the thing rigidly attached to the robot).

**No answer to question 1 exists anywhere in this repository or in `~/rebot_ws` today.**
Checked and confirmed empty of any camera-mount reference:
`~/rebot_ws/src/rebotarm_bringup/description/urdf/*.urdf` (no `camera_link`, no `d405`, no
`camera_mount` — `grep` returns nothing), the MoveIt xacro/SRDF, and every doc in this repo
(`ARCHITECTURE.md`, `TECHNICAL_APPROACH.md`, `docs/motion/*.md`, all `plan/cards/GEOM-*.md`,
`plan/cards/CAM-*.md`). `ARCHITECTURE.md` §"Environment" states plainly: **"No D405 is attached
to this machine right now."** `TECHNICAL_APPROACH.md` §2.1 states: "there is no camera mounted on
this arm yet." GEOM-05's card lists `d405_mounted` as a hardware precondition it still needs, not
a fact already established. No CAD, bracket part number, or operator note records an intended
mounting point or orientation.

Per this card's acceptance criteria, that absence is itself the answer this ADR must give: **this
decision is blocked on the operator**, not resolved by this document. Sections below give a
technical recommendation and a fully specified method *conditioned on* that recommendation, so
that once the operator confirms (or overrides) the mounting, GEOM-04/05 have nothing left to
design — only to implement/execute. Nothing in this ADR may be read as "the camera is mounted
eye-in-hand"; it is a proposal awaiting sign-off (see `docs/calibration/PLAN.md` §0).

## Decision

### Mounting: recommend eye-in-hand (wrist-mounted), pending operator confirmation
Recommended: D405 rigidly bolted to the arm near the tool (a fixed child frame of `gripper_link`
or another arm-side link, per ADR-012's frame table — the exact parent link is itself part of what
the operator confirms, since it depends on the physical bracket used).

Reasoning, all grounded in facts already recorded elsewhere in this repo:

- **The D405's own working range forces a close, moving camera.** `ARCHITECTURE.md`'s D405 section
  records the vendor's "ideal depth range ≈ 7 cm – 50 cm", chosen to match this project's planned
  **10–40 cm evaluation distances**. A single stationary (eye-to-hand) camera would have to see the
  *entire* specimen placement footprint — a 0.20×0.20 m square dilated by 0.02 m tolerance per
  `docs/INTERFACES.md` §7.3 / `docs/motion/REACHABILITY.md` — from one fixed 7–50 cm standoff. That
  is not simultaneously achievable at that range without a lens/FOV that also resolves crack-width
  detail; nothing in this repo specifies such a camera or a stand for it. An eye-in-hand rig instead
  re-uses the arm's own boresight-down sweep (`docs/motion/REACHABILITY.md` §2) to bring the camera
  to within 7–50 cm of whichever patch of the specimen it is currently inspecting.
- **The gripper already moves boresight-down over the whole specimen.** `docs/motion/REACHABILITY.md`
  §2 documents that motion planning already aligns the tool's local `+X` boresight with world `−Z`
  at each swept `(x, y)`. A wrist camera sharing that same approach geometry gets a fresh, close,
  roughly-perpendicular view of the crack at every tool position for free; an eye-to-hand camera
  would need the arm to stay clear of a fixed external sightline instead, adding a second geometric
  constraint on top of the one `docs/motion/REACHABILITY.md` already found infeasible for the
  current bench layout.
- **The device is physically sized for this.** The D405 is Intel's short-range stereo module
  explicitly aimed at close-range, in-hand robotic manipulation/inspection (compact ~42×42×23 mm
  body, USB-C), unlike the longer-range D4xx family. Nothing in the vendor SDK tree
  (`third_party/reBotArm_control_py`) or `~/rebot_ws` references a payload budget that would rule
  out wrist-mounting a device this small.
- **Cost acknowledged, not hidden:** eye-in-hand needs a cable run to the wrist (USB-C, routed along
  or through the arm) and means every capture must happen with the arm at rest (motion blur/vibration
  otherwise) — already this pipeline's convention (`crackvision.realsense_capture` captures still
  frames, not a live feed during motion). This is a real integration cost, not a reason to prefer
  eye-to-hand, which has no camera/stand of its own defined anywhere to absorb that cost instead.

**If the operator instead confirms eye-to-hand** (e.g. an existing camera stand/gantry not yet
documented here), the frame graph, target-motion convention and the robot-world solve in the
Algorithms section below apply unchanged — only the extrinsic parent frame moves from an arm link
to a `base_link`- or world-fixed one, and the pose-diversity source moves from "move the arm, hold
the target still" to "hold the camera still, move the target with the gripper" (§ Poses below).

### Target: ChArUco board
ChArUco (chessboard + ArUco corners) over a plain chessboard or a plain ArUco grid, because it gives
sub-pixel chessboard-corner precision *and* per-square ArUco IDs that disambiguate orientation and
tolerate partial occlusion — important at the D405's short working distance, where the board will
routinely fill or exceed the frame at some poses.

- **Dictionary:** `cv2.aruco.DICT_5X5_250` (5×5-bit markers, 250-symbol dictionary — enough unique
  IDs for the board below with a wide inter-marker Hamming distance, low false-detection rate at
  the close ranges this rig operates in).
- **Board layout:** 7×5 squares (a 7×5 `CharucoBoard`, i.e. 6×4 interior corners), print-oriented
  landscape to roughly match the D405 colour stream's 848×480/1280×720 aspect ratio at the planned
  10–40 cm standoff.
- **Nominal square size:** 30 mm squares, 22 mm markers (`marker_length = 0.75 * square_length`,
  OpenCV's usual ratio so the marker fits inside its square with a white border for detection). At
  30 mm squares, the full 7×5 board is 210×150 mm, comfortably inside the D405's field of view at
  15–30 cm and still resolvable at 40 cm.
- **Print-scale verification (mandatory, not optional):** OpenCV's `calibrateHandEye` inputs
  (`target2cam`) come from `cv2.aruco.estimatePoseCharucoBoard`, which uses the *nominal*
  `square_length` baked into the `CharucoBoard` object to convert pixel detections to metric board
  poses. If the printed square is not exactly 30.000 mm, every `target2cam` translation is wrong by
  that same fractional error, and it propagates directly into the final hand-eye translation. The
  capture procedure (`docs/calibration/PLAN.md` §2) requires measuring **at least 3 printed squares
  along both board axes with digital calipers** (not a ruler), averaging, and setting
  `square_length_m = measured_mean_mm / 1000` in the calibration config **before** any pose is
  captured — never trust the printer/PDF's nominal 30 mm. A >0.5% deviation (>0.15 mm on a 30 mm
  square) fails the print-scale check and the board must be reprinted or the config corrected and
  the check repeated.
- **Mounting the board:** rigid, flat backing (e.g. bonded to a stiff acrylic or aluminium plate) —
  a curled/bent print reintroduces the same kind of metric error the caliper check is guarding
  against, undetectably, because `estimatePoseCharucoBoard` assumes planarity.

### Pose set: ≥15 poses, explicit rotation diversity
`AX = XB` (or the robot-world equivalent) is only well-conditioned when the *rotation* part of the
collected poses spans at least 3 non-parallel rotation axes — a pose set that only rotates about one
axis (e.g. only ever yawing the wrist) leaves the corresponding hand-eye rotation under-determined
even with hundreds of poses, a well-known degeneracy of Tsai/Park-style solvers. Requirements for
`docs/calibration/PLAN.md`'s capture procedure:

- **≥15 poses minimum** (this card's floor), **≥20 recommended** in practice — more poses reduce the
  solve's sensitivity to any single noisy ChArUco detection, and captures are cheap (no motion, just
  a still frame per pose).
- **Rotation diversity, concretely:** vary tilt in at least 3 bands (e.g. ~±15°, ~±30°, ~±45° off
  boresight-down) **and** vary azimuth/roll around the boresight through at least 4 distinct
  headings (e.g. ~0°, 90°, 180°, 270°) so the collected rotations are not coplanar in axis-angle
  space. A pose-generation helper should assert this (e.g. reject a set whose rotation axes, taken
  pairwise, are all within a small angle of each other) rather than trust visual inspection.
- **Translation diversity:** vary the camera-to-target standoff across the D405's valid band
  (roughly 15–40 cm, staying inside the 7–50 cm ideal range with margin) and shift the target/board
  laterally in the frame between poses (not always centred) — this decorrelates the translation
  solve from the rotation solve.
- **Full-board visibility:** every capture must have enough ChArUco corners detected
  (`estimatePoseCharucoBoard` return count) to solve a stable planar pose — reject any capture below
  a minimum interior-corner count (recommend ≥ 12 of the 24 interior corners) rather than silently
  keeping a weak detection in the solve.

### Algorithms: ≥2 cross-checked `calibrateHandEye` methods (+ robot-world variant if eye-to-hand)
For the eye-in-hand case (recommended mounting): for each retained pose, compute
`R_gripper2base, t_gripper2base` from the robot's own forward kinematics/TF at capture time (the
`T_base_link_gripper_link` this ADR's parent, ADR-012, already names), and
`R_target2cam, t_target2cam` from `estimatePoseCharucoBoard`. Solve with `cv2.calibrateHandEye`
using **at least two independent methods** and require them to agree (see Residuals below) before
accepting the result — a single method's output has no way to flag that it converged to a
plausible-looking but wrong answer:

1. **`CALIBRATE_HAND_EYE_TSAI`** (Tsai–Lenz) — the standard baseline; fast, closed-form,
   separates rotation then translation.
2. **`CALIBRATE_HAND_EYE_PARK`** (Park–Martin) — closed-form but solves rotation via the
   Lie-algebra/matrix-log formulation, different numerical behaviour under noise than Tsai; a
   second independent cross-check rather than a variant of the same derivation.
3. **`CALIBRATE_HAND_EYE_DANIILIDIS`** (dual quaternion) — solves rotation and translation
   *simultaneously* rather than sequentially, so it does not inherit any rotation-solve error into
   the translation solve the way Tsai/Park structurally do; used as the third cross-check when
   pose count is comfortably above the 15-pose floor (its dual-quaternion SVD wants more
   well-conditioned data to be reliable at exactly 15).

If the operator instead confirms **eye-to-hand** mounting, the board must instead be rigidly held by
the gripper (camera fixed, target moves) and the solve is the **robot-world** formulation,
`cv2.calibrateRobotWorldHandEye`, which simultaneously recovers `T_base_world` (here, effectively
the fixed camera's pose relative to the robot base — the extrinsic this ADR actually wants) and
`T_gripper2cam`-analogue (the target's fixed pose relative to the gripper), because in this
configuration neither is known ahead of time the way `T_gripper2cam` is assumed fixed in the
eye-in-hand case. Cross-check its two available methods, `CALIBRATE_ROBOT_WORLD_HAND_EYE_SHAH` and
`CALIBRATE_ROBOT_WORLD_HAND_EYE_LI`, the same way as above.

### Residual thresholds and uncertainty propagation
Three residual layers, each with its own threshold, because a small error at one layer can still
compound downstream:

1. **ChArUco reprojection residual** (per pose, from `estimatePoseCharucoBoard`'s own solve):
   **RMS < 0.5 px** across detected corners. This is a target-detection quality gate, run *before*
   the pose is accepted into the hand-eye solve — a pose failing this threshold is dropped and
   re-captured, not down-weighted.
2. **Cross-method agreement** (the accept/reject gate for the calibration itself): pairwise between
   every pair of the ≥2 methods run, **rotation difference < 0.3° (geodesic angle between the two
   solved rotation matrices)** and **translation difference < 1.5 mm (Euclidean norm)**. Methods
   disagreeing beyond this indicate either too few poses, insufficient rotation diversity, or a bad
   print-scale/detection input — the fix is more/better poses, never averaging away a disagreement
   silently.
3. **Held-out verification residual** (the final acceptance number GEOM-05 reports): capture
   additional poses **not used in the solve**, reproject each held-out pose's known board corner
   through the solved `T_base_camera` and the pose's own FK, and compare against the detected pixel
   / the independently measured 3D point. **Position residual < 2 mm, orientation residual < 0.5°**
   — the 0.5° figure matches the FK re-check tolerance `docs/motion/REACHABILITY.md` already uses
   elsewhere in this repo (§ `fk_mismatch`, 1 mm / 0.5°) so calibration and planning report
   comparable numbers; the 2 mm position figure is intentionally tighter than that 1 mm/0.5°'s
   companion 1 mm (which bounds MoveIt's own IK/FK self-consistency, not sensor-plus-calibration
   error) to leave headroom for the D405's own depth noise (approximately 1–2 mm 1σ at 20–30 cm,
   vendor-typical for stereo depth at this range) stacking with calibration error.

**Uncertainty propagation:** the final 3D crack-path points GEOM-02 backprojects are
`p_base = T_base_camera · p_camera`, where `p_camera` itself carries the D405's own per-pixel depth
uncertainty (REQ-GEOM-1, already the subject of GEOM-02's per-point uncertainty). `T_base_camera`'s
calibration uncertainty (layers 2–3 above) composes with that depth uncertainty rather than
replacing it: treat the calibration's rotation/translation residuals as an additional fixed
(pose-independent) covariance contribution, and report a **combined** uncertainty
(`sigma_total ≈ sqrt(sigma_depth² + sigma_calib²)` per axis, calibration residual standard deviations
computed from the held-out verification set in layer 3) alongside every backprojected point — never
report GEOM-02's depth-only uncertainty as if it were the whole picture once GEOM-05 measures actual
calibration residuals. GEOM-04's solver report and GEOM-05's operator execution record are both
required to write these residual numbers (not just "pass/fail") into
`docs/calibration/PLAN.md`-referenced evidence so this propagation step has real numbers to combine
with the depth model's.

## Consequences
+ GEOM-04 (solver software) has an unambiguous algorithm list, pose-diversity contract and
  acceptance thresholds to implement and unit-test on synthetic data before any hardware is
  involved.
+ GEOM-05 (operator execution) has a concrete target spec (with a mandatory print-scale check) and
  a residual report format to fill in, rather than inventing thresholds during a hardware session.
+ Cross-checking ≥2 (eye-in-hand) or both (robot-world) methods turns "the solver produced a number"
  into "the solver produced a number *and* a second independent method agrees with it," catching the
  silent-wrong-answer failure mode a single method solve cannot self-diagnose.
- This ADR does not itself decide the mounting — it recommends eye-in-hand and gives the full method
  for it, but `docs/calibration/PLAN.md` §0 formally blocks GEOM-04/05 until the operator confirms
  (or overrides) that recommendation in writing. No downstream card may treat "eye-in-hand" as
  decided until that confirmation exists.
- The 7×5/30 mm ChArUco spec and the residual thresholds are sized for the D405's 7–50 cm range and
  this project's 10–40 cm working distance; a different camera or working distance would need this
  ADR revisited, not silently reused.

## Revisit when
The operator confirms (or overrides) the camera mounting — at which point
`docs/calibration/PLAN.md` §0's block is lifted and, if the confirmed mounting is eye-to-hand
instead of the eye-in-hand recommended here, this ADR is revised (not superseded) to make the
robot-world branch primary. Also revisit if GEOM-05's measured residuals repeatedly fail the
thresholds above with a properly diverse, print-verified pose set — that would mean the thresholds
themselves, not the capture procedure, were miscalibrated to this rig.

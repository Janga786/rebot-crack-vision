# TCP_BORESIGHT_PROCEDURE.md — tool_tip pivot calibration + physical boresight verification (GEOM-06/07)

**Normative source for frame/transform conventions:** `docs/adr/012-frames-and-conventions.md`
(`base_link`, `tool0` == `gripper_link`, `T_a_b` composition, `xyzw` quaternions, SI units).
**Normative source for the tool and its prior:** `docs/adr/014-end-effector-frames-and-task-phases.md`
§5 / `docs/INTERFACES.md` §8: the tool actually calibrated by this procedure is `tool_tip` —
`config/robot/end_effector.yaml`'s `tool` block, the physical crack-facing point (for the closed
gripper, the fingertip; for a held probe/pen, the probe's own tip) — **not** `gripper_tcp`, the vendor
MoveIt grasp centre 44.3 mm proximal to the fingertips. `gripper_tcp` is documented here only for
regression/reference (§3). **Solver implementation:** `src/crackvision/calibration/tcp.py`
(`pivot_calibrate`, `check_boresight`, `load_tool_tip_prior`). This document is the operational
checklist GEOM-07 (operator execution) follows; it does not re-derive the algorithm, only specifies
fixtures, poses, thresholds and what is operator-measured versus computed.

Per ADR-012, everything below computes/verifies `T_tool0_tool_tip`'s **translation only** (`p_tcp`,
the vector from `tool0`'s origin to the physical tool tip, expressed in `tool0`'s frame) — pivoting a
point about itself carries no information about orientation, so no procedure here can or does produce
one. `tool_tip` remains a point, not an independently oriented frame.

## 0. Fixture

- **`tcp_tip_fixture_ready`** (GEOM-07's hardware precondition): a rigid, fixed pivot reference —
  a divot, cone, or V-block bonded/clamped to a surface that does not move relative to `base_link`
  for the entire capture session. Calibrate the tool that will actually be used: the closed gripper
  jaws (nominal prior at `gripper_link`'s origin) or a probe/pen held in the gripper (its own tip, a
  known distance forward of `gripper_link`'s origin) — either way, the physical contact point must be
  small and repeatable.
- The fixture's own position is **never measured by the operator** — it is one of the two unknowns
  `pivot_calibrate` solves for (`PivotCalibrationResult.pivot_point_m`). The only fixture
  requirement is that it stays physically fixed; any detected or suspected fixture movement mid-run
  invalidates every pose captured after the movement and that portion of the run must be re-captured
  from scratch, not patched.
- Confirm rigidity before starting: apply firm finger pressure to the fixture from several
  directions; any visible or felt movement fails the fixture check.

## 1. Poses

| Parameter | Requirement | Why |
|---|---|---|
| Minimum solve poses | `MIN_POSES = 4` (hard floor enforced by `pivot_calibrate`, raises `ValueError` below it) | Below 4 poses (12 equations) the 6-unknown system has no residual headroom — a solve "succeeds" numerically with zero information about whether it is right. |
| Recommended solve poses | **≥ 12** | Matches this repo's existing pose-count-plus-diversity pattern for hand-eye calibration (`docs/adr/013-calibration-method.md`); more poses reduce sensitivity to any one noisy touch. |
| Orientation diversity | **≥ 3 tilt bands** (e.g. ≈15°, ≈30°, ≈45° off the nominal boresight-down approach) **and ≥ 4 azimuth/roll headings** (e.g. 0°, 90°, 180°, 270°) about the tool axis, while the tip stays seated in the fixture at every pose | `tests/test_tcp.py::test_pivot_calibrate_degenerate_single_orientation_does_not_recover_tcp` demonstrates concretely: a pose set sharing one orientation solves with ~zero residual but recovers the **wrong** tool tip — pose *count* alone cannot catch this, only orientation diversity can. |
| Held-out poses | **≥ 3**, captured the same way, excluded from the solve | Independent check in §3 layer 2 below. |

At every pose: seat the tip in the fixture, hold the arm still, and record the robot's own
`T_base_link_tool0` (FK/TF readout, `xyzw` quaternion + metres translation, ADR-012) at the moment
of contact. `PoseSample(quat_xyzw=..., translation_m=...)` is exactly this recorded pose — feed the
recorded set directly to `crackvision.calibration.tcp.pivot_calibrate`.

## 2. What is operator-measured vs computed

| Quantity | Source |
|---|---|
| `T_base_link_tool0` per pose | **Operator-measured** (read off the robot's FK/TF at the moment of fixture contact) — this is the only raw input. |
| Fixture position (`pivot_point_m`) | **Computed** by `pivot_calibrate` — never separately surveyed. |
| tool-tip offset (`tcp_offset_m`, i.e. `T_tool0_tool_tip` translation) | **Computed** by `pivot_calibrate`. |
| Per-pose residuals, RMS/max residual | **Computed** by `pivot_calibrate` (`PivotCalibrationResult.residuals_m` / `.rms_residual_m` / `.max_residual_m`). |
| Held-out verification agreement (§3 layer 2) | **Computed**: predict the pivot point from each held-out pose using the solved `tcp_offset_m`, compare against the solved `pivot_point_m`. |
| Tool-tip prior (`tool.xyz_m`, `config/robot/end_effector.yaml`) | **Read** by `load_tool_tip_prior` (its `value_status`/`provenance` travel with it). For the closed gripper this is the nominal `(0, 0, 0)` — the canonical URDF finger meshes' distal reach at `gripper_link`'s origin — not `gripper_tcp`'s `-0.0443` m grasp centre. |
| Comparison against the tool-tip prior | **Computed** by `check_boresight` — `position_delta_m`/`position_delta_norm_m` (the primary, always-defined number) and, when both the measured and prior points are farther than `MIN_VECTOR_LENGTH_FOR_ANGLE_M` (5 mm) from `tool0`'s origin, `angle_from_prior_deg`. Near `(0, 0, 0)` a direction is not meaningful, so the angle is withheld rather than reported as `NaN` or a spurious large deviation. This is a sanity comparison against the CAD/mesh prior, not an acceptance gate (the prior is evidence, not calibrated fact, until GEOM-07 replaces it). |
| Fixture rigidity check (§0) | **Operator-judged** (visual/tactile), not computed. |

## 3. Acceptance thresholds

Three layers, each with its own gate — a run failing any layer is **re-captured**, not silently
accepted with a caveat:

1. **Per-pose contact residual** (from the solve, `PivotCalibrationResult.residuals_m`): any single
   pose's residual norm **> 0.5 mm** is dropped and re-captured (bad seating in the fixture), not
   down-weighted in the average.
2. **Held-out verification** (the final reported number): using the solved `tcp_offset_m` and each
   held-out pose's own `T_base_link_tool0`, recompute the predicted pivot point
   (`R_i @ tcp_offset_m + t_i`) and compare it against the solved `pivot_point_m` from the accepted
   poses — **agreement < 0.5 mm** per held-out pose.
3. **Overall solve quality**: `rms_residual_m < 0.3 mm` and `max_residual_m < 0.5 mm` across the
   accepted (non-held-out) poses.

`tests/test_tcp.py::test_pivot_calibrate_recovers_tcp_within_0_1mm_under_noise` demonstrates the
solver itself recovers a known tool tip within **0.1 mm** under plausible FK noise (0.05 mm
translation σ, 0.02° rotation σ, 40 poses) — the looser 0.3/0.5 mm physical thresholds above leave
headroom for real fixture-contact repeatability, which is coarser than that synthetic noise floor.

**Tool-tip prior comparison** (`check_boresight` against `load_tool_tip_prior()`, i.e.
`end_effector.yaml`'s `tool.xyz_m`): report `position_delta_norm_m` (and `angle_from_prior_deg` when
defined) alongside the calibration, but do not fail the run on them alone — flag (investigate the
tool/mount/fixture, do not silently discard) any run where `position_delta_norm_m > 5 mm` or, when
the angle is defined, `angle_from_prior_deg > 5°`, since either indicates the measured tip location
and the prior disagree by more than plausible manufacturing/assembly tolerance.
`tests/test_tcp.py::test_check_boresight_old_grasp_centre_prior_would_have_flagged_44mm` is the
regression check for this: comparing a correct closed-gripper calibration (`~(0,0,0)`) against
`GRASP_CENTRE_OFFSET_M` (`gripper_tcp`'s `-0.0443, 0, 0`) — the prior GEOM-06 wrongly defaulted to —
reports a ~44 mm delta, which is exactly the false "disagreement" this card removes by making
`tool_tip` the default prior instead.

## 4. Physical boresight direction verification (independent of the pivot solve)

The pivot solve (§§1–3) recovers the `tool_tip` **point**; it does not, on its own, independently
confirm that a straight probe/laser mounted along the tool's approach axis actually points *through*
that same point in the commanded `+X` direction (a systematic angular misalignment of the physical
tool relative to `gripper_link` would not necessarily show up as a large pivot residual if the
misalignment happens to be small relative to the pivot's own noise floor). Run this check after §3
passes:

1. Mount a fine pointer (or use the same tip used for the pivot calibration) at the computed
   `tcp_offset_m` from `tool0`.
2. Command the arm to aim `tool_tip`'s local `+X` boresight (`pointing_axis`, ADR-014 §8.1) at
   a fixed target mark (e.g. the same fixture divot, or a distinct marked point) from a **near**
   standoff (≈10 cm along the boresight) and record the tip's lateral offset from the target.
3. Repeat from a **far** standoff (≈35 cm along the boresight) without changing the boresight
   orientation, only the standoff distance along it.
4. **Acceptance:** the lateral offset must **not grow between near and far** by more than **1 mm**
   (a translation-only `tool_tip` error stays constant with standoff; a boresight *direction* error
   grows linearly with distance — this is the parallax signature that isolates a direction error from
   a pure offset error). A growing offset means the physical tool axis and the assumed `+X`
   direction disagree and the `tool_tip` offset above, while internally consistent, is not aligned
   with the commanded approach axis — investigate the tool mount before accepting the calibration.

## 5. Evidence to record (GEOM-07)

- The full pose set (solve + held-out) with each `T_base_link_tool0` and the fixture-contact
  residual reported by `pivot_calibrate`.
- The solved `tcp_offset_m`, `pivot_point_m`, `rms_residual_m`, `max_residual_m`.
- The §3 layer-2 held-out agreement numbers, per held-out pose.
- The `check_boresight` comparison (`position_delta_m`, `position_delta_norm_m`, and
  `angle_from_prior_deg` when defined) against `load_tool_tip_prior()`.
- The §4 near/far lateral-offset numbers and the resulting pass/fail against the 1 mm growth
  threshold.
- GEOM-07 writes the accepted result into `config/robot/end_effector.yaml`'s `tool` block
  (`xyz_m`, `value_status: measured`, `provenance` and `source` naming this evidence) — it never
  edits the xacro.

Until GEOM-07 runs this on real hardware, `tcp_offset_m`/`pivot_point_m` are **synthetic-only**
(`tests/test_tcp.py`) and `end_effector.yaml`'s `tool` block stays `value_status: nominal`, carrying
its own "prior evidence, not calibrated fact" caveat.

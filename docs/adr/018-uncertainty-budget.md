# ADR-018: Pixel-to-tool-pose uncertainty budget — decomposition and Monte-Carlo cross-check

**Status:** accepted · **Date:** 2026-10-10 · **Supersedes:** — · **Superseded by:** —

## Context

§10.4 / `lift3d.py` already propagate a first-order covariance from pixel + depth noise, through the
eye-in-hand chain, to a per-point `cov_base_m2` (`Σ_base`) and a per-waypoint `sigma_along_normal_m`.
That propagation already folds in:

- pixel/skeleton localization noise (`σ_px`, default 1.0 px);
- D405 stereo depth noise (`σ_z = geometry.depth_uncertainty_m(z, fx)`);
- camera intrinsics (`fx, fy` inside the pinhole-linearisation Jacobian `J`);
- hand-eye calibration `sigma_calib` (GEOM-04.3: `calibration.wrist_camera.position_sigma_m` /
  `rotation_sigma_rad`, surfaced through `kinematics.EndEffector.camera_sigma_pos_m` /
  `camera_sigma_rot_rad`, added as the `(σ_cam_pos² + (‖p_opt‖·σ_cam_rot)²)·I₃` term);
- TCP/pivot calibration sigma (GEOM-06: `calibration.tool.position_sigma_m`, added as `σ_tool_pos²`
  inside `σ_along_normal`).

What §10.4 does **not** do, and GEOM-09 adds:

- **FK joint uncertainty.** §10.4's `unmodelled` list names `joint_encoder_and_fk` explicitly — no
  joint-encoder/FK-Jacobian term exists anywhere in the current pipeline.
- **A named contribution breakdown.** `cov_base_m2` is already the *sum* of the pixel/depth term and
  the calibration term; nothing downstream records how much of the final sigma each named source
  contributed. An operator reading `within_budget: false` today cannot tell whether that's a depth
  problem, a calibration problem, or an FK problem.
- **An independent cross-check** that the linearization itself is trustworthy at the actual operating
  point (large depth error, near-singular wrist pose, etc., where a first-order Jacobian can
  mislead).

This card does not reopen any accepted GEOM-04/06/08 sigma or contract. It adds an **aggregation
layer on top of** the already-accepted `paths3d` output: a decomposition of the existing total, plus
a joint-uncertainty term that was already declared out of scope there, plus a cross-check report.

## Decision

### 1. Sources in the budget

| Term | Already in §10.4's `cov_base_m2`? | This card's job |
|---|---|---|
| Pixel/skeleton localization noise (`σ_px`) | yes (part of `Σ_opt`) | attribute its share of `Σ_base` |
| D405 stereo depth noise (`σ_z`) | yes (part of `Σ_opt`) | attribute its share of `Σ_base` |
| Camera intrinsics (`fx, fy` in `J`) | yes (folded into `Σ_opt`'s Jacobian, not a separable additive term) | documented as non-separable — see §2 |
| Hand-eye calibration `sigma_calib` (GEOM-04.3, rot+pos) | yes (the `(σ_cam_pos² + (‖p_opt‖·σ_cam_rot)²)·I₃` term) | attribute its share |
| TCP/pivot calibration sigma (GEOM-06) | yes, but only at the waypoint level (`σ_tool_pos²` inside `σ_along_normal`), not in `cov_base_m2` itself | attribute its share at the waypoint level |
| FK joint uncertainty | **no** — listed under §10.4 `unmodelled: joint_encoder_and_fk` | **add** a new additive term (§3) |

Distortion Jacobian, depth bias, capture-time skew / motion, and thermal drift remain unmodelled by
both §10.4 and this card, for the same reasons §10.4 already gives; GEOM-09 does not attempt them.

### 2. Analytic method: linearized per-term decomposition

Reuse §10.4's propagation unchanged — do not re-derive `Σ_opt`, `R`, or the calibration terms. Instead,
re-run the existing linearized chain once per named term, holding every other term's input covariance
at zero, and record each resulting partial covariance:

- `Σ_base^(pixel)`: `R · (J · diag(σ_px², σ_px², 0) · Jᵀ) · Rᵀ`
- `Σ_base^(depth)`: `R · (J · diag(0, 0, σ_z²) · Jᵀ) · Rᵀ`
- `Σ_base^(handeye)`: `(σ_cam_pos² + (‖p_opt‖·σ_cam_rot)²) · I₃` (already additive and separable in
  §10.4 — no re-run needed, just relabelled)
- `Σ_base^(fk)`: §3 below

Because `J` is linear in `(σ_px², σ_px², σ_z²)` and the hand-eye term is already additive, these four
partial covariances sum exactly to the existing `Σ_base = Σ_base^(pixel) + Σ_base^(depth) +
Σ_base^(handeye) + Σ_base^(fk)` — the decomposition is exact under linearization, not an
approximation on top of one. Camera intrinsics are **not** a fifth additive term: `fx`/`fy` appear
inside `J` itself (multiplying every pixel/depth contribution), so their effect is reported as a
documented caveat (intrinsics uncertainty is not separately budgeted; it would require perturbing
`J` itself, which the Monte-Carlo cross-check does implicitly — see §4's bullet on intrinsics) rather
than invented as a spurious orthogonal term.

At the waypoint level, `σ_along_normal² = nᵀ·Σ_base·n + σ_tool_pos²` is decomposed the same way:
`nᵀ·Σ_base^(pixel)·n`, `nᵀ·Σ_base^(depth)·n`, `nᵀ·Σ_base^(handeye)·n`, `nᵀ·Σ_base^(fk)·n`, plus the
already-separate `σ_tool_pos²` (TCP/pivot) term — five non-negative numbers summing exactly to
`σ_along_normal²`.

Linearization is adequate here because every step in the chain (pinhole deprojection at a known
depth, rigid rotation/translation composition, a fixed-point normal projection) is smooth and the
operating uncertainties are small relative to the length scales involved (millimetre-scale sigmas
against a ~0.1–0.5 m working distance); it is expected to **break down** where `z` is small and
`σ_z/z` is not small (near the 0.07 m valid-depth floor), where a waypoint sits near a kinematic
singularity (FK Jacobian ill-conditioned), or where an annulus plane fit is poorly conditioned
(`normal_rms_m` large, pushing `σ_normal_rad` into the regime where re-orienting a lerped normal is
no longer a small perturbation). The Monte-Carlo cross-check (§4) exists specifically to catch these
cases empirically, rather than deriving a second analytic (and equally linearized) bound for each —
a second analytic derivation would share the same blind spot a nonlinear resample is meant to expose.

### 3. The new FK term

Model joint-encoder/FK uncertainty as an independent per-joint angular sigma `σ_joint_rad` (one
value per joint, from `config/robot/b601_dm_limits.yaml`'s repeatability figure if present, else a
documented nominal **0.001 rad** — encoder-class repeatability, an order of magnitude below the
0.010 rad / 5° hand-eye nominal prior, consistent with FK error being a secondary, not dominant,
contribution). Propagate it through the numeric FK Jacobian `∂p_base/∂q` (finite-difference central
difference, step `1e-6` rad, evaluated at the capture joint configuration — the same `q` already used
to build the §9.2 chain) to get `Σ_base^(fk) = Σ_j (∂p_base/∂q_j · σ_joint_rad)²`, i.e. the outer
product sum over the 6 joints' Jacobian columns scaled by `σ_joint_rad²`. This is the same
linearization style as every other term in §10.4 — no new propagation machinery, just one more
Jacobian evaluated at the same operating point.

### 4. Monte-Carlo cross-check

For each point (or a configurable subsample — see `uncertainty_budget.yaml` below), resample the full
pixel→base_link→tool_tip chain `N` times (default **2000** samples) with independent per-term
perturbations:

- pixel: `(u, v) += N(0, σ_px²)` independently per axis, before annulus resampling and plane fit;
- depth: `z_annulus_samples += N(0, σ_z²)` per sample before averaging (so the plane-fit normal is
  also perturbed, not just the surface depth — this is the one place the MC resample captures a
  nonlinear interaction the analytic method cannot, since `fit_plane`/`depth_uncertainty_m` are not
  re-linearized per sample);
- hand-eye: perturb `T_camera_link_camera_color_optical_frame`'s translation by
  `N(0, σ_cam_pos²)` per axis and its rotation by a random axis-angle rotation of magnitude
  `N(0, σ_cam_rot²)`;
- TCP: perturb `T_gripper_link_tool_tip`'s translation by `N(0, σ_tool_pos²)` per axis;
- FK: perturb each joint `q_j += N(0, σ_joint_rad²)` independently before evaluating `FK(q)`.

Each sample recomputes the full chain (pixel → annulus depth/plane fit → optical point/normal →
`FK(q)` → base point/normal → waypoint projection) exactly as the production code does — intrinsics
(`fx, fy, ppx, ppy`) are held fixed at their nominal values (not perturbed; this is why the §2
intrinsics caveat says their effect is only indirectly exercised, through the nonlinear
deprojection/annulus geometry that uses them, not through an explicit intrinsics-uncertainty
distribution). The sample covariance of the resulting `p_base` points (and, per waypoint, the sample
variance of the resulting signed along-normal displacement) is the empirical `Σ_base^(MC)` /
`σ_along_normal^(MC)`.

**Agreement statistic.** For each point: `ratio_trace = trace(Σ_base^(MC)) / trace(Σ_base^(analytic))`
and `ratio_eig_max = λ_max(Σ_base^(MC)) / λ_max(Σ_base^(analytic))`. For each waypoint:
`ratio_along_normal = σ_along_normal^(MC) / σ_along_normal^(analytic)`. Agreement **passes** iff every
ratio in `[0.5, 2.0]` (one octave either side — loose enough to tolerate MC sampling noise at the
default sample count, tight enough to catch a linearization that is off by more than 2×). A ratio
outside that band is flagged `mc_analytic_disagreement` for that point/waypoint; `N=2000` keeps the MC
standard error of a variance estimate under ~3% (`√(2/N)`), well inside the 2× band, so the flag is
attributable to the chain, not to MC sampling noise.

### 5. Why Monte-Carlo as the cross-check, not a second analytic derivation

A second analytic derivation (e.g. a second-order Taylor expansion) would still be a linearization —
of a higher order, but sharing every structural blind spot of the first (same smoothness assumptions,
same inability to exercise the actual nonlinear annulus-sampling/plane-fit/FK code paths). A
resample exercises the *actual* production code (the same `lift3d.py`/`kinematics.py` functions used
to build `paths3d`, not a re-derived formula), so disagreement is evidence about this pipeline's real
behaviour at this operating point, not about whether a human got a second derivation right. The cost
(2000 reruns of already-fast deterministic geometry per point/waypoint) is cheap relative to that
benefit, and the subsample knob in `uncertainty_budget.yaml` (§14.6) keeps it affordable at full-path
scale.

## Consequences

Makes easy: an operator or later card (GEOM-09.2–.4) can read one report and see, for any
`within_budget: false` waypoint, which named source (pixel, depth, hand-eye, TCP, FK) dominates, and
whether the linearized sigma that drove that flag is itself trustworthy at that point.

Makes hard / foreclosed: this card does not budget camera-intrinsics uncertainty as a separate
additive term (§2) — a future card wanting that must perturb `J` itself inside the MC resample and
define a new analytic term, which is out of scope here. It also does not change any accepted
`paths3d`/`capture_3d`/`end_effector` field — `cov_base_m2` and `sigma_along_normal_m` keep their
existing meaning and values; the new report is a read-only derivative.

## Revisit when

A `paths3d` consumer needs intrinsics uncertainty budgeted explicitly (e.g. a new camera with a
poorly-characterised intrinsic calibration), or when GEOM-09.2's implementation finds the `[0.5, 2.0]`
agreement band too loose/tight in practice against real captures — revisit the tolerance with real
disagreement data before loosening or tightening it.

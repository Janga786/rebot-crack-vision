# GEOM-09.3.1 — Implement crackvision.uncertainty_mc Monte-Carlo cross-check (pixel/depth/handeye/fk/tcp)

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/uncertainty_mc.py",
      "tests/test_uncertainty_mc.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#14. Pixel-to-tool-pose uncertainty budget",
    "src/crackvision/uncertainty.py",
    "src/crackvision/lift3d.py",
    "src/crackvision/tool_waypoints.py",
    "src/crackvision/kinematics.py",
    "src/crackvision/geometry.py",
    "tests/test_uncertainty.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_uncertainty_mc.py -q",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The sampler perturbs exactly §14.2's five named terms (pixel, depth, handeye, tcp, fk) and, for pixel/depth/handeye/fk, rebuilds the perturbed point by calling the real crackvision functions (geometry.deproject_pixels / lift3d / kinematics.KinematicChain.fk) on the perturbed inputs per sample — never a re-derived closed-form Jacobian approximation of those four terms.",
      "The tcp term, which §14.2 documents has no separable point-level recompute (it is the robot's physical tool-offset residual, not a function of pixel/depth/extrinsic/joint inputs), is modelled as a directly-added isotropic N(0, sigma_tool_pos_m^2 I) 3D perturbation applied to the final tool_tip position after the real waypoint geometry is computed, and the module's docstring/comments say why this one term is additive-only rather than routed through a recompute.",
      "A deterministic seed (e.g. numpy Generator(PCG64(seed))) makes every reported covariance and agreement statistic exactly reproducible across runs.",
      "The sample count used by the module's public API is caller-supplied (no hidden default baked into the test), and tests/test_uncertainty_mc.py documents in a comment/docstring the N it uses and why that N keeps the suite's wall-clock bounded (e.g. a few hundred to ~2000 samples, each only a handful of matrix ops).",
      "On a point/waypoint case with small sigmas (sub-millimetre position noise, sub-degree rotation noise) where the linearization is near-exact, the empirical MC covariance/along-normal sigma agrees with crackvision.uncertainty's analytic decomposition within the §14/ADR-018 default agreement band [0.5, 2.0] on both ratio_trace and ratio_eig_max (points) or ratio_along_normal (waypoints), and the test asserts flagged is False.",
      "On a second case engineered to break linearity (e.g. a large hand-eye rotation sigma, tens of degrees, applied to a point far enough from the camera that the resulting rotation-induced displacement is strongly nonlinear) the MC/analytic ratio falls outside the agreement band and the test asserts the module's own flagged computation is True — proving the check has discriminating power, not just a tautological pass.",
      "Every public function is pure (no file I/O, no CLI, no argparse) and only imports from crackvision.{geometry,lift3d,tool_waypoints,kinematics,uncertainty} plus numpy/scipy — matching the sibling crackvision.uncertainty module's constraints."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 3,
    "context": 3,
    "consequence": 2,
    "task_class": "numerical_library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.3.1",
  "title": "Implement crackvision.uncertainty_mc Monte-Carlo cross-check (pixel/depth/handeye/fk/tcp)",
  "parent": "GEOM-09.3",
  "outcome": "crackvision/uncertainty_mc.py draws seeded samples that perturb pixel, depth, hand-eye and FK inputs and rerun the real lift3d/kinematics code paths (plus a directly-added isotropic TCP perturbation at the waypoint level), accumulates empirical covariance, and reports a §14 agreement statistic against crackvision.uncertainty's analytic breakdown; tests/test_uncertainty_mc.py proves the check agrees in a near-linear regime and flags a deliberately non-linear one."
}
```

Implement `crackvision/uncertainty_mc.py`, the Monte-Carlo cross-check companion to `crackvision/uncertainty.py` (GEOM-09.2), per `docs/INTERFACES.md` §14 and ADR-018 §4.

## Scope and non-goals
Pure library only — no CLI, no file I/O, no argparse (GEOM-09.4 wires this into a CLI/report later). Do not modify `uncertainty.py`, `lift3d.py`, `tool_waypoints.py` or `kinematics.py`.

## What to build

Two public entry points mirroring `uncertainty.decompose_point` / `uncertainty.decompose_waypoint`'s shape, but Monte-Carlo instead of analytic:

1. `mc_point_covariance(...)` — for one dense lifted point, draws `n_samples` perturbed realizations of pixel row/col, depth, and hand-eye extrinsic (rotation+translation) and FK joint angles, recomputes the perturbed `p_base` by calling the **real** code each time, and returns the empirical covariance of the resulting `p_base` samples plus the §14.4 agreement numbers (`ratio_trace`, `ratio_eig_max`, `flagged`) against a caller-supplied analytic `Σ_base^(total)` (e.g. a `uncertainty.PointBreakdown.total`).
2. `mc_waypoint_sigma(...)` — same idea for one `tool_waypoints.Waypoint`: perturbs the same four terms for its source point/normal **and** adds the isotropic TCP perturbation to the resulting tool_tip position, returns the empirical standard deviation of the signed along-normal displacement plus `ratio_along_normal`/`flagged` against a caller-supplied analytic `sigma_along_normal_m`.

### How each term must be perturbed (so this is a real rerun, not a re-derived formula)

- **pixel + depth**: draw `(Δrow, Δcol) ~ N(0, sigma_px^2 I_2)` (pixel units, matching `LiftParams.pixel_sigma_px`) and `Δz ~ N(0, sigma_z^2)` (metres, `sigma_z = geometry.depth_uncertainty_m(z, fx)` at the point's own depth — reuse that real function, don't hardcode a constant). Call the real `geometry.deproject_pixels(intrinsics, row+Δrow, col+Δcol, depth_m+Δz)` to get the perturbed `p_optical`. This is the same continuous deprojection `lift3d.lift_polyline` itself calls for its centre pixel (lift3d.py:246) — reuse `geometry.deproject_pixels` directly rather than re-threading a full depth image through `lift_polyline` per sample (unnecessary and slow).
- **handeye**: draw a small rotation vector `δθ ~ N(0, sigma_cam_rot_rad^2 I_3)` and translation `δt ~ N(0, sigma_cam_pos_m^2 I_3)`; build the perturbed extrinsic as `T' = T_nominal @ [[Rotation.from_rotvec(δθ).as_matrix(), δt],[0,1]]` (right-perturbation in the camera's own frame) composed with the real `T_base_link_optical` (or, for the full chain, the real `chain.fk(q) @ T_gripper_link_camera_link @ NOMINAL_T_...` composition from `kinematics`) — use `scipy.spatial.transform.Rotation`, already a project dependency (see `tool_waypoints.py`), not a hand-rolled small-angle matrix.
- **fk**: draw `Δq ~ N(0, sigma_joint_rad^2 I_n)` and call the real `kinematics.KinematicChain.fk(q + Δq)` — never a finite-difference Jacobian (that's `uncertainty.fk_covariance`'s analytic approximation, which this module cross-checks, not reuses for its own samples).
- **tcp**: no recompute exists for this term (§14.2: waypoint-level-only, not a function of pixel/depth/extrinsic/joints). Add `δp_tool ~ N(0, sigma_tool_pos_m^2 I_3)` directly to the final sampled tool_tip position in `mc_waypoint_sigma` only. Document this asymmetry in the module docstring.

Compose these per sample into one perturbed `p_base` (point case) or tool_tip position (waypoint case): i.e. a single sample draws all four (or five) perturbations at once and threads them through one real recomputation of the chain pixel -> optical -> base_link (-> tool_tip for waypoints), then accumulates. Do not perturb/recompute each term in isolation and sum variances yourself — that would silently reintroduce the analytic linearization this module exists to check against.

### Agreement statistic (§14.3/§14.4/§14.5)

`agreement_band` defaults to `[0.5, 2.0]` (ADR-018 §4; accept it as a caller-supplied `(lo, hi)` tuple, don't hardcode only — but default to this pair). `flagged = not (lo <= ratio <= hi)`. Points: compute both `ratio_trace = trace(Σ_mc) / trace(Σ_analytic)` and `ratio_eig_max = λ_max(Σ_mc) / λ_max(Σ_analytic)`; `flagged` is true if either falls outside the band. Waypoints: `ratio_along_normal = sigma_mc / sigma_analytic_along_normal`; `flagged` if outside the band.

### Reproducibility and cost

Take `seed: int` and `n_samples: int` as explicit required or defaulted arguments (caller-controlled, not a hidden module constant) and use `numpy.random.Generator(numpy.random.PCG64(seed))` for every draw so a fixed seed reproduces the exact covariance bit-for-bit. Each sample is O(1) matrix ops (one `deproject_pixels` call, one small rotation composition, one `chain.fk` call) — `tests/test_uncertainty_mc.py` should pick an `n_samples` (document it in a comment, e.g. a few hundred to ~2000) that keeps `pytest tests/test_uncertainty_mc.py -q` well under a minute.

## Tests (`tests/test_uncertainty_mc.py`)

Follow `tests/test_uncertainty.py`'s fixture style (`make_intrinsics`, `_flat_depth_image`, real `kinematics.load_chain()`/`load_end_effector()`). At minimum:

1. **Agreement case**: a point (and a waypoint derived from a short real `lift_polyline`/`segment_waypoints` segment) with small, realistic sigmas (`pixel_sigma_px=1.0`, nominal D405 depth noise, `cam_rot_rad` on the order of a few degrees, `sigma_joint_rad=0.001`, `tool_sigma_pos_m=0.005` — i.e. the project's existing nominal priors from `kinematics.py`). Build the analytic breakdown via the real `uncertainty.decompose_point`/`decompose_waypoint`/`fk_covariance`, run this module's MC with a fixed seed, and assert `flagged is False` and that `ratio_trace`/`ratio_eig_max` (or `ratio_along_normal`) are within the band.
2. **Disagreement case**: the same setup but with `cam_rot_rad` driven large (tens of degrees) on a point far enough from the camera that the rotation-induced displacement is strongly nonlinear in the small-angle linearization `uncertainty.decompose_point` uses for its `handeye` term. Assert `flagged is True` for at least one of the ratios — this is the proof the statistic is not vacuous.
3. A determinism test: same seed + inputs twice -> bit-identical (or `np.array_equal`) covariance/ratios; different seeds -> not identical (sanity that randomness is actually being used).
4. A short test asserting the tcp term only affects `mc_waypoint_sigma` (e.g. `sigma_tool_pos_m=0` vs `>0` changes the waypoint MC sigma but not a point-level MC covariance, since points have no tcp term).

Keep the module and tests self-contained within the two files in scope; do not touch `uncertainty.py`'s dataclasses beyond importing/reading them.

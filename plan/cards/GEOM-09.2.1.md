# GEOM-09.2.1 — Implement crackvision.uncertainty analytic per-term decomposition (pixel, depth, handeye, tcp, fk)

```json card
{
  "kind": "impl",
  "depends_on": [
    "GEOM-09.1"
  ],
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/uncertainty.py",
      "tests/test_uncertainty.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#14. Pixel-to-tool-pose uncertainty budget (`crackvision.uncertainty_budget/1`)",
    "docs/INTERFACES.md#10. Robot-frame 3D crack paths and tool waypoints",
    "docs/adr/018-uncertainty-budget.md",
    "src/crackvision/lift3d.py",
    "src/crackvision/tool_waypoints.py",
    "src/crackvision/kinematics.py",
    "src/crackvision/capture_record.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_uncertainty.py -q",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "For a given valid, measured (non-interpolated) lifted point, the pixel+depth+handeye terms returned by the module sum (np.allclose) to that point's existing lift3d cov_base (§10.4), i.e. the module decomposes rather than recomputes a different total; tested on real lift3d.lift_polyline output.",
      "Each term (pixel σ_px, depth σ_z, hand-eye σ_cam_pos+σ_cam_rot, TCP σ_tool_pos, fk σ_joint_rad) is individually attributable and unit-tested by zeroing all other inputs and comparing the remaining term to a hand-derived/closed-form expectation (e.g. handeye == (σ_cam_pos² + (‖p_opt‖·σ_cam_rot)²)·I₃).",
      "The module is pure (no file I/O, no argparse/CLI) per the lift3d/tool_waypoints convention; it imports but does not modify crackvision.lift3d, crackvision.tool_waypoints or crackvision.kinematics.",
      "Invalid points (valid=False / NaN cov_base) are refused with ValueError rather than silently giving zero/garbage contributions, and decomposing an interpolated point from its own p_optical is refused. Instead, public helpers that take plain arrays (usable on paths3d JSON data by GEOM-09.4) map each waypoint to its §14.5 source point exactly as tool_waypoints does: the nearer dense point per tool_waypoints._resample_segment (`nearer = j-1 if t <= 0.5 else j`; approach/retract use the first/last trace sample). If that point is interpolated, the helpers resolve it per §10.3 to the bounding measured neighbour whose cov_base has the larger trace, with ties going to the left neighbour, exactly as lift3d's `conservative` (lift3d.py:120).",
      "The fk term follows ADR-018 §3: Σ_base^(fk) = Σ_j (∂p_base/∂q_j · σ_joint_rad)², where ∂p_base/∂q is a central finite-difference Jacobian of the base_link point through crackvision.kinematics FK at the capture's joint positions. It is unit-tested to be exactly zero for σ_joint_rad = 0, to scale with σ_joint_rad², and to be positive semi-definite.",
      "Point level: the full analytic total equals pixel+depth+handeye+fk (INTERFACES §14.4). Waypoint level: nᵀΣ^(term)n for pixel/depth/handeye/fk, taken from the waypoint's §14.5 source point, plus σ_tool_pos² sums exactly to the new sigma_along_normal_m² (§14.5). Without fk that sum equals the source waypoint's sigma_along_normal_m² from tool_waypoints.segment_waypoints for every waypoint (approach, trace, retract), and with fk it is ≥ that value.",
      "A test runs real lift3d.lift_polyline + tool_waypoints.segment_waypoints on a polyline containing a §10.3 gap-filled run whose two bounding measured neighbours have different cov_base traces. It asserts that at least one waypoint's nearer dense point is interpolated (the test is not vacuous), and that for every waypoint the without-fk sum equals tool_waypoints' sigma_along_normal_m² (np.allclose)."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 2,
    "task_class": "numerical_library",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.2.1",
  "title": "Implement crackvision.uncertainty analytic per-term decomposition (pixel, depth, handeye, tcp, fk)",
  "parent": "GEOM-09.2",
  "outcome": "src/crackvision/uncertainty.py is a pure library that decomposes a lift3d point's §10.4 cov_base into the §14.2 pixel/depth/handeye terms, computes the ADR-018 §3 fk term via a central finite-difference FK Jacobian, maps every waypoint to its §14.5/§10.3 source point, and decomposes a tool waypoint's along-normal variance into pixel/depth/handeye/fk/tcp scalars per §14.4/§14.5, verified by tests/test_uncertainty.py.",
  "source": "debugger D0006"
}
```

Implement `src/crackvision/uncertainty.py` (+ `tests/test_uncertainty.py`) for parent GEOM-09.2, per INTERFACES §14 and ADR-018. Read §10.3, §10.4, §14.2, §14.4 and §14.5 and ADR-018 §2–§3 in full first. The debugger added this card (D0005/D0006) because the lead decomposition's output schema cannot carry scope/acceptance fields. The content is the lead's own child body, corrected against the contract.

## Context (existing code to reuse, not re-derive)
- `lift3d.lift_polyline` (src/crackvision/lift3d.py ~272-291) builds each point as follows: `J = [[z/fx,0,x/z],[0,z/fy,y/z],[0,0,1]]`, `cov_pix = diag(σ_px², σ_px², σ_z²)` with `σ_z = geometry.depth_uncertainty_m(z, fx)`, `Σ_opt = J·cov_pix·Jᵀ`, `calib_term = σ_cam_pos² + (‖p_opt‖·σ_cam_rot)²`, and `cov_base = R·Σ_opt·Rᵀ + calib_term·I₃` (R = rotation of T_base_link_optical). Where a piece is a local variable rather than an importable helper, replicate exactly that formula in a small private helper with a comment citing the lift3d line it mirrors. Never invent a different one.
- Gap fill (§10.3, lift3d._apply_gap_policy, lift3d.py:118-124): an interior run of ≤ max_gap_px invalid points between measured neighbours `left`/`right` is interpolated. Every point in the run copies `cov_base` from `conservative = left if trace(cov_base[left]) >= trace(cov_base[right]) else right` (ties → left).
- Waypoints (tool_waypoints._resample_segment, :58-97): a trace waypoint at arc length s in dense interval [j-1, j] with fraction t takes `cov[nearer]`, where `nearer = j-1 if t <= 0.5 else j`. That dense point may be interpolated. `segment_waypoints` (:144-183) gives approach the first trace sample's cov and retract the last. `_waypoint` computes `sigma_along_normal = sqrt(nᵀ·cov·n + tool_sigma_pos_m²)`, with n = the waypoint's own normal.
- FK: `kinematics.KinematicChain.fk(positions)` (kinematics.py:102). `capture_record.T_base_link_optical` (capture_record.py:322) shows how the optical frame is composed from FK + end-effector transforms.

## What to build
1. A dataclass (e.g. `PointBreakdown`) with 3×3 `pixel`, `depth`, `handeye` and `fk` (zeros when not requested), and a `total` equal to their elementwise sum. Split `cov_pix` into `diag(σ_px²,σ_px²,0)` (pixel) and `diag(0,0,σ_z²)` (depth), each through the same R·J·(·)·Jᵀ·Rᵀ. handeye = calib_term·I₃.
2. `decompose_point(...)` taking a measured point's p_optical, intrinsics, R_base_optical, lift params, calib sigmas and an optional fk covariance (or a minimal equivalent signature). Raise ValueError for invalid points (valid=False or NaN cov_base) and for interpolated points. For a measured point, pixel+depth+handeye must reproduce `cov_base[i]` (np.allclose).
3. `fk_covariance(T_base_optical_of_q, q, p_optical, sigma_joint_rad, step=1e-6)`: a central finite-difference Jacobian (3×N) of `p_base(q) = T(q)·p_opt`. Build the callable from `kinematics.KinematicChain.fk` and the same chain `capture_record.T_base_link_optical` uses, and document the step. Σ_fk = J_fk·J_fkᵀ·σ_joint_rad². It must be exactly `np.zeros((3,3))` for σ_joint_rad == 0, scale with σ², and be PSD (eigvals ≥ -1e-12).
4. A waypoint-level breakdown (e.g. `decompose_waypoint(point_breakdown, normal, tool_sigma_pos_m)`) returning scalars {pixel, depth, handeye, fk} = nᵀΣ^(term)n and tcp = σ_tool_pos². The sum equals the new sigma_along_normal_m². With fk omitted, it equals tool_waypoints' sigma_along_normal_m² exactly.
5. Source-point helpers implementing the §14.5 + §10.3 rule exactly (NOT 'nearest valid point'). They take plain numpy arrays (p_base, valid, interpolated, cov_base) so GEOM-09.4 can call them on paths3d JSON data:
   a. `waypoint_source_indices(p_base, seg, spacing_m)` (or equivalent) returns, in `segment_waypoints` order (approach, trace…, retract), the dense index whose cov each waypoint carries. It reproduces `_resample_segment`'s arc-length sampling and its `nearer` rule. Reuse its logic by importing the private helper if that is clean, otherwise mirror it with a comment citing tool_waypoints.py:58-97.
   b. `measured_source_index(valid, interpolated, cov_base, k)` returns k if point k is measured. If k is interpolated, it walks to the nearest measured point on each side (the gap-fill's `left`/`right`) and returns `left if trace(cov_base[left]) >= trace(cov_base[right]) else right`, as lift3d.py:120 does. It raises ValueError for an invalid k or if a bounding neighbour is missing.
   The terms for a waypoint (or interpolated point) are then those of `decompose_point` on the resolved measured point. The library never decomposes an interpolated point from its own p_optical. This guarantees the without-fk sum equals tool_waypoints' sigma_along_normal_m² and that §14.5's new sigma is ≥ the base value.
6. Pure: no file I/O, no CLI, no mutation of lift3d/tool_waypoints/kinematics.

## Tests (tests/test_uncertainty.py)
- Build a tiny synthetic depth patch + mask (mirror the fixtures in tests/test_lift3d.py / tests/test_tool_waypoints.py), run the real `lift3d.lift_polyline` and `tool_waypoints.segment_waypoints`, and check the breakdown against that real output.
- Isolate each term: σ_px=0 removes only the pixel term; the pure-depth term matches R·J·diag(0,0,σ_z²)·Jᵀ·Rᵀ; calib sigmas = 0 remove the handeye term; the handeye closed form holds exactly.
- fk: zero at σ=0, ratio 4 for 2σ vs σ, PSD. Use a real KinematicChain (load_chain on the repo's model) or a simple analytic chain whose Jacobian is known in closed form.
- Gap-fill case: a polyline with an interior invalid run of ≤ max_gap_px depth holes, on a tilted surface (or with otherwise different depths) so the two bounding measured neighbours have different cov_base traces. Assert that lifted.interpolated has True entries, and that at least one waypoint's source index from (5a) is interpolated. Then for EVERY waypoint (approach, trace, retract), the without-fk waypoint sum (via 5a → 5b → decompose_point → decompose_waypoint) equals that waypoint's sigma_along_normal_m² from segment_waypoints (np.allclose).
- Waypoint sums with and without fk, as in the criteria. Invalid and interpolated points passed directly to decompose_point raise ValueError.

Style: dataclasses, type hints, numpy only, no new dependencies. Run via `./env.sh pytest tests/test_uncertainty.py -q`.

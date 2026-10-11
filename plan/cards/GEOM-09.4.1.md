# GEOM-09.4.1 — Implement crackvision.uncertainty_budget CLI + report assembly

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-GEOM-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/uncertainty_budget.py",
      "tests/test_uncertainty_budget_cli.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#14. Pixel-to-tool-pose uncertainty budget",
    "docs/INTERFACES.md#10. Robot-frame 3D crack paths and tool waypoints",
    "docs/adr/018-uncertainty-budget.md",
    "src/crackvision/uncertainty.py",
    "src/crackvision/uncertainty_mc.py",
    "src/crackvision/path3d.py",
    "src/crackvision/capture_record.py",
    "src/crackvision/kinematics.py",
    "src/crackvision/lift3d.py",
    "src/crackvision/tool_waypoints.py",
    "src/crackvision/config.py",
    "src/crackvision/logging_setup.py",
    "tests/test_path3d_cli.py",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_uncertainty_budget_cli.py -q",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The CLI (`build_parser`/`main`, same shape as `path3d.py`) follows docs/INTERFACES.md §0 (exit codes, `--root`/`--config`/`-v`/`-q`/`--dry-run`/`--skip-existing` via `add_common_args`) and takes a case id / §10 paths3d file as its primary input, resolving `data/paths3d/{case}_paths3d.json` the same way `path3d.paths3d_path` does.",
      "For every §10 point with `valid: true`, the report calls `uncertainty.decompose_point` (using that point's own `p_optical_m`, intrinsics, `T_base_optical` rotation, `sigma_px`, and `lift3d.CalibSigmas` rebuilt from the paths3d `calibration.*` block) plus `uncertainty.fk_covariance` (finite-difference Jacobian of `chain.fk(q) @ T_gripper_link_camera_link @ T_camera_optical` at the capture record's `positions_rad`, `sigma_joint_rad` from `b601_dm_limits.yaml`'s repeatability figure if present else the ADR-018 §3 nominal 0.001 rad), and skips (never decomposes) interpolated/invalid points directly — for those, `uncertainty.measured_source_index` locates the measured source point whose breakdown is reused, exactly as §14.5 specifies for waypoints.",
      "For every §10 segment waypoint, the report calls `uncertainty.decompose_waypoint` on its measured source point's breakdown and `tool_sigma_pos_m`, and computes `sigma_along_normal_m = sqrt(contributions_m2 sum)`, `>= sigma_along_normal_base_m` (copied unchanged from the source `sigma_along_normal_m`); `within_budget` is copied unchanged from the source, never recomputed against the new total.",
      "Monte-Carlo cross-check calls `uncertainty_mc.mc_point_covariance`/`mc_waypoint_sigma` with `analytic_cov_base`/`analytic_sigma_along_normal_m` set to this report's own new total (including `fk`), not the source paths3d's unchanged `cov_base_m2`/`sigma_along_normal_m`; `n_samples`/`seed`/`subsample`/`agreement_band` come from `config/calibration/uncertainty_budget.yaml` (`crackvision.uncertainty_budget_config/1`) if present, else the §14.6 defaults (2000/0/\"all\"/[0.5,2.0]); an int `subsample` selects that many dense points uniformly by index, each `points[]` entry it omits carries no stale/zero MC fields at all.",
      "The optional `uncertainty_budget.yaml` is loaded with the same strict convention as `cartesian_core.load_config` (unknown key or out-of-range value -> exit 2; `agreement_band` must satisfy `min < 1.0 < max`, both `> 0`); its absence is not an error.",
      "The written `data/uncertainty/{case}_uncertainty_budget.json` has exactly the `crackvision.uncertainty_budget/1` key set at every level (§14.3-§14.5): `schema`, `case_id`, `created_utc`, `producer`, `frame`, `sources.{paths3d,capture_record,end_effector,limits,config}` (each `{path, sha256}` or `null` per §14.3), `parameters.{sigma_joint_rad,sigma_joint_rad_source,mc.*}`, `points[]`, `waypoints[]`, `summary.{points_flagged,waypoints_flagged,overall_pass}`, `caveats[]` (containing the exact verbatim intrinsics sentence from §14.3).",
      "`points[]`/`waypoints[]` entries carry exactly the §14.4/§14.5 fields, correctly indexed by `component_id`/`polyline_kind`/`branch_index`/`point_index` (and `segment_index`/`phase`/`waypoint_index` for waypoints); `contributions_m2` sums exactly (to floating-point tolerance) to `sigma_total_m`/`sigma_along_normal_m` as specified, and `sigma_total_m >= sigma_base_m` of the source point componentwise.",
      "The file is written even when `summary.overall_pass` is `false` (diagnostic, never a gate — nothing here touches the source paths3d's `execution_eligible`), but `main()` returns a documented non-zero exit code (distinct from the usage/precondition codes used for missing inputs) whenever `summary.overall_pass` is `false`, so a disagreeing report is never reported as a clean run.",
      "tests/test_uncertainty_budget_cli.py builds a full case the same way tests/test_path3d_cli.py does (synthetic paths.json + mask + depth PNG + capture record via `capture_record.build_record`), runs the real `path3d` CLI to produce the input `_paths3d.json` fixture, then runs this CLI end-to-end and asserts: the full top-level/point/waypoint key sets, `contributions_m2` summing to the reported totals, `summary.overall_pass is True` and exit 0 with default parameters (using a small `uncertainty_budget.yaml` `mc.n_samples` for test speed, fixed seed), and a second case where an impossibly tight `mc.agreement_band` (e.g. `[0.99, 1.01]`) in the config forces `summary.overall_pass is False` and a non-zero exit while the report file is still written and still schema-valid."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 2,
    "context": 4,
    "consequence": 3,
    "task_class": "cli_tool",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-09.4.1",
  "title": "Implement crackvision.uncertainty_budget CLI + report assembly",
  "parent": "GEOM-09.4",
  "outcome": "`./env.sh python -m crackvision.uncertainty_budget` reads a §10 paths3d file (+ its capture record, end_effector.yaml, kinematic chain, optional limits/config files), computes the §14 analytic per-term decomposition and Monte-Carlo cross-check for every valid point and waypoint, writes `data/uncertainty/{case}_uncertainty_budget.json` validating against `crackvision.uncertainty_budget/1` exactly, and exits non-zero (while still writing the file) whenever any point/waypoint is MC/analytic-disagreement-flagged."
}
```

Implement `crackvision/uncertainty_budget.py` as a CLI (`build_parser`/`main`), modelled on `path3d.py`'s shape but writing to `data/uncertainty/{case}_uncertainty_budget.json` per docs/INTERFACES.md §14.

Input resolution per case: load the §10 paths3d file, its `sources.capture_record` (via `capture_record.load_capture_record`), the kinematic chain (`kinematics.load_chain`), `kinematics.load_end_effector()`, and `config/robot/b601_dm_limits.yaml` (check for a top-level `repeatability_rad` scalar; use it as `sigma_joint_rad` with `sigma_joint_rad_source: "limits_file"` if present, else the ADR-018 §3 nominal `0.001` with `source: "nominal"`). Record every input's `{path, sha256}` in `sources.*` (`sources.limits`/`sources.config` are `null` when those optional files are absent/unused).

For each valid dense point in every polyline: rebuild `lift3d.CalibSigmas` from the paths3d `calibration.*` block, call `uncertainty.decompose_point` with that point's `p_optical_m`, the color intrinsics' `fx`/`fy`, the rotation of `T_base_optical` (recomputed via `capture_record.T_base_link_optical(record, chain, end_effector)`), `sigma_px` from `parameters.sigma_px`, and `fk_cov` from `uncertainty.fk_covariance` (build `T_base_optical_of_q` as `lambda q: chain.fk(q) @ end_effector.T_gripper_link_camera_link @ kinematics.NOMINAL_T_CAMERA_LINK_CAMERA_COLOR_OPTICAL_FRAME`, evaluated at the capture record's `positions_rad`). Interpolated/invalid points are never decomposed directly: for a waypoint whose nearest dense point is interpolated, use `uncertainty.measured_source_index` to find the measured point whose breakdown to reuse (mirroring §10.3's gap-fill trace rule), and skip emitting a `points[]` entry for points this report cannot decompose on its own (invalid points are always skipped per §14.4; only `valid: true` points get an entry, each keyed to its own index — interpolated points ARE still `valid: true` per §10.2, so emit an entry for them too, using their measured source's breakdown, same as waypoints do).

For each waypoint: locate its nearest dense point via `uncertainty.waypoint_source_indices` (reproducing `tool_waypoints`' resampling), decompose it the same way, call `uncertainty.decompose_waypoint` with `tool_sigma_pos_m = calibration.tool.position_sigma_m`.

Run the Monte-Carlo cross-check via `uncertainty_mc.mc_point_covariance`/`mc_waypoint_sigma`, reusing the record's intrinsics, `sigma_px`, `chain`, `positions_rad`, `T_gripper_link_camera_link`, `T_camera_optical` (nominal), the resolved `sigma_joint_rad`, `camera_sigma_pos_m`/`camera_sigma_rot_rad`, and (for waypoints) `normal_base`, `clearance_m`, `tool_sigma_pos_m`. Pass `analytic_cov_base`/`analytic_sigma_along_normal_m` as this report's own new total (point breakdown's `.total` / waypoint breakdown's `.total ** 0.5`), `n_samples`/`seed`/`agreement_band` from the resolved config, honouring an int `mc.subsample` by only emitting `points[]` entries for the uniformly-selected subsample (waypoints always get an MC entry — they are comparatively few).

Load `config/calibration/uncertainty_budget.yaml` with a strict validator in the same style as `ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_core.py`'s `load_config` (schema check, unknown-key rejection, range checks on `agreement_band`), raising a dedicated error that `main()` maps to the usage exit code; its absence is not an error and simply uses the §14.6 defaults.

Assemble and write the report exactly per §14.3-§14.5 (atomic write via a `.tmp` + `.replace()`, matching `path3d.py`). Compute `summary.points_flagged`/`waypoints_flagged`/`overall_pass` and return a documented non-zero exit code when `overall_pass` is `false`, while still writing the file — pick a distinct, documented exit code from `logging_setup`'s existing constants (e.g. `EXIT_RUNTIME`) and make this explicit in `--help`/docstrings so it is never confused with the precondition/usage codes used for missing inputs. Use `RunSummary`/`setup_logging` the same way `path3d.py` does for `logs/uncertainty_budget_latest.json`.

Write `tests/test_uncertainty_budget_cli.py` following `tests/test_path3d_cli.py`'s fixture-building technique: build a synthetic case, run the real `path3d` CLI to produce its `_paths3d.json`, then run this CLI and check the full schema, the arithmetic identities (contributions sum to totals, `sigma_total_m >= sigma_base_m`), a clean `overall_pass: true` / exit 0 run with a small `mc.n_samples` config for speed, and a forced-disagreement run (via an impossibly tight `agreement_band` in `uncertainty_budget.yaml`) that still writes a schema-valid file but returns a non-zero exit.

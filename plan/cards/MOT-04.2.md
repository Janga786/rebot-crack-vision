# MOT-04.2 — Reachability map + placement contract (INTERFACES §7), map I/O, ROS-side CLI common layer

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-04.1"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-2"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
      "ros2_ws/src/crackvision_motion/test/test_reachability_map.py",
      "ros2_ws/src/crackvision_motion/test/test_cli_common.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#6",
    "src/crackvision/config.py",
    "src/crackvision/logging_setup.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "config/motion/reachability.yaml",
    "config/robot/b601_dm_limits.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_reachability_map.py ros2_ws/src/crackvision_motion/test/test_cli_common.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "contract-section",
        "cmd": "grep -qE '^## [0-9]+\\. Reachability map and specimen placement' docs/INTERFACES.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "append-only",
        "cmd": "bash -c 'git diff --unified=0 HEAD~1 -- docs/INTERFACES.md | grep -E \"^-[^-]\" && exit 1 || exit 0'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "INTERFACES.md only gains a new appended numbered section (the next free number, expected §7). No existing line changes. The new section defines both schemas field by field with units, frame (base_link), quaternion order (x,y,z,w) and status enums.",
      "The map schema has `complete` (bool) and a per-target `status` ∈ {reachable, unreachable, prefiltered, fk_mismatch, error}. Consumers are told to refuse incomplete maps.",
      "The placement schema carries value_status nominal, map_sha256, config_sha256, boresight provenance and a caveats list. It says it is a recommendation that MOT-10 replaces with a measured pose.",
      "validate_map rejects: a wrong schema id, an unknown status, a reachable target without 6 joint values or quaternion, joint values outside config/robot/b601_dm_limits.yaml, and duplicate target_ids. Each rejection is tested.",
      "write_map is atomic (tmp + os.replace). A load(write(x)) round-trip is lossless.",
      "cli_common mirrors src/crackvision/config.py add_common_args and logging_setup semantics (same flags, exit codes, logs/<tool>_<UTC>.{log,json} + <tool>_latest.json, status enum) without importing the conda-side package. --dry-run writes nothing except logs."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "contract+python-lib",
    "local_ok": false
  },
  "id": "MOT-04.2",
  "parent": "MOT-04",
  "title": "Reachability map + placement contract (INTERFACES §7), map I/O, ROS-side CLI common layer",
  "outcome": "A new numbered INTERFACES section fixes the reachability-map JSON and specimen-placement YAML schemas. reachability_map.py builds, writes and validates maps. cli_common.py gives the ROS-side (system python3) CLIs the §0.2 exit codes, §0.3 common flags and §0.4 log artefacts."
}
```

Runs under system python3. As in MOT-04.1, the module may use only stdlib, numpy and yaml, and must not import rclpy.

### 1. INTERFACES.md: APPEND a new section
Title: `## <N>. Reachability map and specimen placement (MOT-04)`, where N is the next free number (currently 7). Do not edit §0–§6.

Subsection **Reachability config**: point to `config/motion/reachability.yaml` and the `crackvision.reachability_config/1` validation in reachability_core.load_config. Do not duplicate every key.

Subsection **Reachability map JSON**: default path `data/motion/reachability_map.json`, git-ignored. Fields:
- `schema: "crackvision.reachability_map/1"`, `created_utc`, `git_commit`.
- `config_path`, `config_sha256`.
- `frame: base_link`, `units: {length: m, angle: rad}`, `quaternion_order: xyzw`.
- `robot: {group, ik_link, ik_solver, reach_bound_m, limits_file, limits_sha256}`.
- `boresight: {axis_local, provenance}`.
- `grid`, echoing the normalised config grid + orientation + ik + surface_collision blocks.
- `complete: bool`, `duration_s`.
- `targets: [ … ]`, one per enumerate_targets() entry, in order. Each has `target_id, x, y, surface_z, standoff_m, position[3], status`.
- A target with status reachable also has `tilt_deg, azimuth_rad, roll_rad, quat_xyzw[4], joints{joint1..joint6: rad}, min_joint_limit_margin_rad, fk_position_error_m, fk_axis_error_deg`.
- Every target has `ik_calls`, `ik_time_s`, and `error` (string) for status error / fk_mismatch.
- `summary: {counts_by_status, reachable_fraction_by_surface_z}`.

Status semantics:
- reachable: IK succeeded with collision checking, and an independent FK re-check agreed within 1 mm / 0.5°.
- unreachable: no orientation sample succeeded.
- prefiltered: beyond reach_bound_m, provably unreachable, IK not called.
- fk_mismatch: IK claimed success but FK disagreed.
- error: service failure.

State that consumers MUST refuse `complete: false` maps.

Subsection **Specimen placement YAML**: default path `config/motion/specimen_placement.yaml`. Fields:
- `schema: "crackvision.specimen_placement/1"`, `value_status: nominal`.
- `feasible: bool`, `frame: base_link`.
- `placement: {center_xy_m[2], surface_z_m, yaw_rad, footprint_m[2], tolerance_m, standoffs_m[], score_min_joint_margin_rad}` or null when infeasible.
- `alternatives: [ same shape as placement, up to top_k ]`.
- `max_feasible_square_m` (when infeasible).
- `map_sha256`, `config_sha256`, `boresight_provenance`.
- `caveats: [str]`, `source: str`.

Say explicitly that this is a nominal recommendation. The operator places the specimen there, and MOT-10 measures the real pose, which then supersedes this file. MOT-04.3 fills in the scoring rule text; leave a clearly marked `Scoring rule` subsection stub with the rule stated as in the MOT-04.3 card (feasibility = all grid nodes in the tolerance-dilated footprint reachable at every standoff; score = min joint-limit margin; tie-break |y| asc, yaw asc, x asc, surface_z asc).

Subsection **CLIs**: list `reachability_sweep` (MOT-04.4), `recommend_placement` (MOT-04.3) and `scripts/ros/run_reachability.sh` (MOT-04.4) with their exit codes: 0 ok, 1 runtime / FK mismatch / require-all failed / no feasible placement, 2 config, 3 precondition (services absent, incomplete map). Note that they are rclpy-side tools run via scripts/ros/env_ros.sh and follow §0.

### 2. `reachability_map.py`
- `SCHEMA = "crackvision.reachability_map/1"`, `STATUSES`.
- `new_map(cfg, config_path, config_sha, robot_info, git_commit) -> dict`.
- `add_target(map, target, status, **fields)`.
- `finalize(map, complete, duration_s)`, which computes the summary.
- `write_map(map, path)`: atomic, UTF-8, `indent=None, separators=(',',':')` to keep it compact. Create parent dirs.
- `load_map(path, limits_path='config/robot/b601_dm_limits.yaml') -> dict`, which calls `validate_map`.
- `class MapError(ValueError)`.
- `limits_from_yaml(path) -> dict[joint, (lower, upper)]` and `joint_limit_margin(joints, limits) -> float` = min over joint1..6 of min(q−lower, upper−q). This is shared by the sweep node.
- `validate_placement(dict)` and `load_placement(path)` with `class PlacementError(ValueError)`, implementing the placement schema above.

### 3. `cli_common.py`
- `add_common_args(parser)` with exactly the §0.3 flags. `--config` means config/project.yaml, as in §0.3. Tool-specific inputs use their own flags, e.g. `--reachability-config`.
- `find_root()`: $CRACKVISION_ROOT, else walk up to a directory containing pyproject.toml and docs/INTERFACES.md.
- `run_cli(tool, main_fn, argv) -> int`. It sets up logging to stderr + `logs/<tool>_<UTC>.log`, calls `main_fn(args, log, summary)`, maps ConfigError/MapError/PlacementError to 2, a `PreconditionError` class defined here to 3, and other exceptions to 1 (logging the traceback). It writes `logs/<tool>_<UTC>.json` and `logs/<tool>_latest.json` with the §0.4 fields. The status enum is ok|partial|failed|precondition.
- Read src/crackvision/config.py and logging_setup.py and mirror their behaviour. Do not import them (different interpreter, ADR-007).

### 4. Tests
- Map round-trip and each validate_map rejection.
- Margin math.
- validate_placement accept/reject.
- run_cli exit-code mapping and that summary JSON files are written. Use a tmp root via --root so tests never write into the real logs/.
- --dry-run behaviour of run_cli.

For the append-only check: commit this card's work as a single commit so that `git diff HEAD~1` shows only additions to INTERFACES.md.

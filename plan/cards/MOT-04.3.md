# MOT-04.3 — Specimen placement scorer + recommend_placement CLI (+ verification-grid emitter)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-04.2"
  ],
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/placement.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/recommend_placement.py",
      "ros2_ws/src/crackvision_motion/test/test_placement.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json",
      "ros2_ws/src/crackvision_motion/setup.py",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#7",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_map.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cli_common.py",
    "config/motion/reachability.yaml",
    "ros2_ws/src/crackvision_motion/setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500,
        "expect_exit": 0
      },
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_placement.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "cli-smoke",
        "cmd": "bash scripts/ros/env_ros.sh bash -c 'source ros2_ws/install/setup.bash && ros2 run crackvision_motion recommend_placement --map ros2_ws/src/crackvision_motion/test/fixtures/synthetic_map.json --reachability-config config/motion/reachability.yaml --out logs/mot04_check_placement.yaml --emit-verify-config logs/mot04_check_verify.yaml && ros2 run crackvision_motion recommend_placement --validate-placement logs/mot04_check_placement.yaml'",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "mock-plan-regression",
        "cmd": "bash scripts/ros/test_mock_plan.sh",
        "timeout_s": 600,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The scoring rule is implemented exactly as stated and documented in the INTERFACES §7 'Scoring rule' subsection. Results are deterministic, with an explicit lexicographic tie-break.",
      "Feasibility requires every grid node inside the tolerance-dilated, yaw-rotated footprint to be reachable at every configured standoff, and the dilated footprint to lie fully inside the grid. Out-of-grid candidates are never feasible.",
      "The tests include a hand-built map with a known unique optimum, a tie resolved by the tie-break, a hole in the footprint making a candidate infeasible, the infeasible case (feasible=false, exit 1, max_feasible_square_m correct), and an incomplete map refused with exit 3.",
      "The --emit-verify-config output is a valid crackvision.reachability_config/1 (it passes load_config). Its grid covers the dilated footprint at half the map step, at the placement's surface_z and all standoffs, so it samples points the map never did.",
      "Output placement files pass validate_placement and carry value_status nominal, map_sha256, config_sha256, boresight provenance and caveats.",
      "setup.py changes are additive: the new console script is added and existing entries (plan_joint_goal and anything MOT-03 added) are preserved. test_mock_plan.sh still passes."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "python-algorithm",
    "local_ok": false
  },
  "id": "MOT-04.3",
  "parent": "MOT-04",
  "title": "Specimen placement scorer + recommend_placement CLI (+ verification-grid emitter)",
  "outcome": "`ros2 run crackvision_motion recommend_placement` turns a complete reachability map into a deterministic, nominal specimen placement (or an honest 'infeasible' with the largest feasible square). It also emits a half-step verification grid config for independent re-checking and validates placement files."
}
```

This card is pure Python on the system python3 and involves no ROS graph. It is exposed as a console script so operators run it the same way as the sweep.

### `placement.py` (the scoring rule; normative)
Inputs are a validated map (reachability_map.load_map) and the config `placement` block.

For each `surface_z` in the map, each `yaw` in `yaw_candidates_rad`, and each candidate centre = every grid (x,y) node:
1. The footprint rectangle is `footprint_m` = (w along the specimen's local x, d along its local y), rotated by yaw about the centre, then dilated by `tolerance_m` on every side. This dilation models operator placement error.
2. The candidate is out_of_grid, and therefore infeasible, if any corner of the dilated rectangle lies outside [x.min, x.max] × [y.min, y.max].
3. Sample nodes are all grid nodes inside the dilated rectangle, with an inclusive test using a 1e-9 epsilon.
4. The candidate is feasible iff every (node, standoff) for all configured standoffs has status == reachable.
5. Score = min over those (node, standoff) of `min_joint_limit_margin_rad`.
6. Ranking: score desc (compared after rounding to 1e-6), then |y_center| asc, then yaw asc, then x asc, then surface_z asc.

Best = rank 1. `alternatives` = the next top_k−1 candidates whose centres differ from all better-ranked ones by more than one grid step.

If nothing is feasible: set feasible=false and placement=null. Compute `max_feasible_square_m` as the largest square side s (searched downward from min(footprint) in steps of the grid step, at yaw 0 and the same tolerance) that has a feasible placement, or 0.

Public functions:
- `score_candidates(map, cfg) -> list`
- `recommend(map, cfg, map_sha, config_sha) -> placement dict`, conforming to the §7 schema.
- `verification_config(placement, cfg) -> dict`: a reachability_config/1 with grid x/y equal to the dilated footprint's axis-aligned bounding box (for yaw ≠ 0 use the rotated bbox) at step = map step / 2. Snap min/max so that (max−min)/step is integral. Set `surface_z_m: [placement.surface_z]`, the same standoffs/orientation/ik/surface_collision blocks, and `prefilter` enabled. The emitted file must pass reachability_core.load_config.

The caveats list always includes these points:
- The boresight is prior evidence until GEOM-07.
- The TCP is gripper_tcp from the MoveIt config, not yet calibrated.
- The camera mount is undecided (GEOM-03); re-run if the inspection frame changes.
- The collision scene is only self-collision plus a surface slab, not the MOT-03 scene.
- The values are nominal until MOT-10.

### `recommend_placement.py` (console script `recommend_placement`, built on cli_common.run_cli)
Flags:
- The §0.3 common flags.
- `--map PATH` (default data/motion/reachability_map.json).
- `--reachability-config PATH` (default config/motion/reachability.yaml; the placement block is read from here and grid consistency with the map is asserted, otherwise exit 2).
- `--out PATH` (default config/motion/specimen_placement.yaml).
- `--emit-verify-config PATH` (optional).
- `--validate-placement PATH`: validate only; exit 0 if valid, 2 if not; no other work.

Exit codes:
- 0: a feasible placement was written.
- 1: no feasible placement (the file is still written with feasible=false).
- 2: config or schema error.
- 3: map missing or `complete: false`.

`--dry-run` logs the would-be result and writes nothing but logs. `--skip-existing` skips if --out exists. Write YAML with a header comment stating that the file is nominal, generated, and superseded by MOT-10.

### Fixture
`test/fixtures/synthetic_map.json` is a small (< 50 KB) complete map generated by a deterministic helper inside test_placement.py. Reachable means inside an annulus with a hole, and margins are an analytic function, so the optimum is known by hand. The test regenerates the map and asserts byte equality with the committed fixture, so the fixture cannot drift from the generator.

### INTERFACES
Replace only the 'Scoring rule' stub that MOT-04.2 appended to the reachability section with the full rule text above. Touch no other section.

### setup.py
Add `recommend_placement = crackvision_motion.recommend_placement:main`. MOT-03 may have edited this file concurrently, so merge and never overwrite.

## Amendment 2026-09-30 (technical-lead recovery, ADR-014) — sanctioned changes, re-review against these
- The fixed caveats list now starts with the INTERFACES §7.3 nominal disclaimer verbatim (it was only in the
  file header, which the §7.3 text does not allow) and no longer claims 'the camera mount is undecided' or that
  the TCP is gripper_tcp; recommend() appends the map's task frame and collision model (placement.caveats_for).
- verification_config() also copies an optional `environment` block; view_verification_config() and the
  `--emit-view-config` flag (MOT-04.6) emit the wrist-camera view check. The scoring rule is unchanged.

# MOT-04.1 — Reachability config + boresight-down target/pose generator (pure Python)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-02",
    "GEOM-01"
  ],
  "requirements": [
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py",
      "ros2_ws/src/crackvision_motion/test/test_reachability_core.py",
      "config/motion/reachability.yaml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#6",
    "docs/adr/012-frames-and-conventions.md",
    "docs/motion/ROBOT_MODEL.md#4",
    "config/robot/b601_dm_limits.yaml",
    "docs/TECHNICAL_APPROACH.md#2.5",
    "docs/motion/ROS_WORKSPACE.md",
    "ros2_ws/src/crackvision_motion/setup.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_reachability_core.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "config-valid",
        "cmd": "bash scripts/ros/env_ros.sh python3 -c \"import sys; sys.path.insert(0,'ros2_ws/src/crackvision_motion'); from crackvision_motion.reachability_core import load_config; load_config('config/motion/reachability.yaml')\"",
        "timeout_s": 120,
        "expect_exit": 0
      },
      {
        "id": "no-ros-import",
        "cmd": "bash scripts/ros/env_ros.sh python3 -c \"import sys; sys.path.insert(0,'ros2_ws/src/crackvision_motion'); import crackvision_motion.reachability_core; assert not any(m.split('.')[0] in ('rclpy','moveit_msgs','geometry_msgs') for m in sys.modules)\"",
        "timeout_s": 120,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "reachability_core imports only stdlib + numpy + yaml. There is no rclpy/ROS message import anywhere in it.",
      "Generated orientations satisfy R·axis_local == d_world to 1e-9 for every roll/tilt sample. Quaternions are ROS (x,y,z,w), unit-norm, det(R)=+1. The tests cover a non-trivial axis_local, not only [1,0,0].",
      "Target position = surface point − standoff·d_world, so the TCP sits standoff metres above the surface along the boresight. The tests check this for tilt 0 and for tilt > 0.",
      "Grid enumeration is deterministic and serpentine (adjacent targets are grid neighbours), with inclusive integer-count ranges. A non-integer (max−min)/step raises ConfigError.",
      "The reach bound is the sum of joint-origin norms along base_link→tip. It is provably conservative (triangle inequality), a prismatic joint in the chain raises, and the test uses a synthetic URDF whose bound is known by hand.",
      "Every numeric value in config/motion/reachability.yaml carries value_status nominal and a source/rationale. The boresight block states provenance prior_evidence and cites INTERFACES §6.5.",
      "Invalid configs (bad step, negative standoff, roll_samples < 1, unknown keys, footprint ≤ 0, tolerance < step/2) raise ConfigError, and each case is tested."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 3,
    "task_class": "python-geometry-lib",
    "local_ok": false
  },
  "id": "MOT-04.1",
  "parent": "MOT-04",
  "title": "Reachability config + boresight-down target/pose generator (pure Python)",
  "outcome": "config/motion/reachability.yaml (all values tagged nominal) plus a ROS-free module that validates it, enumerates the sweep targets in a fixed order, builds boresight-down TCP poses with sampled roll, and computes a provable reach bound from the URDF. All of it is unit-tested under the system python3."
}
```

Context: MOT-04 reuses the presentation's §2.5 placement sweep (docs/TECHNICAL_APPROACH.md §2.5: 18/18 waypoints converged at coupon x=0.22 but only 5/18 at x=0.42), but runs it with MoveIt's own IK (trac_ik, group `arm`, tip `gripper_tcp`) against the canonical B601-DM model. This card writes only the pure-Python geometry and config layer. Later cards add the map/CLI contract (MOT-04.2), placement scoring (MOT-04.3), the ROS sweep node (MOT-04.4) and the real run (MOT-04.5).

Why this lives in the ROS package: the sweep node runs under /usr/bin/python3 (ROS Humble, py3.10), so this module must live in `ros2_ws/src/crackvision_motion/crackvision_motion/reachability_core.py` and depend only on stdlib, numpy and PyYAML. All three are verified present in system python3. Do NOT import rclpy or any *_msgs here. Do NOT import from src/crackvision (that is the conda py3.11 env, see ADR-007). Do not edit setup.py in this card, because no console script is added yet.

### 1. `config/motion/reachability.yaml`
Schema id `crackvision.reachability_config/1`. Keys:
- `frame: base_link`, `group: arm`, `ik_link: gripper_tcp` (SRDF chain tip; docs/motion/ROBOT_MODEL.md §4).
- `boresight: {axis_local: [1,0,0], provenance: prior_evidence, source: "docs/INTERFACES.md §6.5 / TECHNICAL_APPROACH §2.3; GEOM-07 measures"}`, plus `down_world: [0,0,-1]`.
- `grid: {x_m: {min: 0.05, max: 0.50, step: 0.025}, y_m: {min: -0.40, max: 0.40, step: 0.025}, surface_z_m: [0.00, 0.04, 0.08, 0.12, 0.16], standoffs_m: [0.01, 0.04]}`. The 0.04 standoff is the presentation's STANDOFF_APPROACH_M. 0.01 is a near-surface inspection clearance. Both are nominal.
- `orientation: {roll_samples: 8, tilt_deg: [0.0], tilt_azimuth_samples: 4}`.
- `ik: {timeout_s: 0.02, avoid_collisions: true, seed: neighbour, roll_search: best}`. `seed` ∈ {neighbour, home}. `roll_search` ∈ {first, best}.
- `surface_collision: {enabled: true, thickness_m: 0.02, margin_m: 0.05, allowed_links: [base_link, link1]}`.
- `prefilter: {enabled: true}`.
- `placement: {footprint_m: [0.20, 0.20], yaw_candidates_rad: [0.0, 1.5707963267948966], tolerance_m: 0.02, top_k: 5}`.
- `value_status: nominal`, plus a `sources:` map giving a one-line rationale per block. None of these values is measured; MOT-10 measures the workcell.

### 2. `reachability_core.py` public API (names are normative; later cards import them)
- `class ConfigError(ValueError)`.
- `load_config(path) -> dict`: parse, validate, and return a normalised dict with floats and tuples. Reject unknown top-level keys. Validate that each grid axis has step > 0, max ≥ min and (max−min)/step integral within 1e-6. Standoffs must be ≥ 0, surface_z_m non-empty, roll_samples ≥ 1, tilt_deg in [0, 60], footprint > 0, top_k ≥ 1, and `placement.tolerance_m ≥ max(x.step, y.step)/2`. The last rule ensures the dilated footprint samples cover the true footprint at grid resolution. `axis_local` and `down_world` must be non-zero and are normalised.
- `config_sha256(path) -> str`: sha256 of the file bytes.
- `axis_values(axis_cfg) -> list[float]`: inclusive, count = round((max−min)/step)+1, value = min + i·step (no float accumulation).
- `orientations(cfg) -> list[Orientation]`: an Orientation is a NamedTuple with `tilt_deg, azimuth_rad, roll_rad, d_world (3,), quat_xyzw (4,)`. For each tilt: tilt 0 uses a single azimuth 0, otherwise `tilt_azimuth_samples` azimuths 2πk/N. For each of those, rolls 2πk/roll_samples. Order: tilt asc, azimuth asc, roll asc. d_world = down_world tilted by `tilt` toward `azimuth`. R = Rot(d_world, roll) · R0, where R0 is the shortest-arc rotation taking axis_local to d_world. Handle the antiparallel case explicitly. The quaternion is (x,y,z,w) with w ≥ 0 canonicalised.
- `target_position(surface_xyz, standoff_m, d_world) -> np.ndarray` = surface_xyz − standoff_m·d_world.
- `enumerate_targets(cfg) -> list[Target]`: a Target is a NamedTuple with `target_id, iz, ix, iy, x, y, surface_z, standoff_m`. Order: surface_z asc, then x asc, then y serpentine (ascending on even ix, descending on odd ix), then standoff asc. target_id = `z{iz:02d}_x{ix:03d}_y{iy:03d}_s{is:d}`. Orientation sampling happens per target in the node, not here.
- `reach_bound_from_urdf(urdf_xml: str, base_link: str, tip_link: str) -> float`: walk the parent chain from tip to base via <joint> parent/child and return Σ‖origin.xyz‖. Raise ValueError if the chain is broken or contains a prismatic/floating/planar joint. This value is a strict upper bound on ‖p_tip‖ in base_link, whatever the joint angles.
- `prefiltered(position, bound) -> bool`: ‖position‖ > bound + 1e-6.

### 3. Tests (`test_reachability_core.py`)
Insert `Path(__file__).resolve().parents[1]` into sys.path so the file runs without a colcon build. Cover:
- R·axis_local == d_world for axis_local [1,0,0] and a skew axis; orthonormality; det +1; distinct rolls.
- Quaternion↔matrix round trip.
- target_position for tilt 0 and 30°.
- axis_values counts and endpoints; the serpentine neighbour property; deterministic ids.
- reach bound on a hand-written 3-joint URDF string with a known answer; the prismatic chain raises.
- Every ConfigError path listed in the criteria.
- Loading the committed config/motion/reachability.yaml succeeds.

Run with: `bash scripts/ros/env_ros.sh python3 -m pytest ros2_ws/src/crackvision_motion/test/test_reachability_core.py -q -p no:cacheprovider`.

No robot motion, no ROS graph and no GPU are involved in this card.

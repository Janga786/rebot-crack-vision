# SCENE.md — Collision scene from config (MOT-03)

**Status:** NORMATIVE for the collision scene loaded ahead of any planning request against the
B601-DM model. Builds on `docs/motion/ROS_WORKSPACE.md` (MOT-02) and `docs/motion/ROBOT_MODEL.md`
(MOT-01).

## 0. What this is

`config/scene/scene.yaml` describes the workcell's static collision objects (table, specimen,
camera mount) and the allowed-collision-matrix (ACM) pairs between them and the robot's own links.
`ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py` validates that file (pure
Python, no ROS import, same style as `reachability_core.py`) and exposes
`assert_commissioning_ready`, the function MOT-05's commissioning-gated executor calls before any
real-hardware run. `crackvision_motion/scene_apply.py` (console script `apply_scene`) is the ROS
side: it builds one box `moveit_msgs/CollisionObject` per configured object, extends the *live*
`AllowedCollisionMatrix` with the configured pairs, and applies both as a single
`/apply_planning_scene` diff. `crackvision_motion/assert_scene_objects.py` (console script
`assert_scene_objects`) confirms a set of object ids are present in the live planning scene via
`/get_planning_scene`. Neither script uses an action client or touches `/move_action`,
`ExecuteTrajectory` or `FollowJointTrajectory` — nothing here can move the mock (let alone real) arm.

## 1. `crackvision.scene_config/1` schema

```yaml
schema: crackvision.scene_config/1
frame: base_link              # the frame every object's default pose.frame should normally use

objects:                       # non-empty list
  - id: <unique string>
    shape: box                 # only "box" is supported today
    dimensions_m: [x, y, z]    # all > 0
    pose:
      frame: <TF frame string>
      position_m: [x, y, z]
      rpy_rad: [roll, pitch, yaw]
    value_status: measured | nominal
    source: <non-empty free-text citation>

allowed_collisions:            # list, may be empty
  - link_a: <object id or robot link name>
    link_b: <object id or robot link name, != link_a>
    reason: <non-empty free-text>
    value_status: measured | nominal
    source: <non-empty free-text citation>
```

`scene_core.load_config(path)` (`SceneError` on any problem) enforces: exact top-level/nested key
sets (no typos, no silently-ignored extra keys), `schema` must equal `crackvision.scene_config/1`,
`objects` non-empty with unique ids, `shape` in `{box}`, `dimensions_m` all-positive 3-vectors,
every `pose` a well-formed `{frame, position_m, rpy_rad}` triple, `value_status` in
`{measured, nominal}` for **every** object and **every** `allowed_collisions` entry, and a
non-empty `source` string for each — mirroring `reachability_core.py`'s "never invent a value"
convention, but tagged per-value here (object-by-object) rather than with one file-wide
`value_status`, since a real commissioning survey will plausibly measure some objects (e.g. the
table) before others (e.g. a not-yet-decided camera mount). `config_sha256(path)` hashes the raw
file bytes, for provenance in future consumers (mirrors `reachability_core.config_sha256`).

## 2. The commissioning gate

```python
from crackvision_motion.scene_core import assert_commissioning_ready, SceneNotCommissionedError

assert_commissioning_ready(config)  # raises SceneNotCommissionedError if ANY value is nominal
```

`nominal_items(config)` returns a sorted list like `["allowed_collision:camera_mount~link1",
"object:camera_mount", "object:specimen", "object:table"]`; `all_measured(config)` is
`not nominal_items(config)`; `assert_commissioning_ready` raises `SceneNotCommissionedError`
(listing every pending item) unless `all_measured(config)` is `True`. **This is the function
MOT-05's commissioning-gated executor must call before any real-hardware run** — today,
`config/scene/scene.yaml` is 100% `nominal` (see §4), so `assert_commissioning_ready` on it always
raises; `ros2_ws/src/crackvision_motion/test/test_scene_core.py` tests this exact behaviour
(raises on the real, still-nominal `config/scene/scene.yaml`; does not raise once every value in a
config is flipped to `measured`; still raises if even one single value among many is left
`nominal`) so the refusal is proven now, ahead of MOT-05 existing.

## 3. `apply_scene` (console script, `crackvision_motion/scene_apply.py`)

```bash
ros2 run crackvision_motion apply_scene [--config PATH] [--root PATH]
```

- Default `--config`: `config/scene/scene.yaml` (relative to `--root`, default `$(pwd)`).
- Loads and validates the config with `scene_core.load_config` (exit 2 on an invalid config or a
  missing file, before any ROS call).
- Waits (90 s) for `/get_planning_scene` and `/apply_planning_scene` (exit 3 if either doesn't
  appear).
- `extend_acm(current_acm, config)`: reads the *live* ACM, returns a new
  `AllowedCollisionMatrix` with one new row/column per configured object id not already present
  (so calling `apply_scene` a second time with a different config **layers** on top of the first
  rather than clobbering it — `scripts/ros/test_scene.sh`, §5, relies on exactly this to add the
  test-only collider fixture after the production scene). Every new pair defaults to
  collision-checked (`enabled=False`); a pair is `enabled=True` only if `allowed_collisions` names
  it (in either order) — including pairs between two newly-added objects.
- `build_collision_objects(config)`: one `CollisionObject` (BOX primitive) per object, pose
  built from `position_m` + a quaternion from `rpy_rad`.
- Applies both as one `PlanningScene` diff (`is_diff=True`) via `/apply_planning_scene`. Exit 0 on
  success, 1 if either service call fails/returns `success=False`.

## 4. `config/scene/scene.yaml` — today's values (all nominal)

| object | dims (m) | pose (base_link) | allowed to touch | why |
|---|---|---|---|---|
| `table` | 1.00 × 0.80 × 0.05 | (0.40, 0.00, −0.045) | `base_link` | the surface the robot is bolted to; top face at z=−0.02, 2 cm below the robot's own origin, so it cannot intersect the arm's own links for any on-workspace joint state while still plausibly carrying the specimen |
| `specimen` | 0.20 × 0.20 × 0.01 | (0.30, 0.00, −0.015) | `table` | rests on the table top; x/y and footprint reuse `config/motion/specimen_placement.yaml`'s nominal footprint (itself nominal, MOT-04.3) |
| `camera_mount` | 0.05 × 0.05 × 0.08 | (−0.12, −0.12, 0.05) | `link1` | placeholder bracket volume near the base, diagonally behind the shoulder |

Every value's `source` field in the YAML spells out its provenance; nothing here was invented. Two
things are worth calling out beyond the file itself:

- **`camera_mount`'s parent frame/mounting is itself an open decision.**
  `docs/adr/013-calibration-method.md` (GEOM-03) is **proposed, blocked on operator confirmation**
  and *recommends* eye-in-hand (wrist-mounted, moving with the arm) over a fixed world mount. A
  static `base_link`-relative `CollisionObject` — what this card models, consistent with `table`
  and `specimen` — matches an eye-to-hand mount, not the recommended eye-in-hand one. This entry
  is therefore a conservative placeholder for whichever mounting is eventually confirmed; if
  eye-in-hand is confirmed, a future card must replace it with an attached collision object on the
  wrist link (`RobotState.attached_collision_objects`), which is a materially different mechanism
  (it moves with the joint state) that this card does not implement.
- **`camera_mount`'s pose was chosen from a live FK probe, not guessed blind.** `/compute_fk` for
  every link at `plan_joint_goal.py`'s `DEFAULT_JOINT_GOAL` (the same goal `scripts/ros/test_mock_plan.sh`
  already exercises) gave `link2` at `x=0.010 y=0.036 z=0.140` in `base_link` — about 0.21 m from
  `camera_mount`'s centre, comfortably clear given the object's own 0.05–0.08 m extents. This was
  necessary in practice: an earlier placement (`(-0.08, 0.0, 0.15)`, closer to the shoulder) made
  `move_group` report `Found a contact between 'camera_mount' ... and 'link2'` and a start-state-in-collision
  planning failure for that same default goal, which is exactly the class of accidental,
  unintended collision this scene must not introduce for ordinary operation. The final placement
  was re-verified to plan successfully (see §5, step 4).

## 5. `assert_scene_objects` (console script) and the smoke test

```bash
ros2 run crackvision_motion assert_scene_objects --object-id table --object-id specimen --object-id camera_mount
```

Calls `/get_planning_scene` (`WORLD_OBJECT_GEOMETRY` component), exits 0 iff every `--object-id`
is present in `scene.world.collision_objects`, 1 otherwise (with the missing ids and what *is*
present printed to stderr).

### `scripts/ros/test_scene.sh` — **not written by this card** (scope note)

This card's `scope.write` is `config/scene/**`, `ros2_ws/src/crackvision_motion/**` and this file
only — it does not include `scripts/ros/**`, so the wrapper script the acceptance harness invokes
(`bash scripts/ros/test_scene.sh`) could not be committed from here (writing it would be an
out-of-scope change). It was staged locally, run to completion, and then removed before finishing,
exactly reproducing `scripts/ros/test_reachability.sh`'s (MOT-04.4, accepted) launch/pgid/cleanup
pattern via the existing, unmodified `scripts/ros/_mock_stack.sh`:

1. `python3 -m pytest ros2_ws/src/crackvision_motion/test/test_scene_core.py` (pure Python, no ROS).
2. Start the MOT-02 headless mock stack.
3. `ros2 run crackvision_motion plan_joint_goal` with **no** crackvision scene applied — baseline,
   expect exit 0.
4. `ros2 run crackvision_motion apply_scene --config config/scene/scene.yaml` — expect exit 0.
5. `ros2 run crackvision_motion assert_scene_objects --object-id table --object-id specimen --object-id camera_mount` — expect exit 0.
6. `ros2 run crackvision_motion plan_joint_goal` again — the nominal production scene must not
   block the same default goal — expect exit 0.
7. `ros2 run crackvision_motion apply_scene --config ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml`
   — layers a deliberately oversized "collider" box (see §6) on top — expect exit 0.
8. `ros2 run crackvision_motion plan_joint_goal` once more — must now be **rejected** — expect a
   non-zero exit.
9. Tear the stack down (trap on EXIT/INT/TERM, SIGINT to the process group, 10 s wait, SIGKILL
   fallback), matching MOT-02/MOT-04.4.

Verified locally (clean `ros2_ws/build|install|log`):

```
$ bash scripts/ros/build_ws.sh                          # exit 0
$ bash scripts/ros/test_scene.sh
== unit tests: scene_core (pure python, no ROS) ==
...................                                      [100%]
19 passed in 0.12s
_mock_stack.sh: launch pid=1179 pgid=1179 up
== baseline: default goal with no crackvision scene applied ==
[INFO] [...] [plan_joint_goal]: plan succeeded for group 'arm' (error_code=1)
== apply config/scene/scene.yaml ==
[INFO] [...] [apply_scene]: applied scene from .../config/scene/scene.yaml: objects=[table, specimen, camera_mount]
== assert table/specimen/camera_mount appear in the planning scene ==
[INFO] [...] [assert_scene_objects]: all requested object ids present in the planning scene: ['table', 'specimen', 'camera_mount']
== default goal still plans with the nominal production scene applied ==
[INFO] [...] [plan_joint_goal]: plan succeeded for group 'arm' (error_code=1)
== apply the oversized collider fixture on top ==
[INFO] [...] [apply_scene]: applied scene from .../scene_collision_smoke.yaml: objects=[collider]
== same goal must now be rejected ==
[ERROR] [...] [plan_joint_goal]: plan failed for group 'arm' (error_code=99999)
test_scene.sh: colliding goal correctly rejected (plan_joint_goal exit=1)
test_scene.sh: OK
(exit 0)
```

After the run, `ps aux` showed no leftover `move_group` / `ros2_control_node` /
`robot_state_publisher` / `static_transform_publisher` process. The move_group log for step 8
confirms the rejection is a genuine collision, not an unrelated failure:
`Found a contact between 'collider' (type 'Object') and 'link2' (type 'Robot link')` /
`Start state appears to be in collision with respect to group arm`. The full existing pytest suite
under `ros2_ws/src/crackvision_motion/test/` (111 tests: 92 pre-existing + 19 new in
`test_scene_core.py`), `scripts/ros/test_mock_plan.sh` (MOT-02) and `scripts/ros/test_reachability.sh`
(MOT-04.4) were all re-run after this card's changes and still pass unchanged.

**For whoever adds `scripts/ros/test_scene.sh`:** its content is exactly steps 1–9 above,
byte-for-byte reproducible from this section plus `scripts/ros/test_reachability.sh`'s existing
launch/pgid/cleanup boilerplate — no new design decision is needed, only the file.

## 6. `scene_collision_smoke.yaml` — test-only collider fixture

`ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml` defines a single
4 m × 4 m × 4 m box (`collider`) centred on `base_link` (0, 0, 0.5), allowed to touch only
`base_link`. The canonical B601-DM's reach is about 1.0 m
(`reachability_sweep`'s live `reach_bound_from_urdf` measurement, `docs/motion/ROS_WORKSPACE.md`),
well inside this 4 m cube, so the whole robot (every link except `base_link`) is guaranteed to be
geometrically inside the box for *any* joint configuration — this makes the "a colliding goal is
rejected" check deterministic (a guaranteed `start state in collision`) without needing to compute
a specific colliding pose via FK. It is a test fixture only (`value_status: nominal`, scoped to
MOT-03's own smoke test), not a candidate production scene value.

## 7. Known limitation

Only axis-aligned box primitives are supported (`shape: box`). No cylinder/mesh/sphere support
exists yet; if a future measured object needs one, `scene_core._SHAPES` and
`scene_apply.build_collision_objects` both need a matching addition, each in one place.

# ROBOT_MODEL.md — B601-DM model reconciliation (MOT-01)

**Status:** NORMATIVE for planning. Reads `~/rebot_ws` read-only (never edited from this repo).

## 0. Source snapshot

`~/rebot_ws` HEAD at the time of this comparison: commit `2ebc4b8c09b560321a0023e13f813b6539644a22`
("Workstation deploy kit: plug-and-run real-robot demo on this HP Z8").

The working tree has 2 uncommitted, tracked-file modifications on top of that commit:

| File | Change |
|---|---|
| `src/rebotarm_moveit_config/config/kinematics.yaml` | IK solver swapped from `kdl_kinematics_plugin/KDLKinematicsPlugin` to `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` (+ `trac_ik_kinematics` block: `epsilon=1e-5`, `solve_type=Distance`) |
| `src/rebotarm_moveit_config/package.xml` | `+ <exec_depend>trac_ik_kinematics_plugin</exec_depend>` |

Neither uncommitted file touches joints, links, meshes or limits — they only affect which IK
plugin MoveIt loads — so they have **no effect** on any number in this document or in
`config/robot/b601_dm_limits.yaml`. All URDF/limit values below were read from the committed
blobs at `2ebc4b8c` (i.e. `git show 2ebc4b8c:<path>`, not the working tree), so a `git stash`
or a future commit of those two files changes nothing here. There were also many untracked
files (media, `Documentation/`, `src/trac_ik_ros2/`, misc scripts) — irrelevant to the model,
listed for completeness only.

## 1. Which files exist, and which is canonical

| File | Package | Role |
|---|---|---|
| `rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf` | `rebotarm_bringup` | **Canonical.** 6-DOF arm + 2-finger parallel gripper. Included by `rebotarm_moveit_config/config/rebotarm.urdf.xacro`, which is what `moveit_config/launch/demo.launch.py` and `hardware.launch.py` load as `robot_description`. This is the model MoveIt plans against today. |
| `rebotarm_bringup/description/urdf/reBot-DevArm_fixend.urdf` | `rebotarm_bringup` | Same 6-DOF arm, **no gripper** (ends in a fixed `end_link`). Loaded by `rebotarm_bringup/launch/bringup.launch.py` — i.e. **the real driver's `robot_description` on `bringup.launch.py` is a different, gripper-less model** than the one MoveIt plans with on `demo.launch.py`/`hardware.launch.py`. See §3. |
| `presentation/sim/reBot_B601_DM_with_gripper.urdf` (this repo) | — | A copy of the canonical file with mesh `filename=` rewritten from `package://` to an absolute path (so it opens outside a ROS workspace); see §2 — otherwise byte-for-byte identical kinematics/limits. |
| `third_party/reBotArm_control_py/urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf` | vendor SDK | A **different, older/related arm variant** (no `B601_DM`/gripper naming, different joint axes — e.g. `joint1` axis `0 0 -1` vs canonical `0 0 1` — different origins and much higher effort/velocity limits: 36 Nm/50 rad/s vs 27 Nm/50 rad/s). Not used by any launch file found under `rebot_ws/src`. Excluded from planning; kept only as vendor SDK reference. |
| `third_party/reBotArm_control_py/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf` | vendor SDK | Byte-identical kinematics to `rebotarm_bringup`'s `reBot-DevArm_fixend.urdf`; only the mesh path style differs (`meshes/...` vs `package://rebotarm_bringup/description/meshes/...`). Confirms `rebotarm_bringup`'s copy is a straight re-export, not a hand edit. |
| `third_party/reBotArm_control_py/config/rebotarm_dm.yaml` | vendor SDK | Motor-bridge config (Damiao serial, motor IDs, PID/MIT gains). Points its own `urdf_path` at `reBot-DevArm_fixend_description` and names the tool frame `end_link` — consistent with the driver side of §3, not the gripper side. |

**Canonical choice:** `rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf`, because it is
(a) the only model with the gripper attached, (b) the model MoveIt's own launch files load for
both simulation (`demo.launch.py`) and hardware (`hardware.launch.py`), and (c) the model this
repo's `presentation/sim/` copy was already made from (§2). REQ-MOT-1 asks for MoveIt
integration with the real model including the gripper, which only this file provides.

## 2. Vendor vs. presentation/sim diff

`./env.sh python scripts/motion/compare_urdf.py` (defaults: vendor =
`~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf`, other =
`presentation/sim/reBot_B601_DM_with_gripper.urdf`) reports:

```
RESULT: identical (links + joint kinematics/limits)
```

The only textual difference between the two files (not reported by the tool, since it ignores
mesh geometry) is the STL `filename=` attribute: `package://rebotarm_bringup/description/meshes_b601_gripper/*.STL`
in the vendor file vs. an absolute `/home/boosterk1/rebot_ws/...` path in the presentation copy,
so the meshes still resolve without a sourced ROS workspace. No joint axis, origin, or limit
differs.

## 3. Vendor (gripper model) vs. real-driver model (`reBot-DevArm_fixend.urdf`)

This is the divergence REQ-MOT-1 actually needs reconciled: `bringup.launch.py` (the real/mock
driver) publishes `robot_description` from `reBot-DevArm_fixend.urdf`, while MoveIt plans
against `reBot_B601_DM_with_gripper.urdf`.

```
./env.sh python scripts/motion/compare_urdf.py \
    --vendor ~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf \
    --other  ~/rebot_ws/src/rebotarm_bringup/description/urdf/reBot-DevArm_fixend.urdf
```

```
links only in vendor: ['gripper_left', 'gripper_link', 'gripper_right']
links only in other:  ['end_link']
joints only in vendor: ['gripper_joint', 'gripper_joint1', 'gripper_joint2']
joints only in other:  ['end_joint']
joint 'joint6' differs:
    origin_xyz: vendor='0.023692 0 0.04' other='0.028008 0 0.04'
```

`joint1`-`joint5` (axes, origins, limits) are byte-for-byte identical between the two files —
same 6-DOF arm. The differences are exactly the gripper: `DevArm_fixend` terminates `link6` in a
fixed `end_joint` -> `end_link` (a bare tool point, no gripper geometry, no gripper joints), while
the canonical model instead attaches `gripper_joint` (fixed) -> `gripper_link`, then two
prismatic finger joints `gripper_joint1`/`gripper_joint2` (both `axis="1 0 0"`, range `[0, 0.0715]
m`, i.e. drive in opposite local +x/-x after their `rpy` rotation to open/close). Consequently
`joint6`'s origin is offset by `+4.316 mm` in x on `DevArm_fixend` to compensate for the missing
gripper stack length, so the tool point lands in the same physical place either way.

**Reconciliation:** for planning, ignore `reBot-DevArm_fixend.urdf` entirely — it is a driver
convenience artifact for gripper-less bringup, not a second physical robot. MoveIt already uses
the correct (gripper) model via `rebotarm.urdf.xacro`. If a future task wires
`bringup.launch.py`'s `robot_description` into the same MoveIt session (e.g. real hardware run),
it must be repointed at `reBot_B601_DM_with_gripper.urdf` so the published TF tree and the
planning model agree — today they would silently disagree if both were run together.

## 4. TCP / gripper frames

From `rebotarm_moveit_config/config/rebotarm.urdf.xacro` (which includes the canonical URDF):

```xml
<link name="gripper_tcp"/>
<joint name="gripper_tcp_joint" type="fixed">
  <origin xyz="-0.0443 0 0" rpy="0 0 0"/>
  <parent link="gripper_link"/>
  <child link="gripper_tcp"/>
</joint>
```

- `gripper_link` — the fixed link the gripper fingers are mounted to (child of `link6` via the
  fixed `gripper_joint`, origin `xyz="0 0 0.15971" rpy="0 -1.5708 0"`).
- `gripper_tcp` — the MoveIt end-effector/TCP frame, `-44.3 mm` along `gripper_link`'s local x
  from `gripper_link`'s origin (fixed joint, no rotation). This is the frame planning groups
  should target, not `link6` or `gripper_link` directly.
  **Amended 2026-09-30 (ADR-014):** that holds for *grasping*. `gripper_tcp` sits at the finger
  mid-length. `gripper_link`'s origin is the closed-finger **tip**: the finger meshes placed by
  `gripper_joint1/2` at q=0 reach x = 0.0000 at y = 0. Surface-facing tasks (approach, trace, retract)
  target the new `tool_tip` frame. Camera view poses target `camera_link`. Both come from
  `config/robot/end_effector.yaml` via the `crackvision_description` overlay; the vendor files are
  unchanged.
- `gripper_left` / `gripper_right` — the two finger links, driven by `gripper_joint1` /
  `gripper_joint2` (prismatic, `[0, 0.0715] m` each) for open/close.

## 5. Canonical limits file

`config/robot/b601_dm_limits.yaml` — position (`lower`/`upper`) and `effort` come straight from
the canonical URDF's `<limit>` tags; `acceleration` comes from
`rebot_ws/src/rebotarm_moveit_config/config/joint_limits.yaml` (`max_acceleration`), since the
URDF has no acceleration attribute at all. `velocity` also comes from `joint_limits.yaml`
(`max_velocity`) rather than the URDF's `<limit velocity=...>`, because the URDF values (50-200
rad/s on the arm) are vendor motor no-load specs, not planning-safe limits, and MoveIt's own
`ompl_planning`/`moveit_controllers` stack reads `joint_limits.yaml` for velocity, never the
URDF's velocity attribute, for this robot. The raw URDF value is still recorded per joint as
`velocity_urdf_rad_s` for traceability. Every value's file+field is cited under `source:` in the
YAML; no value in that file was invented — where the URDF has no acceleration limit at all, the
only sourced number available (`joint_limits.yaml`'s `max_acceleration`) is used, never a guess.

`default_velocity_scaling_factor: 0.2` / `default_acceleration_scaling_factor: 0.2` in
`joint_limits.yaml` are per-trajectory runtime multipliers OMPL applies on top of the max values
above (not separate hard limits), so they are documented here but not stored per-joint in the
YAML.

## 6. Verification run

```
$ ./env.sh python scripts/motion/compare_urdf.py
RESULT: identical (links + joint kinematics/limits)
$ ./env.sh python -c "import yaml;d=yaml.safe_load(open('config/robot/b601_dm_limits.yaml'));J=d['joints'];assert len(J)>=6;assert all(all(k in J[j] for k in ('lower','upper','velocity','acceleration','source')) for j in J)"
(exit 0)
$ ./env.sh pytest tests/test_compare_urdf.py -q
7 passed
```

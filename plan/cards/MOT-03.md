# MOT-03 — Planning scene from config (nominal until measured)

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-02",
    "GEOM-10"
  ],
  "requirements": [
    "REQ-MOT-1"
  ],
  "scope": {
    "write": [
      "config/scene/**",
      "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/scene_apply.py",
      "ros2_ws/src/crackvision_motion/crackvision_motion/assert_scene_objects.py",
      "ros2_ws/src/crackvision_motion/test/test_scene_core.py",
      "ros2_ws/src/crackvision_motion/test/fixtures/scene_collision_smoke.yaml",
      "ros2_ws/src/crackvision_motion/setup.py",
      "scripts/ros/test_scene.sh",
      "docs/motion/SCENE.md"
    ]
  },
  "inputs": [
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "docs/INTERFACES.md#8",
    "config/robot/end_effector.yaml",
    "config/motion/reachability.yaml",
    "docs/motion/ROBOT_MODEL.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "build",
        "cmd": "bash scripts/ros/build_ws.sh",
        "timeout_s": 1500
      },
      {
        "id": "scene-test",
        "cmd": "bash scripts/ros/test_scene.sh",
        "timeout_s": 900
      }
    ],
    "criteria": [
      "Scene objects and ACM entries come from config/scene/scene.yaml; every value is tagged measured|nominal with a source (all nominal today).",
      "The scene models the WORKCELL only: the robot base stands on the table, so the table's top face is exactly at z = 0 in base_link (base_link origin = bottom face of base_link.STL) with an ACM entry table~base_link; the table footprint covers the base and the reachability grid in config/motion/reachability.yaml. The specimen is a box resting on the table (its bottom at z = 0), pose/footprint/height nominal and sourced from config/motion/specimen_placement.yaml when that is feasible, otherwise an explicitly labelled placeholder.",
      "There is NO world-fixed camera or camera_mount object: the D405 is eye-in-hand (ADR-014), and its camera/mount collision proxies are robot links from config/robot/end_effector.yaml (GEOM-10). A world object there would be a second, wrong model of the same hardware.",
      "scripts/ros/test_scene.sh (committed, in scope) builds nothing itself; it runs the scene_core unit tests, brings up the mock stack via scripts/ros/_mock_stack.sh, plans a baseline joint goal, applies the production scene, asserts table and specimen are present in the live planning scene, plans again successfully, then applies the oversized collider fixture and asserts the same goal is rejected, and leaves no stray processes.",
      "The commissioning executor (MOT-05) will refuse scenes with nominal values: scene_core.assert_commissioning_ready is stated and tested here as a function (unchanged contract)."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 4
  },
  "track": "simulation",
  "priority": 40,
  "id": "MOT-03",
  "title": "Planning scene from config (nominal until measured)",
  "parent": "L-MOTION",
  "outcome": "Collision scene (table, specimen, camera mount) loaded from config; UNMEASURED values block commissioning."
}
```

## Re-specified 2026-09-30 (technical-lead recovery)
Attempts 00108/00116 built the scene stack (committed at 5e84657) but could never pass: the `scene-test` check
runs scripts/ros/test_scene.sh, which was outside scope.write, so the implementer had to delete it. It is now in
scope. Attempt 00116's notes_for_reviewer contain a verified test_scene.sh; start from it (it still applies: the
mock stack now also carries the ADR-014 camera proxies, which the baseline plan must clear).

Two modelling corrections from ADR-014 and the operator's decisions:
1. Eye-in-hand: remove the world `camera_mount` object (and its ACM entry). The camera and mount are robot
   geometry in config/robot/end_effector.yaml (GEOM-10), so they move with the wrist and are already in every
   planning request. Update the production-scene test that expects {table, specimen, camera_mount}.
2. The robot stands on the table: table top face at z = 0 exactly (today it is 2 cm below the base, i.e. the
   robot floats), ACM table~base_link. Nominal footprint large enough for the base and the reachability grid
   (x 0.11..0.50 +/- margin, |y| <= 0.44), e.g. 1.20 x 1.00 x 0.05 m centred at (0.35, 0.0, -0.025). The
   specimen rests on the table: nominal 0.20 x 0.20 m, top at a plausible nominal height, centred per
   config/motion/specimen_placement.yaml if that file is feasible, else a clearly labelled placeholder that
   MOT-04.5/MOT-10 will replace. config/motion/reachability.yaml loads only the `table` object from this file
   (environment block); keep that id.

Everything else in the original scope stands: YAML-driven objects + ACM, measured|nominal tagging with sources,
scene_core.assert_commissioning_ready tested as a function, docs/motion/SCENE.md updated to match (including why
there is no world camera object and how the specimen pose is handed over from MOT-04.5 to MOT-10).

# MOT-10.1 — Decision + contract: workcell survey procedure and survey record (INTERFACES §12, crackvision.workcell_survey/1)

```json card
{
  "kind": "decision",
  "depends_on": [
    "MOT-03",
    "MOT-05.1"
  ],
  "requirements": [
    "REQ-MOT-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "docs/motion/WORKCELL_SURVEY.md",
      "docs/INTERFACES.md",
      "config/scene/workcell_survey.template.yaml"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#7",
    "docs/INTERFACES.md#8",
    "docs/INTERFACES.md#11",
    "docs/motion/SCENE.md",
    "docs/motion/ROBOT_MODEL.md",
    "docs/adr/014-end-effector-frames-and-task-phases.md",
    "config/scene/scene.yaml",
    "config/motion/reachability.yaml",
    "config/motion/specimen_placement.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/scene_core.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "sections",
        "cmd": "bash -c 'test $(grep -c \"^## 11\\. \" docs/INTERFACES.md) -eq 1 && test $(grep -c \"^## 12\\. \" docs/INTERFACES.md) -eq 1 && grep -qF crackvision.workcell_survey/1 docs/INTERFACES.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "doc",
        "cmd": "bash -c 'test -s docs/motion/WORKCELL_SURVEY.md && grep -qF crackvision.workcell_survey/1 docs/motion/WORKCELL_SURVEY.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "template-yaml",
        "cmd": "./env.sh python -c \"import yaml,sys; d=yaml.safe_load(open('config/scene/workcell_survey.template.yaml')); sys.exit(0 if d.get('schema')=='crackvision.workcell_survey/1' else 1)\"",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§12 is appended after §11 and §0–§11 are byte-identical to what they were before this card. No existing contract is changed.",
      "The base_link datum is stated explicitly and is something an operator can physically reach. Origin = bottom face of base_link.STL = table top when the base is bolted directly to the table. The x/y reference faces are the physical base's front and side faces, and their base_link offsets are taken from the canonical URDF's base_link.STL bounding box, read from the ~/rebot_ws/install underlay (read-only). The values and the command that produced them go in the doc. The doc says plainly that the operator must confirm those faces are the ones on the real casting/plate.",
      "The schema records RAW readings plus instrument, resolution and stated 1-sigma uncertainty for each reading. It does not ask for operator-computed base_link coordinates. It covers: (a) base mounting: bolted directly to the table yes/no, adapter plate thickness; (b) table extents from the base reference faces, and top-face flatness/level near the specimen; (c) the four specimen top corners as distances from the base front face (x) and the base centreline/side face (y), plus caliper thickness at ≥3 places; (d) clearance obstacles inside a stated radius of base_link (walls, fixtures, cables, the camera USB lead routing), each as an axis-aligned box from the same references; (e) ACM facts (base bolted to table, specimen resting on table) as observations; (f) operator, date, photo evidence paths under data/workcell_survey/ (never committed), and free-text notes.",
      "Derived-value rules are normative and deterministic. Specimen centre/yaw/footprint comes from the four corners, with a stated rectangularity tolerance. Specimen top z = adapter-plate offset + mean thickness. The table box is chosen so its top face is at z = -(adapter plate thickness), i.e. exactly 0 when bolted directly. Obstacle boxes are defined the same way. Every derived scene object gets value_status measured and a source naming the survey file sha256. Refusal conditions are listed, e.g. non-rectangular corners beyond tolerance, specimen not on the table, obstacle overlapping the base keep-out, any uncertainty missing.",
      "The doc states that camera/mount poses are NOT tape-measured here. They come from hand-eye calibration (GEOM-05, ADR-013/014), and the eye-in-hand mount is robot geometry (GEOM-10). It also states that the procedure is motion=false: the arm stays powered off or at rest and nothing is jogged.",
      "The doc gives the operator a step-by-step checklist (tools: steel tape/rule, square, calipers, spirit level), the expected uncertainties, how to fill the template, and the commands that will validate and convert it (survey_to_scene, delivered by MOT-10.3). It names the re-measure triggers: specimen moved, base re-bolted, table changed."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 3,
    "consequence": 4,
    "task_class": "decision+contract",
    "local_ok": false
  },
  "id": "MOT-10.1",
  "parent": "MOT-10",
  "title": "Decision + contract: workcell survey procedure and survey record (INTERFACES §12, crackvision.workcell_survey/1)",
  "outcome": "A normative operator procedure (docs/motion/WORKCELL_SURVEY.md) that says how to measure the table, specimen and clearance obstacles in base_link with hand tools and no robot motion. Also a new INTERFACES §12 contract for the raw survey record those measurements go into, plus a fill-in template."
}
```

Define how MOT-10's physical survey is done and recorded before anyone measures anything.

1. Read docs/motion/SCENE.md, config/scene/scene.yaml, scene_core.py and ADR-014. Derive the base_link.STL axis-aligned bounding box from the canonical B601-DM model in the ~/rebot_ws/install underlay (read-only; MOT-01 / docs/motion/ROBOT_MODEL.md name the canonical model). Use a short throwaway snippet, and record both the snippet and the numbers in WORKCELL_SURVEY.md. These give the base_link coordinates of the physical reference faces (front face x, side faces y, bottom face z=0).
2. Write docs/motion/WORKCELL_SURVEY.md with: purpose; datum and reference faces; tools and expected uncertainty; the measurement checklist (base mounting, table, specimen corners + thickness, clearance obstacles within the stated radius, ACM observations, photos); derived-value rules; refusal rules; how it feeds config/scene/scene.yaml (MOT-10.3 tool → MOT-10.5); what is out of scope (camera pose = GEOM-05, TCP = GEOM-07); re-measure triggers.
3. Append `## 12. Workcell survey record (crackvision.workcell_survey/1)` to docs/INTERFACES.md after MOT-05.1's §11. Give the exact key set, units (metres, radians), required/optional fields, per-reading uncertainty, derived-value formulas and refusal rules. Do not touch §0–§11.
4. Write config/scene/workcell_survey.template.yaml: every key present, values null, and comments that tell the operator what to measure and from which face. It must parse as YAML with schema crackvision.workcell_survey/1.

No code. No robot motion. Commit only the three scoped files.

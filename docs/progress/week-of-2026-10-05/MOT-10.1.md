# MOT-10.1 — Decision + contract: workcell survey procedure and survey record (INTERFACES §12, crackvision.workcell_survey/1)

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** software · **Verified commit:** [`e05be30`](https://github.com/Janga786/rebot-crack-vision/commit/e05be30b2a2624f094532358024551db32375c27)

## What was done

Defined the step-by-step procedure and file format for physically surveying the workcell (table, specimen, nearby obstacles) with a tape measure, calipers and a level, with no robot motion. The base's three reference faces were pinned to exact numbers (front face at x=0.070 m, sides at y=±0.100 m, bottom at z=0) by measuring the real robot's 3D model file. All three required checks passed, confirming the new documentation and the fill-in template are well-formed and consistent.

After the first audit, a repair round (MOT-10.1.R1) fixed the reported issues: Fixed three review-flagged gaps in the workcell-survey measurement spec (MOT-10.1): the rectangularity check that could never actually reject a bad specimen measurement now uses a real residual metric, verified numerically to correctly refuse a 5mm-skewed corner at the 3mm default tolerance. Added explicit, unambiguous formulas and sign conventions for converting every tape-measure/caliper reading into robot-frame coordinates, including a new table-thickness measurement needed to fully define the table's collision geometry. Also resolved a contradiction where the fill-in template told operators to leave a field null in the normal case while the validation rule would have rejected exactly that. All three automated acceptance checks pass.

Files changed:
- [config/scene/workcell_survey.template.yaml](https://github.com/Janga786/rebot-crack-vision/blob/e05be30b2a2624f094532358024551db32375c27/config/scene/workcell_survey.template.yaml) (+128 / −5)
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/e05be30b2a2624f094532358024551db32375c27/docs/INTERFACES.md) (+190 / −19)
- [docs/motion/WORKCELL_SURVEY.md](https://github.com/Janga786/rebot-crack-vision/blob/e05be30b2a2624f094532358024551db32375c27/docs/motion/WORKCELL_SURVEY.md) (+218 / −11)

Commits: [`0ef2eb0`](https://github.com/Janga786/rebot-crack-vision/commit/0ef2eb0afa4e9c59e4c487c5895218228fa53af2), [`e05be30`](https://github.com/Janga786/rebot-crack-vision/commit/e05be30b2a2624f094532358024551db32375c27)

## Why it was done

A normative operator procedure (docs/motion/WORKCELL_SURVEY.md) that says how to measure the table, specimen and clearance obstacles in base_link with hand tools and no robot motion. Also a new INTERFACES §12 contract for the raw survey record those measurements go into, plus a fill-in template.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **6/27** tasks accepted; whole project: **48/104**.
- REQ-MOT-1: 5/24 contributing tasks done
- REQ-INT-2: 2/13 contributing tasks done
- Verification: automated checks run by the pipeline itself (sections ✔, doc ✔, template-yaml ✔); independent audit accepted it (criteria: 4 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “The repair fixes all three MOT-10.1 findings and stays within its write scope. - **Finding 1:** §12.7 rule 1 now uses one exact, non-trivial residual: max(|d0-d1|, |s0-s2|, |s1-s3|). It also gives formulas for centre, yaw and footprint, and a worked example. I recomputed the example independently: residual = 0.005 m, so it is refused at the 3 mm default. - **Finding 2:** Every reading now has an explicit formula into base_link, with sign conventions for x/y and the near edge. Obstacle z uses the z_table_top datum. A new table.thickness reading sets the table box's dimension
…[754 chars clipped]”
- Quality loop: the audit found 3 issue(s) that were fixed before acceptance (repair task MOT-10.1.R1): Template's bolted-directly instruction conflicts with refusal rule 6 (null readings); Rectangularity refusal can never fire: corners are tested against a minimum-area enclosing rectangle; Offset→base_link conversion, sign conventions and table box thickness are not specified, so derivation is not deterministic.

## What it unlocks next

- **Ready to start:** MOT-10.2 — Workcell survey core: validate a crackvision.workcell_survey/1 record and derive measured scene objects (pure Python)
- Closer: MOT-10.4 — Operator: place the specimen at the recommended pose and perform the workcell survey (no motion) (still needs MOT-10.3, MOT-04.5)

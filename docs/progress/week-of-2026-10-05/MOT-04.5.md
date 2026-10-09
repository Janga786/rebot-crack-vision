# MOT-04.5 — Run the full reachability sweep, recommend + independently verify the specimen placement, document

**Accepted:** 2026-10-09 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`9f2951b`](https://github.com/Janga786/rebot-crack-vision/commit/9f2951bff8fe13a906f22e2e4f308890c0095730)

## What was done

Ran the full-resolution MoveIt reachability sweep (3,780 poses, ~27 minutes) for the B601-DM using the corrected tool-tip/specimen-block model and found a feasible specimen placement at x=0.26 m, y=0.0 m on the table, with 61% of all swept poses reachable overall. Independently re-verified that placement at twice the sweep's spatial resolution (578/578 points reachable, reproduced on a repeat run) and confirmed the wrist camera can view the placement from its nominal 0.25 m distance. Documented the full method, results and an honest comparison against both the project's earlier (bugged) reachability attempt and its original hand-written reachability study, without overclaiming agreement that wasn't directly measured.

Files changed:
- [config/motion/placement_verify.yaml](https://github.com/Janga786/rebot-crack-vision/blob/9f2951bff8fe13a906f22e2e4f308890c0095730/config/motion/placement_verify.yaml) (+109 / −0)
- [config/motion/reachability.yaml](https://github.com/Janga786/rebot-crack-vision/blob/9f2951bff8fe13a906f22e2e4f308890c0095730/config/motion/reachability.yaml) (+8 / −4)
- [config/motion/specimen_placement.yaml](https://github.com/Janga786/rebot-crack-vision/blob/9f2951bff8fe13a906f22e2e4f308890c0095730/config/motion/specimen_placement.yaml) (+100 / −9)
- [config/motion/view_verify.yaml](https://github.com/Janga786/rebot-crack-vision/blob/9f2951bff8fe13a906f22e2e4f308890c0095730/config/motion/view_verify.yaml) (+110 / −0)
- [docs/motion/REACHABILITY.md](https://github.com/Janga786/rebot-crack-vision/blob/9f2951bff8fe13a906f22e2e4f308890c0095730/docs/motion/REACHABILITY.md) (+501 / −176)
- [docs/motion/figures/reachability.png](https://github.com/Janga786/rebot-crack-vision/blob/9f2951bff8fe13a906f22e2e4f308890c0095730/docs/motion/figures/reachability.png) (+0 / −0)
- [scripts/motion/plot_reachability.py](https://github.com/Janga786/rebot-crack-vision/blob/9f2951bff8fe13a906f22e2e4f308890c0095730/scripts/motion/plot_reachability.py) (+309 / −2)

Commits: [`796efea`](https://github.com/Janga786/rebot-crack-vision/commit/796efea4a48c25f781db7b35c90ccb79dba5cb99), [`9f2951b`](https://github.com/Janga786/rebot-crack-vision/commit/9f2951bff8fe13a906f22e2e4f308890c0095730)

## Why it was done

A real MoveIt/trac_ik reachability map of the B601-DM over the configured workspace, including a figure and docs/motion/REACHABILITY.md. The result is a committed nominal config/motion/specimen_placement.yaml whose footprint is independently re-verified at half the map's grid step by a re-runnable check.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-2**: Reproducible install/config/tests/evaluation results, handoff docs, requirements→evidence map, honest readiness report

## How it moves the project forward

- Motion planning: **12/27** tasks accepted; whole project: **58/104**.
- REQ-MOT-1: 10/24 contributing tasks done
- REQ-MOT-2: 6/11 contributing tasks done
- REQ-OPS-2: 5/8 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, placement-valid ✔, placement-feasible ✔, placement-verify ✔, view-verify ✔, doc-and-figure ✔, plot-cli-dry-run ✔); independent audit accepted it (criteria: 9 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I re-ran the key steps and they check out. The full map is complete and its hashes match the doc (map, config, limits, scene). Re-running recommend_placement on the committed map in /tmp gives byte-identical specimen_placement.yaml, placement_verify.yaml and view_verify.yaml, so none of them were hand-edited. I recomputed the per-surface_z best placements and the §2.5 comparison numbers at (0.23, 0) and (0.41, 0); both match the doc. The half-step verify map is 578/578 reachable with FK error ≤1e-5 m, and the view check is 1/1 reachable with camera_link. plot_reachability.p
…[368 chars clipped]”

## What it unlocks next

- Closer: MOT-10.4 — Operator: place the specimen at the recommended pose and perform the workcell survey (no motion) (still needs MOT-10.1, MOT-10.3)
- Closer: MOT-10.5 — Apply the measured survey to config/scene/scene.yaml and re-run the scene + reachability verification at the as-placed specimen (still needs MOT-10.4, MOT-10.3)

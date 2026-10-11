# MOT-06.10 — Synthetic end-to-end planning run (GEOM-08 synthetic case → paths3d → view + crack plan) and CARTESIAN_PLANNING.md

**Accepted:** 2026-10-11 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`1110ad7`](https://github.com/Janga786/rebot-crack-vision/commit/1110ad7daf0e7c1f02913f41ce3c32a7be830566)

## What was done

Built a single script that runs the whole synthetic crack-inspection pipeline end to end — from a generated synthetic crack surface, through 3D path lifting, to camera-view planning and full Cartesian trace planning on the mock robot stack — and verified it actually plans a complete, collision-checked path (36/36 waypoints, 100% coverage) in about three minutes. Along the way I found and worked around a real bug where two parts of the system disagreed on a data format (crack-component IDs as numbers vs. text), which had never been caught because no prior test exercised the real pipeline's output end to end; this is now documented for the team to fix properly. I also wrote up the results, caveats, and handoff notes in docs/motion/CARTESIAN_PLANNING.md and added the two new planning commands to the workspace reference docs.</presentation_summary>
</invoke>

Files changed:
- [docs/motion/CARTESIAN_PLANNING.md](https://github.com/Janga786/rebot-crack-vision/blob/1110ad7daf0e7c1f02913f41ce3c32a7be830566/docs/motion/CARTESIAN_PLANNING.md) (+113 / −0)
- [docs/motion/ROS_WORKSPACE.md](https://github.com/Janga786/rebot-crack-vision/blob/1110ad7daf0e7c1f02913f41ce3c32a7be830566/docs/motion/ROS_WORKSPACE.md) (+46 / −0)
- [scripts/ros/run_cartesian_synthetic.sh](https://github.com/Janga786/rebot-crack-vision/blob/1110ad7daf0e7c1f02913f41ce3c32a7be830566/scripts/ros/run_cartesian_synthetic.sh) (+244 / −0)

Commits: [`1110ad7`](https://github.com/Janga786/rebot-crack-vision/commit/1110ad7daf0e7c1f02913f41ce3c32a7be830566)

## Why it was done

One script runs tools/synth_scene3d.py → crackvision.paths → crackvision.path3d → plan_view/plan_crack_path on the mock stack with a scene consistent with the synthetic surface, and docs/motion/CARTESIAN_PLANNING.md documents usage, the measured coverage/fallback behaviour, caveats and the MOT-07 handoff; ROS_WORKSPACE.md lists the new CLIs.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-INT-1**: Simulation and recorded-data integration tests of the full pipeline
- Requirement **REQ-OPS-2**: Reproducible install/config/tests/evaluation results, handoff docs, requirements→evidence map, honest readiness report

## How it moves the project forward

- Motion planning: **27/41** tasks accepted; whole project: **88/128**.
- REQ-MOT-2: 14/22 contributing tasks done
- REQ-INT-1: 3/6 contributing tasks done
- REQ-OPS-2: 6/9 contributing tasks done
- Verification: automated checks run by the pipeline itself (build ✔, e2e ✔, doc ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Reproduced the full build and end-to-end script locally (exit 0 both times), independently confirmed execution_eligible/coverage/ineligible_reasons via direct JSON inspection, and verified the documented scene.yaml/specimen_placement.yaml mismatch and the component_id cross-system bug claim against the actual source code. All three acceptance checks pass, scope is respected (only the three allowed files touched, no config/ or contract files modified), and the scratch output stays gitignored under logs/. No discrepancies found between the implementer's claims and observed behavior.”

## What it unlocks next

- Nothing depends directly on this task; it completes its branch of the plan.

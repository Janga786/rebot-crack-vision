# INT-01.1 — Synthetic RGB-D case through the real segmentation-to-paths-to-3D chain (no ROS)

**Accepted:** 2026-10-14 · **Work package:** Integration & commissioning (L-INT) · **Track:** simulation · **Verified commit:** [`d397a5a`](https://github.com/Janga786/rebot-crack-vision/commit/d397a5a548d51c6ce360858f3dd6afffc3e1aa59)

## What was done

Built a single script that runs a synthetic test crack scene through the project's real image-processing pipeline (mask -> skeleton -> path tracing -> 3D lifting), using the actual command-line tools an operator would run, with no robot or ROS involved. Verified it exits cleanly, finds a 105-pixel skeleton, and correctly reports the resulting 3D path as not yet eligible for real robot execution (since it's synthetic, nominal-calibration data).

Files changed:
- [scripts/perc/run_synthetic_perception.sh](https://github.com/Janga786/rebot-crack-vision/blob/d397a5a548d51c6ce360858f3dd6afffc3e1aa59/scripts/perc/run_synthetic_perception.sh) (+165 / −0)

Commits: [`d397a5a`](https://github.com/Janga786/rebot-crack-vision/commit/d397a5a548d51c6ce360858f3dd6afffc3e1aa59)

## Why it was done

One command (scripts/perc/run_synthetic_perception.sh) builds a GEOM-08.8 synthetic case and runs it, as a stand-in predicted mask, through the real crackvision.visualize -> crackvision.skeleton -> crackvision.paths -> crackvision.visualize_paths -> crackvision.path3d CLIs, asserting exit 0 and the expected output files/fields at every step, with no ROS and no hardware.

- Requirement **REQ-INT-1**: Simulation and recorded-data integration tests of the full pipeline

## How it moves the project forward

- Integration & commissioning: **2/9** tasks accepted; whole project: **118/156**.
- REQ-INT-1: 6/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (run ✔); independent audit accepted it (criteria: 8 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Script correctly chains the real crackvision CLIs over a synthetic GEOM-08.8 case, verified by rerunning it twice independently with exit 0 and all assertions passing.”

## What it unlocks next

- **Ready to start:** INT-01.2 — One command: synthetic perception through MoveIt-mock crack planning, retiming and a limits-checked mock execution

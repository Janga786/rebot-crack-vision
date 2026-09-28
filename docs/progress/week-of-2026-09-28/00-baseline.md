# Baseline — full-pipeline plan and audit-gated automation (Sep 27–28, 2026)

## What was done

- **Adopted an audit-gated engineering workflow** (`claude-auto`). Every task runs as a fresh, sandboxed Claude Code
  session. The pipeline itself runs each task's automated checks, and a task counts as done only after an independent
  audit of the exact commit. Audits that find problems create repair tasks. Steps that move the robot are never
  automated: they are operator procedures whose recorded evidence is audited afterwards.
- **Planned the complete pipeline**: D405 capture → OpenCrack segmentation → ordered crack paths → calibrated 3D robot
  coordinates → MoveIt trajectory planning → gated execution on the reBot B601-DM. The plan has 69 tasks in 9 work packages,
  22 requirements and explicit dependencies (see `plan/`). Tasks that cannot be specified yet are marked for later planning
  instead of being guessed.
- **Re-verified phase 1 instead of trusting old status labels.** The 9 previously approved tasks (TC-001…TC-009) had their
  checks re-run on 2026-09-28, including the GPU smoke test (real OpenCrack nnU-Net inference on CUDA, 13.8 s). All passed.
  The project's test suite passes 46/46 and the environment check passes 20/20.
- **Inspected the actual workstation** rather than relying on older notes: RTX 3090 (24 GB), ROS 2 Humble + MoveIt 2.5.9,
  RealSense SDK 2.57 (ROS) / 2.58 (Python). No D405 camera or arm is attached yet.

## Why it was done

Phase 1 deliberately stopped at the 1-pixel crack skeleton. The capstone goal needs the rest of the chain, up to a validated robot
motion. That work has to keep moving between weekly check-ins, with progress that can be verified rather than claimed.

## How it moves the project forward

- Status: **9/69 tasks accepted** (phase-1 perception). The environment is verified, and every remaining step is either
  specified or explicitly scheduled for detailed planning.
- Hardware-dependent work (camera validation, hand-eye/TCP calibration, physical commissioning) is tracked as explicit
  blockers with named prerequisites, and is reported separately from software and simulation results.
- From now on, each accepted task adds its own page to this folder: what was done, why, how it moves the project, and what it
  unlocks next.

## What it unlocks next

- First tasks in order: ARCH-01 (scope decision record), TC-012 (RealSense software validation), TC-010 (skeletonization),
  MOT-01 (robot model + joint limits), HOST-01 (host version manifest), LLM-01 (local coding-model shortlist),
  HOST-02 (environment lock), HOST-05 (storage decision).
- Needed from the operator: connect the D405, decide how to use the unmounted 1.9 TB drive, and confirm how the camera will be
  mounted on the arm.

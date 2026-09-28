# ADR-009: Robot control, camera-robot calibration and ROS 2 integration are deferred

**Status:** accepted · **Date:** 2026-09-16
Status: Superseded in part by ADR-011 (2026-09-28)

## Context
The capstone target is a reBot-DevArm B601-DM (a 4-DOF-class short-reach arm; an existing ROS 2 +
MoveIt2 + Gazebo workspace for it lives at `~/rebot_ws`, and a second LeRobot-based control stack at
`~/rebot_lerobot`). It is tempting to wire perception straight into that. But the project's actual
open question is whether OpenCrack segments D405 crack imagery usably at all. Building trajectory
generation before knowing that risks discarding the whole downstream design.

There is also a hard technical wedge: ROS 2 Humble here is Python **3.10**, while this project needs
Python **3.11** (`adr/003`). They cannot share one interpreter.

## Decision
This phase produces **files on disk** — masks, skeletons, aligned depth, intrinsics — and stops.
No ROS nodes, no topics, no launch files, no MoveIt2, no hand-eye calibration, no trajectory
generation, no arm motion. Nothing in `~/rebot_ws` or `~/rebot_lerobot` is read, written, or imported.

## Consequences
+ Keeps the risky research question isolated and cheap to answer.
+ Keeps the environment decision clean; no 3.10/3.11 contortions.
+ The on-disk contract (same pixel grid for mask/skeleton/aligned depth, plus per-frame intrinsics
  and depth scale in JSON) is exactly what a future ROS node or offline trajectory tool needs.
− A separate integration phase will be required, most likely a small ROS 2 node in a 3.10 env that
  consumes these files or calls this project over a socket.
− Zero risk of a perception experiment commanding a physical arm. (This also honours the standing
  house rule that robot hardware is never actuated without explicit per-action approval.)

## Revisit when
Level-3 evaluation says OpenCrack is good enough on D405 imagery. Integration is then its own project
phase with its own planning session.

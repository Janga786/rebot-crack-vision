# ADR-003: A dedicated Python 3.11 conda environment named `crackvision`

**Status:** accepted · **Date:** 2026-09-16

## Context
The workstation already carries five conda envs (`base` 3.13, `navila` 3.10, `navila-vila`,
`lerobot` 3.12, `isaaclab` 3.11, `vlnce-isaac`), each pinned to a different torch/CUDA combination for
a different active robotics project. None has `nnunetv2` or `pyrealsense2`. Installing into any of
them risks breaking load-bearing work (NaVILA sim2real, Isaac Lab, LeRobot).

Version constraints that must hold simultaneously:
`nnunetv2` needs `>=3.10` and `torch>=2.1.2,!=2.9.*`; `scikit-image` ≥0.26 needs `>=3.11`;
`pyrealsense2` 2.58.4 ships cp310–cp313 linux wheels.

Machine-specific hazards (both empirically verified — `MACHINE_STATE.md` §4):
`~/.local/lib/python3.10/site-packages` holds 272 packages that shadow **any** 3.10 interpreter;
`/opt/ros/humble` exports a `PYTHONPATH` of 3.10 site-packages that leaks into every interpreter.
`~/.local/lib/python3.11` is empty.

## Decision
Create a new conda env **`crackvision` on Python 3.11**, used by this project exclusively. Install the
project into it editable (`pip install -e .`). Never install into, upgrade, or remove any existing env.

## Consequences
+ 3.11 satisfies every dependency and dodges both the `~/.local` 3.10 user-site and the worst of the
  ROS 3.10 collision.
+ Other projects are provably unaffected.
− ~6–9 GB of disk for a second torch (acceptable: 124 GB free after cleanup).
− Not ROS-compatible: `rclpy` on this box is built for 3.10. Accepted — ROS is out of scope
  (`adr/009`), and a future bridge will be a separate process communicating over files or a socket.
− `PYTHONPATH` still leaks in and must be scrubbed at launch (`adr/007`).

## Revisit when
ROS 2 integration becomes in-scope. The likely answer then is still a separate process, not a merged
environment.

# arm_guard

Last-line safety filter in front of every reBot B601-DM motor command. Stdlib-only Python (3.10–3.12), loaded at
interpreter start in every env that can reach the motors, so that IK, the planner, the vendor SDK, LeRobot or a
script cannot drive the arm outside its limits.

Documentation: [`docs/motion/ARM_GUARD.md`](../../docs/motion/ARM_GUARD.md). Next steps: [`TODO.md`](../../TODO.md).

```
src/arm_guard/core.py        pure rules R1-R7: (command, measured state, config) -> verdict
src/arm_guard/runtime.py     per-process state: joint mapping, feedback freshness, fault latch + hold
src/arm_guard/patch.py       in-place patch of motorbridge Controller/Motor (allowlist)
src/arm_guard/config.py      TOML config + approval hash check (config/robot/arm_guard.toml)
src/arm_guard/kinematics.py  stdlib URDF forward kinematics for the keep-out checks
src/arm_guard/autoload.py    patch motorbridge on import; src/arm_guard_boot.py: fail-closed .pth entry point
tests/                       155 tests incl. closed-loop fuzz and replay of every real run (fake_mb/ = fake motorbridge)
vendor_patches/              the change applied to ~/rebot_ws (vendor ROS driver requires the guard)
```

Run the tests: `bash scripts/safety/test_arm_guard.sh` from the repo root.

# Operator runner snapshot — `~/rebot_setup`, 2026-10-23

Copies of the files that changed in `~/rebot_setup` (not under version control) since the
[2026-10-22 snapshot](../rebot_setup_runner_2026-10-22/README.md). Checksums are in `SHA256SUMS`. The pre-change copies
are in `~/rebot_setup/backup_pre_state_file_2026-10-22/`.

| File | What changed |
|---|---|
| `demo_move.py` (2026-10-23.1) | New `--state-file FILE`: about 20 Hz, an atomic JSON `rebot.arm_state/1` of the measured pose (physical joint angles, J1 reference removed), the segment kind, velocities and wall time. Output only; the control loop is unchanged. Used by `crackvision.crack_trace capture` (the camera capture at the scan pose) |
| `test_guarded_runs.py` | `StateFile` (2 tests). Runner suite: 43 tests, all pass |
| `trajectories/fancy/*.json` | The 11 show trajectories from `tools/make_fancy_trajectory.py --all`, each rehearsed at x1 and x2 with the real guard config (22/22 completed, no guard fault) |

None of it has run on the powered arm. See [docs/motion/CRACK_TRACE_PIPELINE.md](../../../docs/motion/CRACK_TRACE_PIPELINE.md).

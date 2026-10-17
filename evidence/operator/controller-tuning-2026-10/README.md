# Operator evidence: controller tuning, 2026-10-14 and 2026-10-17

Raw material behind [`docs/motion/CONTROLLER_TUNING.md`](../../../docs/motion/CONTROLLER_TUNING.md) (the full
write-up with all the mathematics). Operator-attended hardware sessions on the physical B601-DM; every powered run
was started on the operator's explicit "go". **Not commissioning**: no change to `config/robot/*`, e-stop = 24 V
supply switch only, joint limits not measured.

| Path | Contents |
|---|---|
| `runs/` | **every real hardware log** of the week: per run a `*.csv` (200 Hz samples: reference, position, velocity, torque, feed-forward, every held joint) and a `*.json` summary (parameters, pre-checks, confirmation, outcome, statistics, sweep scores); `demo_live.log` (console of the last demo), `power_on_scan_*`, `estop_test_2026-10-14.md` (the decision not to test the switch) |
| `analysis/analyze.py` | regenerates everything below from `runs/` (system `python3` + numpy + matplotlib; also re-runs the simulator for f14) |
| `analysis/RUNS.md` | index of all 41 runs with gains, feed-forward, outcome, errors, trigger and session notes |
| `analysis/SWEEPS.md` | every gain-sweep table (RMS tracking, hold error, overshoot, oscillation, peak torque, score) |
| `analysis/metrics.json` | every number quoted in the write-up (identification, demo errors, loop timing, feedback resolution) |
| `analysis/feedback_resolution.md` | position/velocity/torque resolution measured from the logs |
| `figures/` | f01–f16 (catalogue in the write-up §13) |
| `code/` | exact copies of the scripts used (operator workspace `~/rebot_setup/`): `first_move_j1.py` (single-joint moves, holds, gravity/friction feed-forward, sweeps, multi-joint), `demo_move.py` (all-joint demos, path checks, park-on-abort), `gravity.py` (URDF gravity model), `fakebus.py` (offline simulator), tests (65), `tuned_gains.yaml`, `model/` (vendor URDF) |

Final gains (`code/tuned_gains.yaml`): J1 100/4.0, J2 100/1.0, J3 80/1.0, J4–J6 40/1.0, gripper 40/0.5 (kp/kd);
gravity scales J2 ×0.6, J3 ×2.5, J4 ×2.5; friction feed-forward J1 1.0, J2 1.0, J3 2.0, J4 0.5, J5 0.3, J6 0.1 Nm.

```bash
python3 analysis/analyze.py                                   # figures + tables
cd code && python3 -m unittest test_first_move_j1 test_demo_move   # offline tests, no hardware
```

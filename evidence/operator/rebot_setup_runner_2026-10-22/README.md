# Operator runner snapshot — `~/rebot_setup`, 2026-10-22

`~/rebot_setup` is the operator's workspace on the arm workstation, and it is not under version control. These are
unmodified copies of its scripts as of 2026-10-22, the code that ran the first guarded trajectories on the
physical arm. Checksums are in `SHA256SUMS`. To restore, copy the files back into `~/rebot_setup/`; the runner
expects that location next to `runs/` and `trajectories/`.

| File | What |
|---|---|
| `demo_move.py` (2026-10-22.1) | The multi-joint controller: 200 Hz MIT, tuned gains, gravity and friction feed-forward, tracking/torque/velocity/stall aborts, slow park. Since 2026-10-21 it is guard-aware: it refuses a real run without arm_guard, runs the guard pre-flight before enabling, and on a guard fault holds and waits for the operator (`stop` = reset + park, `off` via `runs/DEMO_OFF`). `--trajectory` runs `crackvision.joint_trajectory/1` files, `--gains` overrides gains for one run, `--simulate --guard-sim` rehearses |
| `run-trajectory.sh` | Operator wrapper: `--rehearse` (simulation), `--check` (read-only bus check), or the real run (enable, zero, hold, Enter, trajectory, fold, off) |
| `basic-demo.sh` | The show-demo wrapper. The show is refused by arm_guard: J1 −3.76 rad is beyond the URDF range |
| `first_move_j1.py` | The single-joint bring-up and tuning tool. It also holds the motor table, URDF limits, tuned gains (`TUNED`), freshness and pre-check helpers that `demo_move.py` imports |
| `gravity.py` | URDF gravity torques for the feed-forward |
| `fakebus.py` | Offline stand-in for motorbridge, used by the tests and simulations |
| `test_demo_move.py`, `test_guarded_runs.py`, `test_first_move_j1.py` | Offline tests (`env -u PYTHONPATH ~/miniconda3/envs/rebot/bin/python -m unittest test_guarded_runs test_demo_move`: 41 tests) |
| `make_trajectory.py`, `trajectories/` | `first_guarded_run.json` (generated) and `smoke_raise.json` (the repo's MOT-05.2 fixture, unchanged) |
| `tuned_gains.yaml` | The 2026-10-17 tuning result. J2/J3 kd is still 1.0 here; kd 4 was tested on 2026-10-22 and is not adopted yet (an operator decision) |
| `j1_reference.json` | J1's forward reading as re-referenced on 2026-10-22 (+6.2766 = one whole turn plus −0.007 rad). It is valid until J1 loses its turn count again |

Runs that used this code: `evidence/operator/first-guarded-run-2026-10-22/`.

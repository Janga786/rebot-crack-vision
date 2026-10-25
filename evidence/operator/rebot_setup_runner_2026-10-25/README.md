# Operator runner snapshot — `~/rebot_setup`, 2026-10-25

Copies of the files that changed in `~/rebot_setup` (not under version control) since the
[2026-10-23 snapshot](../rebot_setup_runner_2026-10-23/README.md). Checksums are in `SHA256SUMS`. The pre-change copy of
`demo_move.py` is in `~/rebot_setup/backup_pre_state_file_2026-10-22/demo_move.pre_ffbias.py`.

| File | What changed |
|---|---|
| `demo_move.py` (2026-10-25.1) | New `--ff-bias j2=1.4,j3=-0.6,j4=-0.2`: a per-run constant added to the feed-forward torque of the named arm joints, **only while the path moves** (fades in over the first second and out over the last; zero at the fold hold, in settle, park and final hold). Each value is capped at ±2 Nm; the 10 Nm feed-forward cap and arm_guard still apply. Recorded in the run summary as `ff_bias`. Added because the powered runs showed a steady, pose-dependent sag that a gravity-model offset explains (see [powered-show-runs-2026-10-23_25](../powered-show-runs-2026-10-23_25/README.md)) |
| `test_guarded_runs.py` | `FeedForwardBias` (4 tests: parsing and limits, the ramp, the exact offset, applied only during the path and recorded). Runner suite: 47 tests, all pass |

The `--state-file` option from 2026-10-23 is unchanged.

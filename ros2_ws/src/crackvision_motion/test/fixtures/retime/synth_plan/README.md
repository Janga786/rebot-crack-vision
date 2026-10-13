# MOT-07.8 fixture: a committed MOT-06.10 synthetic `cartesian_plan`

`cartesian_plan.json` is a **verbatim, unmodified** copy of the `plan_crack_path` output produced
by a real run of `scripts/ros/run_cartesian_synthetic.sh` (headless mock MoveIt stack, case
`synth_mot06`, seed 0) — nothing in it is hand-edited.

Run stamp (scratch root): `logs/cartesian_synthetic/20261013T052821Z/` (not committed; `logs/` is
scratch space per the script's own header).

Command: `bash scripts/ros/run_cartesian_synthetic.sh`, run on 2026-10-13 (UTC) from a clean
working tree at commit `7884d4a` (`git log -1 --format=%H` at copy time). Script output:

```
run_cartesian_synthetic.sh: cartesian candidates_tried=24 chosen={'direction': 'reverse', 'roll_offset_rad': 1.0471975511965976} coverage=36/36 fraction_by_length=1.000000
run_cartesian_synthetic.sh: timing (s): synth+paths+path3d=8 plan_view=1 plan_crack_path=160
run_cartesian_synthetic.sh: OK
```

`end_effector.yaml` and `b601_dm_limits.yaml` are verbatim copies of `config/robot/end_effector.yaml`
and `config/robot/b601_dm_limits.yaml` as they stood at that same run (and still stand at copy
time) — their sha256 equals `cartesian_plan.json`'s own `sources.end_effector.sha256` /
`sources.limits.sha256` exactly, so `retime_plan`/`trajectory_check` never refuse on a sha
mismatch when pointed at this fixture directory.

## shas

| file | sha256 |
|---|---|
| `cartesian_plan.json` | `e15e5c377a4f8ead681fe0ca22ec2a1832615859e4be3c2ea8a832b4c8615e98` |
| `end_effector.yaml` | `c8133145469e3662c501c5bfe60e7a8cc6b66be7d809b50fc02f3ad70e6bb877` |
| `b601_dm_limits.yaml` | `43f5e2e3a92bcae348b077bfc7d1dd223e555a28869789235813d4c2723aa82b` |

Plan's own recorded sources (`cartesian_plan.json`'s `sources.end_effector.sha256` /
`sources.limits.sha256`): `c8133145469e3662c501c5bfe60e7a8cc6b66be7d809b50fc02f3ad70e6bb877` /
`43f5e2e3a92bcae348b077bfc7d1dd223e555a28869789235813d4c2723aa82b` — equal to the above.

The plan has 5 legs: `transit` (22 pts), `approach` (8 pts), `trace` (21 pts), `retract` (9 pts),
`transit` (22 pts) — one `trace` leg, exercising `retime_plan`'s G-TOOLSPEED bisection and
`trajectory_check`'s C-TOOLSPEED/C-PATH/C-REPORT checks (§16.5).

Used by `scripts/ros/test_retime_crosscheck.sh` (MOT-07.8).

# OPS-01.3 — doctor stage: aggregate check_env/check_host/check_realsense into one operator report

```json card
{
  "kind": "impl",
  "depends_on": [
    "OPS-01.1"
  ],
  "requirements": [
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "scripts/operator_doctor.py",
      "tests/test_operator_doctor.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#19. Operator CLI workflow",
    "scripts/check_env.py",
    "scripts/check_host.py",
    "scripts/check_realsense.py",
    "docs/RUNBOOK.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_operator_doctor.py -v",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "help",
        "cmd": "./env.sh python scripts/operator_doctor.py --help",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "operator_doctor.py never reimplements a check: it runs scripts/check_env.py, scripts/check_host.py and scripts/check_realsense.py as subprocesses (each with --json where supported) and reads their exit code + JSON summary, exactly as §19 specifies.",
      "Adds exactly one new informational row (not delegated to a subprocess): whether ros2_ws/install/setup.bash exists, WARN (not FAIL) if missing, with a pointer to scripts/ros/build_ws.sh.",
      "Prints one combined human table plus a logs/operator_doctor_<stamp>.json and logs/operator_doctor_latest.json per §0.4, and exits 0 if nothing is FAIL, 3 otherwise -- never 1 or 2 except for its own usage errors.",
      "Unit tests stub the three subprocess calls (do not actually require a camera, a conda env rebuild, or ROS) and assert the aggregation/exit-code logic for an all-pass, a WARN-only, and a FAIL case."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 1,
    "context": 2,
    "consequence": 1,
    "task_class": "cli-aggregation",
    "local_ok": true
  },
  "id": "OPS-01.3",
  "title": "doctor stage: aggregate check_env/check_host/check_realsense into one operator report",
  "parent": "OPS-01",
  "outcome": "One command (`scripts/operator_doctor.py`) that runs the existing environment/host/camera diagnostics and the ROS overlay build check, and prints a single pass/warn/fail table with remediation pointers, per docs/INTERFACES.md §19's doctor definition."
}
```

Build `scripts/operator_doctor.py`, modeled on the existing `scripts/check_env.py`/`scripts/check_host.py` table+JSON pattern (reuse their row-rendering style for consistency, do not duplicate their check logic). It must:

1. Invoke `./env.sh`-relative calls are not needed here since this script itself runs inside `./env.sh python scripts/operator_doctor.py`; it subprocess-invokes the three check scripts with `sys.executable` and `--json` (check `check_realsense.py --json`'s and `check_host.py --json`'s existing flags; `check_env.py` also has `--json`).
2. Parse each one's JSON summary (fall back gracefully, with a clear WARN row, if a script doesn't emit valid JSON -- never crash the whole aggregation because one sub-check misbehaves).
3. Add the `ros2_ws/install/setup.bash` presence row described in the acceptance criteria.
4. Roll everything up to a single exit code: 0 if every row is PASS/WARN, 3 if any row is FAIL, following §0.2's precondition-not-met code (doctor never returns 1, since it performs no action that can itself fail at runtime beyond its subprocess calls, and never 2 except bad argv).
5. Write the §0.4 log artefacts.

In `tests/test_operator_doctor.py`, monkeypatch `subprocess.run`/`Popen` to return canned JSON for each of the three scripts and assert the aggregation and exit-code behaviour; do not require a real camera or conda rebuild to pass.

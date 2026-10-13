# MOT-08.4 — crackvision.approve_trajectory: re-verify preview + bindings, typed /dev/tty APPROVE <sha8>, write §11.5 approval

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.2"
  ],
  "requirements": [
    "REQ-OPS-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/approve_trajectory.py",
      "tests/test_approve_trajectory.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#11",
    "docs/adr/016-commissioning-gated-execution.md",
    "src/crackvision/preview_core.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_approve_trajectory.py -q -p no:cacheprovider",
        "timeout_s": 600,
        "expect_exit": 0
      },
      {
        "id": "no-bypass",
        "cmd": "bash -c '! grep -nE \"(--yes|--force|--non-interactive|sys\\.stdin|input\\()\" src/crackvision/approve_trajectory.py && grep -q \"/dev/tty\" src/crackvision/approve_trajectory.py'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "no-tty-refuses",
        "cmd": "bash -c 'setsid -w ./env.sh python -m crackvision.approve_trajectory --preview /nonexistent/preview.json --operator ci </dev/null; rc=$?; test $rc -eq 2 -o $rc -eq 3'",
        "timeout_s": 60,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Every §18 refusal condition is tested and gives exit 3 with nothing written: tampered trajectory, tampered PNG, tampered or failing check report, purpose test, stale limits/scene/end-effector binding, wrong phrase, no controlling tty, and an existing output file.",
      "The confirmation is read only from /dev/tty, opened directly. Tests inject the reader through a function parameter or monkeypatch and never through stdin. No flag or env var bypasses it.",
      "The written file has exactly the §11.5 keys. Feeding it to crackvision_motion.execution_gate.load_approval (loaded by file path through importlib in the test, without importing the package from the src tree) succeeds. preview_artifacts lists the manifest first and then every artefact, with repo-relative paths and correct shas.",
      "--dry-run performs every non-interactive check, never prompts, never writes, and exits 0. The default output path is data/approvals/{case}_{sha8}_approval.json."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 5,
    "task_class": "python-cli-safety",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.4",
  "title": "crackvision.approve_trajectory: re-verify preview + bindings, typed /dev/tty APPROVE <sha8>, write §11.5 approval",
  "parent": "MOT-08",
  "outcome": "`./env.sh python -m crackvision.approve_trajectory --preview PATH --operator ID` re-verifies the preview, trajectory, check report and config bindings, requires the typed /dev/tty phrase, and writes a five-key crackvision.execution_approval/1 file. Any other outcome is a refusal with exit 3 and no file written."
}
```

Implement src/crackvision/approve_trajectory.py per §18 (conda side).

Structure:
- `verify_for_approval(manifest_path, *, root, limits_path, scene_config_path, end_effector_config_path) -> dict`: pure. It raises `preview_core.PreviewRefusal` carrying every failing reason, collecting all of them rather than stopping at the first.
- `build_approval(verified, operator, now_utc) -> dict`: pure. It returns exactly `schema, trajectory_sha256, approved_by, approved_utc, preview_artifacts`.
- `confirm(sha8, reader) -> bool`. The default `reader` opens `/dev/tty` read/write, prints the trajectory path, purpose, duration at the preview speed scale and the manifest path, then reads one line. An OSError while opening means there is no tty, which is a refusal.
- `main(argv)`: §0.3 common flags plus `--preview` (required), `--operator` (required, non-empty), `--out`, `--limits`, `--scene-config`, `--end-effector-config`, `--execution-config`. Write atomically (temp file + rename). Refuse if the output exists.

Stale-binding rule: a non-null trajectory binding that differs from the current file's sha is a refusal. A null binding is allowed, with a printed warning, because real mode's G-STALE-CONFIG/G-LIMITS-HASH refuse it later anyway. Purpose `test` is refused. Approved purposes are crack_task and view_move.

Tests (tmp_path only): build a full synthetic case with preview_core plus fake PNG artefacts whose shas are recorded in a valid manifest. You do not need to call the renderer. Cover:
- the happy path with an injected reader returning `APPROVE <sha8>`;
- each refusal;
- the no-tty path, by monkeypatching the open function to raise OSError;
- the --dry-run path.
For the load_approval compatibility test, import ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py with importlib.util.spec_from_file_location. If it cannot load under the conda interpreter because of its imports, load joint_trajectory.py the same way first, or skip with an explicit reason and add a schema-shape assertion instead. Say which in the PR.

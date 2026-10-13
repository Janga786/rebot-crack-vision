# MOT-08.1 — Decision + contract: trajectory preview manifest, approval CLI and G-APPROVAL preview binding (ADR-020, INTERFACES §18)

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2",
    "REQ-OPS-1",
    "REQ-INT-2"
  ],
  "scope": {
    "write": [
      "docs/adr/020-trajectory-preview-approval.md",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/INTERFACES.md#17",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/adr/019-time-parameterization.md",
    "docs/motion/EXECUTION.md",
    "ros2_ws/src/crackvision_motion/crackvision_motion/execution_gate.py",
    "src/crackvision/trajectory_check.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "adr",
        "cmd": "test -s docs/adr/020-trajectory-preview-approval.md && grep -q 'Status:' docs/adr/020-trajectory-preview-approval.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "section",
        "cmd": "bash -c 'grep -qE \"^## 18\\. \" docs/INTERFACES.md && grep -q \"crackvision.trajectory_preview/1\" docs/INTERFACES.md && grep -q \"APPROVE <\" docs/INTERFACES.md && grep -nE \"^## (17|18)\\. \" docs/INTERFACES.md | awk -F: \"NR==1&&\\$2~/^## 17/{a=\\$1} NR==2&&\\$2~/^## 18/{b=\\$1} END{exit !(a&&b&&b>a)}\"'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§18 is appended after §17. §0–§17 are byte-identical to HEAD before the change. §11.5's five-key approval schema is not changed: any binding to the preview goes through preview_artifacts[] entries.",
      "§18 fully specifies crackvision.trajectory_preview/1. It covers every field and its type, the repo-relative path rules, the output directory and file names, the summary metric definitions (finite-difference velocity/acceleration exactly as §11.7 defines them), the preview speed scale source and the artefact list with sha256 values.",
      "The approve_trajectory contract lists every refusal condition (exit 3), the typed /dev/tty phrase and the rule that no flag, env var or stdin can substitute for it. It also fixes the default output path and --dry-run behaviour, and states that only crack_task and view_move trajectories can be approved.",
      "The G-APPROVAL amendment states what is checked, in what order (only after the existing sha + age checks pass) and its outcomes (warn in mock/dry, fail in real). It requires unreadable or malformed artefacts to become a gate outcome, never an exception.",
      "The RViz contract (preview_trajectory node, topics, QoS, launch, rviz config with no MotionPlanning display) is motion-free by construction: no ActionClient, no service clients, no publishers on joint_states or vendor cmd topics.",
      "The ADR records the alternatives it rejected (a --yes flag, approval inside execute_trajectory, RViz MotionPlanning panel) with reasons, plus the residual risk that an approval certifies only what the operator was shown."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 3,
    "context": 4,
    "consequence": 5,
    "task_class": "motion-contract",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.1",
  "title": "Decision + contract: trajectory preview manifest, approval CLI and G-APPROVAL preview binding (ADR-020, INTERFACES §18)",
  "parent": "MOT-08",
  "outcome": "ADR-020 and a new INTERFACES.md §18 define the crackvision.trajectory_preview/1 manifest and its rendered artefacts, the approve_trajectory CLI that writes the unchanged §11.5 crackvision.execution_approval/1 file, the RViz preview node, and a G-APPROVAL amendment that binds approvals to the reviewed preview artefacts."
}
```

Write docs/adr/020-trajectory-preview-approval.md (follow docs/adr/000-template.md) and append `## 18. Trajectory preview and operator approval (crackvision.trajectory_preview/1; amends §11.5/§11.7 G-APPROVAL)` to docs/INTERFACES.md. Never edit §0–§17. Implementer cards MOT-08.2–.9 will build to §18 verbatim, so be exact.

Decisions already taken by the domain lead. Record them, refine the details, but do not reverse them without saying why in the ADR:

1. **Two preview surfaces.** (a) A headless, deterministic PNG render plus a JSON manifest. This runs on the conda side as `./env.sh python -m crackvision.trajectory_preview`, uses matplotlib Agg and gets FK from crackvision.kinematics, so the operator sees a tool path computed independently of the retimer. This render is what the operator approves. (b) An optional RViz view: `ros2 run crackvision_motion preview_trajectory` publishes moveit_msgs/DisplayTrajectory on `/display_planned_path` and a visualization_msgs/MarkerArray on `/crackvision/preview_markers`, both TRANSIENT_LOCAL + RELIABLE. `preview_rviz.launch.py` = mock_planning.launch.py + rviz2 with `rviz/preview.rviz` + the node. The rviz config uses RobotModel, moveit_rviz_plugin/Trajectory, PlanningScene and MarkerArray only, and never the MotionPlanning display, because it has Plan/Execute buttons. The node never constructs an ActionClient or a service client, never publishes on `/joint_states`, `/rebotarm/**` or vendor cmd topics, and never calls a vendor service. RViz is a convenience and is not an approval artefact.

2. **Manifest `crackvision.trajectory_preview/1`**, written to `data/previews/{case}_{sha8}/preview.json`. `{case}` is the plan's case_id and `{sha8}` is the first 8 hex digits of the trajectory sha256. The PNGs sit next to it: `joints.png`, `toolpath.png`, `toolspeed.png`. Baseline fields (path values are repo-relative to --root):
   - `schema`, `created_utc`, `producer`, `case_id`.
   - `trajectory {path, sha256, purpose}`, `plan {path, sha256, schema}`.
   - `retime_report {path, sha256}`.
   - `check_report {path, sha256, all_pass, failed_checks[]}`. `all_pass` means every §16.5 outcome is pass or skip.
   - `bindings {limits_file_sha256, end_effector_config_sha256, scene_config_sha256, paths3d_sha256|null}`, copied from the trajectory.
   - `preview_speed_scale`: `execution.yaml.speed_scale.real_default`, so the operator sees the duration and tool speed real would actually run at.
   - `summary`, with these parts:
     - `n_points`, `duration_s`, `duration_scaled_s`.
     - `per_joint[6]`: `{joint, min_q, max_q, lower, upper, min_headroom_rad, max_abs_v, v_limit, v_ratio, max_abs_a, a_limit, a_ratio}`, finite differences per §11.7.
     - `tool`: `{path_length_m, max_speed_m_s, max_speed_scaled_m_s, min_z_m, legs[{leg_index, kind, point_range, duration_s, length_m, mean_speed_m_s}]}`. Legs come from the retime report's point_range.
   - `artifacts[{kind: joints|toolpath|toolspeed, path, sha256}]`.
   - `warnings[]`.
   Re-rendering from identical inputs must give byte-identical PNGs. Specify how: fixed DPI/size, `metadata={'Software': None}` or equivalent. Only `created_utc` in the manifest may differ.

3. **Preview refusal rules** (exit 3, nothing written):
   - The trajectory is not a valid §11.2 file.
   - The plan is not a cartesian_plan/1 or view_plan/1.
   - `retime_report.trajectory_sha256` differs from the trajectory's sha256.
   - `check_report.trajectory_sha256` differs from the trajectory's sha256.
   A failing check report does NOT refuse the preview: the operator should see why it fails. It is recorded as `all_pass: false` and drawn as a red banner on every PNG. Default report paths are `<trajectory>.retime.json` and `<trajectory>.check.json`.

4. **approve_trajectory**: `./env.sh python -m crackvision.approve_trajectory --preview PATH --operator ID [--out PATH] [--limits] [--scene-config] [--end-effector-config] [--execution-config] [§0.3]`.
   - It re-verifies everything at approval time. Each of the following refuses with exit 3:
     - The manifest is invalid.
     - The trajectory file's current sha256 differs from the manifest's.
     - Any artefact is missing or its sha256 differs.
     - The check report's file sha256 differs from the manifest's, or it does not name the trajectory sha, or any check is `fail`.
     - The trajectory purpose is `test`.
     - Any non-null binding differs from the current limits, scene or end-effector file (the same integrity rule as G-STALE-CONFIG and G-LIMITS-HASH).
   - Only after all of that does it open `/dev/tty` directly and ask for the exact phrase `APPROVE <sha8>` (after strip()). No tty, or a wrong phrase, gives exit 3. There is no --yes, no env override and no stdin path.
   - It writes exactly the five §11.5 keys. `approved_utc` is now. `preview_artifacts` is the manifest itself followed by every manifest artefact, each as a repo-relative path with its sha256.
   - The default output is `data/approvals/{case}_{sha8}_approval.json`, and an existing file is never overwritten (exit 3).
   - `--dry-run` runs every non-interactive check, prints the result, does not prompt, writes nothing and exits 0.

5. **G-APPROVAL amendment** (§11.7). After the existing sha-match and age checks pass, the gate also requires:
   - `preview_artifacts` is non-empty.
   - Every entry's path, resolved against --root, exists and its sha256 matches.
   - Exactly one entry parses as a crackvision.trajectory_preview/1 manifest, and that manifest has `trajectory.sha256` equal to the current trajectory sha and `check_report.all_pass == true`.
   Any violation is warn in mock/dry and fail in real, with a specific detail. Unreadable files, bad JSON or a UnicodeDecodeError become that outcome and never escape as an exception (see the MOT-05.3.R2 and MOT-07.10.2.R2 precedents). Approvals that already pass today stay valid only if they meet the new rule. Say this explicitly; no approvals exist yet.

6. Include a defaults table with rationale and a non-goals list. Non-goals: no video export, no approval revocation list (expiry is bounded by approval_max_age_s), no multi-operator sign-off.

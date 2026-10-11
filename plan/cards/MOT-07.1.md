# MOT-07.1 — Decision + contract: time parameterization and independent trajectory checker (ADR-019, INTERFACES §16)

```json card
{
  "kind": "decision",
  "requirements": [
    "REQ-MOT-1",
    "REQ-MOT-2"
  ],
  "scope": {
    "write": [
      "docs/adr/019-time-parameterization.md",
      "docs/INTERFACES.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#15",
    "docs/adr/016-commissioning-gated-execution.md",
    "docs/adr/017-cartesian-crack-path-planning.md",
    "docs/motion/CARTESIAN_PLANNING.md",
    "config/robot/b601_dm_limits.yaml",
    "config/robot/end_effector.yaml",
    "config/motion/execution.yaml",
    "ros2_ws/src/crackvision_motion/crackvision_motion/joint_trajectory.py",
    "ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py",
    "src/crackvision/kinematics.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "adr",
        "cmd": "test -s docs/adr/019-time-parameterization.md && grep -q 'Status:' docs/adr/019-time-parameterization.md",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "section",
        "cmd": "grep -qE '^## 16\\. ' docs/INTERFACES.md && grep -q 'crackvision.retime_config/1' docs/INTERFACES.md && grep -q 'crackvision.retime_report/1' docs/INTERFACES.md && grep -q 'crackvision.trajectory_check/1' docs/INTERFACES.md && grep -nE '^## (15|16)\\. ' docs/INTERFACES.md | awk -F: 'NR==1&&$2~/^## 15/{a=$1} NR==2&&$2~/^## 16/{b=$1} END{exit !(a&&b&&b>a)}'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "§16 is appended after §15 and §0–§15 are byte-identical to before; §11.2 is consumed unchanged (no new top-level trajectory keys). Any amendment is stated inside §16.",
      "The ADR records verified facts for rejecting off-the-shelf TOTG/Ruckig: moveit_core TOTG is C++ only with no Humble Python binding, ros-humble-ruckig has no Python module in either interpreter, TOTG blends corners off the checked path, and Ruckig is point-to-point. It also records why the chosen method fits G-LIMITS' finite-difference semantics and the vendor driver's linear interpolation.",
      "Output guarantees are normative and testable. Every plan configuration is an output point, and every other output point linearly interpolates between consecutive plan configurations. The joint-step and sample-period bounds hold. Finite-difference velocity ≤ f_v·limit and acceleration ≤ f_a·limit, using exactly the §11.7 definitions. The rest-to-rest boundary rule holds. The §11.6 margin rule passes, using execution.yaml's position_margin_rad. No explicit velocities/accelerations are written. A self-check with joint_trajectory.check_limits passes, otherwise the run is refused: never clamped and never silently slowed.",
      "Constant tool speed on trace legs is defined precisely. It specifies the per-segment tool speed (tool_tip chord / Δt), the per-leg constant v_c ≤ target, the ±tolerance band outside the ramps, the definition of a ramp and its bound, the never-above-band rule, and refusal below min speed. The contract states that execute_trajectory's speed_scale s multiplies the effective tool speed by s.",
      "The checker contract requires a separate implementation: a different interpreter (conda), no import of crackvision_motion, and FK from crackvision.kinematics. It derives leg membership geometrically and checks provenance, limits, margin, rest, sampling, path fidelity, tool speed and report consistency. The report schema and exit codes 0/2/3 are specified.",
      "Trajectory field mapping from the plan (producer, purpose, limits_file, end_effector/scene shas, source.paths3d verbatim), the refusal conditions, the CLIs (§0 flags, defaults, --dry-run) and a defaults table with rationale are all specified. The view-plan / §11.2 purpose gap is decided and recorded explicitly."
    ]
  },
  "routing": {
    "complexity": 4,
    "ambiguity": 4,
    "context": 4,
    "consequence": 5,
    "task_class": "motion-contract",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-07.1",
  "title": "Decision + contract: time parameterization and independent trajectory checker (ADR-019, INTERFACES §16)",
  "parent": "MOT-07",
  "outcome": "ADR-019 and an appended INTERFACES §16 fix the retiming algorithm and its output guarantees (exact plan-polyline following, finite-difference vel/acc within limit fractions, rest-to-rest, §11.6 margin, sample period/step bounds, constant tool speed on trace legs), the crackvision.retime_config/1, crackvision.retime_report/1 and crackvision.trajectory_check/1 schemas, both CLIs and their exit codes, consistent with §11 and §13/§15."
}
```

Write ADR-019 using the template docs/adr/000-template.md, then APPEND `## 16. Time parameterization and independent trajectory check (`crackvision.retime_config/1`, `crackvision.retime_report/1`, `crackvision.trajectory_check/1`)` to docs/INTERFACES.md. Do not edit §0–§15.

The design below is the lead's recommendation. Adopt it unless a fact you verify forces a change, and record any change in the ADR.

**Facts to verify and cite in the ADR (read-only):**
- `ls /opt/ros/humble/lib/python3.10/site-packages | grep -i moveit` shows only moveit_configs_utils, so there is no moveit_py and TOTG is not callable from Python.
- `dpkg -l ros-humble-ruckig` is installed (0.9.2), but `python3 -c 'import ruckig'` fails under both /usr/bin/python3 and ./env.sh python.
- §11.6/§11.7: G-LIMITS checks finite-difference (FD) velocity/acceleration, and the vendor driver interpolates each segment linearly (ADR-016, ros_actions.py:147-184), so the FD values are what the arm is commanded.
- §11.12 cites a 20 ms driver control tick.
- In the MOT-06.10 synthetic plan (logs/cartesian_synthetic/*/cartesian_plan.json), transit legs step 0.1 rad per configuration and trace legs about 0.02 rad.

**Algorithm (pure Python + numpy, ROS-free, in crackvision_motion; system python3):**
1. **Load and concatenate.** Load the plan with `cartesian_plan.load_plan`. Concatenate `legs[].positions` in `leg_index` order, dropping each leg's first configuration when it is within `cartesian_plan.CONTINUITY_TOL_RAD` of the previous leg's last. The result is the plan polyline. Plan configurations are copied bit-exact.
2. **Refine.** Subdivide every polyline segment linearly into equal sub-steps so that no per-joint step exceeds `sampling.max_joint_step_rad`. There is no corner blending: the arm follows the checked geometric path exactly. Slowing at corners is the accepted cost; document it as ADR-019's main trade-off against TOTG's `path_tolerance` blending.
3. **Discrete forward–backward pass over segment path speeds `u_i`.** Per-joint segment velocity is `v_ij = u_i·d_ij`, with segment duration `τ_i = h_i/u_i`. Constraints:
   - (V) `|v_ij| ≤ f_v·V_j`, plus external caps.
   - (A) at each interior point k, `|v_kj − v_(k−1)j| ≤ f_a·A_j·(τ_(k−1)+τ_k)/2`. This is exactly §11.7's FD acceleration.
   - (R) rest boundary: `|v_0j| ≤ f_a·A_j·τ_0` and `|v_(N−1)j| ≤ f_a·A_j·τ_(N−1)`.
   - (M) §11.6 FD margin rule: at a near-bound point, the toward-bound speed must not increase.
   Then subdivide any segment with `τ_i > sampling.max_period_s` and re-profile until none remains, with a bounded iteration count. Rationale for max_period_s = 0.02 s: FD acceleration only reflects the physical velocity step at a point if the sample period is about the driver tick.
4. **Constant tool speed on trace legs.** Tool speed of a segment is `‖p_tool(q_(i+1)) − p_tool(q_i)‖/τ_i`, with `p_tool` = tool_tip FK. Each trace leg gets a single `v_c ≤ trace.tool_speed_m_s`: the largest value (bisection) for which the profiled leg satisfies all of the following:
   - every segment's tool speed ≤ `v_c(1+tol)`;
   - every segment outside the leading and trailing ramps is within `[v_c(1−tol), v_c(1+tol)]`. A ramp is the maximal prefix/suffix of segments below `v_c(1−tol)`;
   - each ramp's tool arc length is ≤ `trace.max_ramp_m`.
   If `v_c < trace.min_tool_speed_m_s`, refuse. Tool-speed caps feed step 3 as external caps.
5. **Self-check.** Run `joint_trajectory.check_limits` at full limits with execution.yaml `position_margin_rad`. Any violation means refuse (exit 3). Never clamp and never apply a hidden uniform slowdown.
6. **Output.** Write positions + `t_s` only, with no explicit velocities/accelerations. Rationale: the vendor driver's linear interpolation commands the FD values, and explicit fields would be a second, unchecked truth. Time-optimality is not guaranteed. State that the method is near-optimal: a single straight move must come within 10% of the analytic trapezoid/triangle at `f_v`/`f_a`.

**Tool FK in the retimer.** Use a new pure-numpy `crackvision_motion.tool_fk`:
- URDF chain base_link→gripper_link, default `presentation/sim/reBot_B601_DM_with_gripper.urdf` (MOT-01: byte-identical to the vendor URDF), composed with the end_effector.yaml `tool` block via `crackvision_description.end_effector`.
- Parity is tested against crackvision.kinematics.

**§16 contents:**
- **§16.1 `config/motion/retime.yaml`** (`crackvision.retime_config/1`, strict loader, unknown key = exit 2). Recommended defaults:
  - `limit_fraction.velocity` 0.95, `limit_fraction.acceleration` 0.95;
  - `sampling.max_joint_step_rad` 0.01, `sampling.max_period_s` 0.02;
  - `trace.tool_speed_m_s` 0.05 (unscaled; 5 mm/s at the real 0.10 cap);
  - `trace.min_tool_speed_m_s` 0.005, `trace.speed_tolerance` 0.05 (relative), `trace.max_ramp_m` 0.005;
  - `value_status: nominal`, plus `sources`.
  `position_margin_rad` is read from execution.yaml and is never duplicated.
- **§16.2** The algorithm and the normative output guarantees G1…Gn above.
- **§16.3 §11.2 field mapping.**
  - `producer` = `retime_plan`.
  - `purpose` = `crack_task` by default, or `--purpose test`.
  - `limits_file` = path + sha of the limits file used. Refuse if that sha ≠ `plan.sources.limits.sha256`.
  - `end_effector_config_sha256` = `plan.sources.end_effector.sha256`. Refuse if the end_effector file used for FK has a different sha.
  - `scene_config_sha256` = `plan.sources.scene.sha256`, copied and not checked: G-STALE-CONFIG checks it at execution.
  - `source.paths3d` = `plan.sources.paths3d`, verbatim.
  - Other refusals: invalid plan, a plan with no legs, a trace `v_c` below the minimum, a self-check violation.
- **§16.4 `crackvision.retime_report/1`** sidecar at `<trajectory>.retime.json`:
  - shas of plan, limits, end_effector, URDF, retime config and execution config;
  - parameters echo;
  - per leg: `{leg_index, kind, point_range [first,last], duration_s}`, plus for trace legs `tool_speed {target, constant, ramp_in_m, ramp_out_m, min, max}`;
  - totals: n_points, duration_s, per-joint peak FD vel/acc as a fraction of limit;
  - `self_check {violations: 0}`.
- **§16.5 `crackvision.trajectory_check/1`** — the independent checker `crackvision.trajectory_check`. It runs in the conda env, must not import crackvision_motion, and gets FK from crackvision.kinematics. Inputs: trajectory, plan, limits, end_effector, execution config, retime config, optional report. Checks:
  - C-SCHEMA;
  - C-PROVENANCE: trajectory shas vs plan and vs the files given; `source.paths3d` equal to the plan's;
  - C-LIMITS: FD velocity/acceleration at full limits, and at f_v/f_a reported as utilisation;
  - C-MARGIN: §11.6 FD rule;
  - C-REST;
  - C-SAMPLING;
  - C-PATH: every point on the plan polyline within 1e-6 rad; monotone progress; every plan configuration present; start = plan first, end = plan last;
  - C-TOOLSPEED: legs derived from the geometric projection; band, ramp and never-above rules vs the target;
  - C-REPORT: the report's declared `v_c`, ranges and durations agree with what the checker recomputes.
  Report fields: per-check outcome (pass/fail) + detail + numbers. Exit 0 = all pass, 3 = any fail, 2 = usage/config error, 1 = runtime error.
- **§16.6 CLIs.**
  - `ros2 run crackvision_motion retime_plan --plan P [--out PATH (default data/trajectories/{case}_trajectory.json)] [--purpose crack_task|test] [--retime-config] [--limits] [--end-effector-config] [--execution-config] [--urdf] [§0.3 flags]`. It never imports rclpy, so it is also runnable as `bash scripts/ros/env_ros.sh python3 -m crackvision_motion.retime_plan`. `--dry-run` validates inputs and prints the leg summary without writing.
  - `./env.sh python -m crackvision.trajectory_check --trajectory T --plan P [--report R] [...] [--out PATH]`.
  - Exit codes per §0.2.
- **§16.7** Defaults table with rationale.
- **§16.8** Decide and record the view-plan gap. §13.14 `transit_leg` (home → camera view pose) also needs retiming. §11.2 `purpose` allows only `crack_task` (which requires `source.paths3d`) or `test` (refused in real), so a view move cannot run in real today. Recommendation: §16 specifies only cartesian_plan retiming, names the gap, and proposes a `purpose: "view_move"` amendment (with its G-TRAJ/G-ELIGIBLE rules) for card MOT-07.10 to settle. Do not change execution_gate/joint_trajectory behaviour in this card.

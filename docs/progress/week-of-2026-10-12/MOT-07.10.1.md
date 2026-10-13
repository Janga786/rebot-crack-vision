# MOT-07.10.1 — Decide and append the view_move purpose contract (ADR-019 addendum + INTERFACES §17)

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`2b7dcd7`](https://github.com/Janga786/rebot-crack-vision/commit/2b7dcd7847fe29a0e4738323fe0588aeb9651360)

## What was done

Settled the long-standing "view_move purpose" design gap for the robot motion stack by writing a formal decision (ADR-019 addendum) and a detailed specification (INTERFACES.md §17) for how a camera-positioning move can be safely executed on the real robot. This unblocks a future implementation card to let the robot move its camera into position using the exact same safety gates (e-stop, commissioning, collision checks) as crack-inspection moves, without weakening any of them. Verified: all 3 required checks pass, including the existing contract test suite (5/5 tests), and confirmed via diff that no existing documentation text was altered, only appended.

After the first audit, a repair round (MOT-07.10.1.R1) fixed the reported issues: Closed a documentation inconsistency in the motion-planning interface spec: the view-move retime report now correctly documents that it carries through the view plan's own leg index instead of a hard-coded placeholder, and defines the point range using the already-established inclusive convention instead of an undefined variable. All three automated checks (section structure, ADR addendum, and the contract test suite) pass.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/2b7dcd7847fe29a0e4738323fe0588aeb9651360/docs/INTERFACES.md) (+189 / −4)
- [docs/adr/019-time-parameterization.md](https://github.com/Janga786/rebot-crack-vision/blob/2b7dcd7847fe29a0e4738323fe0588aeb9651360/docs/adr/019-time-parameterization.md) (+88 / −0)

Commits: [`6ce6613`](https://github.com/Janga786/rebot-crack-vision/commit/6ce66134bc375bac7fd117d5db19ecb02f7c6091), [`2b7dcd7`](https://github.com/Janga786/rebot-crack-vision/commit/2b7dcd7847fe29a0e4738323fe0588aeb9651360)

## Why it was done

ADR-019 has a MOT-07.10 addendum and docs/INTERFACES.md has a new appended §17. Together they settle §16.8: `purpose: "view_move"`, its `source.view_plan` binding, its G-TRAJ/G-ELIGIBLE rules, and how retime_plan and trajectory_check handle a view_plan.

- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery

## How it moves the project forward

- Motion planning: **34/57** tasks accepted; whole project: **88/144**.
- REQ-MOT-2: 21/38 contributing tasks done
- REQ-OPS-1: 2/11 contributing tasks done
- Verification: automated checks run by the pipeline itself (section17 ✔, adr-addendum ✔, contracts ✔); independent audit accepted it (criteria: 8 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Commit 2b7dcd7 changes only docs/INTERFACES.md. Both hunks are inside §17 (lines 2947+ and 2995+), so §1–§16 are untouched. §17.3's report bullet now gives the single legs[] entry as `{leg_index: transit_leg.leg_index, kind: "transit", point_range: [0, n_points − 1], duration_s}`, carried through from the view plan unchanged, with the range inclusive at both ends per §16.4. §17.4 C-REPORT gained a clause saying §16.5's C-REPORT never compares leg_index. I checked that clause against the §16.5 table: C-REPORT compares only point_range, duration_s and tool_speed, so the claus
…[921 chars clipped]”
- Quality loop: the audit found 1 issue(s) that were fixed before acceptance (repair task MOT-07.10.1.R1): §17.3 report legs[] entry hard-codes leg_index 0 (card says carry the transit_leg's leg_index) and uses an undefined N in point_range [0, N].

## What it unlocks next

- **Ready to start:** MOT-07.10.2 — Accept view_move in joint_trajectory and execution_gate (G-TRAJ, G-ELIGIBLE) with gate regression
- **Ready to start:** MOT-07.10.4 — trajectory_check: accept a view_plan and check view_move trajectories independently
- Closer: MOT-07.10.3 — retime_plan --view-plan: retime the view_plan transit leg into a view_move trajectory (still needs MOT-07.10.2)

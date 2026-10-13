# MOT-08.1 — Decision + contract: trajectory preview manifest, approval CLI and G-APPROVAL preview binding (ADR-020, INTERFACES §18)

**Accepted:** 2026-10-13 · **Work package:** Motion planning (L-MOTION) · **Track:** simulation · **Verified commit:** [`aedea4e`](https://github.com/Janga786/rebot-crack-vision/commit/aedea4e957807c39f1cfc85d374ef3dfe630c027)

## What was done

Defined the full written contract (ADR-020 plus a new §18 in INTERFACES.md) for how a planned robot trajectory gets previewed and approved before it can run on the real arm: a rendered preview (JSON + 3 plots) that an operator reviews, a CLI that records their typed approval tied to that exact preview, and a safety-gate rule that checks the approval still matches the preview at run time. This is a specification-only step — verified that the two required automated checks pass and that no earlier contract text was altered — with the actual preview/approval tools to be built against it next.

After the first audit, a repair round (MOT-08.1.R1) fixed the reported issues: Fixed a documented contradiction in the trajectory-approval design docs: the design-decision record and the formal interface spec now agree that the approval tool must refuse any trajectory with a failing safety check, closing a loophole that could have let a failing robot motion get approved. Also tightened how the approval record stores the file path of the reviewed preview so it resolves correctly regardless of the operator's working directory. Both automated contract checks (document structure and section ordering) pass.

Files changed:
- [docs/INTERFACES.md](https://github.com/Janga786/rebot-crack-vision/blob/aedea4e957807c39f1cfc85d374ef3dfe630c027/docs/INTERFACES.md) (+303 / −1)
- [docs/adr/020-trajectory-preview-approval.md](https://github.com/Janga786/rebot-crack-vision/blob/aedea4e957807c39f1cfc85d374ef3dfe630c027/docs/adr/020-trajectory-preview-approval.md) (+138 / −7)

Commits: [`b399390`](https://github.com/Janga786/rebot-crack-vision/commit/b399390aaac211ca16adbc4dccd915b31e3c2a38), [`aedea4e`](https://github.com/Janga786/rebot-crack-vision/commit/aedea4e957807c39f1cfc85d374ef3dfe630c027)

## Why it was done

ADR-020 and a new INTERFACES.md §18 define the crackvision.trajectory_preview/1 manifest and its rendered artefacts, the approve_trajectory CLI that writes the unchanged §11.5 crackvision.execution_approval/1 file, the RViz preview node, and a G-APPROVAL amendment that binds approvals to the reviewed preview artefacts.

- Requirement **REQ-MOT-1**: MoveIt integration with the real B601-DM model: limits, controllers, collision scene
- Requirement **REQ-MOT-2**: Reachability, Cartesian paths along cracks, time parameterization within vel/acc limits
- Requirement **REQ-OPS-1**: Operator workflow: clean launch → capture → plan → preview → controlled execution, with failure recovery
- Requirement **REQ-INT-2**: Separately gated hardware commissioning with verified limits, collision checks, e-stop, controlled conditions and operator-sourced evidence

## How it moves the project forward

- Motion planning: **38/66** tasks accepted; whole project: **92/156**.
- REQ-MOT-1: 26/45 contributing tasks done
- REQ-MOT-2: 25/42 contributing tasks done
- REQ-OPS-1: 6/19 contributing tasks done
- REQ-INT-2: 9/20 contributing tasks done
- Verification: automated checks run by the pipeline itself (adr ✔, section ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “Accepted. Both MOT-08.1 findings are fixed. The ADR-020 residual-risk paragraph now says approve_trajectory refuses any `fail` outcome under §18.2 check 4, and it no longer refers to a `warn`-only check-report outcome. The §18.2 manifest entry in preview_artifacts[] is now normalised by the §11.2 convention, which matches how §18.3 check 2 resolves paths against --root. The commit changes only one line in §18 and one paragraph in the ADR. §0–§17 are untouched and the original MOT-08.1 content is still in place. Both scheduler checks pass. One optional minor wording issue remains in the ADR.”
- Quality loop: the audit found 2 issue(s) that were fixed before acceptance (repair task MOT-08.1.R1): ADR-020 contradicts §18.2 on whether approve_trajectory refuses a failing check report; Manifest entry in preview_artifacts uses the raw --preview string, not a repo-relative path.

## What it unlocks next

- **Ready to start:** MOT-08.2 — Preview core: bind trajectory/plan/retime/check inputs, FK tool path, summary metrics and trajectory_preview/1 manifest (pure, no plotting)
- **Ready to start:** MOT-08.5 — execution_gate G-APPROVAL: verify preview_artifacts and the bound trajectory_preview/1 manifest (§18 amendment)
- **Ready to start:** MOT-08.6 — preview_trajectory ROS node: DisplayTrajectory + tool-path/crack MarkerArray, latched, motion-free

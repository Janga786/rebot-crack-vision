# HOST-04 — Arm USB adapter access via least-privilege udev rule

**Accepted:** 2026-10-11 · **Work package:** Host platform & reproducibility (L-HOST) · **Track:** physical · **Verified commit:** [`124ce75`](https://github.com/Janga786/rebot-crack-vision/commit/124ce75772fc77230e2e4ae1d5a55c3864ce28b5)

## What was done

The B601-DM arm's USB-to-CAN adapter now has its own least-privilege udev rule (matched by its specific USB vendor/product ID) installed on the workstation, replacing the operator's manual chmod 666 workaround. The rule, its design rationale, and rollback steps are documented in docs/host/DEVICE_ACCESS.md, and all three automated verification checks for the rule's installation, scope, and documentation pass.

Files changed:
- [docs/host/DEVICE_ACCESS.md](https://github.com/Janga786/rebot-crack-vision/blob/124ce75772fc77230e2e4ae1d5a55c3864ce28b5/docs/host/DEVICE_ACCESS.md) (+121 / −0)
- [evidence/operator/HOST-04/OPERATOR_EVIDENCE.md](https://github.com/Janga786/rebot-crack-vision/blob/124ce75772fc77230e2e4ae1d5a55c3864ce28b5/evidence/operator/HOST-04/OPERATOR_EVIDENCE.md) (+1 / −0)
- [evidence/operator/HOST-04/arm_adapter_usb_ids_2026-10-10.md](https://github.com/Janga786/rebot-crack-vision/blob/124ce75772fc77230e2e4ae1d5a55c3864ce28b5/evidence/operator/HOST-04/arm_adapter_usb_ids_2026-10-10.md) (+48 / −0)
- [host/udev/99-rebot-arm.rules](https://github.com/Janga786/rebot-crack-vision/blob/124ce75772fc77230e2e4ae1d5a55c3864ce28b5/host/udev/99-rebot-arm.rules) (+30 / −0)

Commits: [`9de6892`](https://github.com/Janga786/rebot-crack-vision/commit/9de6892f5781f6666a69d5b7aa92d52a86231581), [`124ce75`](https://github.com/Janga786/rebot-crack-vision/commit/124ce75772fc77230e2e4ae1d5a55c3864ce28b5)

## Why it was done

The B601-DM USB adapter is accessible to the user through a VID:PID-specific udev rule (group plugdev), not broad group changes.

- Requirement **REQ-HOST-3**: Device access (RealSense, arm adapter) granted by reviewed least-privilege actions

## How it moves the project forward

- Host platform & reproducibility: **9/10** tasks accepted; whole project: **68/108**.
- REQ-HOST-3: 1/2 contributing tasks done
- Verification: automated checks run by the pipeline itself (installed ✔, least-privilege ✔, documented ✔); independent audit accepted it (criteria: 5 passed, 0 failed, 0 not verifiable without hardware).
- Audit summary: “I'm accepting HOST-04: every criterion holds and nothing needs fixing. I re-ran the three checks and all exit 0. The installed rule is byte-identical to the repo copy. The rule file has one active line with all the required parts and none of the forbidden ones. The commit only adds lines to DEVICE_ACCESS.md, so the D405 section is unchanged. The privileged path was used exactly twice, one request at a time, each approved before it ran: the install with the absolute repo path and a cmp check, then udevadm control --reload-rules with a test -f check. There was no trigger, no 
…[50 chars clipped]”

## What it unlocks next

- Closer: MOT-09 — Arm bring-up commissioning (operator) (still needs MOT-05, MOT-11)

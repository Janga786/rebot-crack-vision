# MOT-07.10.1.R1 — Repair MOT-07.10.1: §17.3 report legs[] entry hard-codes leg_index 0 (card says carry the

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "docs/INTERFACES.md",
      "docs/adr/019-time-parameterization.md"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#11",
    "docs/INTERFACES.md#13",
    "docs/INTERFACES.md#16",
    "docs/adr/019-time-parameterization.md",
    "docs/adr/016-commissioning-gated-execution.md"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "section17",
        "cmd": "bash -c 'grep -q \"^## 17\\. \" docs/INTERFACES.md && for s in 17.1 17.2 17.3 17.4 17.5; do grep -q \"^### $s \" docs/INTERFACES.md || { echo missing $s; exit 1; }; done && grep -q view_move docs/INTERFACES.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "adr-addendum",
        "cmd": "bash -c 'grep -q \"^## Addendum (MOT-07.10\" docs/adr/019-time-parameterization.md && grep -q view_move docs/adr/019-time-parameterization.md'",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "contracts",
        "cmd": "./env.sh pytest tests/test_contracts.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "Change §17.3's report bullet so the single legs[] entry carries `transit_leg.leg_index`, as the card says (or state normatively that transit_leg.leg_index must be 0 and is rejected otherwise, and make §17.4 C-REPORT consistent with that). Define point_range as `[0, n_points − 1]`, inclusive per §16.4. Leave §1–§16 untouched.",
      "All acceptance criteria of MOT-07.10.1 still hold at the new revision"
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
  "priority": 91,
  "repairs": "MOT-07.10.1",
  "findings": [
    "c6d33b2ec95a10b8"
  ],
  "id": "MOT-07.10.1.R1",
  "title": "Repair MOT-07.10.1: §17.3 report legs[] entry hard-codes leg_index 0 (card says carry the",
  "parent": "MOT-07.10",
  "outcome": "Resolve the review findings on MOT-07.10.1 while every acceptance criterion of MOT-07.10.1 still holds."
}
```

Repair work generated deterministically from review attempt `00960-MOT-07.10.1-review` of `MOT-07.10.1`.
Original card: `plan/cards/MOT-07.10.1.md` — its acceptance criteria must still hold.

## Finding 1 [minor] §17.3 report legs[] entry hard-codes leg_index 0 (card says carry the transit_leg's leg_index) and uses an undefined N in point_range [0, N]
- Evidence: docs/INTERFACES.md §17.3: "`legs[]` has exactly one entry, `{leg_index: 0, kind: \"transit\", point_range: [0, N], duration_s}`". The card body says: "`legs[]` has one transit entry carrying the transit_leg's `leg_index`". In ros2_ws/src/crackvision_motion/crackvision_motion/cartesian_plan.py, `_validate_leg_doc`, called for transit_leg by validate_view_plan at about line 901, only enforces `_as_nonneg_int(v["leg_index"], ...)`. It does not require 0, so a valid view plan can have transit_leg.leg_index != 0. retime_core builds its spans from `leg["leg_index"]` (retime_core.py:213, 230). Separately, N is defined nowhere in §17. §16.4 says point_range is inclusive at both ends, so the correct bound is n_points−1. Reading N as the point count, which the --dry-run bullet's "point count" suggests, gives an off-by-one range.
- Consequence: The downstream retime_plan and trajectory_check cards have to interpret this rule. Take a view plan whose transit_leg.leg_index is not 0. A retimer that follows §17.3 literally writes leg_index 0, while a checker that keys its geometric leg assignment on the plan's leg_index, as C-REPORT does for cartesian plans, expects the plan's own value, so C-REPORT fails or the two disagree. An undefined N can give a point_range that is out of range by one, which C-REPORT checks with zero tolerance.
- Affected: docs/INTERFACES.md
- Acceptance condition: Change §17.3's report bullet so the single legs[] entry carries `transit_leg.leg_index`, as the card says (or state normatively that transit_leg.leg_index must be 0 and is rejected otherwise, and make §17.4 C-REPORT consistent with that). Define point_range as `[0, n_points − 1]`, inclusive per §16.4. Leave §1–§16 untouched.

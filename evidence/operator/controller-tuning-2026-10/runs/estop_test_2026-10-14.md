# E-stop test record, 2026-10-14 (MOT-09 steps B5 and C3)

- Date / time (local): 2026-10-14 (MDT); times filled in per step below
- Operator: boosterk1, at the arm. Steps run by the Claude Code session on the operator's "go" in chat.
- Device relied on: [ ] inline latching e-stop in the 24 V line — make/model, rating: ____
                    [x] 24 V supply switch (interim, joint1-only steps; decision recorded below)
- Where it is mounted / distance from the operator position:
- Decision note (if the 24 V switch is the interim stop): operator, in chat on 2026-10-14: "I will just flip the switch off for the E-stop." No inline e-stop is installed.

## B5 — unloaded (all motors DISABLED)
- Pressed at ____ → scan result (expect "0 motor(s) found"): ____
- Released at ____ → scan result (expect 7/7, all DISABLED): ____
- `first_move_j1.py --check-only` afterwards (exit code, max |q|): ____
- Anything unexpected:

## C3 — powered (motor 1 enabled and holding, `first_move_j1.py --estop-test`)
- Script output line (expect "E-STOP OBSERVED: [stale] ..." or "[bus]"): ____
- Time from press to "feedback lost" (from the CSV: last fresh row vs your press): ____
- Did joint1 move when power dropped? (expected: no, it carries no gravity load): ____
- After release: scan 7/7? `--check-only` exit 0? ____
- Files: runs/estop_test_j1_<stamp>.csv / .json

## Verdict
- [ ] B5 passed   [ ] C3 passed   → only then: `claude-auto hw declare arm_powered_estop_verified --note "..."`

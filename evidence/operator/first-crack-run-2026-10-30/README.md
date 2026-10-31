# First crack run on the powered arm, case `print01`, 2026-10-30

Archived 2026-10-31T00:50:37Z by `scripts/crack_pipeline.sh archive print01` (copies; the originals are untouched). Every file
and its sha256 is in `MANIFEST.json` and `SHA256SUMS`.

| | |
|---|---|
| Case | `print01` |
| Session | 2026-10-30T18:56:36Z to 2026-10-30T19:00:52Z, slow 2, DEMO_EXTRA `--gains j2=100/4,j3=80/4`, outcome **failed_at_scan** |
| Preflight | `preflight_20261030T184653Z.json`: GO |
| Runner | demo_move.py 2026-10-29.4; sha256 run-trajectory.sh `699a2d69f7985f5a`, demo_move.py `5352950a792d4681` |
| Guard approval | Janga786 (2026-10-29T21:01:59+00:00) |
| Arm steps | search: completed, scan: aborted_parked |
| Search | found at stop 1, centre [0.3167, -0.0407, -0.0108] |
| Scan / crack gate | no component table |
| Trace plan | no plan |

## What happened (filled in from the logs and the chat, 2026-10-30; Claude drove the session at the operator's request)

- Print #1, matte, taped flat on the table, a little over 25 cm ahead of the base, long side left to right.
- Bench, hand-held before the arm run: the crack was found every time (254 mm / 2.9 mm wide / mean p 0.93 at 23 cm;
  195 mm at 32 cm) but rejected by the flatness gate (ring plane residual 4.3 and 3.2 mm vs the 3.0 mm limit): the sheet
  bowed in the hand (+4 / -4 mm along the crack). D405 noise on flat patches at 32 cm measured ~0.4 mm (40 px tiles),
  so the 3 mm limit was kept. On the table the gate accepted the crack at every stop that saw it.
- Read-only check: all 7 motors read, J1 +0.044 rad (reference known), guard pre-flight OK.
- First session start (18:52Z): the arm enabled and held at the fold, but the background capture found no camera:
  the D405 had dropped off USB at 12:48:51 local (kernel log), two minutes after the preflight saw it. `stop` typed
  at the hold -> parked, off, never left the fold (traj_search_20261030T185229Z, aborted_parked).
- Camera reseated. Second session (18:56Z): **search completed with no guard fault**. Stops 0-3 all found the crack
  (134-149 mm, p 0.89-0.90, accepted); stop 1 chosen: centre [0.3167, -0.0407, -0.0108] m, extent 0.151 m.
  Stop 4 was skipped ("the arm left the scan pose without settling"), stop 5 never captured: the camera dropped off
  USB again at 12:59:55 local, during the return to the fold. The cable is pulled by wrist motion: the plug (Micro-B
  at the camera) needs to be seated and the cable strain-relieved to the forearm before the next run.
- scan-plan from the search: completed (rehearsal). Scan (19:00Z): the arm enabled and held at the fold; the capture
  again reported no camera -> `stop` -> parked, off, never left the fold (traj_scan_20261030T190026Z, aborted_parked).
  The session stopped there; plan and trace did not run. No REFUSED, no GUARD FAULT, no 24 V cut.
- kd 4 override used (DEMO_EXTRA `--gains j2=100/4,j3=80/4`), x2 slower.
- **Tracking during the search** (runner_runs/traj_search_20261030T185643Z.json, max |ref - measured|):
  J1 8, **J2 89**, J3 21, J4 20, J5 4, J6 5 mrad. J2 came within 11 mrad of the 100 mrad tracking abort: this is the
  gravity sag the fitted model predicted (62 mrad predicted for the search, docs/motion/GRAVITY_MODEL.md); the trace
  reaches further (77 mrad predicted). Run the gravity survey and refit before the trace, or expect a tracking abort.
- Not seen today (no trace): trace offset, hover height, wobble.

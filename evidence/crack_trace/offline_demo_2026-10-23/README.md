# Offline crack-trace demo — 2026-10-23

Output of `bash scripts/crack_pipeline.sh demo`, with no hardware (case `demo_20261023T051035Z`):

1. `scan.json`: the scan pose, camera 25 cm above (0.30, 0, 0), held 15 s.
2. A rendered D405 view of crack test print #1 (Wikimedia Commons "Crack (2136639056).jpg", see
   `tools/make_crack_targets.py` for credits), printed 20 cm wide and lying on the table. Intrinsics and depth scale
   come from the real D405 recording of 2026-10-22.
3. demo_move.py simulated the scan run with the real guard config and published its measured pose (`--state-file`).
   `crack_trace capture` waited for the arm to settle, captured, and ran the OpenCrack model: 1 crack
   (`capture.log`, `*_paths_overlay.png`).
4. `crack_trace plan`: path3d gave `*_paths3d.json`, then the trace. One 208 mm segment, hover 14.3 cm (set by
   arm_guard's floor), tilt 10°, 14.6 s (`*_trace.json`, `*_trace_plan.json`). `*_trace_preview.png` is the planned
   path (green) over the model's mask (red).
5. Rehearsal at x1 and x2: completed, no guard fault, keep-out margin at least 18 mm.

The 5 mm vertical offset in this run is expected: the render used the commanded pose, while the simulated arm sags
0.03 rad and the record correctly holds the measured pose. Rendered at an exactly held pose, the path lies on the
print plane to 0.04 mm.

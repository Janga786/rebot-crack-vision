# Offline search demo — 2026-10-23

Output of `bash scripts/crack_pipeline.sh demo` (2 min 44 s, no hardware): crack test print #1 is hidden off-centre at
(0.34, -0.12) on the table, and the pipeline has to find it.

1. `search.json`: 6 camera stops, looking straight down 25 cm up (x 0.26 / 0.36, y -0.15 / 0 / 0.15), 3 s rest each. The
   sweep is rehearsed with the real guard config at x1 and x2 (keep-out margin >= 38 mm).
2. The camera views are rendered from the print's exact plane; the sweep's arm state is replayed in real time
   (`tools/replay_arm_state.py`, no motor) while `crack_trace search-capture` runs the crack model at every stop.
   `search_result.json`: all six stops see the crack; stop 0 (the crack fully inside the view, not touching the image
   edge) puts it at **(0.3367, -0.1141)** against the true (0.34, -0.12), within 6 mm. `stop_00_overlay.png` and
   `stop_05_overlay.png` show the model's mask (red).
3. `scan.json`: the scan pose re-centred on that point (J1 -0.34 rad), camera and mount at least 209 mm above the table
   over the whole run. Captured, segmented, lifted to 3D.
4. `*_trace.json` / `*_trace_preview.png`: 209 mm crack, hover 13.9 cm (set by arm_guard's floor), tilt 10 deg, 14.6 s;
   the preview draws the planned path (green) over the mask (red). Rehearsals at x1 and x2 completed with no guard
   fault (keep-out margin >= 19 mm).

A bug found on the way and fixed: a separately solved stop pose differed by 0.04 rad (J6) from where the leg arrived,
which would have needed 0.86 rad/s; each stop now rests at its leg's arrival pose (J6 peak 0.06 rad/s).

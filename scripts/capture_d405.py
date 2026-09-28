#!/usr/bin/env python3
"""scripts/capture_d405.py — thin CLI wrapper for `crackvision.realsense_capture` (TC-013).

All capture logic lives in src/crackvision/realsense_capture.py; this just lets an operator run
`./env.sh python scripts/capture_d405.py ...` instead of `-m crackvision.realsense_capture`,
consistent with the rest of this repo's operational scripts/.
"""

from __future__ import annotations

from crackvision.realsense_capture import main

if __name__ == "__main__":
    raise SystemExit(main())

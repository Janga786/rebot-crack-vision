"""scripts/camera.py — thin CLI wrapper for `crackvision.camera_pipeline` (the physical D405, end to end).

    ./env.sh python scripts/camera.py check     # first connection: device, firmware, USB 3, validated recording
    ./env.sh python scripts/camera.py snap      # capture + full crack pipeline (overlays under data/)
    ./env.sh python scripts/camera.py live      # model on every frame: open http://127.0.0.1:8090
    ./env.sh python scripts/camera.py record    # lossless RGB-D recording for later replay

All logic lives in src/crackvision/camera_pipeline.py; see docs/camera/D405_SETUP.md.
"""

from crackvision.camera_pipeline import main

if __name__ == "__main__":
    raise SystemExit(main())

# GEOM-04.1 — Fix pinned OpenCV so cv2.calibrateHandEye/calibrateRobotWorldHandEye exist in the crackvision env

```json card
{
  "kind": "impl",
  "requirements": [
    "REQ-GEOM-2",
    "REQ-HOST-1"
  ],
  "scope": {
    "write": [
      "requirements/requirements-core.txt",
      "requirements/crackvision.pip-freeze.txt",
      "docs/host/ENV_LOCK.md",
      "docs/COMPLETION_LOG.md"
    ]
  },
  "inputs": [
    "requirements/requirements-core.txt",
    "requirements/crackvision.pip-freeze.txt",
    "docs/host/ENV_LOCK.md",
    "scripts/verify_lock.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "features",
        "cmd": "./env.sh python -c \"import cv2; assert hasattr(cv2, 'calibrateHandEye'); assert hasattr(cv2, 'calibrateRobotWorldHandEye'); assert hasattr(cv2.aruco, 'CharucoDetector'); assert hasattr(cv2.aruco, 'CharucoBoard'); print('ok')\"",
        "timeout_s": 30,
        "expect_exit": 0
      },
      {
        "id": "pip-check",
        "cmd": "./env.sh python -m pip check",
        "timeout_s": 60,
        "expect_exit": 0
      },
      {
        "id": "lock-verify",
        "cmd": "./env.sh python scripts/verify_lock.py",
        "timeout_s": 60,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The installed OpenCV build was independently verified (before this card, by the lead) to lack cv2.calibrateHandEye/calibrateRobotWorldHandEye entirely at opencv-python-headless==5.0.0.93 and opencv-contrib-python-headless==5.0.0.93 alike (a Python-binding gap in that OpenCV 5.0.0 release, not a contrib-vs-headless flavor issue); opencv-contrib-python-headless==4.11.0.86 was independently confirmed to provide calibrateHandEye, calibrateRobotWorldHandEye, aruco.CharucoDetector and aruco.CharucoBoard together. Pin to that version (or a newer 4.x release re-verified to have all four) unless a newer OpenCV release is confirmed to have closed the 5.0 gap.",
      "Only the opencv package line changes in requirements/requirements-core.txt's diff (same filtered-pip-freeze regeneration command already documented in that file's header). No other package version changed as a side effect of the swap (pip check clean; full test suite green).",
      "requirements/crackvision.pip-freeze.txt regenerated via `./env.sh python -m pip freeze` per docs/host/ENV_LOCK.md's documented procedure, header comment updated with today's date/source, self-editable commit-hash line left as-is (verify_lock.py normalizes it).",
      "requirements/crackvision.conda-explicit.txt is untouched (opencv is pip-installed, not conda-installed, in this env) — do not regenerate it.",
      "docs/host/ENV_LOCK.md gains a short dated note (append, do not rewrite existing prose) recording why/what changed. One entry appended to docs/COMPLETION_LOG.md."
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 1,
    "context": 2,
    "consequence": 3,
    "task_class": "dependency-fix",
    "local_ok": false
  },
  "track": "calibration",
  "id": "GEOM-04.1",
  "parent": "GEOM-04",
  "title": "Fix pinned OpenCV so cv2.calibrateHandEye/calibrateRobotWorldHandEye exist in the crackvision env",
  "outcome": "The crackvision conda env's OpenCV package is swapped for a pinned version that provides cv2.calibrateHandEye, cv2.calibrateRobotWorldHandEye and the cv2.aruco ChArUco API together, with the environment lock files regenerated and the full test suite green."
}
```

Swap the crackvision conda env's OpenCV package: uninstall `opencv-python-headless==5.0.0.93`, install `opencv-contrib-python-headless==4.11.0.86` (`./env.sh pip uninstall -y opencv-python-headless && ./env.sh pip install opencv-contrib-python-headless==4.11.0.86`).

This card exists because `cv2.calibrateHandEye`/`cv2.calibrateRobotWorldHandEye` — the algorithm `docs/calibration/PLAN.md` §3 (GEOM-03, accepted) mandates — do not exist as Python-callable attributes on `cv2` at all in the pinned 5.0.0.93 build, in either the `opencv-python-headless` or `opencv-contrib-python-headless` flavor (both were downloaded and probed in isolation; both expose the `cv2.CALIB_HAND_EYE_TSAI`/`PARK`/`DANIILIDIS` integer constants but no `calibrateHandEye` function — an incomplete Python binding in this very new OpenCV 5.0.0 release, not a contrib-vs-headless packaging choice). `opencv-contrib-python-headless==4.11.0.86` was probed the same way and has both the hand-eye functions and the `cv2.aruco.CharucoDetector`/`CharucoBoard` API GEOM-04.2 needs — no other card in this branch or elsewhere in the repo currently imports `cv2` beyond a couple of debug-preview calls (`imshow`/`waitKey`/`destroyAllWindows` in `visualize.py`), so this downgrade carries very low regression risk; the full suite check below is the actual proof.

After installing: run the acceptance checks above in order (features → pip check → verify_lock → full suite). Regenerate the two lock artefacts exactly as `docs/host/ENV_LOCK.md` §"Regenerating the lock files" already documents (that section already sanctions later cards doing this after a deliberate environment change — this is not a new exception to §5 file ownership, it is the documented normal path). If any existing test breaks under the downgrade, fix the *test/code* only if the break is a trivial `cv2` API-name difference already caused by this swap; if it reveals something else, stop and report rather than papering over it.

Note for GEOM-04.3: PLAN.md §3 spells the method flags `CALIBRATE_HAND_EYE_TSAI` etc. — the real OpenCV symbols (in both 4.x and this 5.0 build) are `cv2.CALIB_HAND_EYE_TSAI`/`PARK`/`DANIILIDIS` (no `RATE`). Use the real names; this is a wording slip in PLAN.md's prose, not a different decision.

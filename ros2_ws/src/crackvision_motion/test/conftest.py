"""Shared pytest setup for crackvision_motion unit tests.

The unit check runs without sourcing ros2_ws/install/setup.bash, so put the in-tree
crackvision_motion and crackvision_description packages on sys.path before any test module
is collected (mirrors test_execution_gate.py).
"""

from __future__ import annotations

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
for _pkg in (_SRC / "crackvision_description", _SRC / "crackvision_motion"):
    if str(_pkg) not in sys.path:
        sys.path.insert(0, str(_pkg))

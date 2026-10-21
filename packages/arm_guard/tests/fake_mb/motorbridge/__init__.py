"""Fake motorbridge for arm_guard tests (see core.py)."""
from .core import Controller, Motor  # noqa: F401
from .errors import CallError, MotorBridgeError  # noqa: F401
from .models import Mode, MotorState  # noqa: F401

__version__ = "0.5.6-fake"

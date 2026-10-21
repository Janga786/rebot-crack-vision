"""arm_guard — last-line safety filter in front of every reBot B601-DM motor command.

    import arm_guard           # normally done for you by the arm_guard.pth hook (python -m arm_guard install-hook)
    arm_guard.install()        # patch motorbridge now (idempotent)
    arm_guard.status()         # config hash, latch state, recent events
    arm_guard.reset("why")     # operator clears a fault latch
    with arm_guard.config_writes("set CAN timeout"):   # allow register/zero/store writes, motors disabled
        ...

See docs/motion/ARM_GUARD.md.
"""

from __future__ import annotations

import sys

from .errors import GuardConfigError, GuardError, GuardFault, GuardRefused  # noqa: F401
from .runtime import Guard

__all__ = ["install", "status", "reset", "config_writes", "guard", "GuardError", "GuardConfigError",
           "GuardRefused", "GuardFault"]

_guard: Guard = None


def _build() -> Guard:
    from . import config
    try:
        return Guard(config.load())
    except GuardConfigError as e:
        print(f"[arm_guard] CONFIG ERROR, all arm motion will be refused: {e}", file=sys.stderr, flush=True)
        return Guard(None, config_error=str(e))
    except Exception as e:  # fail closed on anything unexpected
        print(f"[arm_guard] INTERNAL ERROR loading config, all arm motion will be refused: {e!r}", file=sys.stderr,
              flush=True)
        return Guard(None, config_error=repr(e))


def guard() -> Guard:
    global _guard
    if _guard is None:
        _guard = _build()
    return _guard


def install() -> Guard:
    """Patch motorbridge.core (importing it if needed). Safe to call repeatedly."""
    import motorbridge.core as mc

    from .patch import patch
    g = guard()
    patch(mc.Controller, mc.Motor, g)
    return g


def status() -> dict:
    return guard().status()


def reset(reason: str) -> None:
    guard().reset(reason)


def config_writes(reason: str):
    return guard().config_writes(reason)

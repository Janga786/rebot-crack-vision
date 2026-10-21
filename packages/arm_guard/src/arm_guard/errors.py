"""arm_guard.errors — exception types. All derive from PermissionError, never from motorbridge's CallError, so
vendor code that swallows CallError (`except CallError: pass`) cannot silently swallow a guard refusal."""


class GuardError(PermissionError):
    """Base class: the guard refused an operation."""


class GuardConfigError(GuardError):
    """The configuration is missing, malformed, inconsistent, or not approved. The guard then refuses all motion."""


class GuardRefused(GuardError):
    """A setup/permission refusal (unknown motor, not enabled-safe, config write without permission). Nothing was sent."""


class GuardFault(GuardError):
    """A safety rule tripped while operating. The guard latched: the offending joint was sent a hold command and every
    later motion command on every joint is turned into a hold until the process restarts or reset() is called."""

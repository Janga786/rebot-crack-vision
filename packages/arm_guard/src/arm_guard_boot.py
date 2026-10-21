"""arm_guard_boot — imported by arm_guard.pth at interpreter start. Kept tiny so it cannot fail itself: if
arm_guard cannot load, `import motorbridge` is made to fail instead of running unguarded."""

import sys

try:
    import arm_guard.autoload  # noqa: F401
except Exception as _e:  # fail closed
    _msg = f"arm_guard failed to load ({_e!r}); motorbridge is blocked until this is fixed"
    print(f"[arm_guard] {_msg}", file=sys.stderr)

    class _Block:
        def find_spec(self, fullname, path=None, target=None):
            if fullname == "motorbridge" or fullname.startswith("motorbridge."):
                raise ImportError(_msg)
            return None

    sys.meta_path.insert(0, _Block())

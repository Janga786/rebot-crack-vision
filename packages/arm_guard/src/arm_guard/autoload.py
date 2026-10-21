"""arm_guard.autoload — patch motorbridge.core the moment anything imports it.

Imported at interpreter start by arm_guard_boot (via the arm_guard.pth file that `python -m arm_guard install-hook`
writes into an env's site-packages). If patching fails, the import of motorbridge fails: an unguarded motor bus
is never handed out.
"""

from __future__ import annotations

import importlib.abc
import sys

TARGET = "motorbridge.core"


class _Loader(importlib.abc.Loader):
    def __init__(self, inner) -> None:
        self.inner = inner

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module) -> None:
        self.inner.exec_module(module)
        try:
            from . import guard
            from .patch import patch
            patch(module.Controller, module.Motor, guard())
        except Exception as e:
            raise ImportError(f"arm_guard could not patch motorbridge ({e!r}); refusing to provide an unguarded "
                              f"motor bus") from e


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != TARGET:
            return None
        for f in sys.meta_path:
            if f is self or not hasattr(f, "find_spec"):
                continue
            spec = f.find_spec(fullname, path, target)
            if spec is not None:
                spec.loader = _Loader(spec.loader)
                return spec
        return None


def enable() -> None:
    if TARGET in sys.modules:
        from . import install
        install()
        return
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


enable()

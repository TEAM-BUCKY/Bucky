"""Registry of lab modules, keyed by ``(kind, name)``.

User modules live as plain ``.py`` files in ``bucky/lab/user/``. :func:`discover` re-executes
every file on each call, so editing or adding a formula takes effect on the next sweep without a
server restart. A file that fails to import is reported in :func:`load_errors` instead of
breaking the others.
"""
from __future__ import annotations

import importlib.util
import inspect
import sys
import traceback
from pathlib import Path

from bucky.lab.modules import LabModule
from bucky.lab.params import describe_params

USER_DIR = Path(__file__).parent / "user"
_USER_PKG = "bucky.lab.user"

_MODULES: dict[tuple[str, str], type[LabModule]] = {}
_ERRORS: dict[str, str] = {}


def register(cls: type[LabModule]) -> type[LabModule]:
    """Class decorator: make a module available to the lab."""
    if not cls.kind or not cls.name:
        raise ValueError(f"{cls.__name__} needs both `kind` and `name` set")
    _MODULES[(cls.kind, cls.name)] = cls
    return cls


def discover(user_dir: Path = USER_DIR) -> None:
    """(Re)load every user module file (and register the firmware's programs)."""
    import bucky.firmware.lab  # noqa: F401  (kind "firmware": robot/src programs, no build needed)

    for key, cls in list(_MODULES.items()):
        if cls.__module__.startswith(_USER_PKG + "."):
            del _MODULES[key]
    _ERRORS.clear()
    for path in sorted(user_dir.glob("*.py")):
        if path.name.startswith("_"):
            continue
        mod_name = f"{_USER_PKG}.{path.stem}"
        try:
            spec = importlib.util.spec_from_file_location(mod_name, path)
            mod = importlib.util.module_from_spec(spec)
            sys.modules[mod_name] = mod
            # Compile from source rather than spec.loader.exec_module: the .pyc cache keys on
            # mtime (1 s resolution) + size, so a quick same-length edit would load stale code.
            exec(compile(path.read_text(), str(path), "exec"), mod.__dict__)
        except Exception:  # noqa: BLE001 — surface the error, keep loading the rest
            sys.modules.pop(mod_name, None)
            _ERRORS[path.name] = traceback.format_exc(limit=5)


def load_errors() -> dict[str, str]:
    return dict(_ERRORS)


def get_module(kind: str, name: str) -> type[LabModule]:
    try:
        return _MODULES[(kind, name)]
    except KeyError:
        known = sorted(n for k, n in _MODULES if k == kind)
        raise KeyError(f"unknown {kind} module {name!r}; known: {known}") from None


def list_modules(kind: str | None = None) -> list[dict]:
    out = []
    for (k, n), cls in sorted(_MODULES.items()):
        if kind is not None and k != kind:
            continue
        try:
            src_file = inspect.getsourcefile(cls)
            source = Path(src_file).read_text() if src_file else ""
        except (OSError, TypeError):
            src_file, source = None, ""
        out.append({
            "kind": k,
            "name": n,
            "doc": inspect.cleandoc(cls.__doc__ or ""),
            "params": describe_params(cls.params),
            "sensors": list(cls.sensors),
            "file": Path(src_file).name if src_file else None,
            "source": source,
        })
    return out

"""Declarative tunables shared by modules, sensors and experiments.

A class lists its knobs as ``params = {"name": Param(...)}``. Subclasses inherit and may override
their parent's params (merged by :class:`Configurable`). The web UI renders a form straight from
:meth:`Param.describe`, and the runner can sweep any of them.
"""
from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, ClassVar


@dataclass(frozen=True)
class Param:
    default: Any
    min: float | None = None
    max: float | None = None
    step: float | None = None
    help: str = ""
    options: tuple | None = None   # fixed choices → rendered as a select

    @property
    def type(self) -> str:
        if self.options is not None:
            return "choice"
        d = self.default
        if isinstance(d, bool):
            return "bool"
        if isinstance(d, int):
            return "int"
        if isinstance(d, float):
            return "float"
        if isinstance(d, (list, tuple)):
            return "list"
        return "str"

    def coerce(self, value: Any) -> Any:
        t = self.type
        if t == "bool":
            return value if isinstance(value, bool) else str(value).lower() in ("1", "true", "yes")
        if t == "int":
            return int(round(float(value)))
        if t == "float":
            return float(value)
        if t == "list":
            if isinstance(value, str):
                value = [v for v in value.replace(";", ",").split(",") if v.strip()]
            return [float(v) for v in value]
        if t == "choice":
            if value not in self.options:
                raise ValueError(f"{value!r} not one of {self.options}")
            return value
        return str(value)

    def describe(self) -> dict:
        return {
            "type": self.type, "default": self.default, "min": self.min, "max": self.max,
            "step": self.step, "help": self.help,
            "options": list(self.options) if self.options is not None else None,
        }


class Configurable:
    """Mixin: merges ``params`` down the class hierarchy and resolves overrides into ``self.p``."""

    params: ClassVar[dict[str, Param]] = {}

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        merged: dict[str, Param] = {}
        for base in reversed(cls.__mro__[1:]):
            merged.update(getattr(base, "params", {}) or {})
        merged.update(cls.__dict__.get("params", {}) or {})
        cls.params = merged

    def __init__(self, **overrides: Any) -> None:
        self.p = SimpleNamespace(**resolve_params(self.params, overrides))


def resolve_params(spec: dict[str, Param], overrides: dict[str, Any] | None) -> dict[str, Any]:
    overrides = dict(overrides or {})
    unknown = set(overrides) - set(spec)
    if unknown:
        raise ValueError(f"unknown param(s): {', '.join(sorted(unknown))}")
    out = {}
    for name, param in spec.items():
        out[name] = param.coerce(overrides[name]) if name in overrides else param.default
    return out


def describe_params(spec: dict[str, Param]) -> dict[str, dict]:
    return {name: p.describe() for name, p in spec.items()}

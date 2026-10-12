"""STM32 pin names ⇄ the integer ``PinName`` values the simulator uses.

``PB_14_ALT2`` = (port B = 1) << 4 | 14, plus ALT2 = 0x200 — exactly stm32duino's encoding, so
``pin("PB_14_ALT2") == bucky_fw.board.MOTOR1.inA``.
"""
from __future__ import annotations

import re

PORTS = "ABCDEFGHI"
_RE = re.compile(r"^P([A-I])_(\d{1,2})(?:_ALT([1-7]))?$")


def pin(name: str) -> int:
    m = _RE.match(name.strip().upper())
    if m is None or int(m.group(2)) > 15:
        raise ValueError(f"not an STM32 pin name: {name!r} (e.g. 'PB_14' or 'PA_6_ALT1')")
    value = PORTS.index(m.group(1)) << 4 | int(m.group(2))
    if m.group(3):
        value |= int(m.group(3)) << 8
    return value


def pin_name(value: int) -> str:
    if value == 0xFFFFFFFF:
        return "NC"
    port, line, alt = (value >> 4) & 0xF, value & 0xF, (value >> 8) & 0x7
    name = f"P{PORTS[port]}_{line}"
    return f"{name}_ALT{alt}" if alt else name


def pad(value: int) -> int:
    """The physical pad (drops the _ALTn selector)."""
    return value & 0xFF

"""Import the compiled firmware simulator (building it first when needed)."""
from __future__ import annotations

import importlib.machinery
import importlib.util
import sys
from types import ModuleType

from bucky.firmware.build import DEFAULT_BOARD, DEFAULT_BUILD_TYPE, ensure_built

_LOADED: dict[str, ModuleType] = {}


def load_firmware(board: str = DEFAULT_BOARD, build_type: str = DEFAULT_BUILD_TYPE) -> ModuleType:
    """The ``bucky_fw`` extension for ``board``: the real firmware on a simulated board.

    One process holds one simulated board (the firmware's globals are process-wide); asking for a
    different board in the same process raises. Use :class:`bucky.firmware.process.FirmwareProcess`
    to run several boards side by side.
    """
    if "bucky_fw" in sys.modules:
        mod = sys.modules["bucky_fw"]
        loaded_board = getattr(mod, "__bucky_board__", None)
        if loaded_board not in (None, board):
            raise RuntimeError(f"this process already runs board {loaded_board!r}")
        return mod
    path = ensure_built(board, build_type)
    loader = importlib.machinery.ExtensionFileLoader("bucky_fw", str(path))
    spec = importlib.util.spec_from_file_location("bucky_fw", path, loader=loader)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["bucky_fw"] = mod
    try:
        loader.exec_module(mod)
    except BaseException:
        sys.modules.pop("bucky_fw", None)
        raise
    mod.__bucky_board__ = board
    _LOADED[board] = mod
    return mod

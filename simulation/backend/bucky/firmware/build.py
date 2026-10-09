"""Build the firmware simulator extension (``robot/sim`` → ``bucky_fw``) on demand.

The real firmware in ``robot/src`` is compiled for the host by CMake + Ninja into
``robot/sim/build/<board>-<build_type>/``. :func:`ensure_built` only rebuilds when something that
affects the binary changed — a hash over every source under ``robot/src`` and ``robot/sim``, the
build settings, the pybind11 version and the Python ABI — so calling it at the top of every test
session or sweep is cheap. Concurrent callers (pytest-xdist, sweep workers) serialise on a lock.

Environment:

* ``BUCKY_FW_NO_BUILD=1`` — never build; use whatever is in the build directory.
* ``BUCKY_ROBOT_DIR`` — the ``robot/`` directory, if not the one next to ``simulation/``.

CLI::

    uv run python -m bucky.firmware.build [-v] [--force] [--board h562rg]
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import os
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

BOARDS = ("h562rg",)
DEFAULT_BOARD = "h562rg"
DEFAULT_BUILD_TYPE = "RelWithDebInfo"


class BuildError(RuntimeError):
    """CMake or the compiler failed; the message holds the tail of the build log."""


def robot_dir() -> Path:
    """The firmware repo's ``robot/`` directory. In the repo that is ``<root>/robot`` next to
    ``simulation/``; deployments that ship only the backend (the Docker image's ``/app``) have
    none, and get a path that does not exist (callers check, see :func:`firmware_available`)."""
    env = os.environ.get("BUCKY_ROBOT_DIR")
    if env:
        return Path(env).resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "robot"
        if (candidate / "src" / "main.cpp").is_file():
            return candidate
    return here.parent / "_no_robot_dir"


def firmware_available() -> bool:
    """True when the firmware sources are present (False in backend-only deployments)."""
    return (firmware_dir() / "main.cpp").is_file()


def sim_dir() -> Path:
    return robot_dir() / "sim"


def firmware_dir() -> Path:
    return robot_dir() / "src"


def build_dir(board: str = DEFAULT_BOARD, build_type: str = DEFAULT_BUILD_TYPE) -> Path:
    return sim_dir() / "build" / f"{board}-{build_type}"


def toolchain_available() -> bool:
    """True when the host can build the simulator (C/C++ compiler + CMake)."""
    return bool(shutil.which("cmake") and (shutil.which("c++") or shutil.which("g++")))


def _ext_suffix() -> str:
    return sysconfig.get_config_var("EXT_SUFFIX") or ".so"


def module_path(board: str = DEFAULT_BOARD, build_type: str = DEFAULT_BUILD_TYPE) -> Path:
    return build_dir(board, build_type) / f"bucky_fw{_ext_suffix()}"


def _source_files() -> list[Path]:
    files: list[Path] = []
    for root in (firmware_dir(), sim_dir()):
        for p in sorted(root.rglob("*")):
            if not p.is_file():
                continue
            rel = p.relative_to(root)
            if rel.parts and rel.parts[0] == "build":
                continue
            if p.suffix in {".pyc"} or "__pycache__" in rel.parts:
                continue
            files.append(p)
    return files


def source_hash(board: str = DEFAULT_BOARD, build_type: str = DEFAULT_BUILD_TYPE) -> str:
    import pybind11

    h = hashlib.sha256()
    h.update(f"{board}|{build_type}|{pybind11.__version__}|{_ext_suffix()}|{sys.version}".encode())
    base = robot_dir()
    for p in _source_files():
        h.update(str(p.relative_to(base)).encode())
        h.update(b"\0")
        h.update(p.read_bytes())
        h.update(b"\0")
    return h.hexdigest()


def _run(cmd: list[str], cwd: Path, verbose: bool) -> None:
    if verbose:
        print("+", " ".join(cmd), flush=True)
        res = subprocess.run(cmd, cwd=cwd)
        if res.returncode != 0:
            raise BuildError(f"{cmd[0]} failed (exit {res.returncode})")
        return
    res = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if res.returncode != 0:
        log = (res.stdout + res.stderr).strip().splitlines()
        raise BuildError(f"{' '.join(cmd[:2])} failed:\n" + "\n".join(log[-60:]))


def ensure_built(board: str = DEFAULT_BOARD, build_type: str = DEFAULT_BUILD_TYPE, *,
                 force: bool = False, verbose: bool = False) -> Path:
    """Build ``bucky_fw`` for ``board`` if its sources changed; return the extension's path."""
    if board not in BOARDS:
        raise ValueError(f"unknown board {board!r}; known: {BOARDS}")
    out = module_path(board, build_type)
    if os.environ.get("BUCKY_FW_NO_BUILD") == "1":
        if not out.exists():
            raise BuildError(f"BUCKY_FW_NO_BUILD=1 but {out} does not exist")
        return out
    if not toolchain_available():
        raise BuildError("building the firmware simulator needs cmake and a C++ compiler")

    bdir = build_dir(board, build_type)
    bdir.mkdir(parents=True, exist_ok=True)
    stamp = bdir / ".bucky_stamp"
    with open(bdir / ".lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        digest = source_hash(board, build_type)
        if not force and out.exists() and stamp.exists() and stamp.read_text() == digest:
            return out

        import pybind11

        cfg = [
            "cmake", "-S", str(sim_dir()), "-B", str(bdir),
            f"-DCMAKE_BUILD_TYPE={build_type}", f"-DBUCKY_BOARD={board}",
            f"-Dpybind11_DIR={pybind11.get_cmake_dir()}", f"-DPython_EXECUTABLE={sys.executable}",
        ]
        if shutil.which("ninja") and not (bdir / "Makefile").exists():
            cfg += ["-G", "Ninja"]
        _run(cfg, bdir, verbose)
        jobs = str(max(1, (os.cpu_count() or 2)))
        _run(["cmake", "--build", str(bdir), "--target", "bucky_fw", "bucky_firmware_objects",
              "-j", jobs], bdir, verbose)
        if not out.exists():
            raise BuildError(f"build finished but {out.name} is missing")
        stamp.write_text(digest)
        return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Build the firmware simulator (bucky_fw)")
    ap.add_argument("--board", default=DEFAULT_BOARD, choices=BOARDS)
    ap.add_argument("--build-type", default=DEFAULT_BUILD_TYPE)
    ap.add_argument("--force", action="store_true", help="rebuild even if nothing changed")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    try:
        path = ensure_built(args.board, args.build_type, force=args.force, verbose=args.verbose)
    except BuildError as e:
        sys.exit(str(e))
    print(path)


if __name__ == "__main__":
    main()

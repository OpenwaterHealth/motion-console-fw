"""Pure helper functions for scripts/deploy.py."""
from __future__ import annotations

import platform
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Optional

_PROJECT_NAME_RE = re.compile(
    r"set\s*\(\s*CMAKE_PROJECT_NAME\s+([^\s\)]+)\s*\)"
)


def read_project_name(cmake_path: Path) -> str:
    """Extract CMAKE_PROJECT_NAME from a CMakeLists.txt.

    Mirrors the regex used by generate_vscode_files.py so VS Code and the
    deploy script always agree on the binary name.
    """
    text = cmake_path.read_text(encoding="utf-8")
    m = _PROJECT_NAME_RE.search(text)
    if not m:
        raise RuntimeError(
            f"Could not find CMAKE_PROJECT_NAME set(...) directive in {cmake_path}"
        )
    return m.group(1)


def bin_path_for(repo_root: Path, config: str, project: str) -> Path:
    """Return the absolute path to the produced .bin for a given config."""
    return repo_root / "build" / config / f"{project}.bin"


def _bundled_dfu_util() -> Optional[Path]:
    """Look for the dfu-util binary that ships with the omotion package.

    Returns None if omotion isn't importable or no matching platform binary
    is bundled. The SDK ships binaries for win32, win64, and darwin-x86_64;
    Linux callers fall through to PATH lookup.
    """
    try:
        import omotion
    except ImportError:
        return None

    pkg_dir = Path(omotion.__file__).resolve().parent

    system = platform.system().lower()
    if system.startswith("windows"):
        machine = platform.machine().lower()
        subdir = "win64" if "64" in machine else "win32"
        exe = "dfu-util.exe"
    elif system.startswith("darwin"):
        subdir = "darwin-x86_64"
        exe = "dfu-util"
    else:
        return None

    candidate = pkg_dir / "dfu-util" / subdir / exe
    return candidate if candidate.is_file() else None


def resolve_dfu_util(override: str | None) -> str:
    """Return an absolute path to dfu-util, or raise if not found.

    Search order:
      1. --dfu-util override (must exist if given)
      2. The binary bundled inside the installed omotion package
      3. dfu-util on PATH
    """
    if override:
        if not Path(override).is_file():
            raise RuntimeError(
                f"dfu-util override path does not exist: {override}"
            )
        return override

    bundled = _bundled_dfu_util()
    if bundled is not None:
        return str(bundled)

    found = shutil.which("dfu-util")
    if not found:
        raise RuntimeError(
            "dfu-util not found: not bundled with the installed omotion "
            "package, not on PATH, and no --dfu-util override given. "
            "Install omotion (pip install -e ../openmotion-sdk), install "
            "dfu-util on PATH (Windows: scoop install dfu-util), or pass "
            "--dfu-util PATH."
        )
    return found


DFU_VID_PID_STR = "0483:df11"


def wait_for_dfu_device(dfu_util: str, timeout: float = 10.0,
                        poll_interval: float = 0.3) -> bool:
    """Poll `dfu-util --list` until the STM32 DFU device appears or timeout.

    Returns True if the device was found, False on timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                [dfu_util, "--list"],
                capture_output=True, text=True, check=False,
            )
            if DFU_VID_PID_STR in result.stdout.lower():
                return True
        except FileNotFoundError:
            # dfu-util went missing between resolve and now — fail fast
            return False
        time.sleep(poll_interval)
    return False

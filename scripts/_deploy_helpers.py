"""Pure helper functions for scripts/deploy.py."""
from __future__ import annotations

import re
import shutil
from pathlib import Path

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


def resolve_dfu_util(override: str | None) -> str:
    """Return an absolute path to dfu-util, or raise if not found."""
    if override:
        return override
    found = shutil.which("dfu-util")
    if not found:
        raise RuntimeError(
            "dfu-util not found on PATH. Install it (Windows: scoop install dfu-util, "
            "or download from sourceforge.net/projects/dfu-util) or pass --dfu-util PATH."
        )
    return found

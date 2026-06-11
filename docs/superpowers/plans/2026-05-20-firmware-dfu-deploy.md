# Firmware DFU Deploy Script Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `scripts/deploy.py` and CMake `flash`/`flash-left`/`flash-right` targets to both `openmotion-console-fw` and `openmotion-sensor-fw` so one command can build, DFU-flash, and recover the device.

**Architecture:** Per-repo Python script (~200 lines) that drives: cmake build → omotion `enter_dfu()` → `dfu-util` flash with `:leave` (auto-resets MCU) → wait for re-enumeration → optional soft-reset recovery. Pure-logic helpers get pytest unit tests; hardware paths are covered by manual verification only.

**Tech Stack:** Python 3.12, `omotion` SDK (already on PATH after `pip install -e openmotion-sdk`), `dfu-util` CLI, CMake 3.22+.

**Spec:** `docs/superpowers/specs/2026-05-20-firmware-dfu-deploy-design.md`.

---

## File Structure

```
openmotion-console-fw/
├── scripts/
│   ├── deploy.py              # NEW — main entry point
│   └── _deploy_helpers.py     # NEW — pure helpers (testable)
├── tests/
│   └── test_deploy_helpers.py # NEW — pytest for pure helpers
├── CMakeLists.txt             # MODIFY — add `flash` target
└── ...

openmotion-sensor-fw/
├── scripts/
│   ├── deploy.py              # NEW — same shape as console, sensor-specific
│   └── _deploy_helpers.py     # NEW — copy of console's helpers
├── tests/
│   └── test_deploy_helpers.py # NEW — same tests, sensor project name
├── CMakeLists.txt             # MODIFY — add `flash-left`, `flash-right` targets
└── ...
```

**Why the split:** `deploy.py` is orchestration with side effects (cmake, subprocess, omotion, dfu-util). `_deploy_helpers.py` holds the pure functions (path resolution, project-name parsing, argv validation) — these are unit-testable without hardware. Putting them in their own module keeps `deploy.py` focused on the workflow.

**Repository convention:** This plan is duplicated identically in both `openmotion-console-fw/docs/superpowers/plans/` and `openmotion-sensor-fw/docs/superpowers/plans/`. Execute the console-fw tasks first; sensor-fw tasks reuse code from console-fw.

---

## Task 1: Console — scaffold scripts/ and tests/ directories

**Repo:** `openmotion-console-fw`

**Files:**
- Create: `openmotion-console-fw/scripts/deploy.py` (skeleton)
- Create: `openmotion-console-fw/scripts/_deploy_helpers.py` (empty)
- Create: `openmotion-console-fw/tests/test_deploy_helpers.py` (empty)
- Create: `openmotion-console-fw/tests/conftest.py` (sys.path shim)

- [ ] **Step 1: Create `scripts/_deploy_helpers.py` with placeholder docstring**

```python
"""Pure helper functions for scripts/deploy.py.

All side-effect-free logic lives here so it can be unit tested
without hardware. Imported by deploy.py.
"""
```

- [ ] **Step 2: Create `scripts/deploy.py` skeleton (CLI surface only)**

```python
#!/usr/bin/env python3
"""deploy.py — Build, DFU-flash, and recover the MOTION console firmware.

Usage:
    python scripts/deploy.py [--config Debug|Release] [--no-build]
                             [--dfu-util PATH] [--post-reset] [--no-confirm]
"""
from __future__ import annotations

import argparse
import sys


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Build, enter DFU, flash, and reset the MOTION console."
    )
    p.add_argument("--config", choices=("Debug", "Release"), default="Debug",
                   help="CMake build config (default: Debug)")
    p.add_argument("--no-build", action="store_true",
                   help="Skip 'cmake --build'; flash whatever is already built")
    p.add_argument("--dfu-util", default=None,
                   help="Path to dfu-util binary (default: search PATH)")
    p.add_argument("--post-reset", action="store_true",
                   help="If device doesn't come back after leaveDFU, attempt soft_reset")
    p.add_argument("--no-confirm", action="store_true",
                   help="Skip the y/N confirmation prompt")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    print(f"[deploy] args = {args}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Create `tests/conftest.py` so pytest can import the script package**

```python
"""Add the scripts/ directory to sys.path for tests."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
```

- [ ] **Step 4: Verify the skeleton runs**

Run: `python scripts/deploy.py --config Debug`
Expected: prints `[deploy] args = Namespace(config='Debug', no_build=False, dfu_util=None, post_reset=False, no_confirm=False)` and exits 0.

- [ ] **Step 5: Commit**

```bash
git add scripts/deploy.py scripts/_deploy_helpers.py tests/conftest.py tests/test_deploy_helpers.py
git commit -m "chore: scaffold scripts/deploy.py and tests/"
```

---

## Task 2: Console — pure helpers (project name, bin path, dfu-util resolve) with tests

**Repo:** `openmotion-console-fw`

**Files:**
- Modify: `openmotion-console-fw/scripts/_deploy_helpers.py`
- Modify: `openmotion-console-fw/tests/test_deploy_helpers.py`

- [ ] **Step 1: Write the failing tests in `tests/test_deploy_helpers.py`**

```python
import shutil
import textwrap
from pathlib import Path

import pytest

from _deploy_helpers import (
    read_project_name,
    bin_path_for,
    resolve_dfu_util,
)


def test_read_project_name_extracts_from_set_directive(tmp_path: Path):
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text(textwrap.dedent("""
        cmake_minimum_required(VERSION 3.22)
        set(CMAKE_PROJECT_NAME motion-console-fw)
        project(${CMAKE_PROJECT_NAME})
    """))
    assert read_project_name(cmake) == "motion-console-fw"


def test_read_project_name_raises_when_missing(tmp_path: Path):
    cmake = tmp_path / "CMakeLists.txt"
    cmake.write_text("# nothing here\n")
    with pytest.raises(RuntimeError, match="CMAKE_PROJECT_NAME"):
        read_project_name(cmake)


def test_bin_path_for_returns_repo_relative(tmp_path: Path):
    repo = tmp_path
    result = bin_path_for(repo, "Debug", "motion-console-fw")
    assert result == repo / "build" / "Debug" / "motion-console-fw.bin"


def test_resolve_dfu_util_uses_override_when_given(tmp_path: Path):
    fake = tmp_path / "dfu-util"
    fake.write_text("#!/bin/sh\necho dfu-util fake\n")
    fake.chmod(0o755)
    assert resolve_dfu_util(str(fake)) == str(fake)


def test_resolve_dfu_util_raises_when_missing(monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda _: None)
    with pytest.raises(RuntimeError, match="dfu-util not found"):
        resolve_dfu_util(None)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_deploy_helpers.py -v`
Expected: All tests fail with `ImportError: cannot import name 'read_project_name'`.

- [ ] **Step 3: Implement the helpers in `scripts/_deploy_helpers.py`**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_deploy_helpers.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add scripts/_deploy_helpers.py tests/test_deploy_helpers.py
git commit -m "feat(deploy): add pure helpers for path/config resolution"
```

---

## Task 3: Console — DFU enumeration polling helper with test

**Repo:** `openmotion-console-fw`

**Files:**
- Modify: `openmotion-console-fw/scripts/_deploy_helpers.py`
- Modify: `openmotion-console-fw/tests/test_deploy_helpers.py`

We poll `dfu-util --list` to detect DFU enumeration rather than depending on pyusb directly. Output of `dfu-util --list` contains substrings like `Found DFU: [0483:df11]` when the STM32 ROM bootloader is enumerated.

- [ ] **Step 1: Write the failing test**

Add to `tests/test_deploy_helpers.py`:

```python
import subprocess
from unittest.mock import patch

from _deploy_helpers import wait_for_dfu_device


def _fake_run_returning(stdout: str, returncode: int = 0):
    def _run(cmd, **kw):
        return subprocess.CompletedProcess(cmd, returncode, stdout=stdout, stderr="")
    return _run


def test_wait_for_dfu_device_succeeds_when_listed():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=_fake_run_returning("Found DFU: [0483:df11] ...\n")):
        assert wait_for_dfu_device("dfu-util", timeout=0.5, poll_interval=0.05) is True


def test_wait_for_dfu_device_times_out_when_absent():
    with patch("_deploy_helpers.subprocess.run",
               side_effect=_fake_run_returning("No DFU capable USB device available\n")):
        assert wait_for_dfu_device("dfu-util", timeout=0.2, poll_interval=0.05) is False
```

- [ ] **Step 2: Run tests to verify the new ones fail**

Run: `pytest tests/test_deploy_helpers.py -v`
Expected: 2 new tests fail with `ImportError: cannot import name 'wait_for_dfu_device'`.

- [ ] **Step 3: Implement `wait_for_dfu_device` in `scripts/_deploy_helpers.py`**

Add at the bottom of `_deploy_helpers.py`:

```python
import subprocess
import time

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
```

- [ ] **Step 4: Run tests to verify all pass**

Run: `pytest tests/test_deploy_helpers.py -v`
Expected: 7 passed.

- [ ] **Step 5: Commit**

```bash
git add scripts/_deploy_helpers.py tests/test_deploy_helpers.py
git commit -m "feat(deploy): add wait_for_dfu_device helper"
```

---

## Task 4: Console — wire main() to drive the full workflow

**Repo:** `openmotion-console-fw`

**Files:**
- Modify: `openmotion-console-fw/scripts/deploy.py`

This is orchestration only — the helpers from Tasks 2-3 do the testable work. Hardware paths (omotion, dfu-util invocation, post-flash) are exercised by manual verification (Task 8), not unit tests, matching the SDK pattern where `enter_dfu.py` and `soft_reset_console.py` have no unit tests.

- [ ] **Step 1: Replace the entire body of `scripts/deploy.py`**

```python
#!/usr/bin/env python3
"""deploy.py — Build, DFU-flash, and recover the MOTION console firmware.

Usage:
    python scripts/deploy.py [--config Debug|Release] [--no-build]
                             [--dfu-util PATH] [--post-reset] [--no-confirm]
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from _deploy_helpers import (  # noqa: E402
    bin_path_for,
    read_project_name,
    resolve_dfu_util,
    wait_for_dfu_device,
    DFU_VID_PID_STR,
)

DEVICE_LABEL = "console"
COMEBACK_TIMEOUT_S = 10.0
DFU_ENUM_TIMEOUT_S = 10.0
FLASH_BASE = 0x08000000


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Build, enter DFU, flash, and reset the MOTION console."
    )
    p.add_argument("--config", choices=("Debug", "Release"), default="Debug")
    p.add_argument("--no-build", action="store_true")
    p.add_argument("--dfu-util", default=None)
    p.add_argument("--post-reset", action="store_true")
    p.add_argument("--no-confirm", action="store_true")
    return p.parse_args()


def _run(cmd: list[str], **kw) -> int:
    print(f"+ {' '.join(cmd)}")
    return subprocess.call(cmd, **kw)


def _git_describe() -> str:
    try:
        out = subprocess.check_output(
            ["git", "describe", "--tags", "--dirty", "--always"],
            cwd=REPO_ROOT, text=True,
        )
        return out.strip()
    except Exception:
        return "unknown"


def _confirm(bin_file: Path) -> bool:
    print()
    print(f"  Device : {DEVICE_LABEL}")
    print(f"  Binary : {bin_file}  ({bin_file.stat().st_size} bytes)")
    print(f"  Source : {_git_describe()}")
    print()
    answer = input(f"Deploy {DEVICE_LABEL}? (y/N): ").strip().lower()
    return answer == "y"


def _build(config: str) -> None:
    build_dir = REPO_ROOT / "build" / config
    if not build_dir.exists():
        raise RuntimeError(
            f"Build dir {build_dir} does not exist. Run cmake configure first "
            f"(e.g. cmake --preset {config})."
        )
    rc = _run(["cmake", "--build", str(build_dir), "--config", config])
    if rc != 0:
        raise RuntimeError(f"cmake --build failed with exit code {rc}")


def _enter_dfu_console(timeout: float) -> bool:
    """Trigger the console into DFU using the same omotion API as
    openmotion-sdk/scripts/soft_reset_console.py + scripts/enter_dfu.py."""
    from omotion.Interface import MOTIONInterface

    interface, console_connected, _, _ = MOTIONInterface.acquire_motion_interface()
    if not console_connected:
        print("❌ Console not connected — cannot trigger DFU.")
        if interface is not None:
            interface.disconnect()
        return False

    # Stop telemetry poller before tearing down the serial port, mirroring
    # soft_reset_console.py — avoids a cascade of ClearCommError logs.
    try:
        interface.console_module.telemetry.stop()
    except Exception:
        pass

    print("[*] Requesting DFU mode …")
    try:
        ok = interface.console_module.enter_dfu()
    except Exception as exc:
        print(f"❌ enter_dfu raised: {exc}")
        ok = False
    finally:
        interface.disconnect()
    return bool(ok)


def _wait_for_console_comeback(timeout: float) -> bool:
    """Poll the omotion interface until the console reports connected."""
    from omotion.Interface import MOTIONInterface

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        interface, console_connected, _, _ = MOTIONInterface.acquire_motion_interface()
        if interface is not None:
            interface.disconnect()
        if console_connected:
            return True
        time.sleep(0.5)
    return False


def _soft_reset_console() -> bool:
    """Best-effort soft reset via omotion. Returns True if the call succeeded."""
    from omotion.Interface import MOTIONInterface

    interface, console_connected, _, _ = MOTIONInterface.acquire_motion_interface()
    try:
        if not console_connected:
            return False
        try:
            interface.console_module.telemetry.stop()
        except Exception:
            pass
        return bool(interface.console_module.soft_reset())
    finally:
        if interface is not None:
            interface.disconnect()


def main() -> int:
    args = parse_args()

    # Resolve everything that can fail without touching the device.
    try:
        project = read_project_name(REPO_ROOT / "CMakeLists.txt")
        bin_file = bin_path_for(REPO_ROOT, args.config, project)
        dfu_util = resolve_dfu_util(args.dfu_util)
    except RuntimeError as e:
        print(f"❌ {e}")
        return 1

    try:
        import omotion  # noqa: F401
    except ImportError:
        print("❌ omotion not installed in this Python env. "
              "Run `pip install -e ../openmotion-sdk` (path may differ).")
        return 1

    if not args.no_build:
        try:
            _build(args.config)
        except RuntimeError as e:
            print(f"❌ {e}")
            return 1

    if not bin_file.exists():
        print(f"❌ Binary not found: {bin_file}")
        print("   Run `cmake --build build/{cfg}` first or drop --no-build.".format(cfg=args.config))
        return 1

    if not args.no_confirm:
        if not _confirm(bin_file):
            print("Aborted.")
            return 0

    if not _enter_dfu_console(timeout=DFU_ENUM_TIMEOUT_S):
        return 1

    print(f"[*] Waiting for DFU device ({DFU_VID_PID_STR}) to enumerate …")
    if not wait_for_dfu_device(dfu_util, timeout=DFU_ENUM_TIMEOUT_S):
        print("❌ DFU device did not appear within "
              f"{DFU_ENUM_TIMEOUT_S:.0f}s.")
        print("   On Windows, bind WinUSB to the STM32 DFU interface "
              "(PID 0xDF11) via Zadig.")
        return 1

    print(f"[*] Flashing {bin_file.name} via dfu-util …")
    cmd = [
        dfu_util,
        "-d", DFU_VID_PID_STR,
        "-a", "0",
        "-s", f"0x{FLASH_BASE:08x}:leave",
        "-D", str(bin_file),
    ]
    rc = _run(cmd)
    if rc != 0:
        print(f"❌ dfu-util exited with {rc}. Device is still in DFU; rerun deploy.py to retry.")
        return 1

    print("[*] leaveDFU sent. Waiting for console to come back …")
    if _wait_for_console_comeback(timeout=COMEBACK_TIMEOUT_S):
        print("✅ Console is back online.")
        return 0

    if args.post_reset:
        print("[*] Console did not come back; attempting soft_reset …")
        if _soft_reset_console() and _wait_for_console_comeback(timeout=COMEBACK_TIMEOUT_S):
            print("✅ Console recovered via soft_reset.")
            return 0
        print("⚠️  soft_reset did not bring the console back.")

    print("⚠️  Console did not re-enumerate. Please toggle power on the console.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Verify the script still loads with --help (no hardware needed)**

Run: `python scripts/deploy.py --help`
Expected: argparse help text prints, exits 0. If imports fail (e.g. `_deploy_helpers` not on sys.path), fix before commit.

- [ ] **Step 3: Verify unit tests still pass**

Run: `pytest tests/test_deploy_helpers.py -v`
Expected: 7 passed (unchanged from Task 3).

- [ ] **Step 4: Commit**

```bash
git add scripts/deploy.py
git commit -m "feat(deploy): wire main() for full build/DFU/flash/recover flow"
```

---

## Task 5: Console — add `flash` CMake target

**Repo:** `openmotion-console-fw`

**Files:**
- Modify: `openmotion-console-fw/CMakeLists.txt`

- [ ] **Step 1: Append the flash target after the existing post-build block**

Open `CMakeLists.txt` and append at the end (after the `add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD ...)` block at line 144-148):

```cmake
# Build + DFU-flash target. Invokes scripts/deploy.py which:
#   1. enters DFU via the omotion SDK
#   2. waits for the STM32 ROM bootloader to enumerate
#   3. flashes the .bin via dfu-util with :leave for auto-reset
# Requires: dfu-util on PATH, omotion installed in the active Python env.
find_program(PYTHON_FOR_DEPLOY NAMES python3 python)
if(PYTHON_FOR_DEPLOY)
    add_custom_target(flash
        COMMAND ${PYTHON_FOR_DEPLOY} ${CMAKE_SOURCE_DIR}/scripts/deploy.py
                --config $<CONFIG> --no-build
        DEPENDS ${CMAKE_PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        USES_TERMINAL
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} via DFU"
    )
else()
    message(WARNING "Python not found — `flash` target will not be available")
endif()
```

- [ ] **Step 2: Re-configure CMake to pick up the new target**

Run from repo root: `cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake`
Expected: configure succeeds; output contains `Configuring done`.

- [ ] **Step 3: Verify the target was registered**

Run: `cmake --build build/Debug --target help`
Expected: output lists `... flash` among available targets.

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: add `flash` CMake target invoking scripts/deploy.py"
```

---

## Task 6: Sensor — mirror Tasks 1-4 with sensor-specific bits

**Repo:** `openmotion-sensor-fw`

**Files:**
- Create: `openmotion-sensor-fw/scripts/deploy.py`
- Create: `openmotion-sensor-fw/scripts/_deploy_helpers.py`
- Create: `openmotion-sensor-fw/tests/test_deploy_helpers.py`
- Create: `openmotion-sensor-fw/tests/conftest.py`

Sensor `deploy.py` differs from console in three places:
1. Adds required `--device left|right` argument.
2. `_enter_dfu_sensor()` uses `interface.left` / `interface.right` instead of `console_module`.
3. Post-flash recovery is a printed hint only (no `soft_reset()` equivalent for sensors).

- [ ] **Step 1: Copy `_deploy_helpers.py` from console-fw verbatim**

`scripts/_deploy_helpers.py` is identical to the console version. Copy it. The `read_project_name` regex still works for sensor's `set(CMAKE_PROJECT_NAME motion-sensor-fw)`.

- [ ] **Step 2: Copy `tests/conftest.py` and `tests/test_deploy_helpers.py` verbatim**

These tests use `tmp_path` and don't reference any project-specific name, so they pass unmodified in the sensor repo.

- [ ] **Step 3: Run the tests to verify they pass in the new repo**

Run from `openmotion-sensor-fw/`: `pytest tests/test_deploy_helpers.py -v`
Expected: 7 passed.

- [ ] **Step 4: Write `scripts/deploy.py` for the sensor**

```python
#!/usr/bin/env python3
"""deploy.py — Build, DFU-flash, and recover a MOTION sensor module.

Usage:
    python scripts/deploy.py --device left|right
                             [--config Debug|Release] [--no-build]
                             [--dfu-util PATH] [--post-reset] [--no-confirm]
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from _deploy_helpers import (  # noqa: E402
    bin_path_for,
    read_project_name,
    resolve_dfu_util,
    wait_for_dfu_device,
    DFU_VID_PID_STR,
)

COMEBACK_TIMEOUT_S = 10.0
DFU_ENUM_TIMEOUT_S = 10.0
FLASH_BASE = 0x08000000


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Build, enter DFU, flash, and reset a MOTION sensor module."
    )
    p.add_argument("--device", choices=("left", "right"), required=True,
                   help="Which sensor side to flash (required to avoid wrong-side mistakes)")
    p.add_argument("--config", choices=("Debug", "Release"), default="Debug")
    p.add_argument("--no-build", action="store_true")
    p.add_argument("--dfu-util", default=None)
    p.add_argument("--post-reset", action="store_true",
                   help="If sensor doesn't re-enumerate after leaveDFU, print extra hints")
    p.add_argument("--no-confirm", action="store_true")
    return p.parse_args()


def _run(cmd: list[str], **kw) -> int:
    print(f"+ {' '.join(cmd)}")
    return subprocess.call(cmd, **kw)


def _git_describe() -> str:
    try:
        out = subprocess.check_output(
            ["git", "describe", "--tags", "--dirty", "--always"],
            cwd=REPO_ROOT, text=True,
        )
        return out.strip()
    except Exception:
        return "unknown"


def _confirm(device: str, bin_file: Path) -> bool:
    print()
    print(f"  Device : {device} sensor")
    print(f"  Binary : {bin_file}  ({bin_file.stat().st_size} bytes)")
    print(f"  Source : {_git_describe()}")
    print()
    answer = input(f"Deploy {device} sensor? (y/N): ").strip().lower()
    return answer == "y"


def _build(config: str) -> None:
    build_dir = REPO_ROOT / "build" / config
    if not build_dir.exists():
        raise RuntimeError(
            f"Build dir {build_dir} does not exist. Run cmake configure first."
        )
    rc = _run(["cmake", "--build", str(build_dir), "--config", config])
    if rc != 0:
        raise RuntimeError(f"cmake --build failed with exit code {rc}")


def _sensor_handle(interface, device: str):
    return interface.left if device == "left" else interface.right


def _enter_dfu_sensor(device: str, timeout: float) -> bool:
    """Trigger the named sensor into DFU mode via omotion."""
    from omotion.Interface import MOTIONInterface

    interface, _, left_connected, right_connected = MOTIONInterface.acquire_motion_interface()
    connected = left_connected if device == "left" else right_connected
    if not connected:
        print(f"❌ {device} sensor not connected — cannot trigger DFU.")
        if interface is not None:
            interface.disconnect()
        return False

    print(f"[*] Requesting DFU mode on {device} sensor …")
    try:
        sensor = _sensor_handle(interface, device)
        ok = sensor.enter_dfu()
    except Exception as exc:
        print(f"❌ enter_dfu raised: {exc}")
        ok = False
    finally:
        interface.disconnect()
    return bool(ok)


def _wait_for_sensor_comeback(device: str, timeout: float) -> bool:
    """Poll omotion until the named sensor reports connected."""
    from omotion.Interface import MOTIONInterface

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        interface, _, left_connected, right_connected = MOTIONInterface.acquire_motion_interface()
        if interface is not None:
            interface.disconnect()
        ok = left_connected if device == "left" else right_connected
        if ok:
            return True
        time.sleep(0.5)
    return False


def main() -> int:
    args = parse_args()

    try:
        project = read_project_name(REPO_ROOT / "CMakeLists.txt")
        bin_file = bin_path_for(REPO_ROOT, args.config, project)
        dfu_util = resolve_dfu_util(args.dfu_util)
    except RuntimeError as e:
        print(f"❌ {e}")
        return 1

    try:
        import omotion  # noqa: F401
    except ImportError:
        print("❌ omotion not installed in this Python env. "
              "Run `pip install -e ../openmotion-sdk` (path may differ).")
        return 1

    if not args.no_build:
        try:
            _build(args.config)
        except RuntimeError as e:
            print(f"❌ {e}")
            return 1

    if not bin_file.exists():
        print(f"❌ Binary not found: {bin_file}")
        return 1

    if not args.no_confirm:
        if not _confirm(args.device, bin_file):
            print("Aborted.")
            return 0

    if not _enter_dfu_sensor(args.device, timeout=DFU_ENUM_TIMEOUT_S):
        return 1

    print(f"[*] Waiting for DFU device ({DFU_VID_PID_STR}) to enumerate …")
    if not wait_for_dfu_device(dfu_util, timeout=DFU_ENUM_TIMEOUT_S):
        print("❌ DFU device did not appear within "
              f"{DFU_ENUM_TIMEOUT_S:.0f}s.")
        print("   On Windows, bind WinUSB to the STM32 DFU interface "
              "(PID 0xDF11) via Zadig.")
        return 1

    print(f"[*] Flashing {bin_file.name} via dfu-util …")
    cmd = [
        dfu_util,
        "-d", DFU_VID_PID_STR,
        "-a", "0",
        "-s", f"0x{FLASH_BASE:08x}:leave",
        "-D", str(bin_file),
    ]
    rc = _run(cmd)
    if rc != 0:
        print(f"❌ dfu-util exited with {rc}. Device is still in DFU; rerun deploy.py to retry.")
        return 1

    print(f"[*] leaveDFU sent. Waiting for {args.device} sensor to come back …")
    if _wait_for_sensor_comeback(args.device, timeout=COMEBACK_TIMEOUT_S):
        print(f"✅ {args.device.capitalize()} sensor is back online.")
        return 0

    # No software path to power-cycle a single sensor — surface a clear hint.
    print(f"⚠️  {args.device} sensor did not re-enumerate.")
    print("   Please toggle power on the console "
          "(this power-cycles the sensors).")
    if args.post_reset:
        print("   --post-reset: no additional automatic recovery is available "
              "for sensors; manual power cycle required.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 5: Verify the sensor script loads**

Run: `python scripts/deploy.py --help`
Expected: argparse help prints with `--device {left,right}` listed; exits 0.

- [ ] **Step 6: Verify --device is required**

Run: `python scripts/deploy.py --config Debug`
Expected: argparse error `the following arguments are required: --device`, non-zero exit.

- [ ] **Step 7: Commit**

```bash
git add scripts/deploy.py scripts/_deploy_helpers.py tests/conftest.py tests/test_deploy_helpers.py
git commit -m "feat(deploy): add scripts/deploy.py for sensor module"
```

---

## Task 7: Sensor — add `flash-left` and `flash-right` CMake targets

**Repo:** `openmotion-sensor-fw`

**Files:**
- Modify: `openmotion-sensor-fw/CMakeLists.txt`

- [ ] **Step 1: Append the flash targets after the existing post-build block**

Open `CMakeLists.txt` and append at the end (after the `add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD ...)` block):

```cmake
# Build + DFU-flash targets, one per sensor side. Invokes scripts/deploy.py.
# `--device` is required by the script, so we expose two separate targets.
find_program(PYTHON_FOR_DEPLOY NAMES python3 python)
if(PYTHON_FOR_DEPLOY)
    foreach(SIDE left right)
        add_custom_target(flash-${SIDE}
            COMMAND ${PYTHON_FOR_DEPLOY} ${CMAKE_SOURCE_DIR}/scripts/deploy.py
                    --device ${SIDE} --config $<CONFIG> --no-build
            DEPENDS ${CMAKE_PROJECT_NAME}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            USES_TERMINAL
            COMMENT "Flashing ${CMAKE_PROJECT_NAME} (${SIDE} sensor) via DFU"
        )
    endforeach()
else()
    message(WARNING "Python not found — `flash-left`/`flash-right` targets will not be available")
endif()
```

- [ ] **Step 2: Re-configure CMake**

Run: `cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake`
Expected: configure succeeds.

- [ ] **Step 3: Verify both targets registered**

Run: `cmake --build build/Debug --target help`
Expected: output lists both `... flash-left` and `... flash-right`.

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: add flash-left/flash-right CMake targets"
```

---

## Task 8: Manual hardware verification (both repos)

**Prerequisites:** Bench setup with console + at least one sensor connected over USB. `dfu-util` on PATH. `omotion` installed in the active Python env. Windows: WinUSB bound to STM32 DFU interface (PID 0xDF11) via Zadig — bind it the first time the DFU device shows up if not already done.

These are manual checks. Record observations in the PR description / commit notes.

- [ ] **Step 1: Console happy path**

From `openmotion-console-fw/`:
```
cmake --build build/Debug --target flash
```
Expected sequence:
1. Build runs (or no-op if up-to-date).
2. Prompt: `Deploy console? (y/N):` — answer `y`.
3. `[*] Requesting DFU mode …` — console becomes a DFU device.
4. `[*] Waiting for DFU device (0483:df11) to enumerate …` — succeeds within ~2s.
5. `[*] Flashing motion-console-fw.bin via dfu-util …` — dfu-util progress bar runs to completion.
6. `[*] leaveDFU sent. Waiting for console to come back …`
7. `✅ Console is back online.` — exit 0.

Then verify the new firmware: `python -c "from omotion.Interface import MOTIONInterface; i,c,l,r = MOTIONInterface.acquire_motion_interface(); print(i.console_module.ping())"` — should print True.

- [ ] **Step 2: Sensor happy path (both sides)**

From `openmotion-sensor-fw/`, for each side:
```
cmake --build build/Debug --target flash-left
# then
cmake --build build/Debug --target flash-right
```
Expected: same sequence as console, but with sensor-specific messages and ending in `✅ Left sensor is back online.` / `✅ Right sensor is back online.`.

- [ ] **Step 3: Pre-flight failures (negative tests)**

Console repo:
1. Pass `--config Bogus` → argparse rejects, non-zero exit, no device touched.
2. Hide `dfu-util` (`PATH=...` without it, or rename) → script prints "dfu-util not found …" and exits 1 before any device interaction.
3. Disconnect console, run `flash` → "Console not connected — cannot trigger DFU." Non-zero exit.

Sensor repo:
4. Run `python scripts/deploy.py` (no `--device`) → argparse error, non-zero exit.

- [ ] **Step 4: Cancellation path**

Run `cmake --build build/Debug --target flash`, answer `n` at the prompt.
Expected: `Aborted.`, exit 0, console still in normal operating mode (verify with a ping).

- [ ] **Step 5: --post-reset path (console)**

This is harder to trigger naturally. Optional smoke test: run `python scripts/deploy.py --post-reset --no-confirm` after a known-good flash and confirm the script still exits cleanly when the device comes back without needing the soft-reset.

- [ ] **Step 6: Record verification in the PR**

In the PR description, note: which steps were exercised, on what hardware (console serial number + sensor sides), and any observations. The CMake targets and Python helpers are CI-friendly but the device interaction is not — manual record is the only audit trail.

---

## Self-Review Summary

**Spec coverage:**
- Workflow (build → enter_dfu → wait → dfu-util `:leave` → comeback) — Tasks 4, 6.
- CMake `flash` target (console) — Task 5.
- CMake `flash-left`/`flash-right` (sensor) — Task 7.
- Confirmation prompt — Tasks 4, 6 (in `_confirm`).
- Pre-flight checks (dfu-util, omotion, bin existence) — Task 4 (lines in `main()`), Task 6.
- Post-flash recovery (console soft_reset, sensor power-toggle hint) — Tasks 4, 6.
- Edge cases (wrong-side flash, dfu-util missing, DFU never enumerates, mid-flash failure, wedged after leave) — covered across Tasks 4, 6, 8.
- Tests for pure helpers — Tasks 2, 3.

**No-placeholder scan:** No "TBD"/"TODO"/"implement later" patterns. All code blocks are complete.

**Type consistency:** All helpers use the names `read_project_name`, `bin_path_for`, `resolve_dfu_util`, `wait_for_dfu_device`, and constant `DFU_VID_PID_STR`. These names are referenced identically in `scripts/deploy.py` and `tests/test_deploy_helpers.py`.

**Known small risks:**
- The `omotion` API surface (`interface.console`, `interface.left`, `interface.right` vs `interface.console_module`) varies between `enter_dfu.py` (which uses `interface.console.enter_dfu()`) and `soft_reset_console.py` (which uses `interface.console_module.soft_reset()`). The plan uses `console_module` and `left`/`right` directly per the matching existing scripts. If implementation reveals these don't exist, fall back to the API the working script uses, no design change needed.

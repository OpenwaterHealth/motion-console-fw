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
    """Trigger the console into DFU using the current omotion API."""
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=True, wait_timeout=timeout)
    try:
        if not interface.console.is_connected():
            print("❌ Console not connected — cannot trigger DFU.")
            return False

        # Stop telemetry poller before tearing down the serial port.
        # Mirrors the cleanup pattern from MotionConsole/soft_reset to
        # avoid a cascade of ClearCommError logs.
        try:
            interface.console.telemetry.stop()
        except Exception:
            pass

        print("[*] Requesting DFU mode …")
        try:
            return bool(interface.console.enter_dfu())
        except Exception as exc:
            print(f"❌ enter_dfu raised: {exc}")
            return False
    finally:
        interface.stop()


def _wait_for_console_comeback(timeout: float) -> bool:
    """Construct the interface ONCE and poll the console handle."""
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=False)
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if interface.console.is_connected():
                return True
            time.sleep(0.5)
        return False
    finally:
        interface.stop()


def _soft_reset_console() -> bool:
    """Best-effort soft reset via omotion."""
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=True, wait_timeout=5.0)
    try:
        if not interface.console.is_connected():
            return False
        try:
            interface.console.telemetry.stop()
        except Exception:
            pass
        return bool(interface.console.soft_reset())
    finally:
        interface.stop()


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
    return 1


if __name__ == "__main__":
    sys.exit(main())

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

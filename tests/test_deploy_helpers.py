import shutil
import subprocess
import textwrap
from pathlib import Path
from unittest.mock import patch

import pytest

from _deploy_helpers import (
    read_project_name,
    bin_path_for,
    resolve_dfu_util,
    wait_for_dfu_device,
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

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

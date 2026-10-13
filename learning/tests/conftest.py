"""Fixtures for core tests, which need the C++ build (section 10 of docs/learning_track.md).

Core tests run when FLEET_BUILD names the build folder, for example FLEET_BUILD=build. They
skip when FLEET_BUILD is unset, and fail when it is set and a tool is missing.
"""

import os
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def fleet_build() -> Path:
    """The C++ build folder that FLEET_BUILD names, relative to the repo root or absolute."""
    value = os.environ.get("FLEET_BUILD")
    if not value:
        pytest.skip("core test: set FLEET_BUILD to the C++ build folder")
    return (REPO / value).resolve()


@pytest.fixture
def run_sweep(fleet_build: Path) -> Path:
    """The run_sweep tool in the build folder."""
    path = fleet_build / "run_sweep"
    if not os.access(path, os.X_OK):
        pytest.fail(f"{path} is missing; build fleet_core first (CONTRIBUTING.md)")
    return path


@pytest.fixture
def predict_cost(fleet_build: Path) -> Path:
    """The predict_cost tool in the build folder."""
    path = fleet_build / "predict_cost"
    if not os.access(path, os.X_OK):
        pytest.fail(f"{path} is missing; build fleet_core first (CONTRIBUTING.md)")
    return path

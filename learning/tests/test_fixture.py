"""The fixture is a real run: the core reproduces it from its own config (section 10)."""

import subprocess
from pathlib import Path

import pandas as pd
import pytest
import yaml

FIXTURE = Path(__file__).parent / "fixtures" / "run_tiny"
STALE = "the core no longer reproduces the fixture; regenerate it (tests/fixtures/README.md)"


def summary_without_timing(run_dir: Path) -> pd.DataFrame:
    """summary.csv without the columns that measure wall time (*_ms, *_seconds)."""
    frame = pd.read_csv(run_dir / "summary.csv", dtype=str, keep_default_na=False)
    return frame.drop(columns=[c for c in frame.columns if c.endswith(("_ms", "_seconds"))])


@pytest.mark.core
def test_fixture_config_reproduces_the_fixture(tmp_path, run_sim):
    out = tmp_path / "run_tiny"
    command = [str(run_sim), "--config", str(FIXTURE / "config.yaml"), "--out", str(out)]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    rows = (out / "assignments.csv").read_bytes()
    assert rows == (FIXTURE / "assignments.csv").read_bytes(), STALE
    assert summary_without_timing(out).equals(summary_without_timing(FIXTURE)), STALE
    # The committed config differs from the one the core writes only in map.file, which is
    # relative to the fixture folder instead of absolute.
    written = yaml.safe_load((out / "config.yaml").read_text())
    committed = yaml.safe_load((FIXTURE / "config.yaml").read_text())
    assert written["map"].pop("file") == str((FIXTURE / committed["map"].pop("file")).resolve())
    assert written == committed

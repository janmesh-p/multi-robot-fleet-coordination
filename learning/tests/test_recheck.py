"""Tests for recheck.py: which phases it picks, and that a changed run does not pass."""

import json
import subprocess
from pathlib import Path

import pytest

from fleet_learning import collect, recheck

REPO = Path(__file__).resolve().parents[2]


def git(repo: Path, *args: str) -> str:
    done = subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True, text=True)
    return done.stdout.strip()


def test_find_phases_keeps_commits_with_the_core_and_without_the_new_one(tmp_path):
    git(tmp_path, "init", "-q")
    commit = ["-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m"]
    commits = []
    for name in ("before", "core", "after", "new"):
        git(tmp_path, *commit, name)
        commits.append(git(tmp_path, "rev-parse", "HEAD")[:12])
    for commit in commits:
        phase = tmp_path / collect.DATA_ROOT / commit
        phase.mkdir(parents=True)
        (phase / "manifest.json").write_text(json.dumps({"commit": commit}), encoding="utf-8")

    found = recheck.find_phases(tmp_path, commits[1], commits[3])

    assert [path.name for path in found] == sorted(commits[1:3])


@pytest.mark.core
def test_recheck_sweep_passes_the_same_run_and_flags_a_changed_one(run_sweep, tmp_path):
    manifest = collect.collect(
        REPO,
        tmp_path / "data",
        allocators=["nearest"],
        rates=[0.1],
        fleet=[5],
        seeds=[1000],
        allow_dirty=True,
        run_sweep=run_sweep,
    )
    (record,) = manifest["sweeps"]
    scratch = tmp_path / "scratch"
    scratch.mkdir()

    same = recheck.recheck_sweep(REPO, record, scratch, run_sweep)
    assert same["problems"] == [] and same["runs"] == 1 and same["rows"] > 0

    out = Path(record["command"][record["command"].index("--out") + 1])
    (assignments,) = out.glob("*/assignments.csv")
    assignments.write_bytes(assignments.read_bytes() + b"\n")
    changed = recheck.recheck_sweep(REPO, record, scratch, run_sweep)
    assert changed["problems"] == [f"{assignments.parent.name}: assignments.csv differs"]

    sweep = out / "sweep.csv"
    header, row = sweep.read_text(encoding="utf-8").splitlines()[:2]
    cells = row.split(",")
    cells[header.split(",").index("log_hash")] = "0" * 16
    sweep.write_text(f"{header}\n{','.join(cells)}\n", encoding="utf-8")
    assert len(recheck.recheck_sweep(REPO, record, scratch, run_sweep)["problems"]) == 2

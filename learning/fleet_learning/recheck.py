"""Rerun finished phases on a newer core and check that nothing changed.

Stage 6 added faults, and default.yaml turns them off. This checks that the core still gives
the same runs without them, so the rounds made on the Stage 5 core hold on Stage 6:

    python -m fleet_learning.recheck

It builds the C++ tools at the checked-out commit, then finds every manifest.json under
learning/data whose phase ran on the Stage 5 core: at a commit that contains `--core` but not
`--new`. That is the Stage 5 and late-robot collections and their closed-loop tuning and final
runs. Each sweep runs again with the command its manifest recorded, into a temporary folder.
run_sweep --check compares every sweep.csv column except the timing ones, log_hash included,
and every run's assignments.csv is compared byte for byte. Exit 0 when all of it is identical.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

from fleet_learning import collect

STAGE5_CORE = "6c0e93405694"  # the Stage 5 commit
STAGE6_CORE = "a0f820f"  # the Stage 6 commit


def contains(repo: Path, commit: str, ancestor: str) -> bool:
    """Whether `commit` has `ancestor` in its history (a commit contains itself)."""
    result = subprocess.run(
        ["git", "merge-base", "--is-ancestor", ancestor, commit], cwd=repo, capture_output=True
    )
    return result.returncode == 0


def find_phases(repo: Path, core: str, new: str) -> list[Path]:
    """Folders under learning/data whose manifest commit contains `core` but not `new`."""
    found = []
    for manifest in sorted((repo / collect.DATA_ROOT).rglob("manifest.json")):
        commit = json.loads(manifest.read_text(encoding="utf-8"))["commit"]
        if contains(repo, commit, core) and not contains(repo, commit, new):
            found.append(manifest.parent)
    return found


def recheck_sweep(
    repo: Path, record: dict[str, Any], scratch: Path, run_sweep: str | Path = collect.RUN_SWEEP
) -> dict[str, Any]:
    """Run one recorded sweep again and compare it with what it wrote the first time.

    Returns the runs and assignment rows compared, and one line per difference.
    """
    command = list(record["command"])
    command[0] = str(run_sweep)
    at = command.index("--out")
    old_out = repo / command[at + 1]
    new_out = scratch / record["name"]
    command[at + 1] = str(new_out)
    if "--runs" not in command:
        command.append("--runs")
    command += ["--check", str(old_out / "sweep.csv")]
    done = subprocess.run(command, cwd=repo, capture_output=True, text=True)
    result: dict[str, Any] = {"runs": 0, "rows": 0, "problems": []}
    if done.returncode != 0:
        lines = (done.stdout.strip() or done.stderr.strip()).splitlines()
        result["problems"].append(f"{record['name']}: {lines[-1].strip() if lines else 'failed'}")
    for run in sorted(path for path in old_out.iterdir() if (path / "assignments.csv").is_file()):
        old = (run / "assignments.csv").read_bytes()
        new = new_out / run.name / "assignments.csv"
        if not new.is_file() or new.read_bytes() != old:
            result["problems"].append(f"{run.name}: assignments.csv differs")
        result["runs"] += 1
        result["rows"] += max(old.count(b"\n") - 1, 0)
    shutil.rmtree(new_out, ignore_errors=True)  # run folders hold large traces
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fleet_learning.recheck",
        description="Rerun every phase made on the Stage 5 core with the checked-out core and "
        "check that sweep.csv and assignments.csv come out the same. Run from the repo root.",
    )
    parser.add_argument("--core", default=STAGE5_CORE, help="phases ran on commits with this one")
    parser.add_argument("--new", default=STAGE6_CORE, help="and on commits without this one")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 1, help="sweeps at a time")
    args = parser.parse_args(argv)
    repo = Path.cwd()
    head = collect.git_commit(repo)
    if not contains(repo, head, args.new):
        print(f"error: {head} does not contain {args.new}; check out main", file=sys.stderr)
        return 1
    phases = find_phases(repo, args.core, args.new)
    if not phases:
        print(f"error: no phase under {collect.DATA_ROOT} ran on {args.core}", file=sys.stderr)
        return 1
    collect._print(f"commit     {head}")
    collect.build(repo)
    rows = runs = sweeps = problems = 0
    with tempfile.TemporaryDirectory() as tmp, ThreadPoolExecutor(args.jobs) as pool:
        for phase in phases:
            records = json.loads((phase / "manifest.json").read_text(encoding="utf-8"))["sweeps"]
            results = list(pool.map(lambda r: recheck_sweep(repo, r, Path(tmp)), records))
            found = [p for result in results for p in result["problems"]]
            phase_runs = sum(result["runs"] for result in results)
            state = "identical" if not found else f"{len(found)} differences"
            collect._print(
                f"{phase.relative_to(repo)}: {len(records)} sweeps, {phase_runs} runs, {state}"
            )
            for problem in found[:10]:
                collect._print(f"  {problem}")
            sweeps += len(records)
            runs += phase_runs
            rows += sum(result["rows"] for result in results)
            problems += len(found)
    collect._print(
        f"total      {len(phases)} phases, {sweeps} sweeps, {runs} runs, {rows} assignment rows, "
        f"{problems} differences"
    )
    return 0 if problems == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

"""Collect training data: write sweep configs, build the C++ tools, and run run_sweep.

Follows section 7 of docs/learning_track.md. One collection writes everything under
learning/data/<commit>/: configs/, runs/, and manifest.json. The manifest comes last, and only
when every sweep succeeded, so a folder without one holds an incomplete collection.

Every logging allocator runs since Stage 3, but pibt arrives in Stage 4. Until then, the
plumbing collection of section 11 uses independent_astar:

    python -m fleet_learning.collect --planner independent_astar --seeds 3000-3004
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import shlex
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

ALLOCATORS: tuple[str, ...] = ("nearest", "hungarian", "cost_aware")
ARRIVAL_RATES: tuple[float, ...] = (0.05, 0.1, 0.2)  # orders per step
PLANNERS: tuple[str, ...] = ("pibt", "independent_astar")  # independent_astar: plumbing only
FLEET_SIZES: tuple[int, ...] = (5, 10, 25)
SEEDS: tuple[int, ...] = tuple(range(1000, 1040))  # training, validation, and offline test
TRACE_FILES: tuple[str, ...] = ("positions.csv", "steps.csv")

BASE_CONFIG = Path("benchmarks/configs/default.yaml")
DATA_ROOT = Path("learning/data")
RUN_SWEEP = Path("build/run_sweep")
BUILD_COMMANDS: tuple[tuple[str, ...], ...] = (
    ("cmake", "-S", "fleet_core", "-B", "build", "-G", "Ninja"),
    ("cmake", "--build", "build"),
)
POLL_SECONDS = 0.05

Log = Callable[[str], None]


def _quiet(_: str) -> None:
    pass


def _print(line: str) -> None:
    print(line, flush=True)  # keeps progress in order with errors when output is piped


class CollectError(RuntimeError):
    """A collection that cannot start, or a step that failed. The message says what happened."""


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True)
    if result.returncode != 0:
        raise CollectError(f"git {' '.join(args)} failed in {repo}: {result.stderr.strip()}")
    return result.stdout


def git_commit(repo: Path) -> str:
    """The first 12 characters of `git rev-parse HEAD` in `repo`."""
    return _git(repo, "rev-parse", "HEAD").strip()[:12]


def git_changes(repo: Path) -> list[str]:
    """The lines `git status --porcelain` prints in `repo`; none when the tree is clean."""
    return _git(repo, "status", "--porcelain").splitlines()


def build(repo: Path) -> None:
    """Build the C++ tools with the commands of section 7. Raises CollectError on failure."""
    for command in BUILD_COMMANDS:
        result = subprocess.run(command, cwd=repo, capture_output=True, text=True)
        if result.returncode != 0:
            raise CollectError(
                f"{shlex.join(command)} exited with {result.returncode}:\n"
                f"{result.stdout}{result.stderr}"
            )


def parse_int_list(text: str) -> list[int]:
    """Numbers as run_sweep reads them: "5,10,25", "1000-1039" (inclusive), or "1-3,7"."""
    values: list[int] = []
    for item in text.split(","):
        first, dash, last = item.strip().partition("-")
        try:
            span = range(int(first), int(last) + 1) if dash else [int(first)]
        except ValueError:
            message = f"{text!r}: expected numbers or ranges such as 5,10,25 or 1000-1039"
            raise ValueError(message) from None
        if not span:
            raise ValueError(f"{text!r}: range {item} ends before it starts")
        values.extend(span)
    return values


def format_int_list(values: Sequence[int]) -> str:
    """Numbers for run_sweep, with each run of consecutive values as a range: "1-3,7"."""
    spans: list[list[int]] = []
    for value in values:
        if spans and value == spans[-1][1] + 1:
            spans[-1][1] = value
        else:
            spans.append([value, value])
    return ",".join(str(a) if a == b else f"{a}-{b}" for a, b in spans)


def rate_code(rate: float) -> str:
    """An arrival rate (orders per step) times 100, as three digits: 0.1 gives "010"."""
    hundredths = round(rate * 100)
    if not 1 <= hundredths <= 999 or abs(rate * 100 - hundredths) > 1e-9:
        raise ValueError(f"arrival rate {rate}: use a multiple of 0.01 from 0.01 to 9.99")
    return f"{hundredths:03d}"


def run_name(allocator: str, rate: float) -> str:
    """`collect_<allocator>_r<rate x 100, three digits>`, for example collect_hungarian_r010."""
    return f"collect_{allocator}_r{rate_code(rate)}"


def parse_rates(text: str) -> tuple[float, ...]:
    """Arrival rates such as "0.05,0.1,0.2", each a multiple of 0.01."""
    try:
        rates = tuple(float(rate) for rate in text.split(","))
    except ValueError:
        raise ValueError(f"{text!r}: expected rates such as 0.05,0.1,0.2") from None
    for rate in rates:
        rate_code(rate)
    return rates


def _check(
    values: Sequence[Any], what: str, allowed: Sequence[Any] | None = None, least: int = 0
) -> tuple[Any, ...]:
    values = tuple(values)
    if not values:
        raise ValueError(f"{what}: give at least one")
    if len(set(values)) != len(values):
        raise ValueError(f"{what}: {', '.join(map(str, values))} has duplicates")
    for value in values:
        if allowed is not None and value not in allowed:
            raise ValueError(f"{what}: {value} is not one of {', '.join(allowed)}")
        if allowed is None and value < least:
            raise ValueError(f"{what}: {value} is below {least}")
    return values


def make_config(
    base: Mapping[str, Any], base_dir: Path, changes: Mapping[str, Any]
) -> dict[str, Any]:
    """A copy of `base` with `changes` ({"section.key": value}) applied.

    A relative `map.file` or `allocation.model_file` becomes absolute against `base_dir`, the
    folder of the base config, because the C++ side resolves it against the folder of the config
    that holds it (section 7).
    """
    config = copy.deepcopy(dict(base))
    for dotted, value in changes.items():
        section, key = dotted.split(".")
        config.setdefault(section, {})[key] = value
    for section, key in (("map", "file"), ("allocation", "model_file")):
        value = (config.get(section) or {}).get(key)
        if value is not None and not Path(value).is_absolute():
            config[section][key] = str((base_dir / value).resolve())
    return config


def grid(allocators: Sequence[str], rates: Sequence[float], planner: str) -> list[dict[str, Any]]:
    """The changes for each cell of the section 7 grid: allocators outer, arrival rates inner."""
    return [
        {
            "run.name": run_name(allocator, rate),
            "allocation.method": allocator,
            "planning.method": planner,
            "orders.arrival_rate": rate,
        }
        for allocator in allocators
        for rate in rates
    ]


@dataclass
class Sweep:
    """One run_sweep call: the config it runs, its --out folder, and the command."""

    name: str
    config: Path
    out: Path
    command: list[str]


def _shown(path: Path, repo: Path) -> str:
    """`path` relative to the repo root when it is inside it, else absolute."""
    try:
        return str(path.relative_to(repo))
    except ValueError:
        return str(path)


def delete_traces(out: Path) -> tuple[int, int]:
    """Delete TRACE_FILES from the run directories in `out`. Returns (files, bytes) deleted."""
    files = size = 0
    for run_dir in sorted(p for p in out.iterdir() if p.is_dir()):
        for name in TRACE_FILES:
            path = run_dir / name
            if path.is_file():
                size += path.stat().st_size
                path.unlink()
                files += 1
    return files, size


def run_sweeps(
    sweeps: Sequence[Sweep], repo: Path, jobs: int, keep_traces: bool, log: Log = _quiet
) -> list[dict[str, Any]]:
    """Run the sweeps from `repo`, at most `jobs` at a time, deleting traces as each one ends.

    Returns one record per sweep, in the order they finished. On the first sweep that exits
    with anything but 0, stops the others and raises CollectError with its command and output.
    """
    pending = list(sweeps)
    running: list[tuple[Sweep, subprocess.Popen, Any, float]] = []
    records: list[dict[str, Any]] = []
    try:
        while pending or running:
            while pending and len(running) < jobs:
                sweep = pending.pop(0)
                output = tempfile.TemporaryFile()
                process = subprocess.Popen(
                    sweep.command, cwd=repo, stdout=output, stderr=subprocess.STDOUT
                )
                running.append((sweep, process, output, time.monotonic()))
            finished = [item for item in running if item[1].poll() is not None]
            if not finished:
                time.sleep(POLL_SECONDS)
            for item in finished:
                running.remove(item)
                sweep, process, output, started = item
                seconds = time.monotonic() - started
                output.seek(0)
                text = output.read().decode(errors="replace")
                output.close()
                if process.returncode != 0:
                    raise CollectError(
                        f"sweep {sweep.name} exited with {process.returncode}; stopped the "
                        f"others.\ncommand: {shlex.join(sweep.command)}\noutput:\n{text}"
                    )
                files, size = (0, 0) if keep_traces else delete_traces(sweep.out)
                log(f"done       {sweep.name} in {seconds:.1f} s; deleted {files} trace files")
                records.append(
                    {
                        "name": sweep.name,
                        "command": sweep.command,
                        "wall_seconds": round(seconds, 3),
                        "trace_files_deleted": files,
                        "trace_bytes_deleted": size,
                    }
                )
    finally:
        for _, process, output, _ in running:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            output.close()
    return records


def collect(
    repo: str | Path = ".",
    data_root: str | Path | None = None,
    *,
    allocators: Sequence[str] = ALLOCATORS,
    rates: Sequence[float] = ARRIVAL_RATES,
    planner: str = "pibt",
    fleet: Sequence[int] = FLEET_SIZES,
    seeds: Sequence[int] = SEEDS,
    jobs: int | None = None,
    keep_traces: bool = False,
    allow_dirty: bool = False,
    run_sweep: str | Path | None = None,
    log: Log = _quiet,
) -> dict[str, Any]:
    """Run one collection, the six steps of section 7, and return its manifest.

    `repo` is the repo root. Data goes to <data_root>/<commit>, by default
    learning/data/<commit>. With `run_sweep` given, that tool runs the sweeps and nothing is
    built. Raises CollectError, or ValueError for bad arguments.
    """
    started = time.monotonic()
    repo = Path(repo).resolve()
    allocators = _check(allocators, "allocators", ALLOCATORS)
    rates = _check(rates, "arrival rates")
    _check([planner], "planner", PLANNERS)
    fleet = _check(fleet, "fleet sizes", least=1)
    seeds = _check(seeds, "seeds", least=0)
    cells = grid(allocators, rates, planner)
    cores = os.cpu_count() or 1
    jobs = min(cores, len(cells)) if jobs is None else jobs
    if not 1 <= jobs <= cores:
        raise ValueError(f"jobs: {jobs} is not between 1 and {cores}, the CPU cores here")

    # 1. A clean tree, unless allowed otherwise, and the commit.
    changes = git_changes(repo)
    if changes and not allow_dirty:
        raise CollectError(
            f"git status --porcelain lists {len(changes)} changes, the first: {changes[0]}. "
            "Commit them, or pass --allow-dirty (section 7)."
        )
    commit = git_commit(repo)
    base_path = repo / BASE_CONFIG
    if not base_path.is_file():
        raise CollectError(f"{base_path} not found. Run from the repo root.")
    data_dir = (repo / DATA_ROOT if data_root is None else Path(data_root).resolve()) / commit
    if data_dir.exists():
        raise CollectError(
            f"{data_dir} already exists, and each commit gets one collection (section 7). "
            "Delete or move it to collect again."
        )
    state = f"dirty, {len(changes)} changes" if changes else "clean"
    log(f"commit     {commit} ({state})")

    # 2. The C++ tools.
    if run_sweep is None:
        log(f"build      {' && '.join(shlex.join(command) for command in BUILD_COMMANDS)}")
        build(repo)
        run_sweep = repo / RUN_SWEEP
    run_sweep = Path(run_sweep).resolve()

    # 3. One config per grid cell.
    base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    sweeps = []
    for changes_of_cell in cells:
        name = changes_of_cell["run.name"]
        config = make_config(base, base_path.parent, changes_of_cell)
        path = data_dir / "configs" / f"{name}.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        header = (
            f"# Written by fleet_learning.collect at commit {commit}: {BASE_CONFIG} with\n"
            f"# {', '.join(changes_of_cell)} set, and map.file absolute.\n"
        )
        path.write_text(header + yaml.safe_dump(config, sort_keys=False))
        out = data_dir / "runs" / name
        command = [_shown(run_sweep, repo), "--config", _shown(path, repo)]
        command += ["--fleet", format_int_list(fleet), "--seeds", format_int_list(seeds)]
        command += ["--out", _shown(out, repo), "--runs"]
        sweeps.append(Sweep(name, path, out, command))
    log(f"configs    {len(sweeps)} in {_shown(data_dir / 'configs', repo)}")

    # 4 and 5. The sweeps, then their traces.
    log(
        f"sweeps     {len(sweeps)}, up to {jobs} at a time: fleet {format_int_list(fleet)}, "
        f"seeds {format_int_list(seeds)}, planner {planner}"
    )
    records = run_sweeps(sweeps, repo, jobs, keep_traces, log)

    # 6. The manifest, with the sweeps in grid order.
    order = {sweep.name: i for i, sweep in enumerate(sweeps)}
    manifest = {
        "commit": commit,
        "dirty": bool(changes),
        "changes": changes,
        "base_config": str(BASE_CONFIG),
        "allocators": list(allocators),
        "arrival_rates": list(rates),
        "planner": planner,
        "fleet_sizes": list(fleet),
        "seeds": list(seeds),
        "configs": [str(sweep.config.relative_to(data_dir)) for sweep in sweeps],
        "sweeps": sorted(records, key=lambda record: order[record["name"]]),
        "jobs": jobs,
        "keep_traces": keep_traces,
        "wall_seconds": round(time.monotonic() - started, 3),
    }
    path = data_dir / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    log(f"manifest   {_shown(path, repo)}: {len(sweeps)} sweeps in {manifest['wall_seconds']} s")
    return manifest


def _arg(parse: Callable[[str], Any]) -> Callable[[str], Any]:
    def convert(text: str) -> Any:
        try:
            return parse(text)
        except ValueError as error:
            raise argparse.ArgumentTypeError(str(error)) from error

    return convert


def _jobs(text: str) -> int:
    cores = os.cpu_count() or 1
    jobs = int(text)
    if not 1 <= jobs <= cores:
        raise ValueError(f"{jobs} is not between 1 and {cores}, the CPU cores here")
    return jobs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fleet_learning.collect",
        description="Collect training data as section 7 of docs/learning_track.md says: write "
        "one config per allocator and arrival rate, build the C++ tools, run run_sweep for each "
        "config, and write learning/data/<commit>/manifest.json. Run from the repo root.",
    )
    parser.add_argument(
        "--allocators",
        type=_arg(lambda text: _check(text.split(","), "allocators", ALLOCATORS)),
        default=ALLOCATORS,
        help="comma-separated allocation methods (default: nearest,hungarian,cost_aware)",
    )
    parser.add_argument(
        "--rates",
        type=_arg(parse_rates),
        default=ARRIVAL_RATES,
        help="comma-separated arrival rates, orders per step (default: 0.05,0.1,0.2)",
    )
    parser.add_argument(
        "--planner",
        choices=PLANNERS,
        default="pibt",
        help="planning.method (default: pibt, from Stage 4; independent_astar for plumbing)",
    )
    parser.add_argument(
        "--fleet",
        type=_arg(parse_int_list),
        default=FLEET_SIZES,
        help="fleet sizes (default: 5,10,25)",
    )
    parser.add_argument(
        "--seeds",
        type=_arg(parse_int_list),
        default=SEEDS,
        help="seeds, a list or a range (default: 1000-1039)",
    )
    parser.add_argument(
        "--jobs",
        type=_arg(_jobs),
        default=None,
        help="sweeps at a time, at most one per CPU core (default: one per core)",
    )
    parser.add_argument(
        "--keep-traces", action="store_true", help="keep positions.csv and steps.csv"
    )
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="run even when git status --porcelain prints anything",
    )
    args = parser.parse_args(argv)
    try:
        collect(
            Path.cwd(),
            allocators=args.allocators,
            rates=args.rates,
            planner=args.planner,
            fleet=args.fleet,
            seeds=args.seeds,
            jobs=args.jobs,
            keep_traces=args.keep_traces,
            allow_dirty=args.allow_dirty,
            log=_print,
        )
    except (OSError, ValueError, CollectError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

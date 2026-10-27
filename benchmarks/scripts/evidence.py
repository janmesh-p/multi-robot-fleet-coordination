#!/usr/bin/env python3
"""The benchmark evidence behind README.md: run it, rebuild the README tables from it, verify it.

    python3 benchmarks/scripts/evidence.py run [--only a,b] [--jobs N]
    python3 benchmarks/scripts/evidence.py tables [--check]
    python3 benchmarks/scripts/evidence.py verify [--only a,b] [--sample N]
    python3 benchmarks/scripts/evidence.py list

run     runs every benchmark suite the README reports and writes one CSV per suite to
        benchmarks/evidence/, one row per run, as the bench tool writes it. A suite runs in
        parts, one per planner and fleet size, at most --jobs at a time; the rows come out in the
        same order as from one bench_planning call.
tables  rebuilds every README table between `<!-- table:NAME -->` and `<!-- /table -->` from
        those CSVs. --check changes nothing and exits 1 if a table is out of date.
verify  runs the suites again, or N rows of each picked with a fixed seed, and checks that every
        column except timing comes out the same, log_hash included. The baseline sweep is
        checked against benchmarks/reference/baseline_sweep.csv.

Run from the repo root after `cmake --build build` (FLEET_BUILD names another build folder).
Standard library only. Timing columns (planning and solve times, wall time) depend on the
machine and its load; every other column depends only on the config and the seed.
"""

from __future__ import annotations

import argparse
import csv
import difflib
import math
import os
import random
import re
import statistics
import subprocess
import sys
import tempfile
from collections.abc import Callable, Iterable, Sequence
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

EVIDENCE = Path("benchmarks/evidence")
README = Path("README.md")
REFERENCE = Path("benchmarks/reference/baseline_sweep.csv")
BUILD = Path(os.environ.get("FLEET_BUILD", "build"))
TIMING = {
    "planning_p50_ms",
    "planning_p95_ms",
    "wall_seconds",
    "hungarian_p50_ms",
    "hungarian_p95_ms",
}
BOOTSTRAP_RESAMPLES = 10_000
BOOTSTRAP_SEED = 0

PLANNING = "benchmarks/configs/planning.yaml"
CLOSURE = "benchmarks/configs/closure.yaml"
FAULTS = "benchmarks/configs/faults.yaml"
FOUR = ("pibt", "windowed_coop_astar", "prioritized", "ecbs")
GOALS = ("prioritized", "ecbs")
LARGE = ("--rate", "0.5", "--map", "maps/warehouse_large.txt")
POCKETS = ("--rate", "0.5", "--map", "maps/warehouse_large_pockets.txt")
TO_GOALS = "rolling_replanning"


@dataclass(frozen=True)
class Planning:
    """A bench_planning suite: every planner x fleet size x seed, one run each."""

    config: str
    planners: tuple[str, ...]
    fleet: tuple[int, ...]
    seeds: str
    extra: tuple[str, ...] = ()

    def command(
        self,
        planners: Sequence[str] | None = None,
        fleet: Sequence[int] | None = None,
        seeds: str | None = None,
    ) -> list[str]:
        return [
            str(BUILD / "bench_planning"),
            "--config",
            self.config,
            "--planners",
            ",".join(planners or self.planners),
            "--fleet",
            ",".join(str(n) for n in (fleet or self.fleet)),
            "--seeds",
            seeds or self.seeds,
            *self.extra,
        ]

    def parts(self) -> list[list[str]]:
        """One command per planner and fleet size, in the order bench_planning runs them."""
        return [self.command([p], [n]) for p in self.planners for n in self.fleet]


@dataclass(frozen=True)
class Allocation:
    """A bench_allocation run: random rounds, one row per size."""

    extra: tuple[str, ...] = ()

    def parts(self) -> list[list[str]]:
        return [[str(BUILD / "bench_allocation"), *self.extra]]


@dataclass(frozen=True)
class Sweep:
    """The baseline sweep. Its evidence is the reference file, which CI checks on every push."""

    config: str = "benchmarks/configs/baseline.yaml"


def delay(p: str) -> tuple[str, ...]:
    return ("--delay", p)


def disable(*features: str) -> tuple[str, ...]:
    return ("--disable", ",".join(features))


SUITES: dict[str, Planning | Allocation | Sweep] = {
    # Stage 2
    "baseline": Sweep(),
    # Stage 3
    "allocation": Allocation(),
    "allocation_large": Allocation(
        ("--map", "maps/warehouse_large.txt", "--sizes", "25x25,50x50,100x100,100x200")
        + ("--rounds", "200")
    ),
    # Stage 4
    "planning_r010": Planning(PLANNING, FOUR, (5, 10, 25), "1-10"),
    "planning_r020": Planning(PLANNING, FOUR, (5, 10, 25), "1-10", ("--rate", "0.2")),
    "cbs_r010": Planning(PLANNING, ("cbs",), (5, 10), "1-10"),
    "cbs_r020": Planning(PLANNING, ("cbs",), (5, 10), "1-10", ("--rate", "0.2")),
    "independent_r010": Planning(PLANNING, ("independent_astar",), (5, 10, 25), "1-10"),
    "independent_r020": Planning(
        PLANNING, ("independent_astar",), (5, 10, 25), "1-10", ("--rate", "0.2")
    ),
    # Stage 4 (seeds 1 to 3) and Stage 7
    "large": Planning(PLANNING, FOUR, (25, 50, 100), "1-10", LARGE),
    # Stage 7
    "large_pockets": Planning(PLANNING, FOUR, (25, 50, 100), "1-10", POCKETS),
    # Stage 5
    "delay_005": Planning(PLANNING, FOUR, (10, 25), "1-10", delay("0.05")),
    "delay_010": Planning(PLANNING, FOUR, (10, 25), "1-10", delay("0.1")),
    "delay_020": Planning(PLANNING, FOUR, (10, 25), "1-10", delay("0.2")),
    "delay_030": Planning(PLANNING, FOUR, (10, 25), "1-10", delay("0.3")),
    "rules_no_reservations": Planning(
        PLANNING, FOUR, (10, 25), "1-10", delay("0.1") + disable("intersection_reservations")
    ),
    "rules_no_dependency_graph": Planning(
        PLANNING, FOUR, (10, 25), "1-10", delay("0.1") + disable("dependency_graph")
    ),
    "rules_neither": Planning(
        PLANNING,
        FOUR,
        (10, 25),
        "1-10",
        delay("0.1") + disable("dependency_graph", "intersection_reservations") + ("--record",),
    ),
    "goals_on_time": Planning(PLANNING, GOALS, (10, 25), "1-10", disable(TO_GOALS)),
    "goals_on_time_cbs": Planning(PLANNING, ("cbs",), (10,), "1-10", disable(TO_GOALS)),
    "goals_late": Planning(PLANNING, GOALS, (10, 25), "1-10", delay("0.1") + disable(TO_GOALS)),
    "goals_late_cbs": Planning(PLANNING, ("cbs",), (10,), "1-10", delay("0.1") + disable(TO_GOALS)),
    "goals_no_recovery": Planning(
        PLANNING, GOALS, (10, 25), "1-10", delay("0.1") + disable(TO_GOALS, "deadlock_recovery")
    ),
    "goals_no_recovery_cbs": Planning(
        PLANNING, ("cbs",), (10,), "1-10", delay("0.1") + disable(TO_GOALS, "deadlock_recovery")
    ),
    "goals_no_reservations": Planning(
        PLANNING,
        GOALS,
        (10, 25),
        "1-10",
        delay("0.1") + disable(TO_GOALS, "intersection_reservations") + ("--record",),
    ),
    "goals_no_reservations_cbs": Planning(
        PLANNING,
        ("cbs",),
        (10,),
        "1-10",
        delay("0.1") + disable(TO_GOALS, "intersection_reservations") + ("--record",),
    ),
    "guard_off_r010": Planning(PLANNING, FOUR, (5, 10, 25), "1-10", disable("starvation_guard")),
    "guard_off_r020": Planning(
        PLANNING, FOUR, (5, 10, 25), "1-10", ("--rate", "0.2") + disable("starvation_guard")
    ),
    "closure": Planning(CLOSURE, FOUR, (10, 25), "1-10"),
    "closure_goals": Planning(CLOSURE, GOALS, (10, 25), "1-10", disable(TO_GOALS)),
    # Stage 6
    "loss_005": Planning(PLANNING, FOUR, (10, 25), "1-10", ("--loss", "0.05")),
    "loss_010": Planning(PLANNING, FOUR, (10, 25), "1-10", ("--loss", "0.1")),
    "loss_020": Planning(PLANNING, FOUR, (10, 25), "1-10", ("--loss", "0.2")),
    "loss_030": Planning(PLANNING, FOUR, (10, 25), "1-10", ("--loss", "0.3")),
    "faults": Planning(FAULTS, FOUR, (10, 25), "1-10"),
    "faults_goals": Planning(FAULTS, GOALS, (10, 25), "1-10", disable(TO_GOALS)),
}


# ---------------------------------------------------------------------------------------------
# Running


def _print(line: str) -> None:
    print(line, flush=True)


def _run(command: list[str], out: Path) -> None:
    done = subprocess.run([*command, "--out", str(out)], capture_output=True, text=True)
    if done.returncode != 0:
        tail = (done.stdout + done.stderr).strip().splitlines()[-5:]
        raise RuntimeError(f"exit {done.returncode}: {' '.join(command)}\n" + "\n".join(tail))


def _read(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader.fieldnames or []), list(reader)


def _cost(command: list[str]) -> float:
    """A rough guess at how long a part takes, so the slow ones start first."""
    planner = command[command.index("--planners") + 1] if "--planners" in command else ""
    fleet = command[command.index("--fleet") + 1] if "--fleet" in command else "10"
    weight = {"cbs": 100.0, "ecbs": 3.0, "prioritized": 1.5}.get(planner, 1.0)
    large = 4.0 if any("warehouse_large" in arg for arg in command) else 1.0
    return weight * large * sum(int(n) for n in fleet.split(","))


def run_suites(names: Sequence[str], jobs: int, out_dir: Path) -> dict[str, Path]:
    """Run the named suites; returns the CSV written for each one (none for the baseline)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written: dict[str, Path] = {}
    with tempfile.TemporaryDirectory() as tmp:
        work: list[tuple[str, int, list[str]]] = []
        for name in names:
            suite = SUITES[name]
            if isinstance(suite, Sweep):
                continue
            work += [(name, i, part) for i, part in enumerate(suite.parts())]
        work.sort(key=lambda w: -_cost(w[2]))

        def part_file(name: str, i: int) -> Path:
            return Path(tmp) / f"{name}.{i:03d}.csv"

        with ThreadPoolExecutor(max(jobs, 1)) as pool:
            for future in [pool.submit(_run, part, part_file(n, i)) for n, i, part in work]:
                future.result()
        for name in names:
            suite = SUITES[name]
            if isinstance(suite, Sweep):
                continue
            header: list[str] = []
            rows: list[dict[str, str]] = []
            for i in range(len(suite.parts())):
                header, part_rows = _read(part_file(name, i))
                rows += part_rows
            path = out_dir / f"{name}.csv"
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=header, lineterminator="\n")
                writer.writeheader()
                writer.writerows(rows)
            written[name] = path
    return written


def check_baseline() -> bool:
    with tempfile.TemporaryDirectory() as tmp:
        done = subprocess.run(
            [
                str(BUILD / "run_sweep"),
                "--config",
                Sweep().config,
                "--out",
                tmp,
                "--check",
                str(REFERENCE),
            ],
            capture_output=True,
            text=True,
        )
    last = (done.stdout.strip() or done.stderr.strip()).splitlines()[-1:]
    _print(f"baseline   {last[0].strip() if last else 'failed'}")
    return done.returncode == 0


def compare(old: Sequence[dict[str, str]], new: Sequence[dict[str, str]]) -> list[str]:
    """Differences between two sets of rows in every column but timing."""
    if len(old) != len(new):
        return [f"{len(old)} rows before, {len(new)} now"]
    problems = []
    for i, (a, b) in enumerate(zip(old, new, strict=True)):
        columns = [c for c in a if c not in TIMING]
        diff = [c for c in columns if a[c] != b.get(c)]
        if diff:
            what = ", ".join(f"{c} {a[c]} -> {b.get(c)}" for c in diff[:3])
            problems.append(f"row {i + 1}: {what}")
    return problems


def cmd_run(args: argparse.Namespace) -> int:
    names = _names(args.only)
    if "baseline" in names and not check_baseline():
        return 1
    for name, path in run_suites(names, args.jobs, EVIDENCE).items():
        _print(f"{name:26s} {len(_read(path)[1]):4d} rows  {path}")
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    problems = 0
    for name in _names(args.only):
        suite = SUITES[name]
        if isinstance(suite, Sweep):
            problems += 0 if check_baseline() else 1
            continue
        _, old = _read(EVIDENCE / f"{name}.csv")
        with tempfile.TemporaryDirectory() as tmp:
            if args.sample and isinstance(suite, Planning):
                picked = sorted(
                    random.Random(name).sample(range(len(old)), min(args.sample, len(old)))
                )
                old = [old[i] for i in picked]
                new = []
                for i, row in enumerate(old):
                    out = Path(tmp) / f"{i}.csv"
                    _run(
                        suite.command([row["planner"]], [int(row["fleet_size"])], row["seed"]), out
                    )
                    new += _read(out)[1]
            else:
                new = _read(run_suites([name], args.jobs, Path(tmp))[name])[1]
        found = compare(old, new)
        state = "identical" if not found else f"{len(found)} differences"
        _print(f"{name:26s} {len(old):4d} rows  {state}")
        for problem in found[:5]:
            _print(f"  {problem}")
        problems += len(found)
    return 0 if problems == 0 else 1


def _names(only: str | None) -> list[str]:
    if not only:
        return list(SUITES)
    names = only.split(",")
    unknown = [n for n in names if n not in SUITES]
    if unknown:
        raise SystemExit(f"error: unknown suite {', '.join(unknown)}; see `evidence.py list`")
    return names


def cmd_list(_: argparse.Namespace) -> int:
    for name, suite in SUITES.items():
        if isinstance(suite, Sweep):
            command = f"run_sweep --config {suite.config} --check {REFERENCE}"
        elif isinstance(suite, Allocation):
            command = " ".join(suite.parts()[0][0:1] + list(suite.extra))
        else:
            command = " ".join(suite.command())
        _print(f"{name:26s} {command}")
    return 0


# ---------------------------------------------------------------------------------------------
# Reading the evidence

_cache: dict[str, list[dict[str, str]]] = {}


def rows(*names: str) -> list[dict[str, str]]:
    """The rows of one or more suites, in order."""
    out = []
    for name in names:
        if name not in _cache:
            path = REFERENCE if name == "baseline" else EVIDENCE / f"{name}.csv"
            _cache[name] = _read(path)[1]
        out += _cache[name]
    return out


def where(data: Iterable[dict[str, str]], **match: object) -> list[dict[str, str]]:
    return [r for r in data if all(r[k] == str(v) for k, v in match.items())]


def values(data: Iterable[dict[str, str]], column: str) -> list[float]:
    return [float(r[column]) for r in data]


def mean(data: Iterable[dict[str, str]], column: str) -> float:
    found = values(data, column)
    if not found:
        raise ValueError(f"no rows for {column}")
    return statistics.fmean(found)


def paired(a: Sequence[dict[str, str]], b: Sequence[dict[str, str]], column: str) -> list[float]:
    """Per-seed differences a - b. Both sides must hold the same seeds."""
    by_seed = {r["seed"]: float(r[column]) for r in b}
    seeds = [r["seed"] for r in a]
    if sorted(seeds) != sorted(by_seed) or len(set(seeds)) != len(seeds):
        raise ValueError(f"unpaired seeds: {seeds} and {sorted(by_seed)}")
    return [float(r[column]) - by_seed[r["seed"]] for r in a]


def bootstrap(diffs: Sequence[float]) -> tuple[float, float]:
    """95% percentile interval of the mean, resampling the differences with replacement."""
    rng = random.Random(BOOTSTRAP_SEED)
    n = len(diffs)
    means = sorted(statistics.fmean(rng.choices(diffs, k=n)) for _ in range(BOOTSTRAP_RESAMPLES))
    return _percentile(means, 2.5), _percentile(means, 97.5)


def _percentile(ordered: Sequence[float], q: float) -> float:
    """Linear interpolation between closest ranks, as numpy.percentile does by default."""
    at = (len(ordered) - 1) * q / 100.0
    low = math.floor(at)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (at - low)


# ---------------------------------------------------------------------------------------------
# Formatting


def f1(x: float) -> str:
    return f"{x:.1f}"


def f2(x: float) -> str:
    return f"{x:.2f}"


def whole(x: float) -> str:
    """A count or a mean shown to the step, with thousands separators."""
    return f"{round(x):,}"


def short(x: float) -> str:
    """Whole from 100 up, one decimal below, without a trailing .0."""
    if abs(x) >= 100:
        return f"{round(x)}"
    text = f"{x:.1f}"
    return text[:-2] if text.endswith(".0") else text


def median_step(xs: Sequence[float]) -> str:
    m = statistics.median(xs)
    return str(int(m)) if float(m).is_integer() else f1(m)


def pct(x: float) -> str:
    return f"{x:.1f}%"


def table(header: Sequence[str], align: Sequence[str], body: Iterable[Sequence[str]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join(align) + "|"]
    lines += ["| " + " | ".join(row) + " |" for row in body]
    return "\n".join(lines)


def code(name: str) -> str:
    return f"`{name}`"


# ---------------------------------------------------------------------------------------------
# README tables


def t_baseline() -> str:
    body = []
    data = rows("baseline")
    for n in (5, 10, 25):
        runs = where(data, fleet_size=n)
        dead = [r for r in runs if int(r["deadlocks"]) > 0]
        frozen = [r for r in runs if int(r["frozen_at"]) >= 0]
        body.append(
            [
                str(n),
                f"{len(dead)} of {len(runs)}",
                median_step(values(dead, "first_deadlock_step")),
                f"{len(frozen)} of {len(runs)}",
                median_step(values(frozen, "frozen_at")),
                f1(mean(runs, "orders_delivered")),
            ]
        )
    header = [
        "Robots",
        "Runs that deadlocked",
        "First deadlock, median step",
        "Runs that froze",
        "Frozen from, median step",
        "Orders delivered per run, mean",
    ]
    return table(header, ["---:"] * 6, body)


def t_allocation() -> str:
    body = []
    for r in rows("allocation"):
        body.append(
            [
                r["robots"],
                r["orders"],
                f1(float(r["nearest_mean_total"])),
                f1(float(r["hungarian_mean_total"])),
                pct(float(r["reduction_pct"])),
                pct(100.0 * int(r["rounds_better"]) / int(r["rounds"])),
                f"{float(r['hungarian_p95_ms']):.3f} ms",
            ]
        )
    header = [
        "Robots",
        "Waiting orders",
        "Nearest, mean total",
        "Hungarian, mean total",
        "Hungarian lower by",
        "Rounds where Hungarian is strictly better",
        "Hungarian solve, p95",
    ]
    return table(header, ["---:"] * 7, body)


def t_planning_small() -> str:
    body = []
    for planner in ("independent_astar", *FOUR, "cbs"):
        suites = {
            "independent_astar": ("independent_r010", "independent_r020"),
            "cbs": ("cbs_r010", "cbs_r020"),
        }.get(planner, ("planning_r010", "planning_r020"))
        cells = [code(planner)]
        for suite in suites:
            for n in (5, 10, 25):
                runs = where(rows(suite), planner=planner, fleet_size=n)
                cells.append(f1(mean(runs, "orders_per_hour")) if runs else "not run")
        body.append(cells)
    header = [
        "Planner",
        "0.1/step, 5 robots",
        "10 robots",
        "25 robots",
        "0.2/step, 5 robots",
        "10 robots",
        "25 robots",
    ]
    return table(header, ["---"] + ["---:"] * 6, body)


def t_planning_large() -> str:
    body = []
    first3 = [r for r in rows("large") if int(r["seed"]) <= 3]
    for planner in FOUR:
        cells = [code(planner)]
        for n in (25, 50, 100):
            cells.append(f1(mean(where(first3, planner=planner, fleet_size=n), "orders_per_hour")))
        cells.append(
            f2(mean(where(first3, planner=planner, fleet_size=100), "planning_p95_ms")) + " ms"
        )
        body.append(cells)
    header = ["Planner", "25 robots", "50 robots", "100 robots", "Planning p95 at 100 robots"]
    return table(header, ["---"] + ["---:"] * 4, body)


def _by_level(levels: dict[str, str]) -> str:
    """Robots x planner x one column per suite: orders per hour, mean over the seeds."""
    body = []
    for n in (10, 25):
        for planner in FOUR:
            cells = [str(n), code(planner)]
            for suite in levels.values():
                cells.append(
                    f1(mean(where(rows(suite), planner=planner, fleet_size=n), "orders_per_hour"))
                )
            body.append(cells)
    return table(["Robots", "Planner", *levels], ["---:", "---"] + ["---:"] * len(levels), body)


def t_delay() -> str:
    return _by_level(
        {
            "0": "planning_r010",
            "0.05": "delay_005",
            "0.1": "delay_010",
            "0.2": "delay_020",
            "0.3": "delay_030",
        }
    )


def t_loss() -> str:
    return _by_level(
        {
            "0": "planning_r010",
            "0.05": "loss_005",
            "0.1": "loss_010",
            "0.2": "loss_020",
            "0.3": "loss_030",
        }
    )


def t_rules() -> str:
    body = []
    for label, suite in (
        ("both (the default)", "delay_010"),
        ("dependency graph only", "rules_no_reservations"),
        ("reservations only", "rules_no_dependency_graph"),
        ("neither", "rules_neither"),
    ):
        data = rows(suite)
        cycles = [short(mean(where(data, fleet_size=n), "deadlocks_resolved")) for n in (10, 25)]
        body.append([label, whole(sum(values(data, "conflicts"))), " / ".join(cycles)])
    header = ["Rules on", "Robot-safety stops", "Cycles replanned per run, 10 / 25 robots"]
    return table(header, ["---", "---:", "---:"], body)


def t_goals() -> str:
    body = []
    for label, suite in (
        ("reservations and recovery (the default)", "goals_late"),
        ("the same robots on time", "goals_on_time"),
        ("no deadlock recovery", "goals_no_recovery"),
        ("no reservations", "goals_no_reservations"),
    ):
        data = rows(suite, suite + "_cbs")
        groups = sorted({(r["planner"], r["fleet_size"]) for r in data})
        means = [mean(where(data, planner=p, fleet_size=n), "orders_per_hour") for p, n in groups]
        frozen = sum(1 for r in data if int(r["frozen_at"]) >= 0)
        stops = whole(sum(values(data, "conflicts")))
        body.append([label, str(frozen), stops, f"{short(min(means))} to {short(max(means))}"])
    header = ["Rules", "Frozen runs", "Robot-safety stops", "Orders per hour"]
    return table(header, ["---", "---:", "---:", "---:"], body)


def t_guard() -> str:
    body = []
    for rate, on, off in (
        ("0.1", "planning_r010", "guard_off_r010"),
        ("0.2", "planning_r020", "guard_off_r020"),
    ):
        for n in (5, 10, 25):
            a = where(rows(on), fleet_size=n)
            b = where(rows(off), fleet_size=n)

            def both(column: str, fmt: Callable[[float], str]) -> str:
                return f"{fmt(mean(a, column))} / {fmt(mean(b, column))}"  # noqa: B023

            body.append(
                [
                    rate,
                    str(n),
                    both("oldest_open_steps", whole),
                    both("max_completion_steps", whole),
                    both("orders_per_hour", f1),
                ]
            )
    header = [
        "Orders per step",
        "Robots",
        "Oldest open order at the end, steps",
        "Slowest delivery, steps",
        "Orders per hour",
    ]
    return table(header, ["---:"] * 5, body)


def t_faults() -> str:
    body = []
    for n in (10, 25):
        for planner, replanning, plain, faulty in [
            *((p, "rolling", "planning_r010", "faults") for p in FOUR),
            *((p, "to goals", "goals_on_time", "faults_goals") for p in GOALS),
        ]:
            body.append(
                [
                    str(n),
                    code(planner),
                    replanning,
                    f1(mean(where(rows(plain), planner=planner, fleet_size=n), "orders_per_hour")),
                    f1(mean(where(rows(faulty), planner=planner, fleet_size=n), "orders_per_hour")),
                ]
            )
    header = ["Robots", "Planner", "Replanning", "No faults", "Faults"]
    return table(header, ["---:", "---", "---", "---:", "---:"], body)


# Stage 7

# The time budget classes these runs have. The README says the others never occur in them: at
# 0.5 orders per step an order is always waiting, so no robot is free or idle, and without
# faults no robot is stopped by robot safety or loses a step.
BUDGET = (
    ("time_drive_to_pickup", "To pickup"),
    ("time_drive_loaded", "Loaded"),
    ("time_service", "Service"),
    ("time_traffic", "Traffic"),
    ("time_station_wait", "Station wait"),
)
NEVER = ("time_drive_free", "time_idle", "time_safety_stop", "time_lost")
STATION_WAIT = 4  # index in BUDGET


def budget(runs: Sequence[dict[str, str]]) -> list[float]:
    """Share of the robot-steps after the warmup in each class of BUDGET, in percent."""
    total = sum(int(r[c]) for r in runs for c, _ in BUDGET)
    stray = sum(int(r[c]) for r in runs for c in NEVER)
    if stray:
        raise SystemExit(
            f"error: {stray} robot-steps free, idle, stopped, or lost; the README says none"
        )
    return [100.0 * sum(int(r[c]) for r in runs) / total for c, _ in BUDGET]


def station_busy(runs: Sequence[dict[str, str]]) -> float:
    """Service steps over station-steps after the warmup, in percent."""
    capacity = 0
    for r in runs:  # the classes add up to fleet size x measured steps
        robot_steps = sum(int(r[c]) for c in (*(c for c, _ in BUDGET), *NEVER))
        capacity += int(r["stations"]) * robot_steps // int(r["fleet_size"])
    return 100.0 * sum(int(r["time_service"]) for r in runs) / capacity


def t_budget() -> str:
    body = []
    for n in (25, 50, 100):
        for planner in FOUR:
            runs = where(rows("large"), planner=planner, fleet_size=n)
            body.append(
                [str(n), code(planner), f1(mean(runs, "orders_per_hour"))]
                + [f1(x) for x in budget(runs)]
                + [f1(station_busy(runs))]
            )
    header = ["Robots", "Planner", "Orders per hour", *(name for _, name in BUDGET)]
    return table(header + ["Stations busy"], ["---:", "---"] + ["---:"] * 7, body)


def difference(a: Sequence[dict[str, str]], b: Sequence[dict[str, str]]) -> list[str]:
    """Mean per-seed difference a - b in orders per hour, its 95% interval, seeds a won."""
    diffs = paired(a, b, "orders_per_hour")
    low, high = bootstrap(diffs)
    better = sum(1 for d in diffs if d > 0)
    return [
        f"{statistics.fmean(diffs):+.1f}",
        f"{low:+.1f} to {high:+.1f}",
        f"{better} of {len(diffs)}",
    ]


def t_pockets() -> str:
    body = []
    for n in (25, 50, 100):
        for planner in FOUR:
            a = where(rows("large_pockets"), planner=planner, fleet_size=n)
            b = where(rows("large"), planner=planner, fleet_size=n)
            wait = f"{f1(budget(b)[STATION_WAIT])} / {f1(budget(a)[STATION_WAIT])}"
            body.append(
                [
                    str(n),
                    code(planner),
                    f1(mean(b, "orders_per_hour")),
                    f1(mean(a, "orders_per_hour")),
                ]
                + difference(a, b)
                + [wait]
            )
    header = [
        "Robots",
        "Planner",
        "warehouse_large",
        "Pockets",
        "Difference",
        "95% interval",
        "Seeds better",
        "Station wait %, large / pockets",
    ]
    return table(header, ["---:", "---"] + ["---:"] * 6, body)


def t_intervals() -> str:
    small = {"0.1": "planning_r010", "0.2": "planning_r020"}
    body = []
    for label, rate, n, a, b in (
        ("`pibt` minus `windowed_coop_astar`", "0.1", 10, "pibt", "windowed_coop_astar"),
        ("`pibt` minus `windowed_coop_astar`", "0.2", 25, "pibt", "windowed_coop_astar"),
        ("`ecbs` minus `windowed_coop_astar`", "0.2", 25, "ecbs", "windowed_coop_astar"),
        ("`ecbs` minus `prioritized`", "0.2", 25, "ecbs", "prioritized"),
    ):
        data = rows(small[rate])
        diff = difference(
            where(data, planner=a, fleet_size=n), where(data, planner=b, fleet_size=n)
        )
        body.append([label, "`warehouse_small`", rate, str(n), *diff])
    for planner in FOUR:
        data = rows("large")
        diff = difference(
            where(data, planner=planner, fleet_size=100),
            where(data, planner=planner, fleet_size=50),
        )
        body.append(
            [f"100 robots minus 50, {code(planner)}", "`warehouse_large`", "0.5", "100, 50", *diff]
        )
    header = [
        "Comparison",
        "Map",
        "Orders per step",
        "Robots",
        "Difference",
        "95% interval",
        "Seeds better",
    ]
    return table(header, ["---", "---", "---:", "---:", "---:", "---:", "---:"], body)


TABLES: dict[str, Callable[[], str]] = {
    "baseline": t_baseline,
    "allocation": t_allocation,
    "planning_small": t_planning_small,
    "planning_large": t_planning_large,
    "delay": t_delay,
    "rules": t_rules,
    "goals": t_goals,
    "guard": t_guard,
    "loss": t_loss,
    "faults": t_faults,
    "budget": t_budget,
    "pockets": t_pockets,
    "intervals": t_intervals,
}

_MARKER = re.compile(r"(<!-- table:(\w+) -->\n)(.*?)(<!-- /table -->)", re.DOTALL)


def render(text: str) -> str:
    """The README with every marked table rebuilt from the evidence."""
    seen: set[str] = set()

    def replace(m: re.Match[str]) -> str:
        name = m.group(2)
        if name not in TABLES:
            raise SystemExit(f"error: README.md has a table marker with no generator: {name}")
        seen.add(name)
        return f"{m.group(1)}\n{TABLES[name]()}\n\n{m.group(4)}"

    out = _MARKER.sub(replace, text)
    missing = sorted(set(TABLES) - seen)
    if missing:
        raise SystemExit(f"error: README.md has no marker for {', '.join(missing)}")
    return out


def cmd_tables(args: argparse.Namespace) -> int:
    text = README.read_text(encoding="utf-8")
    new = render(text)
    if new == text:
        _print("tables     up to date")
        return 0
    if args.check:
        diff = difflib.unified_diff(
            text.splitlines(), new.splitlines(), "README.md", "from the evidence", lineterm=""
        )
        sys.stdout.writelines(line + "\n" for line in diff)
        _print("tables     out of date; run benchmarks/scripts/evidence.py tables")
        return 1
    README.write_text(new, encoding="utf-8")
    _print("tables     rewritten")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="evidence.py", description=__doc__.split("\n\n")[0] if __doc__ else None
    )
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run the suites into benchmarks/evidence/")
    run.add_argument("--only", help="comma-separated suite names")
    run.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    run.set_defaults(func=cmd_run)
    tables = sub.add_parser("tables", help="rebuild the README tables from the evidence")
    tables.add_argument("--check", action="store_true", help="exit 1 if a table is out of date")
    tables.set_defaults(func=cmd_tables)
    verify = sub.add_parser("verify", help="rerun and compare every column but timing")
    verify.add_argument("--only", help="comma-separated suite names")
    verify.add_argument("--sample", type=int, default=0, help="rows per suite (0: all)")
    verify.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    verify.set_defaults(func=cmd_verify)
    sub.add_parser("list", help="print every suite and its command").set_defaults(func=cmd_list)
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

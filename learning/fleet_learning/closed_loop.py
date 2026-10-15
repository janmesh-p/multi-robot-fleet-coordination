"""Closed-loop sweeps and paired comparisons: section 9 of docs/learning_track.md.

Two phases, each written the way collect.py writes a collection, under
learning/data/<commit>/closed_loop/<phase>/: configs/, runs/, and manifest.json, which records
the SHA-256 of every model file it ran. Every number comes from sweep.csv and assignments.csv.

    python -m fleet_learning.closed_loop tune --models learning/models/a.json learning/models/b.json
    python -m fleet_learning.closed_loop final --weight 0.5 --model learning/models/a.json
    python -m fleet_learning.closed_loop report learning/data/<commit>/closed_loop/final

`tune` runs cost_aware at every congestion weight and learned with every model on the tuning
seeds, and names the winners: the highest mean orders_per_hour over all tuning runs. `final`
runs hungarian, the tuned cost_aware, and the chosen learned model on the final seeds once.
`report` prints the tables of a finished phase again. Orders per hour are deliveries per
simulated hour after warmup; completion times are in steps (one step is one simulated second).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shlex
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml

from fleet_learning import collect, dataset, evaluate, model_io

PHASES: dict[str, str] = {"tune": "tuning", "final": "final"}  # phase -> seed use (section 7)
SEEDS: dict[str, tuple[int, ...]] = {
    "tune": tuple(range(2000, 2010)),
    "final": tuple(range(1, 11)),
}
FLEET_SIZES: tuple[int, ...] = (5, 10, 25)
ARRIVAL_RATES: tuple[float, ...] = (0.1, 0.2)  # orders per step
WEIGHTS: tuple[float, ...] = (0.25, 0.5, 1.0, 2.0, 4.0)  # allocation.congestion_weight
MAX_MODELS = 5
PLANNER = "pibt"
FINAL_ALLOCATORS: tuple[str, ...] = ("hungarian", "cost_aware", "learned")
BASELINES: tuple[str, ...] = ("hungarian", "cost_aware")

# The success criterion of section 9, written into learning/README.md before the final runs.
CRITERION_FLEET = 25
CRITERION_RATE = 0.2
CRITERION_BASELINE = "cost_aware"

BOOTSTRAP_RESAMPLES = 10_000
BOOTSTRAP_SEED = 0
TIE_TOLERANCE = 1e-9  # orders per hour

# The sweep.csv columns read, by name. Other columns are ignored: they grow between stages.
SWEEP_DTYPES: dict[str, str] = {
    "run": "string",
    "allocator": "string",
    "planner": "string",
    "fleet_size": "int64",
    "seed": "int64",
    "orders_per_hour": "float64",
    "mean_completion_steps": "float64",
    "p95_completion_steps": "float64",
    "aborted": "string",
    "deadlocks": "int64",
    "frozen_at": "int64",
    "log_hash": "string",
    "wall_seconds": "float64",
}
SWEEP_FILE = "sweep.csv"
MANIFEST_FILE = "manifest.json"

Log = collect.Log


class ClosedLoopError(RuntimeError):
    """A phase that cannot start, or results that cannot be read. The message says why."""


def weight_code(weight: float) -> str:
    """A congestion weight times 100, as three digits: 0.5 gives "050", 4 gives "400"."""
    hundredths = round(weight * 100)
    if not 1 <= hundredths <= 999 or abs(weight * 100 - hundredths) > 1e-9:
        raise ValueError(f"weight {weight}: use a multiple of 0.01 from 0.01 to 9.99")
    return f"{hundredths:03d}"


def sha256(path: str | Path) -> str:
    """The SHA-256 of a file, in hex."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _cell(
    name: str, allocator: str, rate: float, weight: float | None, model: Path | None
) -> dict[str, Any]:
    changes: dict[str, Any] = {
        "run.name": name,
        "allocation.method": allocator,
        "planning.method": PLANNER,
        "orders.arrival_rate": rate,
    }
    if weight is not None:
        changes["allocation.congestion_weight"] = weight
    if model is not None:
        changes["allocation.model_file"] = str(model)
    return {
        "name": name,
        "allocator": allocator,
        "arrival_rate": rate,
        "weight": weight,
        "model": model.stem if model is not None else None,
        "changes": changes,
    }


def _check_models(models: Sequence[str | Path], most: int) -> list[Path]:
    paths = [Path(model).resolve() for model in models]
    if not 1 <= len(paths) <= most:
        raise ValueError(f"models: give 1 to {most}, got {len(paths)} (section 9)")
    stems = [path.stem for path in paths]
    if len(set(stems)) != len(stems):
        raise ValueError(f"models: file names {', '.join(stems)} repeat; configs are named by them")
    for path in paths:
        model_io.load_model(path)
    return paths


def tune_cells(
    weights: Sequence[float], models: Sequence[Path], rates: Sequence[float]
) -> list[dict[str, Any]]:
    """Every tuning config: cost_aware per weight, then learned per model, each per rate.

    Names follow section 7: tune_cost_aware_w050_r020 and tune_learned_<model>_r020.
    """
    cells = []
    for weight in weights:
        for rate in rates:
            name = f"tune_cost_aware_w{weight_code(weight)}_r{collect.rate_code(rate)}"
            cells.append(_cell(name, "cost_aware", rate, weight, None))
    for model in models:
        for rate in rates:
            name = f"tune_learned_{model.stem}_r{collect.rate_code(rate)}"
            cells.append(_cell(name, "learned", rate, None, model))
    return cells


def final_cells(weight: float, model: Path, rates: Sequence[float]) -> list[dict[str, Any]]:
    """The final configs: hungarian, cost_aware at `weight`, and learned with `model`, per rate.

    Named like final_learned_r010 (section 7).
    """
    cells = []
    for allocator in FINAL_ALLOCATORS:
        for rate in rates:
            name = f"final_{allocator}_r{collect.rate_code(rate)}"
            cells.append(
                _cell(
                    name,
                    allocator,
                    rate,
                    weight if allocator == "cost_aware" else None,
                    model if allocator == "learned" else None,
                )
            )
    return cells


def check_seeds(phase: str, seeds: Sequence[int]) -> tuple[int, ...]:
    """Raise ValueError unless every seed is for this phase (section 7) or for plumbing."""
    seeds = collect._check(seeds, "seeds", least=0)
    allowed = (PHASES[phase], "plumbing")
    wrong = [f"{s} ({dataset.seed_use(s)})" for s in seeds if dataset.seed_use(s) not in allowed]
    if wrong:
        raise ValueError(
            f"seeds {', '.join(wrong)} are not for {phase}; use {' or '.join(allowed)}"
        )
    return seeds


def run_phase(
    phase: str,
    cells: Sequence[Mapping[str, Any]],
    repo: str | Path = ".",
    data_root: str | Path | None = None,
    *,
    fleet: Sequence[int] = FLEET_SIZES,
    seeds: Sequence[int] | None = None,
    jobs: int | None = None,
    keep_traces: bool = False,
    allow_dirty: bool = False,
    run_sweep: str | Path | None = None,
    extra: Mapping[str, Any] | None = None,
    log: Log = collect._quiet,
) -> dict[str, Any]:
    """Write the configs of `cells`, run them, and write the manifest; return the manifest.

    Like collect.collect: refuses a dirty tree unless `allow_dirty`, builds the C++ tools unless
    `run_sweep` is given, and deletes traces unless `keep_traces`. Output goes to
    <data_root>/<commit>/closed_loop/<phase>, by default under learning/data.
    """
    started = time.monotonic()
    repo = Path(repo).resolve()
    seeds = check_seeds(phase, SEEDS[phase] if seeds is None else seeds)
    fleet = collect._check(fleet, "fleet sizes", least=1)
    cores = os.cpu_count() or 1
    jobs = min(cores, len(cells)) if jobs is None else jobs
    if not 1 <= jobs <= cores:
        raise ValueError(f"jobs: {jobs} is not between 1 and {cores}, the CPU cores here")

    changes = collect.git_changes(repo)
    if changes and not allow_dirty:
        raise ClosedLoopError(
            f"git status --porcelain lists {len(changes)} changes, the first: {changes[0]}. "
            "Commit them, or pass --allow-dirty (section 7)."
        )
    commit = collect.git_commit(repo)
    base_path = repo / collect.BASE_CONFIG
    if not base_path.is_file():
        raise ClosedLoopError(f"{base_path} not found. Run from the repo root.")
    root = repo / collect.DATA_ROOT if data_root is None else Path(data_root).resolve()
    phase_dir = root / commit / "closed_loop" / phase
    if phase_dir.exists():
        raise ClosedLoopError(f"{phase_dir} already exists. Delete or move it to run again.")
    state = f"dirty, {len(changes)} changes" if changes else "clean"
    log(f"commit     {commit} ({state})")

    if run_sweep is None:
        log(f"build      {' && '.join(shlex.join(c) for c in collect.BUILD_COMMANDS)}")
        collect.build(repo)
        run_sweep = repo / collect.RUN_SWEEP
    run_sweep = Path(run_sweep).resolve()

    base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    sweeps = []
    for cell in cells:
        config = collect.make_config(base, base_path.parent, cell["changes"])
        path = phase_dir / "configs" / f"{cell['name']}.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        header = (
            f"# Written by fleet_learning.closed_loop at commit {commit}: {collect.BASE_CONFIG}\n"
            f"# with {', '.join(cell['changes'])} set, and paths absolute.\n"
        )
        path.write_text(header + yaml.safe_dump(config, sort_keys=False))
        out = phase_dir / "runs" / cell["name"]
        command = [collect._shown(run_sweep, repo), "--config", collect._shown(path, repo)]
        command += ["--fleet", collect.format_int_list(fleet)]
        command += ["--seeds", collect.format_int_list(seeds)]
        command += ["--out", collect._shown(out, repo), "--runs"]
        sweeps.append(collect.Sweep(cell["name"], path, out, command))
    log(f"configs    {len(sweeps)} in {collect._shown(phase_dir / 'configs', repo)}")
    log(
        f"sweeps     {len(sweeps)}, up to {jobs} at a time: fleet "
        f"{collect.format_int_list(fleet)}, seeds {collect.format_int_list(seeds)}"
    )
    records = collect.run_sweeps(sweeps, repo, jobs, keep_traces, log)

    order = {sweep.name: i for i, sweep in enumerate(sweeps)}
    models = {}
    for cell in cells:
        model_file = cell["changes"].get("allocation.model_file")
        if model_file is not None:
            models[cell["model"]] = {
                "file": collect._shown(Path(model_file), repo),
                "sha256": sha256(model_file),
            }
    manifest = {
        "commit": commit,
        "dirty": bool(changes),
        "changes": changes,
        "phase": phase,
        "base_config": str(collect.BASE_CONFIG),
        "planner": PLANNER,
        "fleet_sizes": list(fleet),
        "seeds": list(seeds),
        "cells": [{k: v for k, v in cell.items() if k != "changes"} for cell in cells],
        "models": models,
        **dict(extra or {}),
        "configs": [str(sweep.config.relative_to(phase_dir)) for sweep in sweeps],
        "sweeps": sorted(records, key=lambda record: order[record["name"]]),
        "jobs": jobs,
        "keep_traces": keep_traces,
        "wall_seconds": round(time.monotonic() - started, 3),
    }
    path = phase_dir / MANIFEST_FILE
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    log(f"manifest   {collect._shown(path, repo)}: {len(sweeps)} sweeps")
    return manifest


def read_manifest(phase_dir: str | Path) -> dict[str, Any]:
    """The manifest of a finished phase. Raises ClosedLoopError when there is none."""
    path = Path(phase_dir) / MANIFEST_FILE
    if not path.is_file():
        raise ClosedLoopError(f"{path} not found: the phase did not finish, or this is no phase")
    return json.loads(path.read_text(encoding="utf-8"))


def read_results(phase_dir: str | Path, manifest: Mapping[str, Any]) -> pd.DataFrame:
    """One row per run of the phase: SWEEP_DTYPES columns, then the cell's config, arrival_rate,
    weight, and model. Raises ClosedLoopError for a missing sweep.csv or an allocator mismatch."""
    frames = []
    for cell in manifest["cells"]:
        path = Path(phase_dir) / "runs" / cell["name"] / SWEEP_FILE
        if not path.is_file():
            raise ClosedLoopError(f"{path} not found")
        frame = pd.read_csv(
            path, usecols=list(SWEEP_DTYPES), dtype=SWEEP_DTYPES, keep_default_na=False
        )
        if not (frame["allocator"] == cell["allocator"]).all():
            raise ClosedLoopError(f"{path}: allocator is not {cell['allocator']} in every run")
        frames.append(
            frame.assign(
                config=cell["name"],
                arrival_rate=float(cell["arrival_rate"]),
                weight=cell["weight"],
                model=cell["model"],
            )
        )
    results = pd.concat(frames, ignore_index=True)
    results["aborted"] = results["aborted"] == "true"
    return results


def candidate_label(row: Mapping[str, Any]) -> str:
    """cost_aware w=0.5, or learned <model>: how a tuning candidate is named in tables."""
    if row["allocator"] == "cost_aware":
        return f"cost_aware w={row['weight']:g}"
    return f"learned {row['model']}"


def tuning_table(results: pd.DataFrame) -> pd.DataFrame:
    """Mean orders_per_hour per tuning candidate, over all its runs and per fleet and rate.

    Columns: allocator, candidate, runs, mean, then one column per "n<fleet> r<rate>". Rows keep
    the order the candidates ran in.
    """
    results = results.assign(candidate=[candidate_label(r) for r in results.to_dict("records")])
    order = list(dict.fromkeys(results["candidate"]))
    grouped = results.groupby("candidate", sort=False)
    table = pd.DataFrame(
        {
            "allocator": grouped["allocator"].first(),
            "runs": grouped.size(),
            "mean": grouped["orders_per_hour"].mean(),
        }
    )
    cells = results.groupby(["candidate", "fleet_size", "arrival_rate"])["orders_per_hour"].mean()
    for (fleet, rate), values in cells.groupby(level=[1, 2]):
        table[f"n{fleet} r{rate:g}"] = values.droplevel([1, 2])
    return table.loc[order].rename_axis("candidate").reset_index()


def choose_tuned(results: pd.DataFrame) -> dict[str, Any]:
    """The cost_aware weight and the learned model with the highest mean orders_per_hour over
    all their tuning runs (section 9). Ties go to the candidate that ran first."""
    choice: dict[str, Any] = {}
    for allocator, key in (("cost_aware", "weight"), ("learned", "model")):
        runs = results[results["allocator"] == allocator]
        if runs.empty:
            continue
        means = runs.groupby(key, sort=False)["orders_per_hour"].mean()
        best = means.idxmax()
        choice[key] = float(best) if key == "weight" else str(best)
        choice[f"{key}_orders_per_hour"] = float(means[best])
    return choice


def bootstrap_median(
    differences: Sequence[float],
    resamples: int = BOOTSTRAP_RESAMPLES,
    seed: int = BOOTSTRAP_SEED,
) -> tuple[float, float, float]:
    """The median of `differences` and its 95% percentile bootstrap interval (section 9).

    Resamples the differences with replacement `resamples` times with
    numpy.random.default_rng(seed), and returns (median, 2.5th, 97.5th percentile of the
    resampled medians), in the units of the differences.
    """
    values = np.asarray(differences, dtype=np.float64)
    if values.size == 0:
        raise ValueError("bootstrap_median needs at least one difference")
    rng = np.random.default_rng(seed)
    medians = np.median(rng.choice(values, size=(resamples, values.size), replace=True), axis=1)
    low, high = np.percentile(medians, [2.5, 97.5])
    return float(np.median(values)), float(low), float(high)


def paired_differences(results: pd.DataFrame, baseline: str) -> pd.DataFrame:
    """Per fleet size and arrival rate: learned minus `baseline` in orders_per_hour, paired by
    seed. Columns: fleet_size, arrival_rate, seeds, wins, ties, losses, median, low, high.

    Raises ClosedLoopError when the two allocators did not run the same seeds."""
    rows = []
    keys = ["fleet_size", "arrival_rate"]
    for (fleet, rate), group in results.groupby(keys):
        learned = group[group["allocator"] == "learned"].set_index("seed")["orders_per_hour"]
        base = group[group["allocator"] == baseline].set_index("seed")["orders_per_hour"]
        if learned.empty or set(learned.index) != set(base.index):
            raise ClosedLoopError(
                f"{fleet} robots, rate {rate:g}: learned and {baseline} ran different seeds"
            )
        difference = (learned - base.reindex(learned.index)).to_numpy()
        median, low, high = bootstrap_median(difference)
        rows.append(
            {
                "fleet_size": int(fleet),
                "arrival_rate": float(rate),
                "seeds": int(difference.size),
                "wins": int((difference > TIE_TOLERANCE).sum()),
                "ties": int((np.abs(difference) <= TIE_TOLERANCE).sum()),
                "losses": int((difference < -TIE_TOLERANCE).sum()),
                "median": median,
                "low": low,
                "high": high,
            }
        )
    return pd.DataFrame(rows)


def final_table(results: pd.DataFrame) -> pd.DataFrame:
    """Per fleet size, arrival rate, and allocator: runs; mean and median orders_per_hour;
    mean over runs of mean_completion_steps and of p95_completion_steps (steps); runs with a
    deadlock, frozen runs, and aborted runs; and total wall_seconds."""
    order = {name: i for i, name in enumerate(FINAL_ALLOCATORS)}
    grouped = results.groupby(["fleet_size", "arrival_rate", "allocator"])
    table = grouped.agg(
        runs=("seed", "size"),
        mean_orders_per_hour=("orders_per_hour", "mean"),
        median_orders_per_hour=("orders_per_hour", "median"),
        mean_completion_steps=("mean_completion_steps", "mean"),
        p95_completion_steps=("p95_completion_steps", "mean"),
        deadlocked_runs=("deadlocks", lambda d: int((d > 0).sum())),
        frozen_runs=("frozen_at", lambda f: int((f >= 0).sum())),
        aborted_runs=("aborted", "sum"),
        wall_seconds=("wall_seconds", "sum"),
    ).reset_index()
    table["_order"] = table["allocator"].map(order)
    table = table.sort_values(["fleet_size", "arrival_rate", "_order"])
    return table.drop(columns="_order").reset_index(drop=True)


def criterion(paired: pd.DataFrame) -> dict[str, Any]:
    """The success criterion of section 9, from paired_differences against cost_aware: at 25
    robots and rate 0.2, is the whole 95% interval of the median difference above 0?"""
    row = paired[
        (paired["fleet_size"] == CRITERION_FLEET)
        & np.isclose(paired["arrival_rate"], CRITERION_RATE)
    ]
    if row.empty:
        return {"evaluated": False}
    row = row.iloc[0]
    return {
        "evaluated": True,
        "median": float(row["median"]),
        "low": float(row["low"]),
        "high": float(row["high"]),
        "met": bool(row["low"] > 0),
    }


def selection_bias(
    phase_dir: str | Path,
    manifest: Mapping[str, Any],
    repo: str | Path = ".",
    margin: int | None = None,
) -> dict[str, Any]:
    """Prediction error on the pairs the learned allocator chose in the final runs (section 9).

    Loads the learned runs' assignments.csv, applies the section 3 filters with the margin the
    model was trained with (from its metrics file, if `margin` is None), and compares model_io
    costs with steps_to_pickup. Returns the closed-loop metrics, the filter counts, and the
    offline test metrics of the exported model for comparison. Raises ClosedLoopError when the
    model file changed since the runs.
    """
    repo = Path(repo)
    (name,) = manifest["models"]
    entry = manifest["models"][name]
    path = repo / entry["file"]
    if sha256(path) != entry["sha256"]:
        raise ClosedLoopError(f"{path} changed since the runs: its SHA-256 differs")
    metrics_path = path.with_name(f"{path.stem}.metrics.json")
    offline = None
    if metrics_path.is_file():
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        family = metrics["exported"]["family"]
        offline = {k: metrics["models"][family]["test"][k] for k in ("rows", "mae", "bias")}
        if margin is None:
            margin = int(metrics["data"]["margin_steps"])
    margin = dataset.MARGIN_STEPS if margin is None else margin
    learned = [c["name"] for c in manifest["cells"] if c["allocator"] == "learned"]
    runs = dataset.read_runs([Path(phase_dir) / "runs" / cell for cell in learned])
    rows = dataset.read_rows(runs)
    kept, counts = dataset.apply_filters(rows, margin)
    predicted = model_io.predict(model_io.load_model(path), kept)
    observed = kept["steps_to_pickup"]
    by_cell = {}
    for (fleet, rate), index in kept.groupby(["fleet_size", "arrival_rate"]).groups.items():
        positions = kept.index.get_indexer(index)
        by_cell[f"n{fleet} r{rate:g}"] = evaluate.error_summary(
            observed.to_numpy()[positions], predicted[positions]
        )
    return {
        "model": name,
        "margin_steps": margin,
        "closed_loop": evaluate.error_summary(observed, predicted),
        "by_cell": by_cell,
        "offline_test": offline,
        "filters": counts.to_dict("records"),
    }


def _number(value: Any, digits: int = 2) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "-"
    return f"{value:.{digits}f}"


def _table(header: Sequence[str], rows: Sequence[Sequence[str]], text_columns: int = 1) -> list:
    align = ["---"] * text_columns + ["---:"] * (len(header) - text_columns)
    return ["| " + " | ".join(cells) + " |" for cells in [list(header), align, *rows]]


def _seeds_line(manifest: Mapping[str, Any]) -> str:
    seeds = dataset.format_seeds(manifest["seeds"])
    fleets = ", ".join(str(n) for n in manifest["fleet_sizes"])
    models = "; ".join(
        f"{name}: {entry['file']} (SHA-256 {entry['sha256'][:12]})"
        for name, entry in manifest["models"].items()
    )
    return (
        f"Commit {manifest['commit']}, planner {manifest['planner']}, seeds {seeds}, fleets "
        f"{fleets}. Models: {models}."
    )


def tuning_report(results: pd.DataFrame, manifest: Mapping[str, Any]) -> str:
    """Markdown: the tuning table and the choices."""
    table = tuning_table(results)
    cell_columns = [c for c in table.columns if c.startswith("n")]
    lines = [_seeds_line(manifest), "", "Mean orders_per_hour per candidate.", ""]
    body = [
        [r["candidate"], str(r["runs"]), _number(r["mean"]), *[_number(r[c]) for c in cell_columns]]
        for r in table.to_dict("records")
    ]
    lines += _table(["Candidate", "Runs", "All runs", *cell_columns], body)
    choice = choose_tuned(results)
    lines.append("")
    if "weight" in choice:
        lines.append(
            f"Chosen cost_aware weight: {choice['weight']:g} "
            f"({choice['weight_orders_per_hour']:.2f} orders per hour)."
        )
    if "model" in choice:
        lines.append(
            f"Chosen learned model: {choice['model']} "
            f"({choice['model_orders_per_hour']:.2f} orders per hour)."
        )
    if results["aborted"].any():
        lines.append(f"Aborted runs: {', '.join(results.loc[results['aborted'], 'run'])}.")
    return "\n".join(lines)


def final_report(
    results: pd.DataFrame, manifest: Mapping[str, Any], bias: Mapping[str, Any] | None
) -> str:
    """Markdown: the final table, the paired differences, the criterion, and the bias check."""
    lines = [_seeds_line(manifest), f"cost_aware weight {manifest.get('weight')}.", ""]
    header = ["Robots", "Rate", "Allocator", "Runs", "Orders/h mean", "Orders/h median"]
    header += ["Completion mean", "Completion p95", "Deadlocked", "Frozen", "Wall s"]
    body = [
        [
            str(r["fleet_size"]),
            f"{r['arrival_rate']:g}",
            r["allocator"],
            str(r["runs"]),
            _number(r["mean_orders_per_hour"]),
            _number(r["median_orders_per_hour"]),
            _number(r["mean_completion_steps"], 1),
            _number(r["p95_completion_steps"], 1),
            str(r["deadlocked_runs"]),
            str(r["frozen_runs"]),
            _number(r["wall_seconds"]),
        ]
        for r in final_table(results).to_dict("records")
    ]
    lines += _table(header, body, text_columns=3)
    lines += ["", "Completion times are means over runs, in steps. Wall s: all runs, summed."]
    if results["aborted"].any():
        lines.append(f"Aborted runs: {', '.join(results.loc[results['aborted'], 'run'])}.")
    for baseline in BASELINES:
        paired = paired_differences(results, baseline)
        lines += ["", f"learned minus {baseline}, orders_per_hour per seed:", ""]
        body = [
            [
                str(r["fleet_size"]),
                f"{r['arrival_rate']:g}",
                f"{r['wins']}/{r['ties']}/{r['losses']}",
                _number(r["median"]),
                f"[{_number(r['low'])}, {_number(r['high'])}]",
            ]
            for r in paired.to_dict("records")
        ]
        header = ["Robots", "Rate", "Wins/ties/losses", "Median", "95% interval"]
        lines += _table(header, body, text_columns=2)
        if baseline == CRITERION_BASELINE:
            result = criterion(paired)
    lines.append("")
    lines.append(
        f"Bootstrap: {BOOTSTRAP_RESAMPLES} resamples, numpy.random.default_rng({BOOTSTRAP_SEED})."
    )
    if result["evaluated"]:
        verdict = "met" if result["met"] else "not met"
        lines.append(
            f"Criterion ({CRITERION_FLEET} robots, rate {CRITERION_RATE:g}, interval above 0 "
            f"against {CRITERION_BASELINE}): {verdict}, median {result['median']:.2f}, "
            f"interval [{result['low']:.2f}, {result['high']:.2f}]."
        )
    if bias is not None:
        lines += [
            "",
            f"Selection bias check, model {bias['model']}, margin {bias['margin_steps']}:",
        ]
        lines += [""]
        body = []
        if bias["offline_test"] is not None:
            o = bias["offline_test"]
            body.append(["offline test", str(o["rows"]), _number(o["mae"]), _number(o["bias"])])
        c = bias["closed_loop"]
        body.append(["final learned runs", str(c["rows"]), _number(c["mae"]), _number(c["bias"])])
        for cell, m in bias["by_cell"].items():
            body.append([f"  {cell}", str(m["rows"]), _number(m["mae"]), _number(m["bias"])])
        lines += _table(["Rows from", "Rows", "MAE", "Bias"], body)
        lines += ["", "MAE and bias in steps; bias is predicted minus observed steps_to_pickup."]
    return "\n".join(lines)


def report(phase_dir: str | Path, repo: str | Path = ".") -> str:
    """The Markdown report of a finished phase, from its manifest, sweep.csv files, and, for
    the final phase, the learned runs' assignments.csv."""
    manifest = read_manifest(phase_dir)
    results = read_results(phase_dir, manifest)
    if manifest["phase"] == "tune":
        return tuning_report(results, manifest)
    return final_report(results, manifest, selection_bias(phase_dir, manifest, repo))


def _phase_dir(manifest: Mapping[str, Any], repo: Path, data_root: Path | None) -> Path:
    root = repo / collect.DATA_ROOT if data_root is None else data_root.resolve()
    return root / manifest["commit"] / "closed_loop" / manifest["phase"]


def _weights_arg(text: str) -> tuple[float, ...]:
    try:
        weights = tuple(float(w) for w in text.split(","))
        for weight in weights:
            weight_code(weight)
        return collect._check(weights, "weights", least=0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{text!r}: {error}") from error


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fleet_learning.closed_loop",
        description="Closed-loop tuning, final runs, and paired comparisons: section 9 of "
        "docs/learning_track.md. Run from the repo root.",
    )
    sub = parser.add_subparsers(dest="phase", required=True)
    tune = sub.add_parser("tune", help="tune the cost_aware weight and choose a learned model")
    tune.add_argument("--models", nargs="+", required=True, help=f"1 to {MAX_MODELS} model files")
    tune.add_argument(
        "--weights",
        type=_weights_arg,
        default=WEIGHTS,
        help="cost_aware congestion weights (default: 0.25,0.5,1,2,4)",
    )
    final = sub.add_parser("final", help="the final runs, once, with the tuned choices")
    final.add_argument("--weight", type=float, required=True, help="the tuned cost_aware weight")
    final.add_argument("--model", required=True, help="the chosen learned model file")
    for phase in (tune, final):
        phase.add_argument(
            "--rates",
            type=collect._arg(collect.parse_rates),
            default=ARRIVAL_RATES,
            help="arrival rates, orders per step (default: 0.1,0.2)",
        )
        phase.add_argument(
            "--fleet",
            type=collect._arg(collect.parse_int_list),
            default=FLEET_SIZES,
            help="fleet sizes (default: 5,10,25)",
        )
        phase.add_argument(
            "--seeds",
            type=collect._arg(collect.parse_int_list),
            default=None,
            help="seeds (default: 2000-2009 for tune, 1-10 for final; 3000 and up for plumbing)",
        )
        phase.add_argument("--jobs", type=collect._arg(collect._jobs), default=None)
        phase.add_argument("--keep-traces", action="store_true")
        phase.add_argument("--allow-dirty", action="store_true")
    show = sub.add_parser("report", help="print the tables of a finished phase")
    show.add_argument("phase_dir", type=Path, help="learning/data/<commit>/closed_loop/<phase>")
    args = parser.parse_args(argv)

    repo = Path.cwd()
    try:
        if args.phase == "report":
            print(report(args.phase_dir, repo))
            return 0
        if args.phase == "tune":
            models = _check_models(args.models, MAX_MODELS)
            cells = tune_cells(args.weights, models, args.rates)
            extra = {"weights": list(args.weights)}
        else:
            weight_code(args.weight)
            (model,) = _check_models([args.model], 1)
            cells = final_cells(args.weight, model, args.rates)
            extra = {"weight": args.weight}
        manifest = run_phase(
            args.phase,
            cells,
            repo,
            fleet=args.fleet,
            seeds=args.seeds,
            jobs=args.jobs,
            keep_traces=args.keep_traces,
            allow_dirty=args.allow_dirty,
            extra=extra,
            log=collect._print,
        )
        phase_dir = _phase_dir(manifest, repo.resolve(), None)
        print()
        print(report(phase_dir, repo))
    except (OSError, ValueError, collect.CollectError, ClosedLoopError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

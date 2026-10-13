"""Run directories to one table of assignments with run metadata; filters; seed splits.

See sections 3 and 7 of docs/learning_track.md. One step is one simulated second.

A run directory is any folder that holds `summary.csv`, as `run_sim` and `run_sweep --runs`
write them. Point the loader at one collection (`learning/data/<commit>/runs`), a sweep folder,
or single run directories. It refuses two run directories with the same run name, so data from
two collections never mixes.

Usage:
    python -m fleet_learning.dataset learning/data/<commit>/runs
"""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

import pandas as pd
import yaml

from fleet_learning import schema

SUMMARY_FILE = "summary.csv"
CONFIG_FILE = "config.yaml"

MARGIN_STEPS = 300  # rows with t > steps_run - MARGIN_STEPS are dropped (section 3)

# The summary.csv columns read, by name. Other columns are ignored: they grow between stages.
SUMMARY_DTYPES: dict[str, str] = {
    "run": "string",
    "allocator": "string",
    "planner": "string",
    "fleet_size": "int64",
    "seed": "int64",
    "steps_run": "int64",
    "aborted": "string",
}

# Run metadata added to every row. For filtering and grouping only, never a model input.
METADATA_DTYPES: dict[str, str] = {
    "run": "string",
    "allocator": "string",
    "planner": "string",
    "fleet_size": "int64",
    "seed": "int64",
    "arrival_rate": "float64",
    "steps_run": "int64",
    "aborted": "bool",
}
METADATA_COLUMNS: tuple[str, ...] = tuple(METADATA_DTYPES)

# The filters of section 3, in the order they apply. Each counts only rows the earlier ones kept.
FILTERS: tuple[str, ...] = ("aborted", "margin", "unreached")
GROUP_COLUMNS: tuple[str, ...] = ("allocator", "fleet_size")
UNREACHED_NOTE_SHARE = 0.01  # above this share in any group, the write-up must say so
LONGEST_KEPT_SHARE = 0.5  # "well under the margin" (section 3), read as at most half of it

# The one use of each seed (section 7): use -> (first seed, last seed or None for no end).
SEED_USES: dict[str, tuple[int, int | None]] = {
    "train": (1000, 1019),
    "validation": (1020, 1029),
    "test": (1030, 1039),
    "tuning": (2000, 2009),
    "final": (1, 10),
    "plumbing": (3000, None),
}
OFFLINE_SPLITS: tuple[str, ...] = ("train", "validation", "test")


class DatasetError(ValueError):
    """Run directories that cannot be loaded into one table."""


def find_run_dirs(roots: Iterable[str | Path]) -> list[Path]:
    """Every folder at or below `roots` that holds summary.csv: absolute, sorted, each once."""
    roots = [Path(root) for root in roots]
    found: set[Path] = set()
    for root in roots:
        if not root.is_dir():
            raise FileNotFoundError(f"{root}: not a directory")
        found.update(path.parent.resolve() for path in root.rglob(SUMMARY_FILE))
    if not found:
        names = ", ".join(str(root) for root in roots)
        raise FileNotFoundError(f"no run directories (folders with {SUMMARY_FILE}) in {names}")
    return sorted(found)


def _read_summary(run_dir: Path) -> dict[str, Any]:
    path = run_dir / SUMMARY_FILE
    try:
        header = pd.read_csv(path, nrows=0).columns
    except ValueError as error:
        raise DatasetError(f"{path}: no header ({error})") from error
    missing = [name for name in SUMMARY_DTYPES if name not in header]
    if missing:
        raise DatasetError(f"{path}: missing columns {', '.join(missing)}")
    try:
        frame = pd.read_csv(
            path, usecols=list(SUMMARY_DTYPES), dtype=SUMMARY_DTYPES, keep_default_na=False
        )
    except ValueError as error:
        raise DatasetError(f"{path}: a value does not fit its column dtype ({error})") from error
    if len(frame) != 1:
        raise DatasetError(f"{path}: expected one row, found {len(frame)}")
    summary = frame.iloc[0].to_dict()
    if summary["aborted"] not in ("true", "false"):
        raise DatasetError(f"{path}: aborted must be true or false, got {summary['aborted']!r}")
    summary["aborted"] = summary["aborted"] == "true"
    return summary


def _config_value(config: Any, path: Path, *keys: str) -> Any:
    node = config
    for key in keys:
        if not isinstance(node, dict) or key not in node:
            raise DatasetError(f"{path}: missing key {'.'.join(keys)}")
        node = node[key]
    return node


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def _read_config(run_dir: Path) -> dict[str, Any]:
    path = run_dir / CONFIG_FILE
    try:
        with open(path, encoding="utf-8") as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError as error:
        raise DatasetError(f"{path}: not valid YAML ({error})") from error
    arrival_rate = _config_value(config, path, "orders", "arrival_rate")
    packet_loss = _config_value(config, path, "faults", "packet_loss")
    events = _config_value(config, path, "faults", "events") or []
    for name, value in (("orders.arrival_rate", arrival_rate), ("faults.packet_loss", packet_loss)):
        if not _is_finite_number(value):
            raise DatasetError(f"{path}: {name} must be a number")
    if not isinstance(events, list) or not all(isinstance(event, dict) for event in events):
        raise DatasetError(f"{path}: faults.events must be a list of events")
    outage = any(event.get("type") == "comm_outage" for event in events)
    return {"arrival_rate": float(arrival_rate), "lossless": packet_loss == 0 and not outage}


def read_runs(roots: Iterable[str | Path]) -> pd.DataFrame:
    """One row per run directory at or below `roots`, sorted by path.

    Columns: `run_dir`, then METADATA_COLUMNS (`arrival_rate` in orders per step, `steps_run`
    in steps), then `lossless`: true when the run lost no robot reports (`faults.packet_loss`
    is 0 and no `comm_outage` event), so `steps_to_pickup >= dist_to_pickup` must hold.
    """
    records = [
        {"run_dir": str(run_dir), **_read_summary(run_dir), **_read_config(run_dir)}
        for run_dir in find_run_dirs(roots)
    ]
    dtypes = {"run_dir": "string", **METADATA_DTYPES, "lossless": "bool"}
    runs = pd.DataFrame.from_records(records, columns=list(dtypes)).astype(dtypes)
    repeated = runs[runs["run"].duplicated(keep=False)]
    if not repeated.empty:
        name = repeated["run"].iloc[0]
        dirs = ", ".join(repeated.loc[repeated["run"] == name, "run_dir"])
        raise DatasetError(
            f"run {name} is in more than one run directory ({dirs}). "
            "Load one collection at a time (section 7)."
        )
    return runs


def read_rows(runs: pd.DataFrame) -> pd.DataFrame:
    """Every assignment of the runs in `runs` (from `read_runs`), checked, with run metadata.

    Columns: schema.COLUMNS, then METADATA_COLUMNS. Rows follow `runs`, then each file's order
    (t, then robot). Raises FileNotFoundError when a run directory has no assignments.csv, which
    the core writes from Stage 3 on.
    """
    missing = [d for d in runs["run_dir"] if not (Path(d) / schema.FILE_NAME).is_file()]
    if missing:
        raise FileNotFoundError(
            f"{len(missing)} of {len(runs)} run directories have no {schema.FILE_NAME}, "
            f"the first is {missing[0]}. The core writes {schema.FILE_NAME} from Stage 3 on; "
            "check the Stages table in README.md on origin/main."
        )
    frames = []
    for run in runs.to_dict("records"):
        frame = schema.read_assignments(run["run_dir"], lossless=run["lossless"])
        if not frame.empty:
            frames.append(frame.assign(**{name: run[name] for name in METADATA_COLUMNS}))
    if not frames:
        dtypes = {**schema.DTYPES, **METADATA_DTYPES}
        return pd.DataFrame({name: pd.Series(dtype=dtype) for name, dtype in dtypes.items()})
    return pd.concat(frames, ignore_index=True).astype(METADATA_DTYPES)


def drop_reasons(rows: pd.DataFrame, margin: int = MARGIN_STEPS) -> pd.Series:
    """For each row, the first filter in FILTERS that drops it, or "kept".

    - aborted: the run's summary.csv has aborted = true; a safety violation stopped the run.
    - margin: t > steps_run - margin (steps). The end of the run cuts off late assignments
      that take long, which biases the target low.
    - unreached: steps_to_pickup = -1 inside the margin. The robot never reached the pickup,
      usually because it was stuck.
    """
    if margin < 0:
        raise ValueError(f"margin must be 0 or more steps, got {margin}")
    reasons = pd.Series("kept", index=rows.index, dtype="string")
    reasons = reasons.mask(rows["steps_to_pickup"] == -1, "unreached")
    reasons = reasons.mask(rows["t"] > rows["steps_run"] - margin, "margin")
    return reasons.mask(rows["aborted"], "aborted")


def filter_counts(rows: pd.DataFrame, reasons: pd.Series) -> pd.DataFrame:
    """Rows each filter drops, per logging allocator and fleet size, for the metrics file.

    Columns: allocator, fleet_size, rows, one count per filter in FILTERS, kept, and
    unreached_share: unreached rows over the rows inside the margin (unreached + kept), NaN when
    no rows are inside the margin.
    """
    outcomes = [*FILTERS, "kept"]
    if rows.empty:
        counts = pd.DataFrame(columns=[*GROUP_COLUMNS, "rows", *outcomes], dtype="int64")
        return counts.assign(unreached_share=pd.Series(dtype="float64"))
    keys = [rows[name] for name in GROUP_COLUMNS]
    counts = reasons.groupby(keys).value_counts().unstack(fill_value=0)
    counts = counts.reindex(columns=outcomes, fill_value=0).astype("int64")
    counts.insert(0, "rows", counts.sum(axis=1))
    counts["unreached_share"] = counts["unreached"] / (counts["unreached"] + counts["kept"])
    return counts.reset_index().rename_axis(columns=None)


def apply_filters(
    rows: pd.DataFrame, margin: int = MARGIN_STEPS
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Drop the rows section 3 excludes from training. Returns (kept rows, filter_counts)."""
    reasons = drop_reasons(rows, margin)
    kept = rows[reasons == "kept"].reset_index(drop=True)
    return kept, filter_counts(rows, reasons)


def seed_use(seed: int) -> str:
    """The one use section 7 gives `seed`, a key of SEED_USES. Raises ValueError if none."""
    for use, (first, last) in SEED_USES.items():
        if first <= seed and (last is None or seed <= last):
            return use
    raise ValueError(f"seed {seed} has no use in section 7 of docs/learning_track.md")


def split_by_seed(
    rows: pd.DataFrame, splits: Sequence[str] = OFFLINE_SPLITS
) -> dict[str, pd.DataFrame]:
    """Rows by the use of their seed, {split: rows}, in the order of `splits`. Never by row.

    Raises ValueError when a row's seed has a use outside `splits`, so rows from seeds kept for
    another use (closed-loop tuning or final runs, plumbing) never leak into a split.
    """
    unknown = [name for name in splits if name not in SEED_USES]
    if unknown:
        raise ValueError(f"unknown splits {', '.join(unknown)}; use {', '.join(SEED_USES)}")
    uses = {int(seed): seed_use(int(seed)) for seed in rows["seed"].unique()}
    stray = [f"{seed} ({use})" for seed, use in sorted(uses.items()) if use not in splits]
    if stray:
        raise ValueError(f"seeds {', '.join(stray)} are not for {', '.join(splits)}")
    use_of_row = rows["seed"].map(uses)
    return {name: rows[use_of_row == name].reset_index(drop=True) for name in splits}


def format_seeds(seeds: Iterable[int]) -> str:
    """Seeds as sorted ranges, for example `1000-1019, 1025`."""
    spans: list[list[int]] = []
    for seed in sorted({int(seed) for seed in seeds}):
        if spans and seed == spans[-1][1] + 1:
            spans[-1][1] = seed
        else:
            spans.append([seed, seed])
    return ", ".join(str(a) if a == b else f"{a}-{b}" for a, b in spans)


def report(
    runs: pd.DataFrame, rows: pd.DataFrame, kept: pd.DataFrame, counts: pd.DataFrame, margin: int
) -> list[str]:
    """Lines that describe a loaded table: filter counts, seeds, and what needs attention.

    Takes the outputs of read_runs, read_rows, and apply_filters. Raises ValueError for a seed
    with no use in section 7.
    """
    lines: list[str] = []

    def add(label: str, text: str) -> None:
        lines.append(f"{label:<11}{text}".rstrip())

    aborted = runs[runs["aborted"]]
    add("runs", f"{len(runs)} ({len(aborted)} aborted)")
    add("rows", f"{len(rows)} ({len(kept)} kept)")
    add("filters", f"in order, each counts rows the earlier ones kept; margin {margin} steps")
    shares = counts["unreached_share"].map(lambda v: "-" if pd.isna(v) else f"{v:.1%}")
    lines += counts.assign(unreached_share=shares).to_string(index=False).splitlines()
    longest = int(kept["steps_to_pickup"].max()) if not kept.empty else None
    add("longest", "no rows kept" if longest is None else f"kept steps_to_pickup: {longest} steps")

    uses = {seed: seed_use(seed) for seed in runs["seed"]}
    label = "seeds"
    for use in SEED_USES:
        seeds = [seed for seed, use_of_seed in uses.items() if use_of_seed == use]
        if seeds:
            n_runs = int(runs["seed"].isin(seeds).sum())
            n_rows = int(kept["seed"].isin(seeds).sum())
            add(label, f"{use}: {format_seeds(seeds)} (runs {n_runs}, rows kept {n_rows})")
            label = ""

    for run in aborted.itertuples():
        add("aborted", f"{run.run}: a safety violation stopped this run. Open an issue with")
        add("", f"the run name and {Path(run.run_dir) / CONFIG_FILE} (section 3).")
    for group in counts[counts["unreached_share"] > UNREACHED_NOTE_SHARE].itertuples():
        share = f"{group.unreached_share:.1%}"
        add("note", f"{group.allocator}, {group.fleet_size} robots: unreached rows are {share}")
        add("", "of the rows inside the margin, over 1%. Say so in the write-up (section 3).")
    if longest is not None and longest > LONGEST_KEPT_SHARE * margin:
        limit = f"{LONGEST_KEPT_SHARE:.0%} of the margin of {margin} steps"
        add("warning", f"the longest kept steps_to_pickup, {longest} steps, is over {limit}.")
        add("", "Raise --margin (section 3).")
    return lines


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fleet_learning.dataset",
        description="Load run directories into one table, apply the filters of section 3 of "
        "docs/learning_track.md, and report what they drop and how the seeds split.",
    )
    parser.add_argument(
        "roots",
        nargs="+",
        type=Path,
        metavar="DIR",
        help="run directories, sweep folders, or a collection: learning/data/<commit>/runs",
    )
    parser.add_argument(
        "--margin",
        type=int,
        default=MARGIN_STEPS,
        help="drop rows with t > steps_run - MARGIN, in steps (default: %(default)s)",
    )
    args = parser.parse_args(argv)
    if args.margin < 0:
        parser.error("--margin must be 0 or more")

    try:
        runs = read_runs(args.roots)
        rows = read_rows(runs)
        kept, counts = apply_filters(rows, args.margin)
        lines = report(runs, rows, kept, counts, args.margin)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())

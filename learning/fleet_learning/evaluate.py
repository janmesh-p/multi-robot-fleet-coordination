"""Offline metrics and tables for travel-time models.

See section 8 of docs/learning_track.md. Every error is in steps; one step is one simulated
second. Bias is the mean of predicted minus observed `steps_to_pickup`, so a negative bias means
the model predicts too few steps. Delayed rows are rows where `steps_to_pickup >
dist_to_pickup`: the robot lost time to traffic or queues.

Usage:
    python -m fleet_learning.evaluate learning/models/<name>.metrics.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from fleet_learning import dataset

# dist_to_pickup buckets (steps): label, first, last (None: no upper end).
DIST_BUCKETS: tuple[tuple[str, int, int | None], ...] = (
    ("0-5", 0, 5),
    ("6-10", 6, 10),
    ("11-20", 11, 20),
    ("21+", 21, None),
)

# Groups for MAE, as columns of the dataset table, with their table titles.
GROUP_TITLES: dict[str, str] = {
    "fleet_size": "MAE by fleet size (robots)",
    "arrival_rate": "MAE by arrival rate (orders per step)",
    "allocator": "MAE by logging allocator",
    "dist_bucket": "MAE by dist_to_pickup (steps)",
}


def _errors(observed: Any, predicted: Any) -> np.ndarray:
    observed = np.asarray(observed, dtype=np.float64)
    predicted = np.asarray(predicted, dtype=np.float64)
    if observed.shape != predicted.shape:
        raise ValueError(f"{observed.shape[0]} observed values but {predicted.shape[0]} predicted")
    return predicted - observed


def mae(observed: Any, predicted: Any) -> float:
    """Mean absolute error in steps. NaN when there are no rows."""
    errors = _errors(observed, predicted)
    return float(np.mean(np.abs(errors))) if errors.size else math.nan


def rmse(observed: Any, predicted: Any) -> float:
    """Root mean squared error in steps. NaN when there are no rows."""
    errors = _errors(observed, predicted)
    return float(np.sqrt(np.mean(errors**2))) if errors.size else math.nan


def bias(observed: Any, predicted: Any) -> float:
    """Mean of predicted minus observed, in steps. NaN when there are no rows."""
    errors = _errors(observed, predicted)
    return float(np.mean(errors)) if errors.size else math.nan


def error_summary(observed: Any, predicted: Any) -> dict[str, Any]:
    """Rows, then MAE, RMSE, and bias in steps."""
    return {
        "rows": int(np.asarray(observed).size),
        "mae": mae(observed, predicted),
        "rmse": rmse(observed, predicted),
        "bias": bias(observed, predicted),
    }


def dist_bucket(dist: Any) -> np.ndarray:
    """The DIST_BUCKETS label of each `dist_to_pickup` (steps)."""
    dist = np.asarray(dist)
    conditions = [dist <= last for _, _, last in DIST_BUCKETS if last is not None]
    labels = [label for label, _, _ in DIST_BUCKETS]
    return np.select(conditions, labels[:-1], default=labels[-1])


def _label(value: Any) -> str:
    if isinstance(value, float | np.floating):
        return f"{float(value):g}"
    return str(value)


def mae_by(
    keys: Any, observed: Any, predicted: Any, order: Sequence[Any] | None = None
) -> dict[str, dict[str, Any]]:
    """{key: {"rows": n, "mae": steps}} per value of `keys`, sorted, or in `order` if given.

    Keys become strings, floats in their shortest form such as 0.05. A value in `order` with no
    rows gets 0 rows and a NaN MAE.
    """
    keys = np.asarray(keys, dtype=object)
    observed = np.asarray(observed, dtype=np.float64)
    predicted = np.asarray(predicted, dtype=np.float64)
    values = list(order) if order is not None else sorted(set(keys.tolist()))
    result = {}
    for value in values:
        mask = keys == value
        result[_label(value)] = {
            "rows": int(mask.sum()),
            "mae": mae(observed[mask], predicted[mask]),
        }
    return result


def offline_metrics(rows: pd.DataFrame, predicted: Any) -> dict[str, Any]:
    """The section 8 metrics of `predicted` (steps) against the rows' `steps_to_pickup`.

    `rows` comes from the dataset table and needs `steps_to_pickup`, `dist_to_pickup`,
    `fleet_size`, `arrival_rate`, and `allocator`. Returns rows, mae, rmse, and bias over all
    rows; `delayed`, the same on delayed rows; and `mae_by`, the rows and MAE per fleet size,
    arrival rate, logging allocator, and dist_to_pickup bucket.
    """
    observed = rows["steps_to_pickup"].to_numpy(dtype=np.float64)
    predicted = np.asarray(predicted, dtype=np.float64)
    delayed = (rows["steps_to_pickup"] > rows["dist_to_pickup"]).to_numpy()
    buckets = [label for label, _, _ in DIST_BUCKETS]
    return {
        **error_summary(observed, predicted),
        "delayed": error_summary(observed[delayed], predicted[delayed]),
        "mae_by": {
            "fleet_size": mae_by(rows["fleet_size"], observed, predicted),
            "arrival_rate": mae_by(rows["arrival_rate"], observed, predicted),
            "allocator": mae_by(rows["allocator"], observed, predicted),
            "dist_bucket": mae_by(
                dist_bucket(rows["dist_to_pickup"]), observed, predicted, buckets
            ),
        },
    }


def _number(value: Any) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "-"
    return f"{value:.2f}"


def _table(header: list[str], rows: list[list[str]], text_columns: int = 1) -> list[str]:
    align = ["---"] * text_columns + ["---:"] * (len(header) - text_columns)
    lines = [header, align, *rows]
    return ["| " + " | ".join(cells) + " |" for cells in lines]


def tables(metrics: Mapping[str, Any], split: str = "test") -> str:
    """Markdown tables of one split's metrics for every model in a train.py metrics file."""
    data = metrics["data"]
    models = metrics["models"]
    exported = metrics["exported"]["family"]
    seeds = dataset.format_seeds(data["seeds"][split])
    lines = [
        f"Offline {split} split: {data['rows'][split]} rows from seeds {seeds}, data commit "
        f"{data['commit']}. Model file `{metrics['name']}.json` holds the {exported} model.",
        "Errors in steps. Bias is predicted minus observed steps_to_pickup. Delayed rows have "
        "steps_to_pickup > dist_to_pickup.",
        "",
    ]

    def model_label(family: str) -> str:
        notes = [] if models[family]["deployable"] else ["not deployable"]
        notes += ["exported"] if family == exported else []
        return family + (f" ({', '.join(notes)})" if notes else "")

    header = ["Model", "Candidate", "MAE", "RMSE", "Bias"]
    header += ["Delayed MAE", "Delayed RMSE", "Delayed bias"]
    body = []
    for family, model in models.items():
        result = model[split]
        cells = [model_label(family), model["candidate"]]
        cells += [_number(result[key]) for key in ("mae", "rmse", "bias")]
        cells += [_number(result["delayed"][key]) for key in ("mae", "rmse", "bias")]
        body.append(cells)
    lines += _table(header, body, text_columns=2)
    first = next(iter(models.values()))[split]
    lines += ["", f"Delayed rows: {first['delayed']['rows']} of {first['rows']}."]

    for group, title in GROUP_TITLES.items():
        values = list(first["mae_by"][group])
        body = [["rows", *[str(first["mae_by"][group][value]["rows"]) for value in values]]]
        for family, model in models.items():
            by_value = model[split]["mae_by"][group]
            body.append([model_label(family), *[_number(by_value[v]["mae"]) for v in values]])
        lines += ["", title, "", *_table(["Model", *values], body)]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fleet_learning.evaluate",
        description="Print the offline tables of section 8 of docs/learning_track.md, as "
        "Markdown, from a metrics file that train.py wrote.",
    )
    parser.add_argument("metrics", type=Path, help="learning/models/<name>.metrics.json")
    parser.add_argument("--split", choices=("test", "validation"), default="test")
    args = parser.parse_args(argv)
    try:
        metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
        text = tables(metrics, args.split)
    except (OSError, ValueError) as error:
        print(f"error: {args.metrics}: {error}", file=sys.stderr)
        return 1
    except (KeyError, TypeError) as error:
        print(f"error: {args.metrics}: not a train.py metrics file ({error!r})", file=sys.stderr)
        return 1
    print(f"From `python -m fleet_learning.evaluate {args.metrics} --split {args.split}`:\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())

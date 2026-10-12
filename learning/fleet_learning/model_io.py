"""Write, read, and check `fleet_cost_model` files, and predict with them in numpy.

See section 4 of docs/learning_track.md. The prediction here is the Python reference for the
C++ `learned` cost: it must match `predict_cost` within 1e-6 on every row.

Usage:
    python -m fleet_learning.model_io check MODEL.json
    python -m fleet_learning.model_io predict MODEL.json ROWS.csv
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from fleet_learning.schema import FEATURE_COLUMNS

FORMAT = "fleet_cost_model"
VERSION = 1
TARGET = "steps_to_pickup"
REQUIRED_KEYS = (
    "format",
    "version",
    "target",
    "features",
    "mean",
    "scale",
    "layers",
    "base_feature",
)
OPTIONAL_KEYS = ("info",)
LAYER_KEYS = ("weights", "bias", "activation")
MAX_FEATURES = 32
MAX_LAYERS = 4
MAX_UNITS = 64
COST_MIN = 0.0
COST_MAX = 100000.0


class ModelError(ValueError):
    """A model file that the C++ side would reject. The message names the field."""


def _is_number(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool)


def _check_numbers(field: str, values: Any, length: int | None = None) -> None:
    if not isinstance(values, list):
        raise ModelError(f"{field}: must be a list of numbers")
    if length is not None and len(values) != length:
        raise ModelError(f"{field}: has {len(values)} entries, expected {length}")
    for i, value in enumerate(values):
        if not _is_number(value):
            raise ModelError(f"{field}[{i}]: must be a number")
        if not math.isfinite(value):
            raise ModelError(f"{field}[{i}]: must be finite")


def _check_feature_name(field: str, name: Any) -> None:
    if not isinstance(name, str) or name not in FEATURE_COLUMNS:
        raise ModelError(f"{field}: {name!r} is not a feature column of assignments.csv")


def validate_model(model: Any) -> None:
    """Raise ModelError unless `model` follows every rule of format version 1."""
    if not isinstance(model, dict):
        raise ModelError("model: must be a JSON object")
    for key in REQUIRED_KEYS:
        if key not in model:
            raise ModelError(f"{key}: missing")
    for key in model:
        if key not in REQUIRED_KEYS + OPTIONAL_KEYS:
            raise ModelError(f"{key}: unknown key")

    if model["format"] != FORMAT:
        raise ModelError(f"format: must be {FORMAT!r}")
    if not isinstance(model["version"], int) or isinstance(model["version"], bool):
        raise ModelError(f"version: must be the integer {VERSION}")
    if model["version"] != VERSION:
        raise ModelError(f"version: must be {VERSION}")
    if model["target"] != TARGET:
        raise ModelError(f"target: must be {TARGET!r}")

    features = model["features"]
    if not isinstance(features, list) or not 1 <= len(features) <= MAX_FEATURES:
        raise ModelError(f"features: must be a list of 1 to {MAX_FEATURES} names")
    for i, name in enumerate(features):
        _check_feature_name(f"features[{i}]", name)
    if len(set(features)) != len(features):
        raise ModelError("features: names must be distinct")

    _check_numbers("mean", model["mean"], len(features))
    _check_numbers("scale", model["scale"], len(features))
    for i, value in enumerate(model["scale"]):
        if value <= 0:
            raise ModelError(f"scale[{i}]: must be greater than 0")

    layers = model["layers"]
    if not isinstance(layers, list) or not 1 <= len(layers) <= MAX_LAYERS:
        raise ModelError(f"layers: must be a list of 1 to {MAX_LAYERS} layers")
    inputs = len(features)
    for k, layer in enumerate(layers):
        field = f"layers[{k}]"
        if not isinstance(layer, dict) or sorted(layer) != sorted(LAYER_KEYS):
            raise ModelError(f"{field}: must have exactly the keys {', '.join(LAYER_KEYS)}")
        weights = layer["weights"]
        if not isinstance(weights, list) or not weights:
            raise ModelError(f"{field}.weights: must be a non-empty list of rows")
        if len(weights) > MAX_UNITS:
            raise ModelError(f"{field}.weights: has {len(weights)} outputs, at most {MAX_UNITS}")
        for r, row in enumerate(weights):
            _check_numbers(f"{field}.weights[{r}]", row, inputs)
        _check_numbers(f"{field}.bias", layer["bias"], len(weights))
        last = k == len(layers) - 1
        expected = "identity" if last else "relu"
        if layer["activation"] != expected:
            raise ModelError(f"{field}.activation: must be {expected!r}")
        if last and len(weights) != 1:
            raise ModelError(f"{field}.weights: the last layer must have exactly one output")
        inputs = len(weights)

    if model["base_feature"] is not None:
        _check_feature_name("base_feature", model["base_feature"])


def _reject_constant(name: str) -> float:
    raise ModelError(f"model: {name} is not a finite number")


def load_model(path: str | Path) -> dict[str, Any]:
    """Read a model file and check it. Raises ModelError for files the C++ side rejects."""
    with open(path, encoding="utf-8") as f:
        model = json.load(f, parse_constant=_reject_constant)
    validate_model(model)
    return model


def write_model(model: Mapping[str, Any], path: str | Path) -> None:
    """Check a model and write it as JSON. Raises ModelError before writing anything bad."""
    model = dict(model)
    validate_model(model)
    text = json.dumps(model, indent=2, allow_nan=False)
    Path(path).write_text(text + "\n", encoding="utf-8")


def free_flow_model() -> dict[str, Any]:
    """The model whose cost is exactly `dist_to_pickup` (steps)."""
    return {
        "format": FORMAT,
        "version": VERSION,
        "target": TARGET,
        "features": ["dist_to_pickup"],
        "mean": [0],
        "scale": [1],
        "layers": [{"weights": [[0]], "bias": [0], "activation": "identity"}],
        "base_feature": "dist_to_pickup",
    }


def predict(model: Mapping[str, Any], rows: pd.DataFrame | Mapping[str, Any]) -> np.ndarray:
    """Cost of each row in steps, computed exactly as the C++ `learned` allocator does.

    `rows` holds one column per name in `features` (and `base_feature`, if set); other columns
    are ignored. Returns a float64 array, one cost per row, clamped to [0, 100000].
    """
    needed = list(model["features"])
    if model["base_feature"] is not None:
        needed.append(model["base_feature"])
    missing = [name for name in needed if name not in rows]
    if missing:
        raise ModelError(f"rows: missing columns {', '.join(missing)}")

    x = np.column_stack([np.asarray(rows[name], dtype=np.float64) for name in model["features"]])
    h = (x - np.asarray(model["mean"], dtype=np.float64)) / np.asarray(
        model["scale"], dtype=np.float64
    )
    for layer in model["layers"]:
        weights = np.asarray(layer["weights"], dtype=np.float64)
        bias = np.asarray(layer["bias"], dtype=np.float64)
        h = h @ weights.T + bias
        if layer["activation"] == "relu":
            h = np.maximum(h, 0.0)
    y = h[:, 0]
    if model["base_feature"] is not None:
        y = y + np.asarray(rows[model["base_feature"]], dtype=np.float64)
    return np.clip(y, COST_MIN, COST_MAX)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fleet_learning.model_io",
        description="Check fleet_cost_model files and predict with them.",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    check = commands.add_parser("check", help="check a model file against format version 1")
    check.add_argument("model", type=Path)
    run = commands.add_parser(
        "predict", help="print row,prediction for every row of a CSV, like predict_cost"
    )
    run.add_argument("model", type=Path)
    run.add_argument("rows", type=Path)
    args = parser.parse_args(argv)

    try:
        model = load_model(args.model)
    except (OSError, ValueError) as error:
        print(f"error: {args.model}: {error}", file=sys.stderr)
        return 1
    if args.command == "check":
        print(f"ok         {args.model}")
        return 0

    try:
        rows = pd.read_csv(args.rows, dtype="float64", usecols=lambda c: c in FEATURE_COLUMNS)
        costs = predict(model, rows)
    except (OSError, ValueError) as error:
        print(f"error: {args.rows}: {error}", file=sys.stderr)
        return 1
    print("row,prediction")
    for i, cost in enumerate(costs):
        print(f"{i},{cost:.17g}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

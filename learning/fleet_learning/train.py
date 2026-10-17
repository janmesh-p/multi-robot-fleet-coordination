"""Fit the candidate models of section 8 and export the chosen one as a fleet_cost_model file.

Fits every candidate on the training split, chooses by MAE on the validation split, and scores
on the offline test split; the splits are by seed (section 7 of docs/learning_track.md). Costs
and errors are in steps. Writes <out>/<name>.json and <out>/<name>.metrics.json.

Usage:
    python -m fleet_learning.train --data learning/data/<commit> --name cost_model_v1
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import re
import shlex
import sys
from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import sklearn
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.linear_model import LinearRegression
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler

from fleet_learning import dataset, evaluate, model_io, schema

FAMILIES: tuple[str, ...] = ("free_flow", "linear", "mlp", "gbdt")
DEPLOYABLE: tuple[str, ...] = ("free_flow", "linear", "mlp")  # format v1 cannot hold trees
TARGETS: tuple[str, ...] = ("direct", "excess")  # steps_to_pickup, or minus dist_to_pickup
BASE_FEATURE = "dist_to_pickup"
# From the Stage 5 rerun on, order_age is not a model input (section 6): it is the same for every
# robot that could take an order, so it can only reorder the queue.
EXCLUDED_FEATURES: tuple[str, ...] = ("order_age",)
MODEL_FEATURES: tuple[str, ...] = tuple(
    name for name in schema.FEATURE_COLUMNS if name not in EXCLUDED_FEATURES
)

DEFAULT_HIDDEN: tuple[tuple[int, ...], ...] = ((32,), (64, 64), (64, 64, 64))
DEFAULT_SEED = 0
DEFAULT_OUT = Path("learning/models")
PARITY_TOLERANCE = 1e-9  # steps: exported file against scikit-learn on the test split

# Fixed hyperparameters, all written out so the metrics file records them.
MLP_PARAMS: dict[str, Any] = {
    "activation": "relu",
    "solver": "adam",
    "alpha": 1e-4,
    "batch_size": 200,
    "learning_rate_init": 1e-3,
    "max_iter": 200,
    "tol": 1e-4,
    "n_iter_no_change": 10,
    "early_stopping": False,
}
GBDT_PARAMS: dict[str, Any] = {
    "learning_rate": 0.1,
    "max_iter": 200,
    "max_leaf_nodes": 31,
    "min_samples_leaf": 20,
    "early_stopping": True,
    "validation_fraction": 0.1,
    "n_iter_no_change": 10,
}

COMMIT_PATTERN = re.compile(r"[0-9a-f]{12}")
NAME_PATTERN = re.compile(r"[A-Za-z0-9_-]+")

Log = Callable[[str], None]


def _quiet(_: str) -> None:
    pass


def _matrix(rows: pd.DataFrame, features: Sequence[str]) -> np.ndarray:
    return rows[list(features)].to_numpy(dtype=np.float64)


def target_values(rows: pd.DataFrame, target: str) -> np.ndarray:
    """The target in steps: `steps_to_pickup` (direct), or minus `dist_to_pickup` (excess)."""
    steps = rows["steps_to_pickup"].to_numpy(dtype=np.float64)
    if target == "direct":
        return steps
    if target == "excess":
        return steps - rows[BASE_FEATURE].to_numpy(dtype=np.float64)
    raise ValueError(f"unknown target {target!r}; use {', '.join(TARGETS)}")


@dataclass
class FittedModel:
    """One fitted candidate. Its cost is in steps, clamped to [0, 100000] like the C++ side.

    The regressor sees the features standardized by `x_scaler` (if set) and predicts the target
    as (target - y_mean) / y_scale. For the excess target, the cost adds `dist_to_pickup` back.
    Without a regressor the target is predicted as 0, which makes the free-flow model.
    """

    name: str
    family: str
    target: str
    features: tuple[str, ...]
    params: dict[str, Any] = field(default_factory=dict)
    regressor: Any = None
    x_scaler: StandardScaler | None = None
    y_mean: float = 0.0
    y_scale: float = 1.0

    @property
    def deployable(self) -> bool:
        return self.family in DEPLOYABLE

    def predict(self, rows: pd.DataFrame) -> np.ndarray:
        """Cost of each row in steps."""
        y = np.zeros(len(rows))
        if self.regressor is not None:
            x = _matrix(rows, self.features)
            if self.x_scaler is not None:
                x = self.x_scaler.transform(x)
            y = self.regressor.predict(x) * self.y_scale + self.y_mean
        if self.target == "excess":
            y = y + rows[BASE_FEATURE].to_numpy(dtype=np.float64)
        return np.clip(y, model_io.COST_MIN, model_io.COST_MAX)

    def to_model_file(self, info: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """This model as a fleet_cost_model v1 dict. Raises ValueError for gbdt."""
        if self.family == "free_flow":
            model = model_io.free_flow_model()
        elif self.family in ("linear", "mlp"):
            model = {
                "format": model_io.FORMAT,
                "version": model_io.VERSION,
                "target": model_io.TARGET,
                "features": list(self.features),
                "mean": self.x_scaler.mean_.tolist(),
                "scale": self.x_scaler.scale_.tolist(),
                "layers": self._layers(),
                "base_feature": BASE_FEATURE if self.target == "excess" else None,
            }
        else:
            raise ValueError(f"{self.name}: format v1 cannot hold a {self.family} model")
        if info:
            model["info"] = dict(info)
        return model

    def _layers(self) -> list[dict[str, Any]]:
        if self.family == "linear":
            weights = [np.reshape(self.regressor.coef_, (1, -1))]
            biases = [np.reshape(self.regressor.intercept_, (1,))]
        else:
            # scikit-learn stores coefs_[k] as (inputs, outputs); format v1 wants the transpose.
            weights = [w.T for w in self.regressor.coefs_]
            biases = list(self.regressor.intercepts_)
        # Fold the target scaling into the last layer: target = y_scale * output + y_mean.
        weights[-1] = weights[-1] * self.y_scale
        biases[-1] = biases[-1] * self.y_scale + self.y_mean
        last = len(weights) - 1
        return [
            {
                "weights": w.tolist(),
                "bias": b.tolist(),
                "activation": "identity" if k == last else "relu",
            }
            for k, (w, b) in enumerate(zip(weights, biases, strict=True))
        ]


def check_features(features: Sequence[str]) -> tuple[str, ...]:
    """Raise ValueError unless `features` are 1 to 32 distinct feature columns (section 6),
    none of them excluded as model inputs."""
    features = tuple(features)
    unknown = [name for name in features if name not in schema.FEATURE_COLUMNS]
    if unknown:
        raise ValueError(f"not feature columns of assignments.csv: {', '.join(unknown)}")
    excluded = [name for name in features if name in EXCLUDED_FEATURES]
    if excluded:
        raise ValueError(f"not a model input from the Stage 5 rerun on: {', '.join(excluded)}")
    if not 1 <= len(features) <= model_io.MAX_FEATURES or len(set(features)) != len(features):
        raise ValueError(f"features must be 1 to {model_io.MAX_FEATURES} distinct names")
    return features


def check_hidden(hidden: Sequence[int]) -> tuple[int, ...]:
    """Raise ValueError unless `hidden` is 1 to 3 layers of 1 to 64 units (section 8)."""
    hidden = tuple(int(units) for units in hidden)
    layers = model_io.MAX_LAYERS - 1
    if not 1 <= len(hidden) <= layers or not all(1 <= u <= model_io.MAX_UNITS for u in hidden):
        raise ValueError(
            f"hidden layers {hidden}: use 1 to {layers} layers of 1 to {model_io.MAX_UNITS} units"
        )
    return hidden


def free_flow() -> FittedModel:
    """The cost `dist_to_pickup`, with nothing to fit."""
    return FittedModel("free_flow", "free_flow", "excess", (BASE_FEATURE,))


def fit_linear(train: pd.DataFrame, features: Sequence[str], target: str) -> FittedModel:
    """Least squares on standardized features."""
    x = _matrix(train, features)
    scaler = StandardScaler().fit(x)
    regressor = LinearRegression().fit(scaler.transform(x), target_values(train, target))
    return FittedModel(
        f"linear_{target}", "linear", target, tuple(features), regressor=regressor, x_scaler=scaler
    )


def fit_mlp(
    train: pd.DataFrame,
    features: Sequence[str],
    target: str,
    hidden: Sequence[int],
    seed: int = DEFAULT_SEED,
) -> FittedModel:
    """A ReLU MLPRegressor on standardized features and a standardized target."""
    hidden = check_hidden(hidden)
    x = _matrix(train, features)
    scaler = StandardScaler().fit(x)
    y = target_values(train, target)
    y_mean = float(np.mean(y))
    y_scale = float(np.std(y)) or 1.0
    regressor = MLPRegressor(hidden_layer_sizes=hidden, random_state=seed, **MLP_PARAMS)
    regressor.fit(scaler.transform(x), (y - y_mean) / y_scale)
    params = {"hidden_layer_sizes": list(hidden), "random_state": seed, **MLP_PARAMS}
    params["n_iter"] = int(regressor.n_iter_)
    name = f"mlp_{target}_{'x'.join(str(units) for units in hidden)}"
    return FittedModel(
        name, "mlp", target, tuple(features), params, regressor, scaler, y_mean, y_scale
    )


def fit_gbdt(
    train: pd.DataFrame, features: Sequence[str], target: str, seed: int = DEFAULT_SEED
) -> FittedModel:
    """A HistGradientBoostingRegressor on raw features: an offline reference, not deployable."""
    regressor = HistGradientBoostingRegressor(random_state=seed, **GBDT_PARAMS)
    regressor.fit(_matrix(train, features), target_values(train, target))
    params = {"random_state": seed, **GBDT_PARAMS, "n_iter": int(regressor.n_iter_)}
    return FittedModel(f"gbdt_{target}", "gbdt", target, tuple(features), params, regressor)


def fit_candidates(
    train: pd.DataFrame,
    features: Sequence[str] = MODEL_FEATURES,
    hidden_sizes: Sequence[Sequence[int]] = DEFAULT_HIDDEN,
    seed: int = DEFAULT_SEED,
) -> Iterator[FittedModel]:
    """Fit and yield every candidate, one at a time: free_flow, then linear, mlp (one per
    hidden size), and gbdt, each for both targets."""
    features = check_features(features)
    yield free_flow()
    for target in TARGETS:
        yield fit_linear(train, features, target)
    for target in TARGETS:
        for hidden in hidden_sizes:
            yield fit_mlp(train, features, target, hidden, seed)
    for target in TARGETS:
        yield fit_gbdt(train, features, target, seed)


def choose(
    winners: Mapping[str, FittedModel], validation: Mapping[str, Mapping[str, Any]], export: str
) -> FittedModel:
    """The model to export: with `export` "best", the deployable family winner with the lowest
    validation MAE; otherwise the winner of the family `export` names."""
    if export == "best":
        return min((winners[f] for f in DEPLOYABLE), key=lambda m: validation[m.name]["mae"])
    if export in DEPLOYABLE:
        return winners[export]
    raise ValueError(f"cannot export {export!r}; use best, {', '.join(DEPLOYABLE)}")


@dataclass
class TrainingResult:
    """Every candidate with its validation metrics, each family's winner, and the choice."""

    candidates: list[FittedModel]
    validation: dict[str, dict[str, Any]]  # candidate name -> evaluate.offline_metrics
    winners: dict[str, FittedModel]  # family -> candidate with the lowest validation MAE
    chosen: FittedModel
    test: dict[str, dict[str, Any]]  # family -> evaluate.offline_metrics of its winner


def train_models(
    splits: Mapping[str, pd.DataFrame],
    features: Sequence[str] = MODEL_FEATURES,
    hidden_sizes: Sequence[Sequence[int]] = DEFAULT_HIDDEN,
    seed: int = DEFAULT_SEED,
    export: str = "best",
    log: Log = _quiet,
) -> TrainingResult:
    """Fit on splits["train"], choose on splits["validation"], and score on splits["test"].

    Each family's winner has the lowest validation MAE among its candidates: both targets and,
    for mlp, every hidden size. Ties go to the earlier candidate.
    """
    for name in dataset.OFFLINE_SPLITS:
        if splits[name].empty:
            first, last = dataset.SEED_USES[name]
            raise ValueError(
                f"the {name} split has no rows; it needs runs with seeds {first}-{last}"
            )
    if export != "best" and export not in DEPLOYABLE:
        raise ValueError(f"cannot export {export!r}; use best, {', '.join(DEPLOYABLE)}")
    log(f"fit        on {len(splits['train'])} train rows; validation MAE in steps:")
    candidates, validation = [], {}
    for model in fit_candidates(splits["train"], features, hidden_sizes, seed):
        validation[model.name] = evaluate.offline_metrics(
            splits["validation"], model.predict(splits["validation"])
        )
        candidates.append(model)
        log(f"           {model.name:<24}{validation[model.name]['mae']:8.3f}")
    winners = {
        family: min(
            (model for model in candidates if model.family == family),
            key=lambda model: validation[model.name]["mae"],
        )
        for family in FAMILIES
    }
    chosen = choose(winners, validation, export)
    test = {
        family: evaluate.offline_metrics(splits["test"], model.predict(splits["test"]))
        for family, model in winners.items()
    }
    return TrainingResult(candidates, validation, winners, chosen, test)


class ExportError(RuntimeError):
    """An exported model file whose costs differ from the scikit-learn model's."""


def export_model(
    model: FittedModel, path: Path, test: pd.DataFrame, info: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """Write `model` to `path`, read it back, and compare costs on the test rows (section 8).

    Returns the file name, its SHA-256, and the largest cost difference in steps between
    model_io and scikit-learn. Deletes the file and raises ExportError when that difference is
    over PARITY_TOLERANCE.
    """
    model_io.write_model(model.to_model_file(info), path)
    loaded = model_io.load_model(path)
    difference = float(np.max(np.abs(model_io.predict(loaded, test) - model.predict(test))))
    if not difference <= PARITY_TOLERANCE:
        path.unlink()
        raise ExportError(
            f"{path}: model_io and scikit-learn costs differ by up to {difference} steps on the "
            f"test split, over {PARITY_TOLERANCE}. Deleted the file."
        )
    sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"file": path.name, "sha256": sha256, "max_abs_difference": difference}


def data_commit(data_dir: str | Path) -> str:
    """The commit a collection belongs to: the name of `learning/data/<commit>` (section 7)."""
    name = Path(data_dir).resolve().name
    if not COMMIT_PATTERN.fullmatch(name):
        raise ValueError(
            f"{data_dir}: expected learning/data/<commit>, where <commit> is the first 12 "
            "characters of a commit hash (section 7)"
        )
    return name


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, bool | np.bool_):
        return bool(value)
    if isinstance(value, int | np.integer):
        return int(value)
    if isinstance(value, float | np.floating):
        return None if math.isnan(value) else float(value)
    return value


def _summary(metrics: Mapping[str, Any]) -> dict[str, Any]:
    return {key: metrics[key] for key in ("rows", "mae", "rmse", "bias", "delayed")}


def train(
    data_dir: str | Path,
    name: str,
    out_dir: str | Path = DEFAULT_OUT,
    *,
    features: Sequence[str] = MODEL_FEATURES,
    hidden_sizes: Sequence[Sequence[int]] = DEFAULT_HIDDEN,
    seed: int = DEFAULT_SEED,
    margin: int = dataset.MARGIN_STEPS,
    export: str = "best",
    command: str | None = None,
    log: Log = _quiet,
) -> dict[str, Any]:
    """Train on the collection `data_dir` and write <out_dir>/<name>.json and .metrics.json.

    Returns the metrics as written: data commit, seeds and rows per split, the rows each filter
    dropped, features, hyperparameters, random seed, and validation and test metrics. JSON has
    no NaN, so a metric without rows is written as null.
    """
    if not NAME_PATTERN.fullmatch(name):
        raise ValueError(f"model name {name!r}: use letters, digits, _ and -")
    features = check_features(features)
    hidden_sizes = [check_hidden(hidden) for hidden in hidden_sizes]
    commit = data_commit(data_dir)
    runs = dataset.read_runs([Path(data_dir) / "runs"])
    rows = dataset.read_rows(runs)
    kept, counts = dataset.apply_filters(rows, margin)
    for line in dataset.report(runs, rows, kept, counts, margin):
        log(line)
    splits = dataset.split_by_seed(kept)
    result = train_models(splits, features, hidden_sizes, seed, export, log)

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    metrics_file = out_dir / f"{name}.metrics.json"
    info = {"name": name, "candidate": result.chosen.name, "data_commit": commit}
    exported = export_model(result.chosen, out_dir / f"{name}.json", splits["test"], info)
    metrics = {
        "name": name,
        "command": command,
        "data": {
            "dir": str(data_dir),
            "commit": commit,
            "runs": len(runs),
            "aborted_runs": sorted(runs.loc[runs["aborted"], "run"]),
            "margin_steps": margin,
            "filters": counts.to_dict("records"),
            "longest_kept_steps_to_pickup": int(kept["steps_to_pickup"].max()),
            "seeds": {
                split: sorted(set(frame["seed"].tolist())) for split, frame in splits.items()
            },
            "rows": {split: len(frame) for split, frame in splits.items()},
        },
        "features": list(features),
        "targets": list(TARGETS),
        "hidden_sizes": [list(hidden) for hidden in hidden_sizes],
        "random_seed": seed,
        "selection": {"metric": "mae", "split": "validation", "export": export},
        "candidates": [
            {
                "name": model.name,
                "family": model.family,
                "target": model.target,
                "params": model.params,
                "validation": _summary(result.validation[model.name]),
            }
            for model in result.candidates
        ],
        "models": {
            family: {
                "candidate": model.name,
                "deployable": model.deployable,
                "validation": result.validation[model.name],
                "test": result.test[family],
            }
            for family, model in result.winners.items()
        },
        "exported": {"family": result.chosen.family, "candidate": result.chosen.name, **exported},
        "versions": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "pandas": pd.__version__,
            "scikit-learn": sklearn.__version__,
        },
    }
    metrics = _jsonable(metrics)
    metrics_file.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    log(f"chosen     {result.chosen.name}, by validation MAE (export: {export})")
    test_mae = ", ".join(f"{f} {result.test[f]['mae']:.3f}" for f in FAMILIES)
    log(f"test       MAE in steps: {test_mae} (gbdt is not deployable)")
    difference = f"{exported['max_abs_difference']:.1e}"
    log(f"parity     model_io matches scikit-learn on the test split within {difference} steps")
    log(f"wrote      {out_dir / exported['file']}")
    log(f"           {metrics_file}")
    log(f"tables     python -m fleet_learning.evaluate {metrics_file}")
    return metrics


def _hidden_arg(text: str) -> tuple[int, ...]:
    try:
        return check_hidden(int(units) for units in text.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"{text!r}: {error}") from error


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m fleet_learning.train",
        description="Fit the candidate models of section 8 of docs/learning_track.md on one "
        "collection, choose on the validation split, score on the test split, and export the "
        "chosen model.",
    )
    parser.add_argument("--data", type=Path, required=True, help="learning/data/<commit>")
    parser.add_argument(
        "--name", required=True, help="model name: writes <out>/<name>.json and .metrics.json"
    )
    parser.add_argument(
        "--out", type=Path, default=DEFAULT_OUT, help="output folder (default: %(default)s)"
    )
    parser.add_argument(
        "--features",
        type=lambda text: tuple(text.split(",")),
        default=MODEL_FEATURES,
        help="comma-separated feature columns (default: all of them except order_age)",
    )
    parser.add_argument(
        "--hidden",
        type=_hidden_arg,
        nargs="+",
        default=DEFAULT_HIDDEN,
        help="mlp hidden layer sizes to try, for example 32 64,64 (default: 32 64,64 64,64,64)",
    )
    parser.add_argument(
        "--export",
        choices=("best", *DEPLOYABLE),
        default="best",
        help="best: the deployable model with the lowest validation MAE (default: best)",
    )
    parser.add_argument(
        "--seed", type=int, default=DEFAULT_SEED, help="mlp and gbdt random_state (default: 0)"
    )
    parser.add_argument(
        "--margin",
        type=int,
        default=dataset.MARGIN_STEPS,
        help="drop rows with t > steps_run - MARGIN, in steps (default: %(default)s)",
    )
    args = parser.parse_args(argv)
    words = sys.argv[1:] if argv is None else argv
    command = f"python -m fleet_learning.train {shlex.join(words)}"
    try:
        train(
            args.data,
            args.name,
            args.out,
            features=args.features,
            hidden_sizes=args.hidden,
            seed=args.seed,
            margin=args.margin,
            export=args.export,
            command=command,
            log=print,
        )
    except (OSError, ValueError, ExportError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

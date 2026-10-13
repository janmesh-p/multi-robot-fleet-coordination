"""Python and C++ agree on the learned cost (core tests, section 10 of docs/learning_track.md)."""

import io
import itertools
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

from fleet_learning import collect, model_io, schema

REPO = Path(__file__).resolve().parents[2]
FIXTURE = Path(__file__).parent / "fixtures" / "run_tiny"


def model(features, mean, scale, layers, base_feature="dist_to_pickup"):
    return {
        "format": model_io.FORMAT,
        "version": model_io.VERSION,
        "target": model_io.TARGET,
        "features": features,
        "mean": mean,
        "scale": scale,
        "layers": layers,
        "base_feature": base_feature,
    }


def layer(weights, bias, activation="identity"):
    return {"weights": weights, "bias": bias, "activation": activation}


def mlp(seed: int = 0) -> dict:
    """A ReLU MLP over every feature column, 19-16-8-1, with random weights from `seed`."""
    rng = np.random.default_rng(seed)
    features = list(schema.FEATURE_COLUMNS)
    sizes = [len(features), 16, 8, 1]
    layers = [
        layer(
            rng.normal(0, 1 / np.sqrt(inputs), (outputs, inputs)).tolist(),
            rng.normal(0, 0.5, outputs).tolist(),
            "identity" if outputs == 1 else "relu",
        )
        for inputs, outputs in itertools.pairwise(sizes)
    ]
    mean = rng.uniform(0, 20, len(features)).tolist()
    return model(features, mean, rng.uniform(0.5, 10, len(features)).tolist(), layers)


MODELS = {
    "free_flow": model_io.free_flow_model(),
    "linear": model(
        ["dist_to_pickup", "manhattan_to_pickup", "robots_in_box", "pickup_assigned", "order_age"],
        [14.2, 13.1, 1.3, 0.9, 6.0],
        [7.5, 7.1, 1.6, 1.1, 5.5],
        [layer([[0.9, -0.35, 1.7, 3.1, 0.06]], [2.25])],
    ),
    "mlp": mlp(),
    # 100000 * (dist - 20): below 0 for short trips and above 100000 for long ones.
    "clamped": model(["dist_to_pickup"], [20], [1], [layer([[1e5]], [0])], None),
    # Overflows to inf - inf, a NaN that counts as 100000.
    "overflow": model(
        ["dist_to_pickup"],
        [0],
        [1e-300],
        [layer([[1e300], [1e300]], [0, 0], "relu"), layer([[1, -1]], [0])],
        None,
    ),
}


def run_predict_cost(tool: Path, model_path: Path, rows_path: Path) -> pd.DataFrame:
    result = subprocess.run(
        [str(tool), "--model", str(model_path), "--rows", str(rows_path)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.startswith("row,prediction\n")
    return pd.read_csv(io.StringIO(result.stdout), dtype={"row": "int64", "prediction": "float64"})


@pytest.mark.core
@pytest.mark.parametrize("name", MODELS)
def test_predict_cost_and_model_io_agree_on_the_fixture(tmp_path, predict_cost, name):
    path = tmp_path / f"{name}.json"
    model_io.write_model(MODELS[name], path)
    rows = schema.read_assignments(FIXTURE)
    cpp = run_predict_cost(predict_cost, path, FIXTURE / "assignments.csv")
    assert cpp["row"].tolist() == list(range(len(rows)))
    python = model_io.predict(model_io.load_model(path), rows)
    np.testing.assert_allclose(cpp["prediction"], python, rtol=0, atol=1e-6)


def test_the_clamp_and_overflow_models_reach_both_bounds():
    rows = schema.read_assignments(FIXTURE)
    clamped = model_io.predict(MODELS["clamped"], rows)
    assert {model_io.COST_MIN, model_io.COST_MAX} <= set(clamped)
    assert set(model_io.predict(MODELS["overflow"], rows)) == {model_io.COST_MAX}


@pytest.mark.core
def test_both_skip_empty_lines(tmp_path, predict_cost, capsys):
    path = tmp_path / "free_flow.json"
    model_io.write_model(MODELS["free_flow"], path)
    lines = (FIXTURE / "assignments.csv").read_text().splitlines()
    rows_path = tmp_path / "rows.csv"
    rows_path.write_text("\n".join([lines[0], lines[1], "", *lines[2:], ""]) + "\n")
    cpp = run_predict_cost(predict_cost, path, rows_path)
    assert model_io.main(["predict", str(path), str(rows_path)]) == 0
    python = pd.read_csv(io.StringIO(capsys.readouterr().out))
    assert cpp["row"].tolist() == python["row"].tolist() == list(range(len(lines) - 1))
    np.testing.assert_array_equal(cpp["prediction"], python["prediction"])


@pytest.mark.core
def test_free_flow_model_replays_hungarian(tmp_path, run_sweep):
    # Its cost is exactly dist_to_pickup, so learned must make hungarian's every decision.
    model_path = tmp_path / "free_flow.json"
    model_io.write_model(MODELS["free_flow"], model_path)
    base_path = REPO / collect.BASE_CONFIG
    base = yaml.safe_load(base_path.read_text())
    sweeps = {}
    for allocator, extra in [
        ("hungarian", {}),
        ("learned", {"allocation.model_file": str(model_path)}),
    ]:
        changes = {"run.name": f"replay_{allocator}", "allocation.method": allocator, **extra}
        config_path = tmp_path / f"{allocator}.yaml"
        config = collect.make_config(base, base_path.parent, changes)
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        out = tmp_path / allocator
        command = [str(run_sweep), "--config", str(config_path), "--fleet", "5,10"]
        command += ["--seeds", "3000-3001", "--out", str(out)]
        result = subprocess.run(command, capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        sweeps[allocator] = pd.read_csv(out / "sweep.csv", dtype=str, keep_default_na=False)

    hungarian, learned = sweeps["hungarian"], sweeps["learned"]
    assert set(hungarian["allocator"]) == {"hungarian"}
    assert set(learned["allocator"]) == {"learned"}
    runs = ["fleet_size", "seed"]
    assert hungarian[runs].values.tolist() == [
        ["5", "3000"],
        ["5", "3001"],
        ["10", "3000"],
        ["10", "3001"],
    ]
    pd.testing.assert_frame_equal(hungarian[[*runs, "log_hash"]], learned[[*runs, "log_hash"]])

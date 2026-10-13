import json
import math

import numpy as np
import pandas as pd
import pytest

from fleet_learning import evaluate

# Three rows. Rows 1 and 2 are delayed: steps_to_pickup > dist_to_pickup.
ROWS = pd.DataFrame(
    {
        "steps_to_pickup": [4, 10, 30],
        "dist_to_pickup": [4, 6, 25],
        "fleet_size": [5, 5, 10],
        "arrival_rate": [0.1, 0.2, 0.2],
        "allocator": ["nearest", "hungarian", "nearest"],
    }
)
PREDICTED = [5, 8, 30]  # errors 1, -2, 0
FREE_FLOW_ROW = "| free_flow (exported) | free_flow | 1.00 | 1.29 | -0.33 | 1.00 | 1.41 | -1.00 |"
GBDT_ROW = "| gbdt (not deployable) | gbdt_excess | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |"


def test_error_metrics_on_known_values():
    observed, predicted = [10, 20, 30], [12, 18, 33]  # errors 2, -2, 3
    assert evaluate.mae(observed, predicted) == pytest.approx(7 / 3)
    assert evaluate.rmse(observed, predicted) == pytest.approx(math.sqrt(17 / 3))
    assert evaluate.bias(observed, predicted) == pytest.approx(1.0)
    assert evaluate.error_summary(observed, predicted) == {
        "rows": 3,
        "mae": pytest.approx(7 / 3),
        "rmse": pytest.approx(math.sqrt(17 / 3)),
        "bias": pytest.approx(1.0),
    }


def test_metrics_of_no_rows_are_nan():
    for metric in (evaluate.mae, evaluate.rmse, evaluate.bias):
        assert math.isnan(metric([], []))
    assert evaluate.error_summary([], [])["rows"] == 0


def test_lengths_must_match():
    with pytest.raises(ValueError, match="2 observed values but 1 predicted"):
        evaluate.mae([1, 2], [1])


def test_dist_buckets_follow_section_8():
    buckets = evaluate.dist_bucket([0, 5, 6, 10, 11, 20, 21, 300])
    assert buckets.tolist() == ["0-5", "0-5", "6-10", "6-10", "11-20", "11-20", "21+", "21+"]


def test_mae_by_sorts_keys_and_names_them_as_strings():
    assert evaluate.mae_by([10, 5, 10], [1, 1, 1], [2, 4, 1]) == {
        "5": {"rows": 1, "mae": 3.0},
        "10": {"rows": 2, "mae": 0.5},
    }
    assert list(evaluate.mae_by(np.array([0.2, 0.05, 0.1]), [0, 0, 0], [0, 0, 0])) == [
        "0.05",
        "0.1",
        "0.2",
    ]


def test_mae_by_keeps_empty_groups_in_order():
    result = evaluate.mae_by(["b"], [1], [3], order=["a", "b"])
    assert list(result) == ["a", "b"]
    assert result["a"]["rows"] == 0 and math.isnan(result["a"]["mae"])
    assert result["b"] == {"rows": 1, "mae": 2.0}


def test_offline_metrics_on_a_tiny_table():
    metrics = evaluate.offline_metrics(ROWS, PREDICTED)
    assert metrics["rows"] == 3
    assert metrics["mae"] == pytest.approx(1.0)
    assert metrics["rmse"] == pytest.approx(math.sqrt(5 / 3))
    assert metrics["bias"] == pytest.approx(-1 / 3)
    assert metrics["delayed"] == {
        "rows": 2,
        "mae": pytest.approx(1.0),
        "rmse": pytest.approx(math.sqrt(2)),
        "bias": pytest.approx(-1.0),
    }
    by = metrics["mae_by"]
    assert by["fleet_size"] == {"5": {"rows": 2, "mae": 1.5}, "10": {"rows": 1, "mae": 0.0}}
    assert by["arrival_rate"] == {"0.1": {"rows": 1, "mae": 1.0}, "0.2": {"rows": 2, "mae": 1.0}}
    assert by["allocator"] == {
        "hungarian": {"rows": 1, "mae": 2.0},
        "nearest": {"rows": 2, "mae": 0.5},
    }
    assert list(by["dist_bucket"]) == ["0-5", "6-10", "11-20", "21+"]
    assert [group["rows"] for group in by["dist_bucket"].values()] == [1, 1, 0, 1]
    assert by["dist_bucket"]["6-10"]["mae"] == 2.0
    assert math.isnan(by["dist_bucket"]["11-20"]["mae"])


def metrics_file_content():
    """What train.py writes, for two models scored on ROWS."""
    free_flow = evaluate.offline_metrics(ROWS, PREDICTED)
    free_flow["mae_by"]["dist_bucket"]["11-20"]["mae"] = None  # how JSON stores NaN
    return {
        "name": "m",
        "data": {"commit": "0123456789ab", "seeds": {"test": [1030, 1031]}, "rows": {"test": 3}},
        "models": {
            "free_flow": {"candidate": "free_flow", "deployable": True, "test": free_flow},
            "gbdt": {
                "candidate": "gbdt_excess",
                "deployable": False,
                "test": evaluate.offline_metrics(ROWS, ROWS["steps_to_pickup"]),
            },
        },
        "exported": {"family": "free_flow"},
    }


def test_tables_show_every_model_and_group():
    lines = evaluate.tables(metrics_file_content()).splitlines()
    assert lines[0] == (
        "Offline test split: 3 rows from seeds 1030-1031, data commit 0123456789ab. "
        "Model file `m.json` holds the free_flow model."
    )
    assert "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |" in lines
    assert FREE_FLOW_ROW in lines
    assert GBDT_ROW in lines
    assert "Delayed rows: 2 of 3." in lines
    bucket_header = lines.index("| Model | 0-5 | 6-10 | 11-20 | 21+ |")
    assert lines[bucket_header + 2] == "| rows | 1 | 1 | 0 | 1 |"
    assert lines[bucket_header + 3] == "| free_flow (exported) | 1.00 | 2.00 | - | 0.00 |"
    assert lines[bucket_header + 4] == "| gbdt (not deployable) | 0.00 | 0.00 | - | 0.00 |"
    assert "| Model | 5 | 10 |" in lines
    assert "| Model | 0.1 | 0.2 |" in lines
    assert "| Model | hungarian | nearest |" in lines


def test_cli_prints_tables_with_their_command(tmp_path, capsys):
    path = tmp_path / "m.metrics.json"
    path.write_text(json.dumps(metrics_file_content()))
    assert evaluate.main([str(path)]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0] == f"From `python -m fleet_learning.evaluate {path} --split test`:"
    assert FREE_FLOW_ROW in out


def test_cli_rejects_other_files(tmp_path, capsys):
    path = tmp_path / "m.metrics.json"
    path.write_text(json.dumps(metrics_file_content()))
    assert evaluate.main([str(path), "--split", "validation"]) == 1
    assert "not a train.py metrics file" in capsys.readouterr().err
    assert evaluate.main([str(tmp_path / "missing.json")]) == 1
    path.write_text("{")
    assert evaluate.main([str(path)]) == 1

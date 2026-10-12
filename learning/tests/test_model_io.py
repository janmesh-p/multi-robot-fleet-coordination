import copy
import json
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from fleet_learning import model_io, schema

FIXTURE = Path(__file__).parent / "fixtures" / "run_tiny"

# The example model from section 4 of docs/learning_track.md.
EXAMPLE = {
    "format": "fleet_cost_model",
    "version": 1,
    "target": "steps_to_pickup",
    "features": ["dist_to_pickup", "robots_in_box", "pickup_assigned"],
    "mean": [14.2, 1.3, 0.9],
    "scale": [7.5, 1.6, 1.1],
    "layers": [
        {"weights": [[0.8, 0.1, 0.3], [0.2, -0.4, 0.6]], "bias": [0.1, -0.2], "activation": "relu"},
        {"weights": [[1.5, 0.7]], "bias": [0.4], "activation": "identity"},
    ],
    "base_feature": "dist_to_pickup",
    "info": {"notes": "free-form; the C++ side ignores this key"},
}

# Small integer weights, so the predictions below can be worked out by hand.
SMALL = {
    "format": "fleet_cost_model",
    "version": 1,
    "target": "steps_to_pickup",
    "features": ["dist_to_pickup", "robots_in_box"],
    "mean": [10, 1],
    "scale": [2, 1],
    "layers": [
        {"weights": [[1, 2], [-1, 1]], "bias": [0, 1], "activation": "relu"},
        {"weights": [[3, -2]], "bias": [1], "activation": "identity"},
    ],
    "base_feature": None,
}
SMALL_ROWS = pd.DataFrame({"dist_to_pickup": [14, 6, 12], "robots_in_box": [3, 0, 1]})


def test_worked_example_from_the_contract():
    row = {"dist_to_pickup": [20], "robots_in_box": [3], "pickup_assigned": [1]}
    assert model_io.predict(EXAMPLE, row)[0] == pytest.approx(21.678284091, abs=1e-9)


def test_two_layer_model_matches_hand_computation():
    # Row 0: x = (2, 2), hidden = relu(6, 1), y = 18 - 2 + 1 = 17.
    # Row 1: x = (-2, -1), hidden = relu(-4, 2) = (0, 2), y = -4 + 1 = -3, clamped to 0.
    # Row 2: x = (1, 0), hidden = relu(1, 0), y = 3 + 1 = 4.
    np.testing.assert_array_equal(model_io.predict(SMALL, SMALL_ROWS), [17, 0, 4])


def test_base_feature_is_added_raw_before_clamping():
    model = dict(SMALL, base_feature="dist_to_pickup")
    np.testing.assert_array_equal(model_io.predict(model, SMALL_ROWS), [31, 3, 16])


def test_clamp_at_upper_bound():
    model = model_io.free_flow_model()
    rows = {"dist_to_pickup": [250000]}
    assert model_io.predict(model, rows)[0] == model_io.COST_MAX


def test_free_flow_equals_dist_to_pickup_on_fixture():
    frame = schema.read_assignments(FIXTURE)
    costs = model_io.predict(model_io.free_flow_model(), frame)
    np.testing.assert_array_equal(costs, frame["dist_to_pickup"].to_numpy(dtype=float))


def test_missing_input_column_is_named():
    with pytest.raises(model_io.ModelError, match="pickup_assigned"):
        model_io.predict(EXAMPLE, {"dist_to_pickup": [1], "robots_in_box": [1]})


@pytest.mark.parametrize("model", [EXAMPLE, SMALL, model_io.free_flow_model()])
def test_write_read_round_trip(tmp_path, model):
    path = tmp_path / "model.json"
    model_io.write_model(model, path)
    loaded = model_io.load_model(path)
    assert loaded == model
    np.testing.assert_array_equal(
        model_io.predict(loaded, SMALL_ROWS.assign(pickup_assigned=[1, 0, 2])),
        model_io.predict(model, SMALL_ROWS.assign(pickup_assigned=[1, 0, 2])),
    )


def mutate(edit):
    model = copy.deepcopy(EXAMPLE)
    edit(model)
    return model


def set_key(key, value):
    return lambda m: m.__setitem__(key, value)


def set_layer(k, key, value):
    return lambda m: m["layers"][k].__setitem__(key, value)


def deep_model(m):
    hidden = {"weights": [[1.0]], "bias": [0.0], "activation": "relu"}
    m["features"], m["mean"], m["scale"] = ["dist_to_pickup"], [0], [1]
    m["layers"] = [dict(hidden, weights=[[1.0]]) for _ in range(4)]
    m["layers"].append({"weights": [[1.0]], "bias": [0.0], "activation": "identity"})


def wide_model(m):
    m["layers"][0]["weights"] = [[0.0, 0.0, 0.0]] * 65
    m["layers"][0]["bias"] = [0.0] * 65
    m["layers"][1]["weights"] = [[0.0] * 65]


BROKEN = {
    "missing key": (lambda m: m.pop("scale"), "scale: missing"),
    "extra key": (set_key("notes", "x"), "notes: unknown key"),
    "wrong format": (set_key("format", "onnx"), "format"),
    "wrong version": (set_key("version", 2), "version"),
    "version as float": (set_key("version", 1.0), "version"),
    "version as bool": (set_key("version", True), "version"),
    "wrong target": (set_key("target", "delivered_t"), "target"),
    "no features": (set_key("features", []), "features"),
    "too many features": (set_key("features", list(schema.FEATURE_COLUMNS) * 2), "features"),
    "duplicate feature": (
        set_key("features", ["dist_to_pickup", "dist_to_pickup", "robots_in_box"]),
        "distinct",
    ),
    "identification column": (
        set_key("features", ["t", "robots_in_box", "pickup_assigned"]),
        "features\\[0\\]",
    ),
    "outcome column": (
        set_key("features", ["dist_to_pickup", "arrived_t", "pickup_assigned"]),
        "features\\[1\\]",
    ),
    "unknown feature": (
        set_key("features", ["dist_to_pickup", "robots_in_box", "fleet_size"]),
        "features\\[2\\]",
    ),
    "short mean": (set_key("mean", [1.0, 2.0]), "mean: has 2 entries"),
    "long scale": (set_key("scale", [1.0, 1.0, 1.0, 1.0]), "scale: has 4 entries"),
    "zero scale": (set_key("scale", [1.0, 0.0, 1.0]), "scale\\[1\\]: must be greater than 0"),
    "negative scale": (set_key("scale", [-1.0, 1.0, 1.0]), "scale\\[0\\]"),
    "string in mean": (set_key("mean", [1.0, "2", 3.0]), "mean\\[1\\]: must be a number"),
    "infinite mean": (set_key("mean", [1.0, float("inf"), 3.0]), "mean\\[1\\]: must be finite"),
    "nan weight": (set_layer(1, "bias", [float("nan")]), "layers\\[1\\].bias\\[0\\]"),
    "no layers": (set_key("layers", []), "layers"),
    "too many layers": (deep_model, "layers"),
    "extra layer key": (set_layer(0, "dropout", 0.1), "layers\\[0\\]: must have exactly"),
    "missing layer key": (lambda m: m["layers"][0].pop("bias"), "layers\\[0\\]: must have"),
    "ragged weights": (
        set_layer(0, "weights", [[0.8, 0.1, 0.3], [0.2, -0.4]]),
        "layers\\[0\\].weights\\[1\\]: has 2 entries",
    ),
    "wrong input count": (
        set_layer(0, "weights", [[0.8, 0.1], [0.2, -0.4]]),
        "layers\\[0\\].weights\\[0\\]: has 2 entries, expected 3",
    ),
    "later layer input count": (
        set_layer(1, "weights", [[1.5, 0.7, 0.1]]),
        "layers\\[1\\].weights\\[0\\]: has 3 entries, expected 2",
    ),
    "bias length": (set_layer(0, "bias", [0.1]), "layers\\[0\\].bias: has 1 entries"),
    "empty weights": (set_layer(0, "weights", []), "layers\\[0\\].weights"),
    "hidden identity": (set_layer(0, "activation", "identity"), "layers\\[0\\].activation"),
    "last relu": (set_layer(1, "activation", "relu"), "layers\\[1\\].activation"),
    "unknown activation": (set_layer(0, "activation", "tanh"), "layers\\[0\\].activation"),
    "two outputs": (
        lambda m: m["layers"][1].update(weights=[[1.5, 0.7], [1.0, 1.0]], bias=[0.4, 0.0]),
        "exactly one output",
    ),
    "too wide": (wide_model, "at most 64"),
    "bad base feature": (set_key("base_feature", "steps_to_pickup"), "base_feature"),
}


@pytest.mark.parametrize("name", list(BROKEN))
def test_rule_rejects_broken_model(name):
    edit, message = BROKEN[name]
    with pytest.raises(model_io.ModelError, match=message):
        model_io.validate_model(mutate(edit))


def test_rejects_non_object():
    with pytest.raises(model_io.ModelError, match="JSON object"):
        model_io.validate_model([])


def test_info_is_optional_and_base_feature_may_be_null():
    model = mutate(lambda m: m.pop("info"))
    model["base_feature"] = None
    model_io.validate_model(model)


def test_base_feature_need_not_be_in_features():
    model_io.validate_model(mutate(set_key("base_feature", "manhattan_to_pickup")))


def test_load_rejects_bad_file_naming_the_field(tmp_path):
    path = tmp_path / "model.json"
    path.write_text(json.dumps(mutate(set_key("scale", [1.0, 0.0, 1.0]))))
    with pytest.raises(model_io.ModelError, match="scale\\[1\\]"):
        model_io.load_model(path)


@pytest.mark.parametrize("token", ["NaN", "Infinity", "-Infinity"])
def test_load_rejects_non_finite_tokens(tmp_path, token):
    path = tmp_path / "model.json"
    path.write_text(json.dumps(EXAMPLE).replace("0.4]", f"{token}]"))
    with pytest.raises(model_io.ModelError, match="finite"):
        model_io.load_model(path)


def test_load_rejects_overflowing_number(tmp_path):
    path = tmp_path / "model.json"
    path.write_text(json.dumps(EXAMPLE).replace("14.2", "1e400"))
    with pytest.raises(model_io.ModelError, match="mean\\[0\\]: must be finite"):
        model_io.load_model(path)


def test_write_refuses_bad_model_and_writes_nothing(tmp_path):
    path = tmp_path / "model.json"
    with pytest.raises(model_io.ModelError):
        model_io.write_model(mutate(set_key("mean", [float("nan"), 0, 0])), path)
    assert not path.exists()


def test_cli_check_and_predict(tmp_path, capsys):
    path = tmp_path / "model.json"
    model_io.write_model(model_io.free_flow_model(), path)
    assert model_io.main(["check", str(path)]) == 0
    capsys.readouterr()
    assert model_io.main(["predict", str(path), str(FIXTURE / "assignments.csv")]) == 0
    lines = capsys.readouterr().out.splitlines()
    assert lines[0] == "row,prediction"
    assert lines[1] == "0,6"
    assert len(lines) == 17


def test_cli_bad_model_exits_1(tmp_path, capsys):
    path = tmp_path / "model.json"
    path.write_text(json.dumps(mutate(set_key("format", "onnx"))))
    assert model_io.main(["check", str(path)]) == 1
    assert "format" in capsys.readouterr().err

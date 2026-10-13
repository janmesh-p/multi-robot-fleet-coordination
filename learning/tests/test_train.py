import hashlib
import json

import numpy as np
import pytest
import synthetic

from fleet_learning import dataset, evaluate, model_io, schema, train

SEEDS = {"train": range(1000, 1010), "validation": range(1020, 1024), "test": range(1030, 1034)}
HIDDEN = ((16,), (8, 8))
NAMES = [
    "free_flow",
    "linear_direct",
    "linear_excess",
    "mlp_direct_16",
    "mlp_direct_8x8",
    "mlp_excess_16",
    "mlp_excess_8x8",
    "gbdt_direct",
    "gbdt_excess",
]


@pytest.fixture(scope="module")
def splits():
    return dataset.split_by_seed(synthetic.table([s for seeds in SEEDS.values() for s in seeds]))


@pytest.fixture(scope="module")
def result(splits):
    return train.train_models(splits, hidden_sizes=HIDDEN)


def candidate(result, name):
    return next(model for model in result.candidates if model.name == name)


def strict_json(text: str):
    def reject(token):
        raise ValueError(f"{token} is not JSON")

    return json.loads(text, parse_constant=reject)


# Fitting and choosing


def test_linear_and_mlp_reach_test_mae_below_one_step(result):
    # Section 10: steps_to_pickup = dist_to_pickup + 3 * pickup_assigned plus small noise.
    assert result.test["linear"]["mae"] < 1
    assert result.test["mlp"]["mae"] < 1
    assert result.test["free_flow"]["mae"] > 2  # it cannot see pickup_assigned


def test_every_candidate_is_fit_in_order(result):
    assert [model.name for model in result.candidates] == NAMES
    assert list(result.validation) == NAMES
    assert [result.winners[family].family for family in train.FAMILIES] == list(train.FAMILIES)


def test_winners_and_choice_have_the_lowest_validation_mae(result):
    def mae(model):
        return result.validation[model.name]["mae"]

    for family, winner in result.winners.items():
        assert mae(winner) == min(mae(m) for m in result.candidates if m.family == family)
    deployable = [result.winners[family] for family in train.DEPLOYABLE]
    assert result.chosen in deployable
    assert mae(result.chosen) == min(mae(m) for m in deployable)


def test_choose_can_name_a_family(result):
    assert train.choose(result.winners, result.validation, "linear") is result.winners["linear"]
    with pytest.raises(ValueError, match="cannot export 'gbdt'"):
        train.choose(result.winners, result.validation, "gbdt")


def test_missing_split_fails(splits):
    with pytest.raises(ValueError, match="validation split has no rows; it needs runs with seeds"):
        train.train_models({**splits, "validation": splits["validation"].iloc[:0]})


def test_free_flow_cost_is_dist_to_pickup(splits):
    test = splits["test"]
    np.testing.assert_array_equal(train.free_flow().predict(test), test["dist_to_pickup"])
    assert train.free_flow().to_model_file() == model_io.free_flow_model()


def test_costs_are_clamped(result, splits):
    rows = splits["test"].head(3).assign(dist_to_pickup=[0, 0, 10**9], pickup_assigned=[-99] * 3)
    costs = candidate(result, "linear_excess").predict(rows)
    np.testing.assert_array_equal(costs, [model_io.COST_MIN, model_io.COST_MIN, model_io.COST_MAX])


@pytest.mark.parametrize(
    ("features", "message"),
    [
        (("dist_to_pickup", "fleet_size"), "not feature columns of assignments.csv: fleet_size"),
        (("steps_to_pickup",), "not feature columns"),
        ((), "1 to 32 distinct names"),
        (("dist_to_pickup", "dist_to_pickup"), "distinct"),
    ],
)
def test_features_come_only_from_feature_columns(features, message):
    with pytest.raises(ValueError, match=message):
        train.check_features(features)


@pytest.mark.parametrize("hidden", [(), (64, 64, 64, 64), (65,), (0,)])
def test_hidden_layers_fit_format_v1(hidden):
    with pytest.raises(ValueError, match="1 to 3 layers of 1 to 64 units"):
        train.check_hidden(hidden)


# Export


@pytest.mark.parametrize("name", NAMES[:-2])
def test_export_matches_scikit_learn_on_test_rows(tmp_path, result, splits, name):
    model = candidate(result, name)
    path = tmp_path / "model.json"
    exported = train.export_model(model, path, splits["test"], {"note": "test"})
    assert exported["max_abs_difference"] <= train.PARITY_TOLERANCE
    assert exported["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    loaded = model_io.load_model(path)
    assert loaded["base_feature"] == ("dist_to_pickup" if model.target == "excess" else None)
    assert loaded["info"] == {"note": "test"}
    costs = model_io.predict(loaded, splits["test"])
    np.testing.assert_allclose(costs, model.predict(splits["test"]), rtol=0, atol=1e-9)


def test_mlp_export_transposes_weights(result):
    layers = candidate(result, "mlp_excess_8x8").to_model_file()["layers"]
    assert [np.shape(layer["weights"]) for layer in layers] == [(8, 19), (8, 8), (1, 8)]
    assert [layer["activation"] for layer in layers] == ["relu", "relu", "identity"]


def test_target_scaling_is_folded_into_the_last_layer(result, splits):
    model = candidate(result, "mlp_direct_16")
    assert model.y_scale > 1  # the target was scaled, so the fold matters
    weights = np.asarray(model.to_model_file()["layers"][-1]["weights"])
    np.testing.assert_allclose(weights, model.regressor.coefs_[-1].T * model.y_scale)


def test_gbdt_cannot_be_exported(result):
    with pytest.raises(ValueError, match="format v1 cannot hold a gbdt model"):
        result.winners["gbdt"].to_model_file()


def test_parity_failure_deletes_the_file(tmp_path, result, splits, monkeypatch):
    monkeypatch.setattr(train, "PARITY_TOLERANCE", -1.0)
    path = tmp_path / "model.json"
    with pytest.raises(train.ExportError, match="Deleted the file"):
        train.export_model(candidate(result, "linear_excess"), path, splits["test"])
    assert not path.exists()


# Command line


def collection(tmp_path, seeds=(1000, 1001, 1020, 1030)):
    return synthetic.write_collection(tmp_path / "0123456789ab", list(seeds), rows_per_run=200)


@pytest.mark.filterwarnings("ignore::sklearn.exceptions.ConvergenceWarning")  # 400 rows
def test_cli_writes_model_and_metrics(tmp_path, capsys):
    data = collection(tmp_path)
    out = tmp_path / "models"
    args = ["--data", str(data), "--name", "m", "--out", str(out), "--hidden", "8"]
    assert train.main(args) == 0
    model = model_io.load_model(out / "m.json")
    metrics = strict_json((out / "m.metrics.json").read_text())

    assert metrics["command"] == "python -m fleet_learning.train " + " ".join(args)
    data_info = metrics["data"]
    assert data_info["commit"] == "0123456789ab"
    assert data_info["seeds"] == {"train": [1000, 1001], "validation": [1020], "test": [1030]}
    assert data_info["rows"] == {"train": 400, "validation": 200, "test": 200}
    assert sum(group["kept"] for group in data_info["filters"]) == 800
    assert metrics["features"] == list(schema.FEATURE_COLUMNS)
    assert metrics["random_seed"] == 0
    assert [c["name"] for c in metrics["candidates"]] == [
        "free_flow",
        "linear_direct",
        "linear_excess",
        "mlp_direct_8",
        "mlp_excess_8",
        "gbdt_direct",
        "gbdt_excess",
    ]
    assert list(metrics["models"]) == list(train.FAMILIES)
    exported = metrics["exported"]
    assert exported["sha256"] == hashlib.sha256((out / "m.json").read_bytes()).hexdigest()
    assert model["info"]["candidate"] == exported["candidate"]
    assert model["info"]["data_commit"] == "0123456789ab"

    # The test metrics of the exported model follow from the file and the data alone.
    rows = dataset.read_rows(dataset.read_runs([data / "runs"]))
    test = dataset.split_by_seed(dataset.apply_filters(rows)[0])["test"]
    recorded = metrics["models"][exported["family"]]["test"]["mae"]
    assert evaluate.mae(test["steps_to_pickup"], model_io.predict(model, test)) == pytest.approx(
        recorded, abs=1e-9
    )

    out_text = capsys.readouterr().out
    assert "seeds      train: 1000-1001 (runs 2, rows kept 400)" in out_text
    assert f"tables     python -m fleet_learning.evaluate {out / 'm.metrics.json'}" in out_text


@pytest.mark.parametrize(
    ("make", "message"),
    [
        (lambda tmp: synthetic.write_collection(tmp / "data", [1000, 1020, 1030], 50), "commit"),
        (lambda tmp: collection(tmp, (1000, 1030)), "validation split has no rows"),
        (lambda tmp: collection(tmp, (1000, 1020, 1030, 1)), "seeds 1 (final)"),
    ],
)
def test_cli_rejects_bad_collections(tmp_path, capsys, make, message):
    data = make(tmp_path)
    assert train.main(["--data", str(data), "--name", "m", "--out", str(tmp_path / "m")]) == 1
    err = capsys.readouterr().err
    assert message in err, err
    assert not (tmp_path / "m").exists()


def test_cli_before_stage_3_exits_1(tmp_path, capsys):
    data = collection(tmp_path)
    for path in (data / "runs").rglob("assignments.csv"):
        path.unlink()
    assert train.main(["--data", str(data), "--name", "m", "--out", str(tmp_path / "m")]) == 1
    assert "from Stage 3 on" in capsys.readouterr().err


def test_cli_rejects_hidden_layers_outside_format_v1(tmp_path):
    with pytest.raises(SystemExit) as exit_info:
        train.main(["--data", str(tmp_path), "--name", "m", "--hidden", "64,64,64,64"])
    assert exit_info.value.code == 2

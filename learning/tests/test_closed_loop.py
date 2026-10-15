import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

from fleet_learning import closed_loop, model_io

REPO = Path(__file__).resolve().parents[2]
FIXTURE = Path(__file__).resolve().parent / "fixtures" / "run_tiny"

# Stands in for build/run_sweep. Each run directory gets the fixture's assignments.csv and
# config.yaml and a summary.csv named for the run. sweep.csv gives learned 6 more orders per hour
# than hungarian, and cost_aware 2 more per unit of weight, so every answer is known.
FAKE_RUN_SWEEP = """#!PYTHON
import shutil
import sys
from pathlib import Path

import pandas as pd
import yaml

FIXTURE = Path("@FIXTURE@")
args = dict(zip(sys.argv[1::2], sys.argv[2::2]))
config = yaml.safe_load(Path(args["--config"]).read_text())
allocation = config["allocation"]
name = config["run"]["name"]


def numbers(text):
    for item in text.split(","):
        first, _, last = item.partition("-")
        yield from range(int(first), int(last or first) + 1)


bonus = {"hungarian": 0.0, "learned": 6.0, "cost_aware": 2.0 * allocation["congestion_weight"]}
out = Path(args["--out"])
out.mkdir(parents=True)
summary = pd.read_csv(FIXTURE / "summary.csv", dtype=str, keep_default_na=False)
rows = []
for fleet in numbers(args["--fleet"]):
    for seed in numbers(args["--seeds"]):
        run = f"{name}_n{fleet}_s{seed}"
        run_dir = out / run
        run_dir.mkdir()
        for file in ("assignments.csv", "config.yaml"):
            shutil.copy(FIXTURE / file, run_dir / file)
        (run_dir / "positions.csv").write_text("x\\n")
        summary.assign(run=run, fleet_size=fleet, seed=seed).to_csv(
            run_dir / "summary.csv", index=False
        )
        rows.append({
            "run": run, "allocator": allocation["method"], "planner": "pibt",
            "fleet_size": fleet, "seed": seed,
            "orders_per_hour": 100.0 + fleet + seed % 3 + bonus[allocation["method"]],
            "mean_completion_steps": 50.0, "p95_completion_steps": 90.0, "aborted": "false",
            "deadlocks": 1 if seed == 3000 and fleet == 5 else 0, "frozen_at": -1,
            "log_hash": "0", "wall_seconds": 0.5, "extra_column": 7,
        })
pd.DataFrame(rows).to_csv(out / "sweep.csv", index=False)
(out / "aggregate.csv").write_text("x\\n")
"""


@pytest.fixture
def fake_run_sweep(tmp_path) -> Path:
    path = tmp_path / "bin" / "run_sweep"
    path.parent.mkdir()
    text = FAKE_RUN_SWEEP.replace("PYTHON", sys.executable).replace("@FIXTURE@", str(FIXTURE))
    path.write_text(text)
    path.chmod(0o755)
    return path


@pytest.fixture
def models(tmp_path) -> list[Path]:
    """Two model files: free flow, and free flow plus 2 steps per pickup_assigned."""
    folder = tmp_path / "models"
    folder.mkdir()
    flat = folder / "flat.json"
    model_io.write_model(model_io.free_flow_model(), flat)
    queue = model_io.free_flow_model()
    queue["features"] = ["pickup_assigned"]
    queue["layers"][0]["weights"] = [[2.0]]
    model_io.write_model(queue, folder / "queue.json")
    return [flat, folder / "queue.json"]


def run(tmp_path, phase, cells, tool, seeds=(3000, 3001), fleet=(5, 25)):
    return closed_loop.run_phase(
        phase,
        cells,
        REPO,
        tmp_path / "data",
        fleet=fleet,
        seeds=seeds,
        allow_dirty=True,
        run_sweep=tool,
    )


def phase_dir(tmp_path, manifest) -> Path:
    return tmp_path / "data" / manifest["commit"] / "closed_loop" / manifest["phase"]


# Names and seeds


def test_names_follow_section_7(models):
    assert closed_loop.weight_code(0.5) == "050"
    assert closed_loop.weight_code(4) == "400"
    assert closed_loop.weight_code(0.25) == "025"
    for weight in (0.0, 0.125, 10.0):
        with pytest.raises(ValueError, match="multiple of 0.01"):
            closed_loop.weight_code(weight)
    names = [c["name"] for c in closed_loop.tune_cells((0.5,), models[:1], (0.1, 0.2))]
    assert names == [
        "tune_cost_aware_w050_r010",
        "tune_cost_aware_w050_r020",
        "tune_learned_flat_r010",
        "tune_learned_flat_r020",
    ]
    cells = closed_loop.final_cells(2.0, models[0], (0.1,))
    assert [c["name"] for c in cells] == [
        "final_hungarian_r010",
        "final_cost_aware_r010",
        "final_learned_r010",
    ]
    assert cells[1]["changes"]["allocation.congestion_weight"] == 2.0
    assert "allocation.congestion_weight" not in cells[0]["changes"]
    assert cells[2]["changes"]["allocation.model_file"] == str(models[0])
    assert "allocation.model_file" not in cells[1]["changes"]


def test_seeds_must_fit_the_phase():
    assert closed_loop.check_seeds("tune", range(2000, 2010)) == tuple(range(2000, 2010))
    assert closed_loop.check_seeds("final", (1, 10, 3000)) == (1, 10, 3000)
    with pytest.raises(ValueError, match=r"1 \(final\) are not for tune"):
        closed_loop.check_seeds("tune", (1, 2000))
    with pytest.raises(ValueError, match="1030"):
        closed_loop.check_seeds("final", (1030,))


def test_at_most_five_models_with_distinct_names(models, tmp_path):
    with pytest.raises(ValueError, match="1 to 5"):
        closed_loop._check_models(models * 3, 5)
    other = tmp_path / "other"
    other.mkdir()
    (other / "flat.json").write_text(models[0].read_text())
    with pytest.raises(ValueError, match="repeat"):
        closed_loop._check_models([models[0], other / "flat.json"], 5)
    bad = tmp_path / "bad.json"
    bad.write_text("{}")
    with pytest.raises(model_io.ModelError):
        closed_loop._check_models([bad], 5)


# Statistics


def test_bootstrap_matches_the_section_9_recipe():
    differences = [3.0, -1.0, 4.0, 1.0, 5.0, 9.0, 2.0, 6.0, 5.0, 3.0]
    median, low, high = closed_loop.bootstrap_median(differences)
    rng = np.random.default_rng(0)
    resampled = np.median(rng.choice(differences, size=(10_000, 10), replace=True), axis=1)
    assert median == 3.5
    assert (low, high) == tuple(np.percentile(resampled, [2.5, 97.5]))
    assert low < median < high
    assert closed_loop.bootstrap_median([2.0] * 10) == (2.0, 2.0, 2.0)
    with pytest.raises(ValueError):
        closed_loop.bootstrap_median([])


def results_frame(learned, baseline):
    rows = []
    for allocator, values in (("learned", learned), ("hungarian", baseline)):
        for seed, value in enumerate(values, start=1):
            rows.append(
                {
                    "allocator": allocator,
                    "fleet_size": 25,
                    "arrival_rate": 0.2,
                    "seed": seed,
                    "orders_per_hour": value,
                }
            )
    return pd.DataFrame(rows)


def test_paired_differences_count_wins_ties_and_losses():
    paired = closed_loop.paired_differences(results_frame([5, 5, 5, 1], [4, 5, 6, 0]), "hungarian")
    row = paired.iloc[0]
    assert (row["wins"], row["ties"], row["losses"], row["seeds"]) == (2, 1, 1, 4)
    assert row["median"] == 0.5
    result = closed_loop.criterion(paired.assign(low=0.1))
    assert result == {
        "evaluated": True,
        "median": 0.5,
        "low": 0.1,
        "high": row["high"],
        "met": True,
    }
    assert not closed_loop.criterion(paired.assign(low=0.0))["met"]
    assert closed_loop.criterion(paired.assign(fleet_size=10)) == {"evaluated": False}


def test_paired_differences_refuse_unpaired_seeds():
    results = results_frame([5, 5], [4, 5, 6])
    with pytest.raises(closed_loop.ClosedLoopError, match="different seeds"):
        closed_loop.paired_differences(results, "hungarian")


# Phases, end to end with the fake run_sweep


def test_tune_writes_configs_and_chooses_the_best(tmp_path, fake_run_sweep, models):
    cells = closed_loop.tune_cells((0.5, 4.0), models, (0.1,))
    manifest = run(tmp_path, "tune", cells, fake_run_sweep)
    folder = phase_dir(tmp_path, manifest)
    config = yaml.safe_load((folder / "configs" / "tune_learned_queue_r010.yaml").read_text())
    assert config["allocation"]["method"] == "learned"
    assert config["allocation"]["model_file"] == str(models[1])
    assert config["planning"]["method"] == "pibt"
    assert Path(config["map"]["file"]).is_absolute()
    assert set(manifest["models"]) == {"flat", "queue"}
    assert manifest["models"]["queue"]["sha256"] == closed_loop.sha256(models[1])
    assert not (
        folder
        / "runs"
        / "tune_learned_flat_r010"
        / "tune_learned_flat_r010_n5_s3000"
        / "positions.csv"
    ).exists()

    results = closed_loop.read_results(folder, manifest)
    assert len(results) == len(cells) * 4
    # cost_aware w=4 gets +8, learned +6 for both models: the first model wins the tie.
    choice = closed_loop.choose_tuned(results)
    assert choice["weight"] == 4.0
    assert choice["model"] == "flat"
    table = closed_loop.tuning_table(results)
    assert list(table["candidate"]) == [
        "cost_aware w=0.5",
        "cost_aware w=4",
        "learned flat",
        "learned queue",
    ]
    assert table.loc[1, "mean"] - table.loc[0, "mean"] == pytest.approx(7.0)
    text = closed_loop.report(folder, REPO)
    assert "Chosen cost_aware weight: 4" in text
    assert "Chosen learned model: flat" in text


def test_final_reports_tables_pairs_and_selection_bias(tmp_path, fake_run_sweep, models):
    cells = closed_loop.final_cells(1.0, models[1], (0.2,))
    manifest = run(tmp_path, "final", cells, fake_run_sweep)
    folder = phase_dir(tmp_path, manifest)
    results = closed_loop.read_results(folder, manifest)

    table = closed_loop.final_table(results)
    assert list(table["allocator"][:3]) == ["hungarian", "cost_aware", "learned"]
    first = table.iloc[0]
    assert (first["fleet_size"], first["runs"], first["deadlocked_runs"]) == (5, 2, 1)
    assert first["wall_seconds"] == 1.0

    paired = closed_loop.paired_differences(results, "cost_aware")
    assert list(paired["median"]) == [4.0, 4.0]
    assert list(paired["wins"]) == [2, 2]
    assert closed_loop.criterion(paired)["met"]

    bias = closed_loop.selection_bias(folder, manifest, REPO)
    assert bias["model"] == "queue"
    assert bias["offline_test"] is None  # no metrics file next to the model
    fixture = pd.read_csv(FIXTURE / "assignments.csv")
    kept = fixture[
        (fixture["steps_to_pickup"] != -1) & (fixture["t"] <= 400 - bias["margin_steps"])
    ]
    predicted = kept["dist_to_pickup"] + 2 * kept["pickup_assigned"]
    assert bias["closed_loop"]["rows"] == 4 * len(kept)
    assert bias["closed_loop"]["mae"] == pytest.approx(
        np.mean(np.abs(predicted - kept["steps_to_pickup"]))
    )
    assert "Criterion (25 robots, rate 0.2" in closed_loop.report(folder, REPO)


def test_selection_bias_uses_the_training_margin(tmp_path, fake_run_sweep, models):
    metrics = {
        "exported": {"family": "free_flow"},
        "models": {"free_flow": {"test": {"rows": 10, "mae": 1.5, "bias": -0.5}}},
        "data": {"margin_steps": 0},
    }
    (models[0].parent / "flat.metrics.json").write_text(json.dumps(metrics))
    manifest = run(
        tmp_path, "final", closed_loop.final_cells(1.0, models[0], (0.2,)), fake_run_sweep
    )
    bias = closed_loop.selection_bias(phase_dir(tmp_path, manifest), manifest, REPO)
    assert bias["margin_steps"] == 0
    assert bias["offline_test"] == {"rows": 10, "mae": 1.5, "bias": -0.5}


def test_selection_bias_refuses_a_changed_model(tmp_path, fake_run_sweep, models):
    manifest = run(
        tmp_path, "final", closed_loop.final_cells(1.0, models[0], (0.2,)), fake_run_sweep
    )
    models[0].write_text(models[0].read_text() + "\n")
    with pytest.raises(closed_loop.ClosedLoopError, match="changed since the runs"):
        closed_loop.selection_bias(phase_dir(tmp_path, manifest), manifest, REPO)


def test_a_phase_runs_once_per_commit(tmp_path, fake_run_sweep, models):
    cells = closed_loop.final_cells(1.0, models[0], (0.2,))
    run(tmp_path, "final", cells, fake_run_sweep, fleet=(5,))
    with pytest.raises(closed_loop.ClosedLoopError, match="already exists"):
        run(tmp_path, "final", cells, fake_run_sweep, fleet=(5,))


def test_report_needs_a_finished_phase(tmp_path):
    with pytest.raises(closed_loop.ClosedLoopError, match="did not finish"):
        closed_loop.report(tmp_path, REPO)

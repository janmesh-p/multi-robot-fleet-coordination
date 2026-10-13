import itertools
import math
import shutil
from pathlib import Path

import pandas as pd
import pytest
import yaml

from fleet_learning import dataset, schema

FIXTURES = Path(__file__).parent / "fixtures"
RUN_TINY = FIXTURES / "run_tiny"


def make_run(
    root: Path,
    name: str,
    *,
    seed: int = 3000,
    fleet_size: int = 5,
    allocator: str = "hungarian",
    aborted: bool = False,
    edit_config=None,
) -> Path:
    """A copy of the fixture run at root/name, with these summary values and config edits."""
    run_dir = root / name
    shutil.copytree(RUN_TINY, run_dir)
    path = run_dir / "summary.csv"
    summary = pd.read_csv(path, dtype=str, keep_default_na=False)
    values = {"run": name, "allocator": allocator, "fleet_size": fleet_size, "seed": seed}
    values["aborted"] = "true" if aborted else "false"
    for column, value in values.items():
        summary.loc[0, column] = str(value)
    summary.to_csv(path, index=False)
    if edit_config is not None:
        path = run_dir / "config.yaml"
        config = yaml.safe_load(path.read_text())
        edit_config(config)
        path.write_text(yaml.safe_dump(config))
    return run_dir


def load(*roots: Path) -> pd.DataFrame:
    return dataset.read_rows(dataset.read_runs(roots))


def orders_by_reason(rows: pd.DataFrame, reasons: pd.Series) -> dict[str, list[int]]:
    outcomes = [*dataset.FILTERS, "kept"]
    return {reason: sorted(rows.loc[reasons == reason, "order"]) for reason in outcomes}


# Loading


def test_find_run_dirs_at_any_level_and_once():
    assert dataset.find_run_dirs([FIXTURES, RUN_TINY]) == [RUN_TINY.resolve()]


def test_find_run_dirs_fails_without_runs(tmp_path):
    with pytest.raises(FileNotFoundError, match="no run directories"):
        dataset.find_run_dirs([tmp_path])
    with pytest.raises(FileNotFoundError, match="not a directory"):
        dataset.find_run_dirs([tmp_path / "missing"])


def test_read_runs_takes_metadata_from_summary_and_config():
    assert dataset.read_runs([RUN_TINY]).to_dict("records") == [
        {
            "run_dir": str(RUN_TINY.resolve()),
            "run": "run_tiny",
            "allocator": "hungarian",
            "planner": "independent_astar",
            "fleet_size": 5,
            "seed": 3113,
            "arrival_rate": 0.1,
            "steps_run": 400,
            "aborted": False,
            "lossless": True,
        }
    ]


def test_read_rows_adds_metadata_to_checked_rows():
    rows = load(RUN_TINY)
    assert list(rows.columns) == [*schema.COLUMNS, *dataset.METADATA_COLUMNS]
    dtypes = {name: str(rows[name].dtype) for name in dataset.METADATA_COLUMNS}
    assert dtypes == dataset.METADATA_DTYPES
    pd.testing.assert_frame_equal(rows[list(schema.COLUMNS)], schema.read_assignments(RUN_TINY))
    assert (rows["run"] == "run_tiny").all()
    assert (rows["seed"] == 3113).all()


def test_metadata_never_shadows_an_assignments_column():
    assert not set(dataset.METADATA_COLUMNS) & set(schema.COLUMNS)


def test_rows_follow_run_directory_order(tmp_path):
    make_run(tmp_path, "b", seed=3001)
    make_run(tmp_path, "a", seed=3000)
    rows = load(tmp_path)
    assert len(rows) == 30
    assert list(rows["run"].unique()) == ["a", "b"]


def test_missing_assignments_points_to_stage_3(tmp_path):
    (make_run(tmp_path, "a") / "assignments.csv").unlink()
    make_run(tmp_path, "b", seed=3001)
    runs = dataset.read_runs([tmp_path])
    with pytest.raises(FileNotFoundError, match="1 of 2 run directories have no assignments.csv"):
        dataset.read_rows(runs)
    with pytest.raises(FileNotFoundError, match="from Stage 3 on"):
        dataset.read_rows(runs)


def test_runs_without_rows_give_an_empty_table(tmp_path):
    path = make_run(tmp_path, "a") / "assignments.csv"
    path.write_text(path.read_text().splitlines()[0] + "\n")
    rows = load(tmp_path)
    assert rows.empty
    assert list(rows.columns) == [*schema.COLUMNS, *dataset.METADATA_COLUMNS]
    kept, counts = dataset.apply_filters(rows)
    assert kept.empty and counts.empty


def test_summary_and_config_are_read_by_name(tmp_path):
    run_dir = make_run(tmp_path, "a", edit_config=lambda c: c.update(stage_9={"new_key": 1}))
    path = run_dir / "summary.csv"
    summary = pd.read_csv(path, dtype=str, keep_default_na=False).assign(new_metric="1.5")
    summary[list(reversed(summary.columns))].to_csv(path, index=False)
    runs = dataset.read_runs([tmp_path])
    assert runs.loc[0, "run"] == "a"
    assert runs.loc[0, "steps_run"] == 400


@pytest.mark.parametrize(
    ("edit", "message"),
    [
        (lambda s: s.drop(columns=["steps_run"]), "missing columns steps_run"),
        (lambda s: s.assign(aborted="yes"), "aborted must be true or false"),
        (lambda s: s.assign(fleet_size="five"), "dtype"),
        (lambda s: pd.concat([s, s]), "expected one row, found 2"),
        (lambda s: pd.DataFrame(), "no header"),
    ],
)
def test_bad_summary_fails(tmp_path, edit, message):
    path = make_run(tmp_path, "a") / "summary.csv"
    edit(pd.read_csv(path, dtype=str, keep_default_na=False)).to_csv(path, index=False)
    with pytest.raises(dataset.DatasetError, match=message):
        dataset.read_runs([tmp_path])


@pytest.mark.parametrize(
    ("edit", "message"),
    [
        (lambda c: c["orders"].pop("arrival_rate"), "missing key orders.arrival_rate"),
        (lambda c: c.pop("faults"), "missing key faults.packet_loss"),
        (lambda c: c["orders"].update(arrival_rate="fast"), "arrival_rate must be a number"),
        (lambda c: c["faults"].update(events={"at": 5}), "faults.events must be a list"),
    ],
)
def test_bad_config_fails(tmp_path, edit, message):
    make_run(tmp_path, "a", edit_config=edit)
    with pytest.raises(dataset.DatasetError, match=message):
        dataset.read_runs([tmp_path])


OUTAGE = {"at": 50, "type": "comm_outage", "robot": 1, "duration": 20}
DEATH = {"at": 50, "type": "robot_death", "robot": 1}


@pytest.mark.parametrize(
    ("edit", "lossless"),
    [
        (None, True),
        (lambda c: c["faults"].update(packet_loss=0.05), False),
        (lambda c: c["faults"].update(events=[OUTAGE]), False),
        (lambda c: c["faults"].update(events=[DEATH]), True),
    ],
)
def test_free_flow_bound_is_checked_only_without_lost_reports(tmp_path, edit, lossless):
    path = make_run(tmp_path, "a", edit_config=edit) / "assignments.csv"
    rows = pd.read_csv(path, dtype=str, keep_default_na=False)
    rows.loc[0, "dist_to_pickup"] = str(int(rows.loc[0, "steps_to_pickup"]) + 1)
    rows.to_csv(path, index=False)
    runs = dataset.read_runs([tmp_path])
    assert runs.loc[0, "lossless"] == lossless
    if lossless:
        with pytest.raises(schema.SchemaError, match="steps_to_pickup < dist_to_pickup"):
            dataset.read_rows(runs)
    else:
        assert len(dataset.read_rows(runs)) == 15


def test_same_run_name_in_two_collections_fails(tmp_path):
    make_run(tmp_path / "commit_a", "collect_hungarian_r010_n5_s1000", seed=1000)
    make_run(tmp_path / "commit_b", "collect_hungarian_r010_n5_s1000", seed=1000)
    with pytest.raises(dataset.DatasetError, match="more than one run directory"):
        dataset.read_runs([tmp_path])


# Filters


def test_each_filter_drops_the_expected_fixture_rows():
    rows = load(RUN_TINY)
    assert orders_by_reason(rows, dataset.drop_reasons(rows)) == {
        "aborted": [],
        "margin": [10, 11, 13, 14, 17],  # t > 400 - 300
        "unreached": [5, 8],  # steps_to_pickup = -1 at t = 74 and 85
        "kept": [0, 1, 2, 3, 4, 6, 7, 9],
    }


@pytest.mark.parametrize(
    ("margin", "expected"),
    [
        (0, {"margin": [], "unreached": [5, 8, 17]}),
        (320, {"margin": [7, 8, 10, 11, 13, 14, 17], "unreached": [5]}),  # t > 80
    ],
)
def test_margin_filter_applies_before_unreached(margin, expected):
    rows = load(RUN_TINY)
    dropped = orders_by_reason(rows, dataset.drop_reasons(rows, margin))
    assert {name: dropped[name] for name in expected} == expected
    assert dropped["aborted"] == []
    assert len(dropped["kept"]) == 15 - sum(len(orders) for orders in expected.values())


def test_aborted_run_drops_every_row(tmp_path):
    make_run(tmp_path, "a", aborted=True)
    rows = load(tmp_path)
    assert orders_by_reason(rows, dataset.drop_reasons(rows)) == {
        "aborted": sorted(rows["order"]),
        "margin": [],
        "unreached": [],
        "kept": [],
    }


def test_negative_margin_fails():
    with pytest.raises(ValueError, match="margin"):
        dataset.drop_reasons(load(RUN_TINY), margin=-1)


def test_apply_filters_counts_per_allocator_and_fleet_size(tmp_path):
    make_run(tmp_path, "h5_a", seed=3000)
    make_run(tmp_path, "h5_b", seed=3001, aborted=True)
    make_run(tmp_path, "c10", seed=3000, fleet_size=10, allocator="cost_aware")
    rows = load(tmp_path)
    kept, counts = dataset.apply_filters(rows)
    common = {"margin": 5, "unreached": 2, "kept": 8, "unreached_share": 0.2}
    assert counts.to_dict("records") == [
        {"allocator": "cost_aware", "fleet_size": 10, "rows": 15, "aborted": 0, **common},
        {"allocator": "hungarian", "fleet_size": 5, "rows": 30, "aborted": 15, **common},
    ]
    assert list(kept.columns) == list(rows.columns)
    assert sorted(set(kept["run"])) == ["c10", "h5_a"]
    assert len(kept) == 16


def test_unreached_share_is_nan_without_rows_inside_the_margin(tmp_path):
    make_run(tmp_path, "a", aborted=True)
    _, counts = dataset.apply_filters(load(tmp_path))
    assert counts.loc[0, "kept"] == 0
    assert math.isnan(counts.loc[0, "unreached_share"])


# Seeds


def test_no_seed_has_two_uses():
    spans = sorted((a, math.inf if b is None else b) for a, b in dataset.SEED_USES.values())
    for (_, end), (start, _) in itertools.pairwise(spans):
        assert end < start


@pytest.mark.parametrize(
    ("seed", "use"),
    [
        (1, "final"),
        (10, "final"),
        (1000, "train"),
        (1019, "train"),
        (1020, "validation"),
        (1029, "validation"),
        (1030, "test"),
        (1039, "test"),
        (2000, "tuning"),
        (2009, "tuning"),
        (3000, "plumbing"),
        (123456, "plumbing"),
    ],
)
def test_seed_use_follows_section_7(seed, use):
    assert dataset.seed_use(seed) == use


@pytest.mark.parametrize("seed", [0, 11, 999, 1040, 1999, 2010, 2999])
def test_seed_without_a_use_fails(seed):
    with pytest.raises(ValueError, match=f"seed {seed} has no use"):
        dataset.seed_use(seed)


def test_split_by_seed_puts_each_seed_in_one_split(tmp_path):
    for seed in (1000, 1019, 1020, 1030, 1039):
        make_run(tmp_path, f"r_s{seed}", seed=seed)
    rows = load(tmp_path)
    splits = dataset.split_by_seed(rows)
    assert list(splits) == ["train", "validation", "test"]
    seeds = {name: set(frame["seed"]) for name, frame in splits.items()}
    assert seeds == {"train": {1000, 1019}, "validation": {1020}, "test": {1030, 1039}}
    assert sum(len(frame) for frame in splits.values()) == len(rows)


def test_split_by_seed_refuses_seeds_kept_for_other_uses(tmp_path):
    make_run(tmp_path, "a", seed=1000)
    make_run(tmp_path, "b", seed=1)
    with pytest.raises(ValueError, match=r"seeds 1 \(final\) are not for train, validation, test"):
        dataset.split_by_seed(load(tmp_path))


def test_split_by_seed_takes_other_uses_by_name():
    rows = load(RUN_TINY)
    assert len(dataset.split_by_seed(rows, splits=("plumbing",))["plumbing"]) == 15
    with pytest.raises(ValueError, match="unknown splits holdout"):
        dataset.split_by_seed(rows, splits=("holdout",))


def test_format_seeds():
    assert dataset.format_seeds([1003, 1000, 1001, 1002, 1025, 1000]) == "1000-1003, 1025"


# Command line


def test_cli_reports_the_fixture(capsys):
    assert dataset.main([str(RUN_TINY)]) == 0
    out = capsys.readouterr().out
    assert "runs       1 (0 aborted)" in out
    assert "rows       15 (8 kept)" in out
    assert "kept steps_to_pickup: 40 steps" in out
    assert "seeds      plumbing: 3113 (runs 1, rows kept 8)" in out
    assert "hungarian, 5 robots: unreached rows are 20.0%" in out
    assert "warning" not in out


def test_cli_names_aborted_runs_and_a_short_margin(tmp_path, capsys):
    make_run(tmp_path, "a", aborted=True)
    make_run(tmp_path, "b", seed=3001)
    assert dataset.main([str(tmp_path), "--margin", "60"]) == 0
    out = capsys.readouterr().out
    assert "a: a safety violation stopped this run" in out
    assert "40 steps, is over" in out  # order 9, over half of 60
    assert "Raise --margin" in out


def test_cli_before_stage_3_exits_1(tmp_path, capsys):
    (make_run(tmp_path, "a") / "assignments.csv").unlink()
    assert dataset.main([str(tmp_path)]) == 1
    assert "Stage 3" in capsys.readouterr().err

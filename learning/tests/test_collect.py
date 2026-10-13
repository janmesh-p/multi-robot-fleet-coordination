import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pandas as pd
import pytest
import yaml

from fleet_learning import collect, schema

REPO = Path(__file__).resolve().parents[2]
TRACES = {"positions.csv", "steps.csv"}

# Stands in for build/run_sweep. It writes what a sweep with --runs writes, fails for hungarian
# the way Stage 2 does, and hangs for cost_aware, so a test can check that collect stops it.
FAKE_RUN_SWEEP = """#!PYTHON
import json
import sys
import time
from pathlib import Path

import yaml

args = dict(zip(sys.argv[1::2], sys.argv[2::2]))
config = yaml.safe_load(Path(args["--config"]).read_text())
method = config["allocation"]["method"]
if method == "hungarian":
    print("error: allocation method 'hungarian' is not implemented yet", file=sys.stderr)
    sys.exit(1)
if method == "cost_aware":
    time.sleep(120)


def numbers(text):
    for item in text.split(","):
        first, _, last = item.partition("-")
        yield from range(int(first), int(last or first) + 1)


out = Path(args["--out"])
out.mkdir(parents=True)
(out / "argv.json").write_text(json.dumps(sys.argv[1:]))
for fleet in numbers(args["--fleet"]):
    for seed in numbers(args["--seeds"]):
        run_dir = out / f"{config['run']['name']}_n{fleet}_s{seed}"
        run_dir.mkdir()
        for name in ("positions.csv", "steps.csv", "summary.csv", "config.yaml"):
            (run_dir / name).write_text("x\\n")
for name in ("sweep.csv", "aggregate.csv"):
    (out / name).write_text("x\\n")
print("sweep      done")
"""


@pytest.fixture
def fake_run_sweep(tmp_path) -> Path:
    path = tmp_path / "bin" / "run_sweep"
    path.parent.mkdir()
    path.write_text(FAKE_RUN_SWEEP.replace("PYTHON", sys.executable))
    path.chmod(0o755)
    return path


def run(tmp_path, tool, **changes):
    """A small collection into tmp_path/data. The repo may have uncommitted work in progress."""
    options = {
        "allocators": ("nearest",),
        "rates": (0.1,),
        "planner": "independent_astar",
        "fleet": (5,),
        "seeds": (3000, 3001),
        "allow_dirty": True,
        "run_sweep": tool,
    }
    return collect.collect(REPO, tmp_path / "data", **{**options, **changes})


def files(folder: Path) -> list[str]:
    return sorted(path.name for path in folder.iterdir())


# Names, lists, and configs


def test_run_names_follow_section_7():
    assert collect.run_name("hungarian", 0.1) == "collect_hungarian_r010"
    assert collect.run_name("nearest", 0.05) == "collect_nearest_r005"
    assert collect.run_name("cost_aware", 0.2) == "collect_cost_aware_r020"
    for rate in (0.125, 0.0, 10.0, -0.1):
        with pytest.raises(ValueError, match="multiple of 0.01"):
            collect.rate_code(rate)


def test_int_lists_read_like_run_sweep():
    assert collect.parse_int_list("5,10,25") == [5, 10, 25]
    assert collect.parse_int_list("1000-1039") == list(range(1000, 1040))
    assert collect.parse_int_list("1-3,7") == [1, 2, 3, 7]
    for text in ("3-1", "", "a", "5,,6", "1-"):
        with pytest.raises(ValueError):
            collect.parse_int_list(text)
    assert collect.format_int_list(range(1000, 1040)) == "1000-1039"
    assert collect.format_int_list([5, 10, 25]) == "5,10,25"
    assert collect.format_int_list([1, 2, 3, 7]) == "1-3,7"


def test_grid_puts_allocators_outside_rates():
    cells = collect.grid(("nearest", "hungarian"), (0.1, 0.2), "pibt")
    assert [cell["run.name"] for cell in cells] == [
        "collect_nearest_r010",
        "collect_nearest_r020",
        "collect_hungarian_r010",
        "collect_hungarian_r020",
    ]


def leaves(config, prefix=""):
    """{"section.key": value} for every leaf of a config."""
    out = {}
    for key, value in config.items():
        if isinstance(value, dict):
            out.update(leaves(value, f"{prefix}{key}."))
        else:
            out[f"{prefix}{key}"] = value
    return out


def test_configs_change_only_the_grid_keys_and_the_map_path():
    base_path = REPO / collect.BASE_CONFIG
    base = yaml.safe_load(base_path.read_text())
    cell = collect.grid(("hungarian",), (0.2,), "pibt")[0]
    config = collect.make_config(base, base_path.parent, cell)
    before, after = leaves(base), leaves(config)
    assert before.keys() == after.keys()
    changed = {key for key in before if before[key] != after[key]}
    assert changed == {*cell, "map.file"}
    assert after["map.file"] == str(REPO / "maps" / "warehouse_small.txt")
    assert {key: after[key] for key in cell} == cell
    assert leaves(base) == before  # the base is not modified


def test_model_file_is_made_absolute_too(tmp_path):
    base = {"map": {"file": "/maps/a.txt"}, "allocation": {"model_file": "models/m.json"}}
    config = collect.make_config(base, tmp_path, {"allocation.method": "learned"})
    assert config["map"]["file"] == "/maps/a.txt"
    assert config["allocation"]["model_file"] == str(tmp_path / "models" / "m.json")


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"allocators": ("learned",)}, "allocators: learned is not one of"),
        ({"allocators": ()}, "allocators: give at least one"),
        ({"fleet": (5, 5)}, "fleet sizes: 5, 5 has duplicates"),
        ({"fleet": (0,)}, "fleet sizes: 0 is below 1"),
        ({"seeds": (-1,)}, "seeds: -1 is below 0"),
        ({"rates": (0.125,)}, "multiple of 0.01"),
        ({"planner": "cbs"}, "planner: cbs is not one of"),
        ({"jobs": 0}, "jobs: 0 is not between 1"),
    ],
)
def test_bad_arguments_fail_before_anything_runs(tmp_path, changes, message):
    with pytest.raises(ValueError, match=message):
        run(tmp_path, tmp_path / "missing_tool", **changes)
    assert not (tmp_path / "data").exists()


# One collection, with a fake run_sweep


def test_collection_writes_configs_runs_and_manifest(tmp_path, fake_run_sweep):
    manifest = run(tmp_path, fake_run_sweep, rates=(0.1, 0.2))
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout[:12]
    data = tmp_path / "data" / commit
    names = ["collect_nearest_r010", "collect_nearest_r020"]
    assert files(data) == ["configs", "manifest.json", "runs"]
    assert files(data / "configs") == [f"{name}.yaml" for name in names]
    assert files(data / "runs") == names

    for name, rate in zip(names, (0.1, 0.2), strict=True):
        config_path = data / "configs" / f"{name}.yaml"
        assert config_path.read_text().startswith("# Written by fleet_learning.collect at commit")
        config = yaml.safe_load(config_path.read_text())
        assert config["run"]["name"] == name
        assert config["orders"]["arrival_rate"] == rate
        assert config["planning"]["method"] == "independent_astar"
        out = data / "runs" / name
        argv = ["--config", str(config_path), "--fleet", "5", "--seeds", "3000-3001"]
        assert json.loads((out / "argv.json").read_text()) == [*argv, "--out", str(out), "--runs"]
        run_dirs = [f"{name}_n5_s3000", f"{name}_n5_s3001"]
        assert files(out) == ["aggregate.csv", "argv.json", *run_dirs, "sweep.csv"]
        for run_dir in run_dirs:
            assert files(out / run_dir) == ["config.yaml", "summary.csv"]  # traces deleted

    assert manifest == json.loads((data / "manifest.json").read_text())
    assert manifest["commit"] == commit
    assert manifest["allocators"] == ["nearest"]
    assert manifest["arrival_rates"] == [0.1, 0.2]
    assert manifest["planner"] == "independent_astar"
    assert manifest["fleet_sizes"] == [5]
    assert manifest["seeds"] == [3000, 3001]
    assert manifest["configs"] == [f"configs/{name}.yaml" for name in names]
    assert [sweep["name"] for sweep in manifest["sweeps"]] == names
    assert [sweep["trace_files_deleted"] for sweep in manifest["sweeps"]] == [4, 4]
    assert manifest["sweeps"][0]["command"][0] == str(fake_run_sweep)
    assert manifest["wall_seconds"] > 0


def test_keep_traces_keeps_them(tmp_path, fake_run_sweep):
    manifest = run(tmp_path, fake_run_sweep, keep_traces=True)
    run_dir = tmp_path / "data" / manifest["commit"] / "runs" / "collect_nearest_r010"
    assert TRACES <= set(files(run_dir / "collect_nearest_r010_n5_s3000"))
    assert manifest["sweeps"][0]["trace_files_deleted"] == 0


def test_a_failed_sweep_stops_the_collection(tmp_path, fake_run_sweep):
    with pytest.raises(collect.CollectError) as error:
        run(tmp_path, fake_run_sweep, allocators=("nearest", "hungarian"), rates=(0.1, 0.2), jobs=1)
    message = str(error.value)
    assert "sweep collect_hungarian_r010 exited with 1; stopped the others." in message
    assert "command: " in message and "collect_hungarian_r010.yaml --fleet 5" in message
    assert "allocation method 'hungarian' is not implemented yet" in message
    data = next((tmp_path / "data").iterdir())
    assert not (data / "manifest.json").exists()
    assert files(data / "runs") == ["collect_nearest_r010", "collect_nearest_r020"]


def test_a_failed_sweep_stops_the_running_ones(tmp_path, fake_run_sweep):
    if (os.cpu_count() or 1) < 2:
        pytest.skip("needs two CPU cores")
    started = time.monotonic()
    with pytest.raises(collect.CollectError, match="collect_hungarian_r010 exited with 1"):
        run(tmp_path, fake_run_sweep, allocators=("cost_aware", "hungarian"), jobs=2)
    # The cost_aware sweep would run for 120 s, and an unanswered terminate falls back to kill
    # after 10 s. Stopping it at once takes well under a second.
    assert time.monotonic() - started < 8


def test_one_collection_per_commit(tmp_path, fake_run_sweep):
    run(tmp_path, fake_run_sweep)
    with pytest.raises(collect.CollectError, match="already exists"):
        run(tmp_path, fake_run_sweep)


def git(repo: Path, *args: str) -> None:
    command = ["git", "-c", "user.name=test", "-c", "user.email=test@example.com", *args]
    subprocess.run(command, cwd=repo, check=True, capture_output=True)


def test_a_dirty_tree_needs_allow_dirty(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    git(repo, "init")
    (repo / "notes.txt").write_text("a\n")
    git(repo, "add", "notes.txt")
    git(repo, "commit", "-m", "Add notes")
    assert collect.git_changes(repo) == []
    assert len(collect.git_commit(repo)) == 12

    (repo / "notes.txt").write_text("b\n")
    assert collect.git_changes(repo) == [" M notes.txt"]
    with pytest.raises(collect.CollectError, match="lists 1 changes.*--allow-dirty"):
        collect.collect(repo, tmp_path / "data", run_sweep=tmp_path / "tool")
    with pytest.raises(collect.CollectError, match="Run from the repo root"):
        collect.collect(repo, tmp_path / "data", run_sweep=tmp_path / "tool", allow_dirty=True)


# Command line


@pytest.fixture
def recorded(monkeypatch):
    calls = []
    monkeypatch.setattr(collect, "collect", lambda repo, **kwargs: calls.append(kwargs))
    return calls


def test_cli_defaults_follow_section_7(recorded):
    assert collect.main([]) == 0
    options = recorded[0]
    assert options["allocators"] == ("nearest", "hungarian", "cost_aware")
    assert options["rates"] == (0.05, 0.1, 0.2)
    assert options["planner"] == "pibt"
    assert options["fleet"] == (5, 10, 25)
    assert options["seeds"] == tuple(range(1000, 1040))
    assert options["keep_traces"] is False and options["allow_dirty"] is False


def test_cli_plumbing_options(recorded):
    args = ["--allocators", "nearest", "--planner", "independent_astar", "--fleet", "5"]
    args += ["--seeds", "3000-3004", "--rates", "0.1,0.2", "--jobs", "1"]
    args += ["--keep-traces", "--allow-dirty"]
    assert collect.main(args) == 0
    options = recorded[0]
    assert options["allocators"] == ("nearest",)
    assert options["planner"] == "independent_astar"
    assert options["fleet"] == [5]
    assert options["seeds"] == [3000, 3001, 3002, 3003, 3004]
    assert options["rates"] == (0.1, 0.2)
    assert options["jobs"] == 1
    assert options["keep_traces"] is True and options["allow_dirty"] is True


@pytest.mark.parametrize(
    "args",
    [
        ["--allocators", "nearest,learned"],
        ["--rates", "0.125"],
        ["--seeds", "10-1"],
        ["--jobs", "0"],
        ["--planner", "cbs"],
    ],
)
def test_cli_rejects_bad_arguments(recorded, args):
    with pytest.raises(SystemExit) as exit_info:
        collect.main(args)
    assert exit_info.value.code == 2
    assert recorded == []


def test_cli_reports_errors(monkeypatch, capsys):
    def fail(repo, **kwargs):
        raise collect.CollectError("sweep collect_hungarian_r010 exited with 1")

    monkeypatch.setattr(collect, "collect", fail)
    assert collect.main([]) == 1
    assert "error: sweep collect_hungarian_r010 exited with 1" in capsys.readouterr().err


# The real run_sweep


@pytest.mark.core
def test_sweep_step_with_every_allocator(tmp_path, run_sweep):
    # Every logging allocator runs since Stage 3. Until Stage 4, only independent_astar does.
    manifest = run(tmp_path, run_sweep, allocators=collect.ALLOCATORS)
    runs = tmp_path / "data" / manifest["commit"] / "runs"
    for allocator in collect.ALLOCATORS:
        out = runs / f"collect_{allocator}_r010"
        run_dirs = [f"{out.name}_n5_s3000", f"{out.name}_n5_s3001"]
        assert files(out) == ["aggregate.csv", *run_dirs, "sweep.csv"]
        for name, seed in zip(run_dirs, (3000, 3001), strict=True):
            assert not TRACES & set(files(out / name))
            summary = pd.read_csv(out / name / "summary.csv", dtype=str, keep_default_na=False)
            columns = ["run", "allocator", "planner", "fleet_size", "seed"]
            assert summary.loc[0, columns].tolist() == [
                name,
                allocator,
                "independent_astar",
                "5",
                str(seed),
            ]
            config = yaml.safe_load((out / name / "config.yaml").read_text())
            assert config["orders"]["arrival_rate"] == 0.1
            assert config["map"]["file"] == str(REPO / "maps" / "warehouse_small.txt")
            schema.read_assignments(out / name)  # written since Stage 3; passes every check
    assert [sweep["trace_files_deleted"] for sweep in manifest["sweeps"]] == [4, 4, 4]

from pathlib import Path

import pandas as pd
import pytest

from fleet_learning import schema

FIXTURE = Path(__file__).parent / "fixtures" / "run_tiny"


def fixture_text() -> str:
    return (FIXTURE / "assignments.csv").read_text()


def write_csv(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "assignments.csv"
    path.write_text(text)
    return path


def edit_header(text: str, edit) -> str:
    header, rest = text.split("\n", 1)
    return ",".join(edit(header.split(","))) + "\n" + rest


def test_fixture_passes():
    frame = schema.read_assignments(FIXTURE)
    assert list(frame.columns) == list(schema.COLUMNS)
    assert len(frame) == 15
    assert frame["t"].dtype == "int64"
    assert frame["end_status"].dtype == "string"


def test_column_groups_cover_the_header_once():
    assert len(schema.COLUMNS) == 26
    assert len(set(schema.COLUMNS)) == 26
    assert schema.COLUMNS[0] == "t" and schema.COLUMNS[-1] == "end_status"


def test_missing_column_fails(tmp_path):
    frame = pd.read_csv(FIXTURE / "assignments.csv").drop(columns=["robots_in_box"])
    path = tmp_path / "assignments.csv"
    frame.to_csv(path, index=False)
    with pytest.raises(schema.SchemaError, match="missing columns: robots_in_box"):
        schema.read_assignments(path)


def test_extra_column_fails(tmp_path):
    frame = pd.read_csv(FIXTURE / "assignments.csv")
    frame["queue_len"] = 0
    path = tmp_path / "assignments.csv"
    frame.to_csv(path, index=False)
    with pytest.raises(schema.SchemaError, match="unexpected columns: queue_len"):
        schema.read_assignments(path)


def test_renamed_column_fails(tmp_path):
    text = edit_header(
        fixture_text(), lambda cols: ["order_id" if c == "order" else c for c in cols]
    )
    with pytest.raises(
        schema.SchemaError, match="missing columns: order; unexpected columns: order_id"
    ):
        schema.read_assignments(write_csv(tmp_path, text))


def test_reordered_columns_fail(tmp_path):
    def swap(cols):
        cols[0], cols[1] = cols[1], cols[0]
        return cols

    with pytest.raises(schema.SchemaError, match="out of order"):
        schema.read_assignments(write_csv(tmp_path, edit_header(fixture_text(), swap)))


def test_duplicated_column_fails():
    with pytest.raises(schema.SchemaError, match="duplicated columns: t"):
        schema.check_columns(["t", *schema.COLUMNS])


def test_non_integer_value_fails(tmp_path):
    header, first, rest = fixture_text().split("\n", 2)
    text = "\n".join([header, "3.5" + first[first.index(",") :], rest])
    with pytest.raises(schema.SchemaError, match="dtype"):
        schema.read_assignments(write_csv(tmp_path, text))


def test_missing_file_names_stage_3(tmp_path):
    with pytest.raises(FileNotFoundError, match="Stage 3"):
        schema.read_assignments(tmp_path)


def broken(column: str, value, where=None) -> pd.DataFrame:
    """The fixture with `column` set to value(row) in the first row where `where` holds."""
    frame = schema.read_assignments(FIXTURE)
    row = 0 if where is None else frame.index[where(frame)][0]
    frame.loc[row, column] = value(frame.loc[row])
    return frame


def unreached(frame):
    return frame["arrived_t"] == -1


def delivered(frame):
    return frame["end_status"] == "delivered"


@pytest.mark.parametrize(
    ("frame", "message"),
    [
        (broken("end_status", lambda row: "lost"), "end_status must be one of"),
        (broken("t", lambda row: 10**6), "not sorted"),
        (broken("busy_robots", lambda row: row["busy_robots"] + 1), "available_robots"),
        (broken("steps_to_pickup", lambda row: row["steps_to_pickup"] + 1), "arrived_t - t \\+ 1"),
        (broken("steps_to_pickup", lambda row: 5, unreached), "must be -1"),
        (broken("delivered_t", lambda row: -1, delivered), "delivered_t"),
        (
            broken("dist_to_pickup", lambda row: row["steps_to_pickup"] + 1),
            "steps_to_pickup < dist_to_pickup",
        ),
    ],
)
def test_broken_rows_fail(frame, message):
    with pytest.raises(schema.SchemaError, match=message):
        schema.check_rows(frame)


def test_free_flow_bound_only_checked_without_lost_reports():
    frame = broken("dist_to_pickup", lambda row: row["steps_to_pickup"] + 1)
    schema.check_rows(frame, lossless=False)


def test_already_on_pickup_gives_zero_steps():
    frame = schema.read_assignments(FIXTURE)
    t = frame.loc[0, "t"]
    frame.loc[0, ["dist_to_pickup", "arrived_t", "steps_to_pickup"]] = [0, t - 1, 0]
    schema.check_rows(frame)
    frame.loc[0, ["arrived_t", "steps_to_pickup"]] = [t - 2, -1]
    with pytest.raises(schema.SchemaError, match="arrived_t is before t - 1"):
        schema.check_rows(frame)

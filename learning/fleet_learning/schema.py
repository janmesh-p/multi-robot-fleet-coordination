"""Columns, dtypes, and checks for `assignments.csv`, version 1.

See section 3 of docs/learning_track.md. One step is one simulated second; distances are in
steps (one cell per step).
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import pandas as pd

ID_COLUMNS: tuple[str, ...] = ("t", "order", "robot")

FEATURE_COLUMNS: tuple[str, ...] = (
    "dist_to_pickup",
    "manhattan_to_pickup",
    "dist_pickup_to_dropoff",
    "robot_row",
    "robot_col",
    "pickup_row",
    "pickup_col",
    "robot_on_endpoint",
    "robots_near_robot",
    "robots_near_pickup",
    "robots_in_box",
    "box_free_cells",
    "pickup_assigned",
    "pickup_pending",
    "pending_orders",
    "available_robots",
    "busy_robots",
    "alive_robots",
    "order_age",
)

OUTCOME_COLUMNS: tuple[str, ...] = (
    "arrived_t",
    "steps_to_pickup",
    "delivered_t",
    "end_status",
)

COLUMNS: tuple[str, ...] = ID_COLUMNS + FEATURE_COLUMNS + OUTCOME_COLUMNS

DTYPES: dict[str, str] = {name: "int64" for name in COLUMNS}
DTYPES["end_status"] = "string"

END_STATUSES: tuple[str, ...] = ("delivered", "picked_up", "assigned", "requeued")

FILE_NAME = "assignments.csv"


class SchemaError(ValueError):
    """An `assignments.csv` that does not match version 1 of the contract."""


def check_columns(columns: Sequence[str]) -> None:
    """Raise SchemaError unless `columns` is exactly the version 1 header, in order."""
    columns = list(columns)
    if columns == list(COLUMNS):
        return
    missing = [c for c in COLUMNS if c not in columns]
    extra = [c for c in columns if c not in COLUMNS]
    duplicated = sorted({c for c in columns if columns.count(c) > 1})
    problems = []
    if missing:
        problems.append(f"missing columns: {', '.join(missing)}")
    if extra:
        problems.append(f"unexpected columns: {', '.join(extra)}")
    if duplicated:
        problems.append(f"duplicated columns: {', '.join(duplicated)}")
    if not problems:
        problems.append("columns are out of order")
    raise SchemaError(
        f"{FILE_NAME} does not match schema version 1 ({'; '.join(problems)}). "
        "If the core added a column, update fleet_learning/schema.py to match "
        "docs/learning_track.md section 3."
    )


def check_rows(frame: pd.DataFrame, lossless: bool = True) -> None:
    """Raise SchemaError if the rows break a guarantee from section 3 of the contract.

    `lossless` says the run lost no robot reports (`faults.packet_loss: 0` and no
    `comm_outage` events). Only then must `steps_to_pickup >= dist_to_pickup` hold.
    """
    check_columns(frame.columns)

    def fail(what: str, mask: pd.Series) -> None:
        rows = list(frame.index[mask][:5])
        raise SchemaError(f"{FILE_NAME}: {what} (first rows: {rows})")

    bad_status = ~frame["end_status"].isin(END_STATUSES)
    if bad_status.any():
        fail(f"end_status must be one of {', '.join(END_STATUSES)}", bad_status)

    key = list(zip(frame["t"], frame["robot"], strict=True))
    if key != sorted(key):
        raise SchemaError(f"{FILE_NAME}: rows are not sorted by t, then robot")

    load = frame["available_robots"] + frame["busy_robots"] != frame["alive_robots"]
    if load.any():
        fail("available_robots + busy_robots != alive_robots", load)

    reached = frame["arrived_t"] != -1
    no_steps = ~reached & (frame["steps_to_pickup"] != -1)
    if no_steps.any():
        fail("steps_to_pickup must be -1 when arrived_t is -1", no_steps)
    wrong_steps = reached & (frame["steps_to_pickup"] != frame["arrived_t"] - frame["t"] + 1)
    if wrong_steps.any():
        fail("steps_to_pickup != arrived_t - t + 1", wrong_steps)
    early = reached & (frame["arrived_t"] < frame["t"] - 1)
    if early.any():
        fail("arrived_t is before t - 1", early)

    delivered = frame["end_status"] == "delivered"
    status_mismatch = delivered != (frame["delivered_t"] != -1)
    if status_mismatch.any():
        fail("delivered_t must be set exactly when end_status is delivered", status_mismatch)

    if lossless:
        slow = reached & (frame["steps_to_pickup"] < frame["dist_to_pickup"])
        if slow.any():
            fail("steps_to_pickup < dist_to_pickup in a run without lost reports", slow)


def read_assignments(path: str | Path, lossless: bool = True) -> pd.DataFrame:
    """Read one `assignments.csv` (a file, or a run directory that holds one) and check it.

    Returns one row per assignment with the dtypes in `DTYPES`. Raises SchemaError when the
    header or the rows break the contract, and FileNotFoundError when the file is missing.
    """
    path = Path(path)
    if path.is_dir():
        path = path / FILE_NAME
    if not path.is_file():
        raise FileNotFoundError(
            f"{path} not found. The core writes {FILE_NAME} from Stage 3 on; check the "
            "Stages table in README.md on origin/main."
        )
    header = pd.read_csv(path, nrows=0).columns
    check_columns(header)
    try:
        frame = pd.read_csv(path, dtype=DTYPES, keep_default_na=False, na_values=[])
    except ValueError as error:
        raise SchemaError(f"{path}: a value does not fit its column dtype ({error})") from error
    check_rows(frame, lossless=lossless)
    return frame

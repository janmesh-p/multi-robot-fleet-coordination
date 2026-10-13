"""Synthetic dataset tables and run directories for tests.

Rows pass every check in schema.py. Their target follows section 10 of docs/learning_track.md:
steps_to_pickup = dist_to_pickup + 3 * pickup_assigned + noise, where the noise is 1 step with
probability 0.2 and 0 otherwise. The other features are random. Seeds only label the runs for
the splits; no simulation runs with them.
"""

from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from fleet_learning import dataset, schema

ALLOCATORS = ("nearest", "hungarian", "cost_aware")
FLEET_SIZES = (5, 10, 25)
ARRIVAL_RATES = (0.05, 0.1, 0.2)
STEPS_RUN = 3600


def assignments(n: int, rng: np.random.Generator, alive: int) -> pd.DataFrame:
    """`n` valid assignments.csv rows for a fleet of `alive` robots, one per step at most."""
    t = np.cumsum(rng.integers(1, 4, n))
    dist = rng.integers(0, 41, n)
    pickup_assigned = rng.integers(0, 3, n)
    steps = dist + 3 * pickup_assigned + rng.binomial(1, 0.2, n)
    to_dropoff = rng.integers(20, 60, n)
    busy = rng.integers(0, alive + 1, n)
    columns = {
        "t": t,
        "order": np.arange(n),
        "robot": rng.integers(0, alive, n),
        "dist_to_pickup": dist,
        "manhattan_to_pickup": np.maximum(dist - rng.integers(0, 4, n), 0),
        "dist_pickup_to_dropoff": to_dropoff,
        "robot_row": rng.integers(0, 15, n),
        "robot_col": rng.integers(0, 38, n),
        "pickup_row": rng.integers(0, 15, n),
        "pickup_col": rng.integers(0, 38, n),
        "robot_on_endpoint": rng.integers(0, 2, n),
        "robots_near_robot": rng.integers(0, 4, n),
        "robots_near_pickup": rng.integers(0, 4, n),
        "robots_in_box": rng.integers(0, 4, n),
        "box_free_cells": rng.integers(1, 120, n),
        "pickup_assigned": pickup_assigned,
        "pickup_pending": rng.integers(0, 3, n),
        "pending_orders": rng.integers(1, 8, n),
        "available_robots": alive - busy,
        "busy_robots": busy,
        "alive_robots": np.full(n, alive),
        "order_age": rng.integers(0, 30, n),
        "arrived_t": t + steps - 1,
        "steps_to_pickup": steps,
        "delivered_t": t + steps + to_dropoff + 3,
        "end_status": "delivered",
    }
    return pd.DataFrame(columns)[list(schema.COLUMNS)].astype(schema.DTYPES)


def table(seeds: list[int], rows_per_run: int = 300, seed: int = 0) -> pd.DataFrame:
    """A dataset table, like dataset.read_rows returns: one run per seed in `seeds`."""
    rng = np.random.default_rng(seed)
    frames = []
    for i, run_seed in enumerate(seeds):
        fleet_size = FLEET_SIZES[i % 3]
        metadata = {
            "run": f"synthetic_n{fleet_size}_s{run_seed}",
            "allocator": ALLOCATORS[i % 3],
            "planner": "pibt",
            "fleet_size": fleet_size,
            "seed": run_seed,
            "arrival_rate": ARRIVAL_RATES[i % 3],
            "steps_run": STEPS_RUN,
            "aborted": False,
        }
        frames.append(assignments(rows_per_run, rng, fleet_size).assign(**metadata))
    return pd.concat(frames, ignore_index=True).astype(dataset.METADATA_DTYPES)


def write_collection(root: Path, seeds: list[int], rows_per_run: int = 300) -> Path:
    """Run directories under root/runs/synthetic/, the layout collect.py writes. Returns root."""
    for run, frame in table(seeds, rows_per_run).groupby("run", sort=False):
        run_dir = root / "runs" / "synthetic" / run
        run_dir.mkdir(parents=True)
        frame[list(schema.COLUMNS)].to_csv(run_dir / "assignments.csv", index=False)
        first = frame.iloc[0]
        summary = {name: first[name] for name in ("run", "allocator", "planner", "fleet_size")}
        summary |= {"seed": first["seed"], "steps_run": first["steps_run"], "aborted": "false"}
        pd.DataFrame([summary]).to_csv(run_dir / "summary.csv", index=False)
        config = {
            "orders": {"arrival_rate": float(first["arrival_rate"])},
            "faults": {"packet_loss": 0.0, "events": []},
        }
        (run_dir / "config.yaml").write_text(yaml.safe_dump(config))
    return root

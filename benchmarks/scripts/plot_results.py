#!/usr/bin/env python3
"""Aggregate benchmarks/results/*/summary.csv into comparison plots (Stage 7)."""
import argparse
import glob
import os

import pandas as pd


def load(results_dir: str) -> pd.DataFrame:
    files = glob.glob(os.path.join(results_dir, "*", "summary.csv"))
    if not files:
        raise SystemExit(f"no summary.csv under {results_dir}")
    return pd.concat((pd.read_csv(f) for f in files), ignore_index=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default="benchmarks/results")
    ap.add_argument("--out", default="benchmarks/plots")
    args = ap.parse_args()

    df = load(args.results)
    os.makedirs(args.out, exist_ok=True)
    print(df.groupby(["allocator", "planner", "fleet_size"]).mean(numeric_only=True))
    # TODO(stage7): throughput vs fleet size, conflicts vs planner, replan latency CDF.


if __name__ == "__main__":
    main()

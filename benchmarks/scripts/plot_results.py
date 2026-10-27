#!/usr/bin/env python3
"""The README figures, drawn from benchmarks/evidence/ (Stage 7).

    pip install matplotlib
    python3 benchmarks/scripts/plot_results.py [--out docs/figures]

Writes throughput_large.png, time_budget_large.png, delay_loss.png, and planning_time_large.png.
Every value comes from the same CSVs and functions as the README tables (evidence.py), so a
figure and its table always agree. Run from the repo root.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.axes import Axes  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import evidence as ev  # noqa: E402

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
# Categorical slots in their fixed order (validated: adjacent CVD and normal-vision separation).
SLOTS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
ON_FILL = {"#eda100": INK}  # label ink inside a fill; white elsewhere

PLANNERS = ev.FOUR
FLEETS = (25, 50, 100)
MAPS = (("large", "warehouse_large"), ("large_pockets", "warehouse_large_pockets"))
# The time budget classes of evidence.py, in stacking order, each with its own color. This order
# of hues is validated for adjacent segments (CVD and normal-vision separation, light surface).
CLASS_COLOR = {
    "time_drive_to_pickup": "#2a78d6",
    "time_drive_loaded": "#eb6834",
    "time_service": "#6250d6",
    "time_traffic": "#e87ba4",
    "time_station_wait": "#eda100",
}

plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.size": 9,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK_2,
        "axes.titlecolor": INK,
        "axes.titlesize": 10,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelcolor": INK_2,
        "ytick.labelcolor": INK_2,
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "legend.frameon": False,
        "legend.labelcolor": INK_2,
    }
)


def style(ax: Axes, grid_axis: str = "y") -> None:
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.grid(axis=grid_axis, color=GRID, linewidth=0.8, linestyle="-")
    ax.set_axisbelow(True)
    ax.tick_params(length=0)


def line(ax: Axes, xs: Sequence[float], ys: Sequence[float], color: str, label: str) -> None:
    ax.plot(xs, ys, color=color, linewidth=2, solid_capstyle="round", solid_joinstyle="round")
    ax.plot(
        xs,
        ys,
        linestyle="none",
        marker="o",
        markersize=7,
        markerfacecolor=color,
        markeredgecolor=SURFACE,
        markeredgewidth=2,
        label=label,
    )


def throughput_large(out: Path) -> None:
    """Orders per hour by fleet size, both maps, one panel per planner, 95% intervals."""
    fig, axes = plt.subplots(1, 4, figsize=(10, 3.2), sharey=True)
    x = list(range(len(FLEETS)))
    widest = 0.0
    for ax, planner in zip(axes, PLANNERS, strict=True):
        style(ax)
        for (suite, label), color, dx in zip(MAPS, SLOTS[:2], (-0.06, 0.06), strict=True):
            means, lows, highs = [], [], []
            for n in FLEETS:
                runs = ev.where(ev.rows(suite), planner=planner, fleet_size=n)
                m = ev.mean(runs, "orders_per_hour")
                lo, hi = ev.bootstrap(ev.values(runs, "orders_per_hour"))
                widest = max(widest, hi - lo)
                means.append(m)
                lows.append(m - lo)
                highs.append(hi - m)
            xs = [i + dx for i in x]
            ax.errorbar(xs, means, yerr=[lows, highs], fmt="none", ecolor=color, elinewidth=1)
            line(ax, xs, means, color, label)
        ax.set_title(planner, loc="left")
        ax.set_xticks(x, [str(n) for n in FLEETS])
        ax.set_xlabel("robots")
        ax.set_xlim(-0.4, len(FLEETS) - 0.6)
    axes[0].set_ylabel("orders per hour")
    axes[0].set_ylim(bottom=0)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", ncol=2, bbox_to_anchor=(0.06, 1.02))
    fig.suptitle(
        "Orders per hour at 0.5 orders per step, mean of seeds 1 to 10. The widest 95% interval "
        f"spans {widest:.0f} orders per hour, about the size of a dot.",
        x=0.06,
        y=1.1,
        ha="left",
        color=INK,
        fontsize=10,
    )
    fig.tight_layout()
    fig.savefig(out / "throughput_large.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def time_budget_large(out: Path, planner: str = "prioritized") -> None:
    """Share of robot-steps by class, one bar per map and fleet size, for one planner."""
    bars = []
    for suite, label in MAPS:
        for n in FLEETS:
            runs = ev.where(ev.rows(suite), planner=planner, fleet_size=n)
            bars.append((f"{label}, {n} robots", ev.budget(runs)))
    fig, ax = plt.subplots(figsize=(10, 3.4))
    style(ax, grid_axis="x")
    for row, (_, shares) in enumerate(bars):
        left = 0.0
        y = len(bars) - 1 - row
        for k, share in enumerate(shares):
            color = CLASS_COLOR[ev.BUDGET[k][0]]
            ax.barh(y, share, left=left, height=0.45, color=color, edgecolor=SURFACE, linewidth=1)
            if share >= 6:
                ax.text(
                    left + share / 2,
                    y,
                    f"{share:.0f}%",
                    ha="center",
                    va="center",
                    fontsize=8,
                    color=ON_FILL.get(color, "#ffffff"),
                )
            left += share
    ax.set_yticks(range(len(bars)), [name for name, _ in reversed(bars)])
    ax.set_xlim(0, 100)
    ax.set_xticks(range(0, 101, 20), [f"{v}%" for v in range(0, 101, 20)])
    handles = [plt.Rectangle((0, 0), 1, 1, color=CLASS_COLOR[c]) for c, _ in ev.BUDGET]
    ax.legend(
        handles,
        [name.lower() for _, name in ev.BUDGET],
        loc="lower left",
        ncol=len(ev.BUDGET),
        bbox_to_anchor=(0, 1.0),
        handlelength=1,
        columnspacing=1.2,
    )
    ax.set_title(
        f"Where the robot time goes after the warmup, {planner}, seeds 1 to 10",
        loc="left",
        pad=28,
    )
    fig.tight_layout()
    fig.savefig(out / "time_budget_large.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def delay_loss(out: Path) -> None:
    """Orders per hour by the chance of a missed step and of a lost message."""
    levels = {
        "late robots: chance a robot misses a step": (
            ("0", "planning_r010"),
            ("0.05", "delay_005"),
            ("0.1", "delay_010"),
            ("0.2", "delay_020"),
            ("0.3", "delay_030"),
        ),
        "lost messages: chance a report or command is lost": (
            ("0", "planning_r010"),
            ("0.05", "loss_005"),
            ("0.1", "loss_010"),
            ("0.2", "loss_020"),
            ("0.3", "loss_030"),
        ),
    }
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharey=True)
    for r, (title, suites) in enumerate(levels.items()):
        xs = [float(p) for p, _ in suites]
        for c, n in enumerate((10, 25)):
            ax = axes[r][c]
            style(ax)
            for planner, color in zip(PLANNERS, SLOTS[:4], strict=True):
                ys = [
                    ev.mean(ev.where(ev.rows(s), planner=planner, fleet_size=n), "orders_per_hour")
                    for _, s in suites
                ]
                line(ax, xs, ys, color, planner)
            ax.set_title(f"{n} robots, {title}", loc="left")
            ax.set_xticks(xs, [p for p, _ in suites])
            ax.set_ylim(0, 400)
        axes[r][0].set_ylabel("orders per hour")
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", ncol=4, bbox_to_anchor=(0.06, 1.03))
    fig.suptitle(
        "warehouse_small, 0.1 orders per step, mean of seeds 1 to 10",
        x=0.06,
        y=1.07,
        ha="left",
        color=INK,
        fontsize=10,
    )
    fig.tight_layout()
    fig.savefig(out / "delay_loss.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def planning_time_large(out: Path) -> None:
    """Planning time per call at p95 by fleet size. Wall time: depends on the machine."""
    fig, ax = plt.subplots(figsize=(6, 3.6))
    style(ax)
    x = list(range(len(FLEETS)))
    for planner, color in zip(PLANNERS, SLOTS[:4], strict=True):
        ys = [
            ev.mean(ev.where(ev.rows("large"), planner=planner, fleet_size=n), "planning_p95_ms")
            for n in FLEETS
        ]
        line(ax, x, ys, color, planner)
    ax.set_yscale("log")
    low, high = ax.get_ylim()
    ticks = [t for t in (0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100) if low <= t <= high]
    ax.set_yticks(ticks, [f"{t:g}" for t in ticks])
    ax.set_ylim(low, high)
    ax.minorticks_off()
    ax.set_xticks(x, [str(n) for n in FLEETS])
    ax.set_xlabel("robots")
    ax.set_ylabel("ms per call, p95 (log scale)")
    ax.legend(loc="upper left", ncol=2)
    ax.set_title(
        "Planning time on warehouse_large, mean of each run's p95\n"
        "wall time on one core of the build machine; depends on the machine",
        loc="left",
    )
    fig.tight_layout()
    fig.savefig(out / "planning_time_large.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0] if __doc__ else None)
    parser.add_argument("--out", type=Path, default=Path("docs/figures"))
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    throughput_large(args.out)
    time_budget_large(args.out)
    delay_loss(args.out)
    planning_time_large(args.out)
    for name in ("throughput_large", "time_budget_large", "delay_loss", "planning_time_large"):
        print(f"wrote      {args.out / name}.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())

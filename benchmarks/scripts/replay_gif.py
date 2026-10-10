#!/usr/bin/env python3
"""Replay a run directory (map.txt + positions.csv + steps.csv) as an animated GIF.

  python3 benchmarks/scripts/replay_gif.py RUN_DIR [--start 0] [--end 300] [--stride 1]
                                                   [--fps 8] [--out FILE] [--png STEP]

Robots are numbered dots: blue heading to a pickup, orange carrying an order, aqua idle.
A square is a robot servicing a station. A black x marks a robot stopped by another robot.
--png STEP writes that single step as a PNG instead of a GIF.

Needs matplotlib (pip install matplotlib).
"""
import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

# Light theme from the project's chart palette (validated: blue, orange, aqua all-pairs).
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
MUTED = "#898781"
SHELF = "#52514e"
PARKING = "#ecebe7"
STATION = "#e1e0d9"
TO_PICKUP = "#2a78d6"
CARRYING = "#eb6834"
IDLE = "#1baf7a"

STATUS_COLOR = {
    "idle": IDLE,
    "to_pickup": TO_PICKUP,
    "picking": TO_PICKUP,
    "to_dropoff": CARRYING,
    "dropping": CARRYING,
    "safe_stop": MUTED,
    "dead": MUTED,
}
SERVICING = {"picking", "dropping"}
BLOCKED = {"blocked_vertex", "blocked_swap", "blocked_occupied", "blocked_cell"}


def load_map(path):
    rows = [line.rstrip("\n") for line in open(path) if line.strip() and not line.startswith(";")]
    codes = {".": 0, "#": 1, "R": 2, "P": 3, "D": 3}
    grid = np.array([[codes[ch] for ch in row] for row in rows])
    stations = [(r, c, ch) for r, row in enumerate(rows) for c, ch in enumerate(row) if ch in "PD"]
    return grid, stations


def load_positions(path):
    frames = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            frames.setdefault(int(row["t"]), []).append(
                (int(row["robot"]), int(row["row"]), int(row["col"]), row["status"], row["outcome"])
            )
    return frames


def load_delivered(path):
    total, delivered = 0, {}
    with open(path) as f:
        for row in csv.DictReader(f):
            total += int(row["deliveries"])
            delivered[int(row["t"])] = total
    return delivered


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--end", type=int, default=300, help="last step, inclusive")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--out", type=Path, help="default: RUN_DIR/replay.gif or replay_STEP.png")
    parser.add_argument("--png", type=int, metavar="STEP", help="render one step as a PNG")
    args = parser.parse_args()

    grid, stations = load_map(args.run_dir / "map.txt")
    frames = load_positions(args.run_dir / "positions.csv")
    delivered = load_delivered(args.run_dir / "steps.csv")
    if not frames:
        sys.exit("positions.csv is empty")
    last = max(frames)
    steps = [args.png] if args.png is not None else list(
        range(max(args.start, 0), min(args.end, last) + 1, max(args.stride, 1))
    )
    if not steps or any(t not in frames for t in steps):
        sys.exit(f"steps must lie within 0..{last}")

    rows, cols = grid.shape
    cell_px, dpi = 18, 100
    fig = plt.figure(figsize=(cols * cell_px / dpi, (rows * cell_px + 70) / dpi), dpi=dpi)
    fig.patch.set_facecolor(SURFACE)
    ax = fig.add_axes([0, 50 / (rows * cell_px + 70), 1, rows * cell_px / (rows * cell_px + 70)])
    ax.imshow(grid, cmap=ListedColormap([SURFACE, SHELF, PARKING, STATION]), vmin=0, vmax=3,
              interpolation="nearest")
    for r, c, ch in stations:
        ax.text(c, r, ch, ha="center", va="center", fontsize=7, color=MUTED, fontweight="bold")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    marker_size = (cell_px * 0.85) ** 2
    dots = ax.scatter([], [], s=marker_size, marker="o", edgecolors=INK, linewidths=0.6, zorder=3)
    squares = ax.scatter([], [], s=marker_size, marker="s", edgecolors=INK, linewidths=0.6, zorder=3)
    stops = ax.scatter([], [], s=28, marker="x", c=INK, linewidths=1.4, zorder=5)
    fleet = max(robot for robot, *_ in frames[steps[0]]) + 1
    labels = [ax.text(0, 0, str(i), ha="center", va="center", fontsize=6, color=INK, zorder=4)
              for i in range(fleet)]
    title = fig.text(0.01, 0.985, "", ha="left", va="top", fontsize=9, color=INK)

    legend = [
        Line2D([], [], marker="o", ls="", markerfacecolor=TO_PICKUP, markeredgecolor=INK, label="heading to pickup"),
        Line2D([], [], marker="o", ls="", markerfacecolor=CARRYING, markeredgecolor=INK, label="carrying"),
        Line2D([], [], marker="o", ls="", markerfacecolor=IDLE, markeredgecolor=INK, label="idle"),
        Line2D([], [], marker="s", ls="", markerfacecolor="none", markeredgecolor=INK, label="at station"),
        Line2D([], [], marker="x", ls="", color=INK, label="stopped by robot"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=5, frameon=False, fontsize=7,
               labelcolor=INK, handletextpad=0.3, columnspacing=1.2)

    def draw(t):
        robots = sorted(frames[t])
        circle_xy, circle_c, square_xy, square_c, stop_xy = [], [], [], [], []
        for robot, r, c, status, outcome in robots:
            color = STATUS_COLOR.get(status, MUTED)
            if status in SERVICING:
                square_xy.append((c, r))
                square_c.append(color)
            else:
                circle_xy.append((c, r))
                circle_c.append(color)
            if outcome in BLOCKED:
                stop_xy.append((c + 0.38, r - 0.38))
            labels[robot].set_position((c, r))
        dots.set_offsets(np.array(circle_xy).reshape(-1, 2))
        dots.set_facecolor(circle_c)
        squares.set_offsets(np.array(square_xy).reshape(-1, 2))
        squares.set_facecolor(square_c)
        stops.set_offsets(np.array(stop_xy).reshape(-1, 2))
        title.set_text(f"step {t}   delivered {delivered.get(t, 0)}   robots {len(robots)}")
        return [dots, squares, stops, title, *labels]

    if args.png is not None:
        out = args.out or args.run_dir / f"replay_{args.png}.png"
        draw(args.png)
        fig.savefig(out, dpi=dpi, facecolor=SURFACE)
    else:
        out = args.out or args.run_dir / "replay.gif"
        animation = FuncAnimation(fig, draw, frames=steps, blit=False)
        animation.save(out, writer=PillowWriter(fps=args.fps), dpi=dpi)
    print(out)


if __name__ == "__main__":
    main()

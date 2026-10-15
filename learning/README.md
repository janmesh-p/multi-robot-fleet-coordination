# Learning track: a learned travel-time cost for task allocation

The contract is [docs/learning_track.md](../docs/learning_track.md). Section numbers below refer
to it. One step is one simulated second.

Status: Stage 4 (pibt). The success criterion below was committed before the final runs.

## Question

When orders wait, the allocator matches free robots to orders with the Hungarian method. The
`learned` allocator uses a model's prediction of the steps a robot needs to reach the pickup,
traffic and station queues included, as the cost of a pair. On seeds the model never saw, does
it deliver more orders per hour than `cost_aware`, the hand-tuned congestion cost?

## Success criterion

Written and committed before the final runs (section 9, step 2):

> At 25 robots and arrival rate 0.2, the per-seed difference in `orders_per_hour` (learned minus
> tuned `cost_aware`, seeds 1-10) has a median whose 95% bootstrap interval lies entirely above 0.

The interval is the 2.5th to 97.5th percentile of the medians of 10,000 resamples of the 10
differences, drawn with replacement using `numpy.random.default_rng(0)`.

## Data

One collection with `planning.method: pibt` at commit `430e08a9cd77` (Stage 4), on
`maps/warehouse_small.txt`, from `benchmarks/configs/default.yaml` with the section 7 grid:
logging allocators `nearest`, `hungarian`, `cost_aware`; arrival rates 0.05, 0.1, 0.2 orders per
step; fleets 5, 10, 25; seeds 1000-1039. That is 1080 runs of 3600 steps. No run aborted,
deadlocked, or froze.

| Split | Seeds | Runs | Rows after filters |
|---|---|---:|---:|
| train | 1000-1019 | 540 | 106334 |
| validation | 1020-1029 | 270 | 53595 |
| offline test | 1030-1039 | 270 | 54525 |

From `python -m fleet_learning.dataset learning/data/430e08a9cd77/runs --margin 900` and
`learning/models/cost_model_linear.metrics.json`.

The runs logged 284028 assignments. Filters, in order (section 3):

- aborted runs: 0 rows.
- margin: rows with `t > steps_run - 900`, 69574 rows. The margin is 900 steps, not 300. The
  longest `steps_to_pickup` is 446 steps with any margin from 300 to 1200, which is not well
  under 300. 900 is the smallest multiple of 100 that is at least twice 446.
- unreached (`steps_to_pickup = -1` inside the margin): 0 rows in every group. With margin 300
  there was 1 such row (hungarian, 25 robots). No group is near the 1% threshold.

Per logging allocator and fleet size, rows dropped by the margin filter and rows kept:

| Allocator | Robots | Rows | Margin | Unreached | Kept |
|---|---:|---:|---:|---:|---:|
| cost_aware | 5 | 21369 | 5320 | 0 | 16049 |
| cost_aware | 10 | 32186 | 7889 | 0 | 24297 |
| cost_aware | 25 | 41970 | 10212 | 0 | 31758 |
| hungarian | 5 | 21203 | 5229 | 0 | 15974 |
| hungarian | 10 | 31705 | 7790 | 0 | 23915 |
| hungarian | 25 | 41394 | 10072 | 0 | 31322 |
| nearest | 5 | 20988 | 5222 | 0 | 15766 |
| nearest | 10 | 31808 | 7758 | 0 | 24050 |
| nearest | 25 | 41405 | 10082 | 0 | 31323 |

From the `filters` entry of `learning/models/cost_model_linear.metrics.json`.

## Models

`train.py` fits every candidate on the training split and keeps, per family, the one with the
lowest validation MAE: both targets (`steps_to_pickup`, or the excess over `dist_to_pickup`)
and, for `mlp`, hidden layers 32, 64x64, and 64x64x64. Random seed 0. Three deployable models
went to closed-loop tuning:

| File | What it is | Validation MAE (steps) |
|---|---|---:|
| `cost_model_v1.json` | the deployable model with the lowest validation MAE: mlp, direct target, one hidden layer of 32, all 19 features | 8.64 |
| `cost_model_linear.json` | linear, direct target, all 19 features | 10.38 |
| `cost_model_nocells.json` | mlp, direct target, 64x64, without `robot_row`, `robot_col`, `pickup_row`, `pickup_col` | 9.06 |

From `"validation"` of each model's metrics file. Tuning chose `cost_model_linear` (below).

## Offline test

The 54525 rows of seeds 1030-1039, scored once. Errors in steps. Bias is predicted minus
observed `steps_to_pickup`. Delayed rows (49934 of 54525) have `steps_to_pickup >
dist_to_pickup`. Each row is the family winner on validation.

| Model | Candidate | MAE | RMSE | Bias | Delayed MAE | Delayed RMSE | Delayed bias |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| free_flow | free_flow | 20.76 | 36.04 | -20.76 | 22.66 | 37.66 | -22.66 |
| linear (deployed) | linear_direct | 10.57 | 15.63 | 0.42 | 11.07 | 16.14 | 0.10 |
| mlp | mlp_direct_32 | 8.86 | 13.79 | -0.30 | 9.19 | 14.25 | -0.81 |
| gbdt (not deployable) | gbdt_excess | 8.88 | 13.86 | 0.01 | 9.23 | 14.34 | -0.46 |

MAE by fleet size (robots):

| Model | 5 | 10 | 25 |
| --- | ---: | ---: | ---: |
| rows | 12011 | 18274 | 24240 |
| free_flow | 7.41 | 13.89 | 32.54 |
| linear (deployed) | 7.25 | 9.94 | 12.70 |
| mlp | 5.09 | 8.40 | 11.08 |
| gbdt (not deployable) | 5.27 | 8.36 | 11.07 |

MAE by arrival rate (orders per step):

| Model | 0.05 | 0.1 | 0.2 |
| --- | ---: | ---: | ---: |
| rows | 12537 | 19648 | 22340 |
| free_flow | 5.23 | 13.54 | 35.81 |
| linear (deployed) | 5.90 | 9.71 | 13.95 |
| mlp | 3.93 | 7.94 | 12.44 |
| gbdt (not deployable) | 3.81 | 7.88 | 12.61 |

MAE by logging allocator:

| Model | cost_aware | hungarian | nearest |
| --- | ---: | ---: | ---: |
| rows | 18339 | 18093 | 18093 |
| free_flow | 19.36 | 21.79 | 21.13 |
| linear (deployed) | 10.05 | 10.99 | 10.68 |
| mlp | 8.66 | 9.01 | 8.92 |
| gbdt (not deployable) | 8.71 | 9.11 | 8.82 |

MAE by `dist_to_pickup` (steps):

| Model | 0-5 | 6-10 | 11-20 | 21+ |
| --- | ---: | ---: | ---: | ---: |
| rows | 1525 | 4545 | 9917 | 38538 |
| free_flow | 4.63 | 5.81 | 8.08 | 26.42 |
| linear (deployed) | 3.37 | 4.97 | 7.26 | 12.37 |
| mlp | 3.18 | 3.93 | 4.93 | 10.68 |
| gbdt (not deployable) | 2.92 | 3.84 | 4.68 | 10.79 |

From `python -m fleet_learning.evaluate learning/models/cost_model_linear.metrics.json`.

The free-flow cost predicts 20.76 steps too few on average: most of a robot's trip to the
pickup is waiting, not driving. The mlp and the gbdt reference are within 0.02 steps MAE of each
other, so a tree model would not help much here.

## Closed-loop tuning

Seeds 2000-2009, fleets 5, 10, 25, arrival rates 0.1 and 0.2, `planning.method: pibt`: 60 runs
per candidate. Mean `orders_per_hour` (deliveries per simulated hour after the 600-step warmup):

| Candidate | Runs | All runs | n5 r0.1 | n5 r0.2 | n10 r0.1 | n10 r0.2 | n25 r0.1 | n25 r0.2 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cost_aware w=0.25 | 60 | 301.14 | 178.32 | 176.64 | 302.52 | 302.40 | 365.16 | 481.80 |
| cost_aware w=0.5 | 60 | 303.10 | 178.08 | 175.80 | 305.40 | 305.40 | 365.64 | 488.28 |
| cost_aware w=1 | 60 | 303.28 | 177.96 | 177.72 | 306.84 | 306.48 | 365.76 | 484.92 |
| cost_aware w=2 | 60 | 304.50 | 178.44 | 179.40 | 306.24 | 307.56 | 367.08 | 488.28 |
| cost_aware w=4 | 60 | 305.58 | 179.52 | 178.44 | 307.08 | 308.76 | 366.12 | 493.56 |
| learned cost_model_v1 | 60 | 306.52 | 178.68 | 177.36 | 312.96 | 312.60 | 365.76 | 491.76 |
| learned cost_model_linear | 60 | 309.36 | 181.92 | 185.16 | 313.44 | 318.84 | 366.48 | 490.32 |
| learned cost_model_nocells | 60 | 304.52 | 177.96 | 175.92 | 309.84 | 308.76 | 365.88 | 488.76 |

From `python -m fleet_learning.closed_loop report learning/data/9134958a33c1/closed_loop/tune`.

Chosen: `cost_aware` with `congestion_weight` 4, and `learned` with `cost_model_linear.json`.
The linear model has the highest offline MAE of the three but delivered the most orders in
tuning. Offline accuracy did not rank the models the way the closed loop did.

## Final evaluation

To be filled in from the final runs on seeds 1-10.

## How to reproduce

From the repo root, after the setup in section 6 and the build in CONTRIBUTING.md.

```bash
# Data. The folder is named after the commit, so collect at the Stage 4 commit.
git checkout 430e08a9cd77
learning/.venv/bin/python -m fleet_learning.collect
git checkout -   # learning/data/ is ignored by git and stays

# Models (about 2.5 minutes each).
P=learning/.venv/bin/python
$P -m fleet_learning.train --data learning/data/430e08a9cd77 --name cost_model_v1 --margin 900
$P -m fleet_learning.train --data learning/data/430e08a9cd77 --name cost_model_linear \
  --margin 900 --export linear
$P -m fleet_learning.train --data learning/data/430e08a9cd77 --name cost_model_nocells \
  --margin 900 --export mlp --features dist_to_pickup,manhattan_to_pickup,\
dist_pickup_to_dropoff,robot_on_endpoint,robots_near_robot,robots_near_pickup,robots_in_box,\
box_free_cells,pickup_assigned,pickup_pending,pending_orders,available_robots,busy_robots,\
alive_robots,order_age

# Offline tables and filter counts.
$P -m fleet_learning.evaluate learning/models/cost_model_linear.metrics.json
$P -m fleet_learning.dataset learning/data/430e08a9cd77/runs --margin 900

# Closed-loop tuning. It writes under learning/data/<commit>/closed_loop/tune, where <commit>
# is the checked-out commit; the numbers do not depend on it.
$P -m fleet_learning.closed_loop tune --models learning/models/cost_model_v1.json \
  learning/models/cost_model_linear.json learning/models/cost_model_nocells.json
```

Every number except wall time comes out the same on a clean checkout. The SHA-256 of each model
file is in the closed-loop manifest.

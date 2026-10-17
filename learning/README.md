# Learning track: a learned travel-time cost for task allocation

The contract is [docs/learning_track.md](../docs/learning_track.md). Section numbers below refer
to it. One step is one simulated second.

Status: Stage 5 rerun in progress. The Stage 4 round below is complete; its success criterion
was committed before its final runs (commit `8133d47`). The Stage 5 round's criterion and
tuning results are in [Stage 5 round](#stage-5-round-before-the-final-runs), committed before
its final runs.

## Stage 5 round, before the final runs

Collection at commit `6c0e93405694` (Stage 5), the same grid and seeds as the Stage 4 round:
1080 runs, none aborted, deadlocked, or frozen. Margin 1000 steps: the longest kept
`steps_to_pickup` is 457 steps, and 1000 is the smallest multiple of 100 that is at least twice
that. `order_age` is not a model input (section 6). From
`python -m fleet_learning.dataset learning/data/6c0e93405694/runs --margin 1000`.

Success criterion, the same as the Stage 4 round, written before the final runs:

> At 25 robots and arrival rate 0.2, the per-seed difference in `orders_per_hour` (learned minus
> tuned `cost_aware`, seeds 1-10) has a median whose 95% bootstrap interval lies entirely above 0.

Closed-loop tuning, seeds 2000-2009, fleets 5, 10, 25, arrival rates 0.1 and 0.2,
`planning.method: pibt`, 60 runs per candidate. Mean `orders_per_hour`:

| Candidate | Runs | All runs | n5 r0.1 | n5 r0.2 | n10 r0.1 | n10 r0.2 | n25 r0.1 | n25 r0.2 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cost_aware w=1 | 60 | 299.36 | 175.32 | 173.40 | 306.84 | 301.92 | 365.76 | 472.92 |
| cost_aware w=2 | 60 | 299.24 | 175.08 | 174.96 | 305.88 | 304.08 | 367.08 | 468.36 |
| cost_aware w=4 | 60 | 299.94 | 174.96 | 175.56 | 306.12 | 303.60 | 366.12 | 473.28 |
| cost_aware w=8 | 60 | 299.26 | 175.20 | 174.48 | 308.64 | 302.28 | 366.60 | 468.36 |
| cost_aware w=16 | 60 | 298.68 | 175.20 | 174.48 | 307.08 | 301.44 | 365.64 | 468.24 |
| learned cost_model_s5 | 60 | 298.14 | 174.12 | 174.48 | 306.96 | 300.84 | 365.88 | 466.56 |
| learned cost_model_s5_linear | 60 | 298.42 | 174.96 | 174.84 | 307.20 | 300.60 | 365.16 | 467.76 |
| learned cost_model_s5_nocells | 60 | 299.76 | 175.68 | 175.68 | 307.56 | 301.92 | 366.48 | 471.24 |

From `python -m fleet_learning.closed_loop report learning/data/85a7438f2a57/closed_loop/tune`.

Chosen for the final runs: `cost_aware` with `congestion_weight` 4, and `learned` with
`cost_model_s5_nocells.json`.

## Result

The success criterion is not met. At 25 robots and arrival rate 0.2, learned minus tuned
`cost_aware` has a median of -6.00 orders per hour, with a 95% interval of [-20.40, 7.20] (4
wins, 6 losses). At 5 and 10 robots, `learned` delivers more orders per hour than both
baselines in every cell, with intervals above 0. But it gets there partly by serving the newest
orders first, and old orders wait much longer (see [Order age](#order-age-at-assignment)).

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

Seeds 1-10, fleets 5, 10, 25, arrival rates 0.1 and 0.2, `planning.method: pibt`, each
allocator run once: `hungarian`, `cost_aware` with weight 4, and `learned` with
`cost_model_linear.json` (SHA-256 `fbf675a96ca3...`, full hash in the manifest). Nothing changed
after these numbers came in. The order age table below is reporting added afterwards, from the
same runs.

All tables in this section come from
`python -m fleet_learning.closed_loop report learning/data/8133d475c5b3/closed_loop/final`.

Orders per hour are deliveries per simulated hour after the 600-step warmup. Completion times are
the means over runs of each run's `mean_completion_steps` and `p95_completion_steps`, in steps.
Wall s is the wall time of all 10 runs, summed.

| Robots | Rate | Allocator | Runs | Orders/h mean | Orders/h median | Completion mean | Completion p95 | Deadlocked | Frozen | Wall s |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 5 | 0.1 | hungarian | 10 | 177.00 | 176.40 | 1212.9 | 1993.1 | 0 | 0 | 0.37 |
| 5 | 0.1 | cost_aware | 10 | 179.40 | 179.40 | 1227.7 | 1954.5 | 0 | 0 | 0.37 |
| 5 | 0.1 | learned | 10 | 182.40 | 183.00 | 181.8 | 407.5 | 0 | 0 | 0.40 |
| 5 | 0.2 | hungarian | 10 | 176.64 | 177.00 | 2159.3 | 2732.5 | 0 | 0 | 0.61 |
| 5 | 0.2 | cost_aware | 10 | 178.44 | 178.80 | 2267.8 | 2727.9 | 0 | 0 | 0.45 |
| 5 | 0.2 | learned | 10 | 185.52 | 186.00 | 149.6 | 259.5 | 0 | 0 | 0.72 |
| 10 | 0.1 | hungarian | 10 | 299.64 | 298.80 | 445.6 | 864.2 | 0 | 0 | 0.55 |
| 10 | 0.1 | cost_aware | 10 | 308.52 | 310.20 | 399.8 | 745.0 | 0 | 0 | 0.54 |
| 10 | 0.1 | learned | 10 | 314.64 | 313.80 | 214.1 | 583.8 | 0 | 0 | 0.67 |
| 10 | 0.2 | hungarian | 10 | 299.64 | 298.80 | 1467.0 | 2200.0 | 0 | 0 | 0.80 |
| 10 | 0.2 | cost_aware | 10 | 308.76 | 309.00 | 1470.2 | 2092.3 | 0 | 0 | 0.68 |
| 10 | 0.2 | learned | 10 | 315.96 | 315.60 | 169.4 | 322.6 | 0 | 0 | 0.79 |
| 25 | 0.1 | hungarian | 10 | 356.52 | 363.60 | 94.9 | 160.4 | 0 | 0 | 1.03 |
| 25 | 0.1 | cost_aware | 10 | 358.32 | 363.00 | 96.1 | 160.7 | 0 | 0 | 1.02 |
| 25 | 0.1 | learned | 10 | 357.24 | 363.60 | 95.2 | 160.7 | 0 | 0 | 1.28 |
| 25 | 0.2 | hungarian | 10 | 472.56 | 472.20 | 849.9 | 1395.6 | 0 | 0 | 1.35 |
| 25 | 0.2 | cost_aware | 10 | 488.40 | 490.20 | 816.3 | 1326.2 | 0 | 0 | 1.34 |
| 25 | 0.2 | learned | 10 | 481.80 | 488.40 | 246.3 | 462.2 | 0 | 0 | 1.36 |

No run deadlocked, froze, or aborted. In every cell except 25 robots at rate 0.1, orders pile
up: the baselines assign them a median 187 to 1316 steps after they arrive (order age table
below). Completion times then
depend on which orders get served, and they only count delivered orders. Do not read the short
`learned` completion times as faster service; see the order age table.

### Paired differences

Learned minus each baseline in `orders_per_hour`, paired by seed (a seed gives the same orders
under every allocator). Median over the 10 seeds, with a 95% percentile bootstrap interval:
10,000 resamples with `numpy.random.default_rng(0)`.

learned minus `hungarian`:

| Robots | Rate | Wins/ties/losses | Median | 95% interval |
| --- | --- | ---: | ---: | ---: |
| 5 | 0.1 | 10/0/0 | 5.40 | [3.60, 7.20] |
| 5 | 0.2 | 10/0/0 | 9.00 | [7.20, 10.80] |
| 10 | 0.1 | 10/0/0 | 13.80 | [10.80, 18.00] |
| 10 | 0.2 | 9/0/1 | 18.60 | [13.20, 20.40] |
| 25 | 0.1 | 6/3/1 | 1.20 | [0.00, 2.40] |
| 25 | 0.2 | 7/0/3 | 8.40 | [-3.60, 20.40] |

learned minus tuned `cost_aware`:

| Robots | Rate | Wins/ties/losses | Median | 95% interval |
| --- | --- | ---: | ---: | ---: |
| 5 | 0.1 | 9/1/0 | 2.40 | [1.20, 4.80] |
| 5 | 0.2 | 9/0/1 | 7.20 | [3.60, 10.80] |
| 10 | 0.1 | 8/1/1 | 4.20 | [0.60, 12.60] |
| 10 | 0.2 | 9/0/1 | 6.00 | [3.60, 12.00] |
| 25 | 0.1 | 3/2/5 | -0.60 | [-2.40, 1.20] |
| 25 | 0.2 | 4/0/6 | -6.00 | [-20.40, 7.20] |

Criterion (25 robots, rate 0.2, interval above 0 against `cost_aware`): not met. The median is
negative and the interval contains 0, so this run shows no difference in either direction at
that cell. Against `cost_aware`, the intervals lie above 0 at 5 and 10 robots, with medians of
2.40 to 7.20 orders per hour (1% to 4%).

### Selection bias check

`model_io` costs for the pairs `learned` chose in the final runs, against their
`steps_to_pickup`, after the section 3 filters with the training margin of 900 steps. Errors in
steps; bias is predicted minus observed.

| Rows from | Rows | MAE | Bias |
| --- | ---: | ---: | ---: |
| offline test | 54525 | 10.57 | 0.42 |
| final learned runs | 14158 | 10.77 | -2.20 |
|   n5 r0.1 | 1393 | 7.03 | -5.52 |
|   n5 r0.2 | 1432 | 7.53 | -6.42 |
|   n10 r0.1 | 2370 | 9.37 | -1.03 |
|   n10 r0.2 | 2435 | 9.47 | -4.04 |
|   n25 r0.1 | 2745 | 8.88 | 1.84 |
|   n25 r0.2 | 3783 | 16.48 | -1.85 |

The MAE on its own choices (10.77) is close to the offline test (10.57), so the model is not
badly fooled overall. The bias turns negative: the pairs it chose take 2.20 steps longer than it
predicted, and 5 to 6 steps longer at 5 robots. That is the expected direction for a model that
picks the pairs it scores lowest. At 25 robots and rate 0.2 the MAE is 16.48, against 12.70 for
all 25-robot rows offline (all three rates, so not a like-for-like cell).

### Order age at assignment

`order_age` at assignment, in steps, over every assignment of the final runs:

| Robots | Rate | Allocator | Assignments | Median | p90 | Max |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 5 | 0.1 | hungarian | 1803 | 793 | 1681 | 2431 |
| 5 | 0.1 | cost_aware | 1825 | 826 | 1640 | 2335 |
| 5 | 0.1 | learned | 1852 | 45 | 211 | 897 |
| 5 | 0.2 | hungarian | 1807 | 1314 | 2453 | 3041 |
| 5 | 0.2 | cost_aware | 1830 | 1316 | 2418 | 2959 |
| 5 | 0.2 | learned | 1888 | 35 | 120 | 589 |
| 10 | 0.1 | hungarian | 3034 | 223 | 647 | 1133 |
| 10 | 0.1 | cost_aware | 3115 | 187 | 557 | 891 |
| 10 | 0.1 | learned | 3172 | 28 | 218 | 2304 |
| 10 | 0.2 | hungarian | 3072 | 962 | 1850 | 2559 |
| 10 | 0.2 | cost_aware | 3146 | 981 | 1818 | 2401 |
| 10 | 0.2 | learned | 3219 | 30 | 126 | 696 |
| 25 | 0.1 | hungarian | 3612 | 0 | 0 | 12 |
| 25 | 0.1 | cost_aware | 3612 | 0 | 0 | 69 |
| 25 | 0.1 | learned | 3612 | 0 | 0 | 6 |
| 25 | 0.2 | hungarian | 4881 | 512 | 1090 | 1764 |
| 25 | 0.2 | cost_aware | 5003 | 469 | 1057 | 1470 |
| 25 | 0.2 | learned | 4970 | 23 | 143 | 1000 |

When orders pile up, `learned` assigns orders a median 23 to 45 steps after they arrive, and the
baselines 469 to 1316 steps after. The linear model gives `order_age` a weight of +0.0014 steps
per step of age (from `learning/models/cost_model_linear.json`, weight over scale). That is
small, but with hundreds of pending orders at almost the same distance it decides the order, so
the newest orders go first and the oldest wait. A deployed allocator would starve old orders.
What this run does not show: how much of the throughput gain at 5 and 10 robots comes from that
ordering, and how much from the `pickup_assigned` term (+15.1 steps per order already assigned at
the same pickup), which spreads robots over pickups. An ablation without `order_age` would
separate the two; it was not run.

## Limits

- One map (`warehouse_small`), one planner (`pibt`), no traffic rules or faults (Stages 5 and
  6). All of these change how robots move, and the protocol must be rerun after each stage.
- Selection bias: the training data comes from `nearest`, `hungarian`, and `cost_aware`, never
  from `learned`. The negative bias on its own choices above is a sign of that.
- The best `cost_aware` weight, 4, is the largest in the section 9 grid. A larger weight might do
  better; the grid was fixed in advance and both sides had the same budget.
- Offline accuracy did not predict closed-loop throughput: the model with the highest offline
  MAE won tuning.
- Most cells are overloaded, so `orders_per_hour` is close to the fleet's capacity and the
  completion times mostly measure queue order. Only 25 robots at rate 0.1 is
  below capacity, and there the three allocators are within 2 orders per hour of each other.
- 10 seeds per cell. Intervals at 25 robots and rate 0.2 are 24 to 28 orders per hour wide.
- The model uses `order_age`, which changes which orders get served, not only how long a trip
  takes. The question in section 1 is about trip cost; this side effect is not part of it.

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

# Final runs, once, with the tuned choices, then the tables again.
$P -m fleet_learning.closed_loop final --weight 4 --model learning/models/cost_model_linear.json
$P -m fleet_learning.closed_loop report learning/data/<commit>/closed_loop/final
```

Every number except wall time comes out the same on a clean checkout. The SHA-256 of each model
file is in the closed-loop manifest.

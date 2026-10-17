# Learning track: a learned travel-time cost for task allocation

The contract is [docs/learning_track.md](../docs/learning_track.md). Section numbers below refer
to it. One step is one simulated second.

Status: Stage 5 (pibt). Two rounds, reported side by side: the Stage 4 round and the Stage 5
rerun. Each round's success criterion was committed before its final runs: commit `8133d47`
for Stage 4 and commit `6471ec6` for Stage 5.

## Result

The success criterion is not met in either round. At 25 robots and arrival rate 0.2, learned
minus tuned `cost_aware` has a median of -6.00 orders per hour in the Stage 4 round (95% interval
[-20.40, 7.20]) and -1.80 in the Stage 5 round ([-10.80, 8.40]). Both intervals contain 0.

In the Stage 4 round, `learned` won at 5 and 10 robots, but partly by serving the newest orders
first. In the Stage 5 round, without `order_age` as an input and with the starvation guard in
the core, those wins are gone: against tuned `cost_aware`, no interval in any cell lies entirely
above or below 0. Against `hungarian`, `learned` wins clearly only at 10 robots and rate 0.1.

## Question

When orders wait, the allocator matches free robots to orders with the Hungarian method. The
`learned` allocator uses a model's prediction of the steps a robot needs to reach the pickup,
traffic and station queues included, as the cost of a pair. On seeds the model never saw, does
it deliver more orders per hour than `cost_aware`, the hand-tuned congestion cost?

## Success criterion

The same in both rounds, written and committed before each round's final runs (section 9,
step 2):

> At 25 robots and arrival rate 0.2, the per-seed difference in `orders_per_hour` (learned minus
> tuned `cost_aware`, seeds 1-10) has a median whose 95% bootstrap interval lies entirely above 0.

The interval is the 2.5th to 97.5th percentile of the medians of 10,000 resamples of the 10
differences, drawn with replacement using `numpy.random.default_rng(0)`.

## What changed between the rounds

In the core (Stage 5, section 2): traffic rules for robots that run late, aisle closures, and the
starvation guard. The guard matches orders that have waited more than 600 steps first, oldest
first. `benchmarks/configs/default.yaml` keeps `execution.delay_probability: 0.0` and has no
closure events, so in these runs the guard and the new execution rules are what changed. Late
robots and closures were not exercised.

In the learning track (sections 6 and 9): `order_age` is not a model input, and `cost_aware` is
tuned on weights 1, 2, 4, 8, and 16 instead of 0.25 to 4. The margin filter is 1000 steps instead
of 900 (see Data). Everything else, including the grid, seeds, candidates, and protocol, is the
same.

## Data

Each round is one collection with `planning.method: pibt` on `maps/warehouse_small.txt`, from
`benchmarks/configs/default.yaml` with the section 7 grid: logging allocators `nearest`,
`hungarian`, `cost_aware`; arrival rates 0.05, 0.1, 0.2 orders per step; fleets 5, 10, 25; seeds
1000-1039. That is 1080 runs of 3600 steps per round. No run in either round aborted,
deadlocked, or froze.

| | Stage 4 | Stage 5 |
|---|---|---|
| collection commit | `430e08a9cd77` | `6c0e93405694` |
| assignments logged | 284028 | 282477 |
| longest kept `steps_to_pickup` (steps) | 446 | 457 |
| margin (steps) | 900 | 1000 |
| rows dropped by the margin | 69574 | 76710 |
| rows dropped as unreached | 0 | 0 |
| train rows, seeds 1000-1019 (540 runs) | 106334 | 102010 |
| validation rows, seeds 1020-1029 (270 runs) | 53595 | 51386 |
| offline test rows, seeds 1030-1039 (270 runs) | 54525 | 52371 |

From `python -m fleet_learning.dataset learning/data/<commit>/runs --margin <margin>` and the
`data` entry of `learning/models/cost_model_linear.metrics.json` (Stage 4) and
`learning/models/cost_model_s5_nocells.metrics.json` (Stage 5).

The margin is the smallest multiple of 100 that is at least twice the longest kept
`steps_to_pickup`, because 300 steps is not well above it (section 3). No aborted runs. No row
inside the margin has `steps_to_pickup = -1` in either round, so no group is near the 1%
threshold. In the Stage 4 round, margin 300 left 1 such row (hungarian, 25 robots).

Rows dropped by the margin filter and rows kept, per logging allocator and fleet size:

| Allocator | Robots | Stage 4 rows | Stage 4 margin | Stage 4 kept | Stage 5 rows | Stage 5 margin | Stage 5 kept |
|---|---:|---:|---:|---:|---:|---:|---:|
| cost_aware | 5 | 21369 | 5320 | 16049 | 21135 | 5817 | 15318 |
| cost_aware | 10 | 32186 | 7889 | 24297 | 31927 | 8641 | 23286 |
| cost_aware | 25 | 41970 | 10212 | 31758 | 41397 | 11114 | 30283 |
| hungarian | 5 | 21203 | 5229 | 15974 | 21055 | 5807 | 15248 |
| hungarian | 10 | 31705 | 7790 | 23915 | 31652 | 8632 | 23020 |
| hungarian | 25 | 41394 | 10072 | 31322 | 41110 | 11136 | 29974 |
| nearest | 5 | 20988 | 5222 | 15766 | 20988 | 5793 | 15195 |
| nearest | 10 | 31808 | 7758 | 24050 | 31808 | 8640 | 23168 |
| nearest | 25 | 41405 | 10082 | 31323 | 41405 | 11130 | 30275 |

From the `filters` entry of the same metrics files. `nearest` logs the same number of rows in
both rounds; its rows still differ, because the guard and the execution rules changed.

## Models

`train.py` fits every candidate on the training split and keeps, per family, the one with the
lowest validation MAE: both targets (`steps_to_pickup`, or the excess over `dist_to_pickup`)
and, for `mlp`, hidden layers 32, 64x64, and 64x64x64. Random seed 0. Each round sent three
deployable models to closed-loop tuning, built the same way:

| Variant | Stage 4 file | Stage 4 model | Validation MAE | Stage 5 file | Stage 5 model | Validation MAE |
|---|---|---|---:|---|---|---:|
| best deployable | `cost_model_v1.json` | mlp, direct, 32 | 8.64 | `cost_model_s5.json` | mlp, excess, 32 | 8.75 |
| linear | `cost_model_linear.json` | linear, direct | 10.38 | `cost_model_s5_linear.json` | linear, excess | 10.78 |
| no cells | `cost_model_nocells.json` | mlp, direct, 64x64 | 9.06 | `cost_model_s5_nocells.json` | mlp, direct, 64x64 | 9.10 |

Validation MAE in steps, from `"validation"` of each model's metrics file. Stage 4 models start
from all 19 feature columns, Stage 5 models from the 18 without `order_age`. The no-cells models
also drop `robot_row`, `robot_col`, `pickup_row`, and `pickup_col`. Tuning chose `cost_model_linear`
(Stage 4) and `cost_model_s5_nocells` (Stage 5).

## Offline test

Each round's offline test split, scored once, using the family winners from the deployed model's
metrics file: all 19 features in the Stage 4 round, the 14 no-cells features in the Stage 5
round. Errors in steps. Bias is predicted minus observed `steps_to_pickup`. Delayed rows have
`steps_to_pickup > dist_to_pickup`: 49934 of 54525 (Stage 4) and 47923 of 52371 (Stage 5).

| Round | Model | Candidate | MAE | RMSE | Bias | Delayed MAE | Delayed RMSE | Delayed bias |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Stage 4 | free_flow | free_flow | 20.76 | 36.04 | -20.76 | 22.66 | 37.66 | -22.66 |
| Stage 4 | linear (deployed) | linear_direct | 10.57 | 15.63 | 0.42 | 11.07 | 16.14 | 0.10 |
| Stage 4 | mlp | mlp_direct_32 | 8.86 | 13.79 | -0.30 | 9.19 | 14.25 | -0.81 |
| Stage 4 | gbdt (not deployable) | gbdt_excess | 8.88 | 13.86 | 0.01 | 9.23 | 14.34 | -0.46 |
| Stage 5 | free_flow | free_flow | 21.31 | 37.72 | -21.31 | 23.29 | 39.43 | -23.29 |
| Stage 5 | linear | linear_direct | 10.92 | 16.15 | 0.69 | 11.52 | 16.71 | 0.43 |
| Stage 5 | mlp (deployed) | mlp_direct_64x64 | 9.29 | 14.62 | -0.02 | 9.68 | 15.16 | -0.48 |
| Stage 5 | gbdt (not deployable) | gbdt_excess | 9.19 | 14.52 | 0.33 | 9.57 | 15.04 | -0.11 |

MAE by fleet size (robots), arrival rate (orders per step), logging allocator, and
`dist_to_pickup` (steps):

| Round | Model | n5 | n10 | n25 | r0.05 | r0.1 | r0.2 | cost_aware | hungarian | nearest | d0-5 | d6-10 | d11-20 | d21+ |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Stage 4 | rows | 12011 | 18274 | 24240 | 12537 | 19648 | 22340 | 18339 | 18093 | 18093 | 1525 | 4545 | 9917 | 38538 |
| Stage 4 | free_flow | 7.41 | 13.89 | 32.54 | 5.23 | 13.54 | 35.81 | 19.36 | 21.79 | 21.13 | 4.63 | 5.81 | 8.08 | 26.42 |
| Stage 4 | linear (deployed) | 7.25 | 9.94 | 12.70 | 5.90 | 9.71 | 13.95 | 10.05 | 10.99 | 10.68 | 3.37 | 4.97 | 7.26 | 12.37 |
| Stage 4 | mlp | 5.09 | 8.40 | 11.08 | 3.93 | 7.94 | 12.44 | 8.66 | 9.01 | 8.92 | 3.18 | 3.93 | 4.93 | 10.68 |
| Stage 4 | gbdt | 5.27 | 8.36 | 11.07 | 3.81 | 7.88 | 12.61 | 8.71 | 9.11 | 8.82 | 2.92 | 3.84 | 4.68 | 10.79 |
| Stage 5 | rows | 11502 | 17608 | 23261 | 12059 | 18937 | 21375 | 17576 | 17342 | 17453 | 1462 | 4394 | 9617 | 36898 |
| Stage 5 | free_flow | 7.67 | 13.86 | 33.70 | 5.23 | 13.72 | 37.12 | 20.39 | 22.43 | 21.13 | 4.67 | 5.85 | 8.16 | 27.24 |
| Stage 5 | linear | 7.88 | 10.13 | 13.03 | 5.91 | 10.07 | 14.50 | 10.51 | 11.33 | 10.94 | 3.77 | 5.01 | 7.17 | 12.89 |
| Stage 5 | mlp (deployed) | 5.51 | 8.64 | 11.65 | 3.98 | 8.18 | 13.26 | 9.07 | 9.59 | 9.20 | 3.15 | 3.99 | 5.13 | 11.24 |
| Stage 5 | gbdt | 5.44 | 8.60 | 11.49 | 3.83 | 8.09 | 13.18 | 8.94 | 9.54 | 9.09 | 2.93 | 3.83 | 4.79 | 11.22 |

From `python -m fleet_learning.evaluate learning/models/cost_model_linear.metrics.json` (Stage
4) and `python -m fleet_learning.evaluate learning/models/cost_model_s5_nocells.metrics.json`
(Stage 5).

In both rounds the free-flow cost predicts about 21 steps too few on average: most of a robot's
trip to the pickup is waiting, not driving. The best mlp and the gbdt reference are within 0.1
steps MAE of each other in both rounds, so a tree model would not help much. The two rounds use
different feature sets and margins, so their offline errors are not a like-for-like comparison.

## Closed-loop tuning

Seeds 2000-2009, fleets 5, 10, 25, arrival rates 0.1 and 0.2, `planning.method: pibt`: 60 runs
per candidate. Mean `orders_per_hour` (deliveries per simulated hour after the 600-step warmup).

Stage 4 round, from
`python -m fleet_learning.closed_loop report learning/data/9134958a33c1/closed_loop/tune`:

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

Stage 5 round, from
`python -m fleet_learning.closed_loop report learning/data/85a7438f2a57/closed_loop/tune`:

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

Chosen: `cost_aware` with weight 4 in both rounds; `learned` with `cost_model_linear.json`
(Stage 4) and `cost_model_s5_nocells.json` (Stage 5).

In the Stage 4 round, weight 4 was the top of the grid. In the Stage 5 round, the wider grid
still picks 4, and weights 8 and 16 do no better. All eight Stage 5 candidates are within 1.80
orders per hour of each other over all runs (298.14 to 299.94), against 8.22 in the Stage 4
round (301.14 to 309.36). In both rounds, offline accuracy did not rank the models the way the
closed loop did: the chosen model is not the one with the lowest validation MAE.

## Final evaluation

Seeds 1-10, fleets 5, 10, 25, arrival rates 0.1 and 0.2, `planning.method: pibt`, each
allocator run once: `hungarian`, `cost_aware` with weight 4, and the chosen `learned` model.
Nothing changed after these numbers came in. The Stage 4 order age table was reporting added
afterwards, from the same runs.

| Round | Final runs | Model | Model SHA-256 |
|---|---|---|---|
| Stage 4 | `learning/data/8133d475c5b3/closed_loop/final` | `cost_model_linear.json` | `fbf675a96ca3...` |
| Stage 5 | `learning/data/6471ec663788/closed_loop/final` | `cost_model_s5_nocells.json` | `f3617beeaa7f...` |

The full hashes are in each manifest. All tables in this section come from
`python -m fleet_learning.closed_loop report <final runs>`.

Orders per hour are deliveries per simulated hour after the 600-step warmup. Completion times are
the means over runs of each run's `mean_completion_steps` and `p95_completion_steps`, in steps.
Wall s is the wall time of all 10 runs, summed.

Stage 4 round:

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

Stage 5 round:

| Robots | Rate | Allocator | Runs | Orders/h mean | Orders/h median | Completion mean | Completion p95 | Deadlocked | Frozen | Wall s |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 5 | 0.1 | hungarian | 10 | 175.44 | 175.20 | 1295.7 | 1834.1 | 0 | 0 | 0.44 |
| 5 | 0.1 | cost_aware | 10 | 173.88 | 174.00 | 1295.6 | 1836.7 | 0 | 0 | 0.67 |
| 5 | 0.1 | learned | 10 | 173.28 | 172.80 | 1288.0 | 1853.4 | 0 | 0 | 0.80 |
| 5 | 0.2 | hungarian | 10 | 174.60 | 174.60 | 2381.7 | 2716.1 | 0 | 0 | 0.48 |
| 5 | 0.2 | cost_aware | 10 | 174.00 | 174.00 | 2384.4 | 2724.1 | 0 | 0 | 0.53 |
| 5 | 0.2 | learned | 10 | 174.24 | 174.60 | 2379.5 | 2716.3 | 0 | 0 | 2.13 |
| 10 | 0.1 | hungarian | 10 | 298.92 | 298.20 | 458.0 | 760.0 | 0 | 0 | 0.79 |
| 10 | 0.1 | cost_aware | 10 | 307.08 | 307.20 | 405.9 | 699.9 | 0 | 0 | 0.77 |
| 10 | 0.1 | learned | 10 | 310.68 | 310.80 | 392.0 | 733.2 | 0 | 0 | 0.97 |
| 10 | 0.2 | hungarian | 10 | 302.40 | 301.80 | 1531.4 | 2073.6 | 0 | 0 | 0.84 |
| 10 | 0.2 | cost_aware | 10 | 303.24 | 303.00 | 1524.3 | 2075.3 | 0 | 0 | 0.88 |
| 10 | 0.2 | learned | 10 | 301.08 | 303.00 | 1519.9 | 2073.3 | 0 | 0 | 2.25 |
| 25 | 0.1 | hungarian | 10 | 356.52 | 363.60 | 94.9 | 160.4 | 0 | 0 | 1.84 |
| 25 | 0.1 | cost_aware | 10 | 358.32 | 363.00 | 96.1 | 160.7 | 0 | 0 | 1.64 |
| 25 | 0.1 | learned | 10 | 357.84 | 363.00 | 95.4 | 163.1 | 0 | 0 | 1.61 |
| 25 | 0.2 | hungarian | 10 | 465.12 | 469.20 | 892.0 | 1294.0 | 0 | 0 | 1.80 |
| 25 | 0.2 | cost_aware | 10 | 477.72 | 477.00 | 853.8 | 1240.0 | 0 | 0 | 2.10 |
| 25 | 0.2 | learned | 10 | 477.00 | 474.00 | 832.4 | 1240.0 | 0 | 0 | 2.87 |

No run in either round deadlocked, froze, or aborted. In every cell except 25 robots at rate 0.1,
orders pile up: the baselines assign them a median 187 to 1337 steps after they arrive (order age
table below). Completion times then depend on which orders get served, and they only count
delivered orders. The short Stage 4 `learned` completion times are not faster service; see the
order age table. In the Stage 5 round, the three allocators have completion times within 3% of
each other at 5 robots and at 10 robots with rate 0.2.

At 25 robots and rate 0.1, `hungarian` and `cost_aware` give the same means in both rounds. That
cell is below capacity, so no order waits 600 steps and the guard never acts.

### Paired differences

Learned minus each baseline in `orders_per_hour`, paired by seed (a seed gives the same orders
under every allocator). Median over the 10 seeds, with a 95% percentile bootstrap interval:
10,000 resamples with `numpy.random.default_rng(0)`. W/T/L is wins, ties, losses.

learned minus `hungarian`:

| Robots | Rate | Stage 4 W/T/L | Stage 4 median | Stage 4 95% interval | Stage 5 W/T/L | Stage 5 median | Stage 5 95% interval |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 5 | 0.1 | 10/0/0 | 5.40 | [3.60, 7.20] | 2/0/8 | -2.40 | [-4.80, 0.60] |
| 5 | 0.2 | 10/0/0 | 9.00 | [7.20, 10.80] | 4/0/6 | -1.20 | [-4.80, 3.00] |
| 10 | 0.1 | 10/0/0 | 13.80 | [10.80, 18.00] | 10/0/0 | 12.60 | [7.80, 15.00] |
| 10 | 0.2 | 9/0/1 | 18.60 | [13.20, 20.40] | 4/2/4 | 0.00 | [-4.80, 3.60] |
| 25 | 0.1 | 6/3/1 | 1.20 | [0.00, 2.40] | 6/3/1 | 1.20 | [0.00, 3.00] |
| 25 | 0.2 | 7/0/3 | 8.40 | [-3.60, 20.40] | 7/1/2 | 14.40 | [0.00, 21.60] |

learned minus tuned `cost_aware`:

| Robots | Rate | Stage 4 W/T/L | Stage 4 median | Stage 4 95% interval | Stage 5 W/T/L | Stage 5 median | Stage 5 95% interval |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 5 | 0.1 | 9/1/0 | 2.40 | [1.20, 4.80] | 3/1/6 | -1.20 | [-3.60, 2.40] |
| 5 | 0.2 | 9/0/1 | 7.20 | [3.60, 10.80] | 5/1/4 | 0.60 | [-3.00, 2.40] |
| 10 | 0.1 | 8/1/1 | 4.20 | [0.60, 12.60] | 7/0/3 | 4.20 | [-1.20, 8.40] |
| 10 | 0.2 | 9/0/1 | 6.00 | [3.60, 12.00] | 6/0/4 | 1.20 | [-9.60, 3.60] |
| 25 | 0.1 | 3/2/5 | -0.60 | [-2.40, 1.20] | 3/2/5 | -0.60 | [-2.40, 1.20] |
| 25 | 0.2 | 4/0/6 | -6.00 | [-20.40, 7.20] | 4/0/6 | -1.80 | [-10.80, 8.40] |

Criterion (25 robots, rate 0.2, interval above 0 against `cost_aware`): not met in either round.
Both medians are negative and both intervals contain 0, so neither round shows a difference in
either direction at that cell.

The Stage 4 wins at 5 and 10 robots, with intervals above 0 against both baselines, do not come
back in the Stage 5 round. Against `cost_aware`, every Stage 5 interval contains 0. Two things
changed at once: the model lost `order_age`, and the core gained the starvation guard. These runs
cannot say how much of the change each one caused.

### Selection bias check

`model_io` costs for the pairs `learned` chose in the final runs, against their
`steps_to_pickup`, after the section 3 filters with each round's training margin (900 and 1000
steps). Errors in steps; bias is predicted minus observed.

| Rows from | Stage 4 rows | Stage 4 MAE | Stage 4 bias | Stage 5 rows | Stage 5 MAE | Stage 5 bias |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| offline test | 54525 | 10.57 | 0.42 | 52371 | 9.29 | -0.02 |
| final learned runs | 14158 | 10.77 | -2.20 | 13375 | 10.45 | -1.69 |
|   n5 r0.1 | 1393 | 7.03 | -5.52 | 1292 | 5.24 | -0.32 |
|   n5 r0.2 | 1432 | 7.53 | -6.42 | 1308 | 5.23 | -0.05 |
|   n10 r0.1 | 2370 | 9.37 | -1.03 | 2283 | 8.34 | -1.13 |
|   n10 r0.2 | 2435 | 9.47 | -4.04 | 2254 | 10.08 | -0.65 |
|   n25 r0.1 | 2745 | 8.88 | 1.84 | 2632 | 7.99 | -2.98 |
|   n25 r0.2 | 3783 | 16.48 | -1.85 | 3606 | 17.57 | -2.85 |

In both rounds the MAE on the model's own choices is within 1.2 steps of the offline test (10.77
against 10.57, and 10.45 against 9.29), so the model is not badly fooled overall. The bias is
negative in both: the pairs it chose take longer than it predicted, which is the expected
direction for a model that picks the pairs it scores lowest. At 25 robots and rate 0.2 the MAE is
16.48 and 17.57, against 12.70 and 11.65 for all 25-robot rows offline (all three rates, so not a
like-for-like cell). That is the cell the criterion is about, and the one where the model is
least accurate on its own choices.

### Order age at assignment

`order_age` at assignment, in steps, over every assignment of the final runs:

| Robots | Rate | Allocator | Stage 4 assignments | Stage 4 median | Stage 4 p90 | Stage 4 max | Stage 5 assignments | Stage 5 median | Stage 5 p90 | Stage 5 max |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 5 | 0.1 | hungarian | 1803 | 793 | 1681 | 2431 | 1791 | 896 | 1660 | 1973 |
| 5 | 0.1 | cost_aware | 1825 | 826 | 1640 | 2335 | 1780 | 892 | 1653 | 1987 |
| 5 | 0.1 | learned | 1852 | 45 | 211 | 897 | 1778 | 885 | 1662 | 1974 |
| 5 | 0.2 | hungarian | 1807 | 1314 | 2453 | 3041 | 1790 | 1337 | 2462 | 2787 |
| 5 | 0.2 | cost_aware | 1830 | 1316 | 2418 | 2959 | 1793 | 1327 | 2457 | 2816 |
| 5 | 0.2 | learned | 1888 | 35 | 120 | 589 | 1791 | 1328 | 2452 | 2826 |
| 10 | 0.1 | hungarian | 3034 | 223 | 647 | 1133 | 3026 | 244 | 619 | 773 |
| 10 | 0.1 | cost_aware | 3115 | 187 | 557 | 891 | 3102 | 194 | 589 | 709 |
| 10 | 0.1 | learned | 3172 | 28 | 218 | 2304 | 3132 | 133 | 614 | 712 |
| 10 | 0.2 | hungarian | 3072 | 962 | 1850 | 2559 | 3095 | 1018 | 1868 | 2176 |
| 10 | 0.2 | cost_aware | 3146 | 981 | 1818 | 2401 | 3100 | 1017 | 1862 | 2153 |
| 10 | 0.2 | learned | 3219 | 30 | 126 | 696 | 3095 | 1009 | 1863 | 2188 |
| 25 | 0.1 | hungarian | 3612 | 0 | 0 | 12 | 3612 | 0 | 0 | 12 |
| 25 | 0.1 | cost_aware | 3612 | 0 | 0 | 69 | 3612 | 0 | 0 | 69 |
| 25 | 0.1 | learned | 3612 | 0 | 0 | 6 | 3612 | 0 | 0 | 36 |
| 25 | 0.2 | hungarian | 4881 | 512 | 1090 | 1764 | 4820 | 604 | 1086 | 1441 |
| 25 | 0.2 | cost_aware | 5003 | 469 | 1057 | 1470 | 4915 | 601 | 1044 | 1351 |
| 25 | 0.2 | learned | 4970 | 23 | 143 | 1000 | 4909 | 615 | 1048 | 1386 |

In the Stage 4 round, when orders piled up, `learned` assigned orders a median 23 to 45 steps
after they arrived, and the baselines 469 to 1316 steps after. The linear model gave `order_age`
a weight of +0.0014 steps per step of age (from `learning/models/cost_model_linear.json`, weight
over scale). That is small, but with hundreds of pending orders at almost the same distance it
decided the order, so the newest orders went first and the oldest waited.

In the Stage 5 round, `learned` assigns orders at about the same age as the baselines: medians
within 14 steps of `cost_aware` in every overloaded cell except 10 robots at rate 0.1 (133 against
194). The largest ages also drop in every overloaded baseline cell, for example from 3041 to 2787 steps for
`hungarian` at 5 robots and rate 0.2. In overloaded cells most orders wait past 600 steps, so the
guard serves them oldest first and the median age stays high: it bounds how far an order can be
passed over, not how long the queue is.

## Limits

- One map (`warehouse_small`) and one planner (`pibt`). Faults (Stage 6) are not in yet, and the
  protocol must be rerun after it lands.
- `default.yaml` keeps late robots (`execution.delay_probability`) and closures off, and the
  section 7 grid does not change them. So the Stage 5 round tests the starvation guard and the
  new execution rules, not late robots or closed aisles.
- Between the rounds, the model lost `order_age` and the core gained the guard. The runs cannot
  separate the two effects on the Stage 4 wins at 5 and 10 robots.
- Selection bias: the training data comes from `nearest`, `hungarian`, and `cost_aware`, never
  from `learned`. The negative bias on its own choices in both rounds is a sign of that.
- Offline accuracy did not predict closed-loop throughput in either round. The Stage 5 tuning
  candidates are within 1.80 orders per hour of each other, so the choice among them is close to
  noise at 60 runs each.
- Most cells are overloaded, so `orders_per_hour` is close to the fleet's capacity and the
  completion times mostly measure queue order. Only 25 robots at rate 0.1 is below capacity, and
  there the three allocators are within 2 orders per hour of each other in both rounds.
- 10 seeds per cell. At 25 robots and rate 0.2, the intervals against `cost_aware` are 27.60
  (Stage 4) and 19.20 (Stage 5) orders per hour wide.
- The Stage 4 model used `order_age`, which changes which orders get served, not only how long a
  trip takes. The question in section 1 is about trip cost; the Stage 5 model does not read it.

## How to reproduce

From the repo root, after the setup in section 6 and the build in CONTRIBUTING.md. Each
collection folder is named after the commit it ran at, so collect at that commit.
`learning/data/` is ignored by git and stays across checkouts. Closed-loop phases write under
`learning/data/<commit>/closed_loop/<phase>`, where `<commit>` is the checked-out commit; the
numbers do not depend on it.

Stage 5 round:

```bash
P=learning/.venv/bin/python

# Data, at the Stage 5 commit.
git checkout 6c0e93405694
$P -m fleet_learning.collect
git checkout -

# Models (about 2.5 minutes each).
$P -m fleet_learning.train --data learning/data/6c0e93405694 --name cost_model_s5 --margin 1000
$P -m fleet_learning.train --data learning/data/6c0e93405694 --name cost_model_s5_linear \
  --margin 1000 --export linear
$P -m fleet_learning.train --data learning/data/6c0e93405694 --name cost_model_s5_nocells \
  --margin 1000 --export mlp --features dist_to_pickup,manhattan_to_pickup,\
dist_pickup_to_dropoff,robot_on_endpoint,robots_near_robot,robots_near_pickup,robots_in_box,\
box_free_cells,pickup_assigned,pickup_pending,pending_orders,available_robots,busy_robots,\
alive_robots

# Offline tables and filter counts.
$P -m fleet_learning.evaluate learning/models/cost_model_s5_nocells.metrics.json
$P -m fleet_learning.dataset learning/data/6c0e93405694/runs --margin 1000

# Closed-loop tuning on weights 1, 2, 4, 8, 16 (the default).
$P -m fleet_learning.closed_loop tune --models learning/models/cost_model_s5.json \
  learning/models/cost_model_s5_linear.json learning/models/cost_model_s5_nocells.json

# Final runs, once, with the tuned choices, then the tables again.
$P -m fleet_learning.closed_loop final --weight 4 \
  --model learning/models/cost_model_s5_nocells.json
$P -m fleet_learning.closed_loop report learning/data/<commit>/closed_loop/final
```

Stage 4 round. Since the Stage 5 rerun, `train.py` rejects `order_age` and `closed_loop tune`
defaults to weights 1 to 16, so check out `5e92202` (the last Stage 4 round commit) for the
learning code:

```bash
P=learning/.venv/bin/python

# Data, at the Stage 4 commit.
git checkout 430e08a9cd77
$P -m fleet_learning.collect

# Models, tuning, and final runs with the Stage 4 round code.
git checkout 5e92202
$P -m fleet_learning.train --data learning/data/430e08a9cd77 --name cost_model_v1 --margin 900
$P -m fleet_learning.train --data learning/data/430e08a9cd77 --name cost_model_linear \
  --margin 900 --export linear
$P -m fleet_learning.train --data learning/data/430e08a9cd77 --name cost_model_nocells \
  --margin 900 --export mlp --features dist_to_pickup,manhattan_to_pickup,\
dist_pickup_to_dropoff,robot_on_endpoint,robots_near_robot,robots_near_pickup,robots_in_box,\
box_free_cells,pickup_assigned,pickup_pending,pending_orders,available_robots,busy_robots,\
alive_robots,order_age
$P -m fleet_learning.evaluate learning/models/cost_model_linear.metrics.json
$P -m fleet_learning.dataset learning/data/430e08a9cd77/runs --margin 900
$P -m fleet_learning.closed_loop tune --models learning/models/cost_model_v1.json \
  learning/models/cost_model_linear.json learning/models/cost_model_nocells.json
$P -m fleet_learning.closed_loop final --weight 4 --model learning/models/cost_model_linear.json
$P -m fleet_learning.closed_loop report learning/data/<commit>/closed_loop/final
git checkout -
```

`5e92202` sits on the Stage 4 core, and `closed_loop` builds the checked-out C++ tools before
it runs, so the Stage 4 closed-loop runs use the Stage 4 core. Every number except wall time
comes out the same on a clean checkout. The SHA-256 of each model file is in the closed-loop
manifests.

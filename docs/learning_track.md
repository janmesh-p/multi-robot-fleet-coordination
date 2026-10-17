# Learning track: a learned travel-time cost for task allocation

This file is the contract between the C++ core and `learning/`. The repo owner maintains it. To
change it, open an issue titled `learning: <what you need>`. Read
[CONTRIBUTING.md](../CONTRIBUTING.md) first.

This is version 1. Stage 3 implements sections 3 and 4, and core tests check the example row,
the worked example, and the free-flow replay. If the core ever has to differ, this file changes
in the same commit, and the commit message says what changed.

## 1. Goal

When orders are waiting, the allocator decides which free robot fetches which order. Stage 3 adds
three allocators that share one solver, the Hungarian method (minimum total cost over all
robot-order pairs), and differ only in the cost of a pair:

| Allocator | Cost of a robot-order pair |
|---|---|
| `hungarian` | `dist_to_pickup`: shortest-path steps to the pickup, as if no other robot existed |
| `cost_aware` | `dist_to_pickup + congestion_weight * (robots_in_box + service_steps * pickup_assigned)`, a hand-tuned congestion penalty; plain `dist_to_pickup` when `features.congestion_cost` is off |
| `learned` | a model's prediction of the steps the robot will really need to reach the pickup, traffic and station queues included |

From Stage 5 all three share a starvation guard (`features.starvation_guard`, on by default):
orders that have waited more than 600 steps are matched first, oldest first, and the other
orders after them. The cost of a pair still decides which robot takes each order.

The learning track builds the `learned` cost: a small regression model trained on logged runs.
The question: on seeds the model never saw, does it deliver more orders per hour than the
hand-tuned cost? Either answer is a result, and both get reported the same way.

## 2. What the core provides, stage by stage

| Stage | What you get |
|---|---|
| 2 (done) | `run_sim` and `run_sweep`, with `nearest` allocation and `independent_astar` planning only. Every run deadlocks within minutes. No per-order log yet. |
| 3 (done) | Allocators `hungarian`, `cost_aware`, `learned`; config key `allocation.model_file`; `assignments.csv` in every run directory; the `predict_cost` tool. Planning is still `independent_astar`, so runs still deadlock. Use this data to test your plumbing, not to train. |
| 4 (done) | Coordinated planners, including `pibt`. With `pibt` the fleet keeps moving: no frozen run and no safety stop in its 69 runs of the planning benchmark in README.md. One rule changed with them: a robot whose station has no free waiting cell waits at home instead of where it stands. This is the first data worth training on. |
| 5 (done) | Traffic rules that keep plans safe when robots run late (`execution.delay_probability`), aisle closures (`block_cells` and `unblock_cells` events), and the starvation guard (section 1). Under heavy load the guard changes which pairs get chosen, and `dist_to_pickup` now routes around closed cells. No column or config key changed. Recollect, retrain, and rerun the evaluation (section 11). |
| 6 | Faults. They change how robots move, so recollect, retrain, and rerun the evaluation after it lands. |

The README Stages table says which stages have landed.

## 3. Data: `assignments.csv`

From Stage 3, every run directory (from `run_sim`, or `run_sweep --runs`) holds
`assignments.csv`: one row per assignment the coordinator made, sorted by `t`, then `robot`.
Every value is an integer except `end_status`. One step is one simulated second. The header, in
this exact order, and one example row from `maps/warehouse_small.txt`:

```
t,order,robot,dist_to_pickup,manhattan_to_pickup,dist_pickup_to_dropoff,robot_row,robot_col,pickup_row,pickup_col,robot_on_endpoint,robots_near_robot,robots_near_pickup,robots_in_box,box_free_cells,pickup_assigned,pickup_pending,pending_orders,available_robots,busy_robots,alive_robots,order_age,arrived_t,steps_to_pickup,delivered_t,end_status
612,57,3,14,14,39,0,9,5,0,1,2,1,2,39,1,0,3,4,6,10,5,631,20,674,delivered
```

Each step `t` of a run does this, in order: orders arrive, robots report, the coordinator ticks
and may assign, and robots move one cell. Row `t` of `positions.csv` holds positions after that
move.

### Identification

| Column | Meaning |
|---|---|
| `t` | step whose coordinator tick made the assignment |
| `order` | order id |
| `robot` | robot id |

### Features

The coordinator computes the features at step `t`, from its belief (the robots' last reports),
before it makes any assignment in that tick. Three terms:

- other robots: robots believed alive, not counting this robot
- near: Manhattan distance 3 or less
- box: the rectangle with the robot's cell and the pickup cell as opposite corners, edges included

| Column | Meaning |
|---|---|
| `dist_to_pickup` | shortest-path steps from the robot's cell to the pickup, ignoring other robots, around the cells closed at step `t` (Stage 5; without closures, the static map). A path may start or end on a station or parking cell but never pass through one, the same rule the planners follow. Manhattan distance when `features.distance_oracle` is off. |
| `manhattan_to_pickup` | absolute row difference plus absolute column difference |
| `dist_pickup_to_dropoff` | shortest-path steps from the pickup to the dropoff, same rule as `dist_to_pickup` |
| `robot_row`, `robot_col` | the robot's cell |
| `pickup_row`, `pickup_col` | the pickup cell |
| `robot_on_endpoint` | 1 if the robot is on a station or parking cell, else 0 |
| `robots_near_robot` | other robots near the robot |
| `robots_near_pickup` | other robots near the pickup |
| `robots_in_box` | other robots inside the box |
| `box_free_cells` | cells inside the box that are not shelves |
| `pickup_assigned` | orders at the same pickup that are assigned but not yet picked up |
| `pickup_pending` | other unassigned orders at the same pickup |
| `pending_orders` | unassigned orders, this one included |
| `available_robots` | robots this tick could assign: believed alive and holding no order |
| `busy_robots` | robots believed alive that hold an order |
| `alive_robots` | robots believed alive |
| `order_age` | `t` minus the step the order arrived |

### Outcomes

These come from ground truth, recorded as the run goes.

| Column | Meaning |
|---|---|
| `arrived_t` | first step at or after `t` whose move ends with the robot on the pickup cell; `t - 1` if the robot already stood there when assigned; -1 if it never gets there |
| `steps_to_pickup` | `arrived_t - t + 1` (0 if the robot already stood there), or -1. The training target. |
| `delivered_t` | step in which the service at the dropoff ends and the order counts as delivered (the `deliveries` column of `steps.csv` counts it in that step); -1 if the order is not delivered |
| `end_status` | state of this assignment when the run ends: `delivered`, `picked_up`, `assigned`, or `requeued` |

What the core guarantees, so your tests can rely on it:

- A robot that nothing slows down has `steps_to_pickup` equal to `dist_to_pickup`. Anything above
  that is traffic, queueing, and missed steps. So `steps_to_pickup >= dist_to_pickup` on every
  row where `steps_to_pickup` is not -1, as long as no robot reports are lost
  (`faults.packet_loss: 0` and no `comm_outage` events) and no closed cell reopens on the way
  (no `unblock_cells` event), the defaults. In the example row, the robot lost 6 steps.
- `available_robots + busy_robots == alive_robots` on every row.
- An order requeued after its robot dies (Stage 6) ends that row as `requeued` and gets a new row
  when it is assigned again.
- The column set and order are fixed for this version. A stage that adds a column says so here,
  and your schema check must fail loudly until you update it.

### Rows to drop before training

- Runs whose `summary.csv` has `aborted` = `true`. A safety violation stopped the run. Open an
  issue with the run name and config.
- Rows with `steps_to_pickup` = -1.
- Rows with `t > steps_run - 300`, where `steps_run` comes from `summary.csv`. Late assignments
  that take long get cut off by the end of the run, which biases the target low. Check that the
  longest `steps_to_pickup` you keep is well under 300. If it is not, raise the margin.

Count the rows each filter drops, per logging allocator and fleet size, and put the counts in the
metrics file. Rows with -1 that the margin filter did not catch are robots that never reached
the pickup, usually because they were stuck. Dropping them hides the worst cases, so if they are
more than 1% of the rows in any group, say so in the write-up.

## 4. Model file: `fleet_cost_model`, version 1

The `learned` allocator reads one JSON file. It is the only thing the C++ side knows about your
model.

```json
{
  "format": "fleet_cost_model",
  "version": 1,
  "target": "steps_to_pickup",
  "features": ["dist_to_pickup", "robots_in_box", "pickup_assigned"],
  "mean": [14.2, 1.3, 0.9],
  "scale": [7.5, 1.6, 1.1],
  "layers": [
    {"weights": [[0.8, 0.1, 0.3], [0.2, -0.4, 0.6]], "bias": [0.1, -0.2], "activation": "relu"},
    {"weights": [[1.5, 0.7]], "bias": [0.4], "activation": "identity"}
  ],
  "base_feature": "dist_to_pickup",
  "info": {"notes": "free-form; the C++ side ignores this key"}
}
```

How the C++ side turns one row into a cost, in double precision. Your Python reference must do
exactly the same.

1. `x[i] = (row[features[i]] - mean[i]) / scale[i]`
2. Start from `h = x`. For each layer in order, `h = activation(weights @ h + bias)`, where `@`
   is the matrix-vector product. `weights` has one row per output unit, and each row has one
   entry per input. `relu(v) = max(v, 0)`, `identity(v) = v`.
3. `y = h[0]`. If `base_feature` is not `null`, add the raw, unscaled `row[base_feature]`.
4. The cost is `y` clamped to `[0, 100000]`. A `y` that is not a number, which only an overflow
   can produce, counts as 100000.

Worked example: for the row `dist_to_pickup = 20`, `robots_in_box = 3`, `pickup_assigned = 1`,
the model above gives a cost of 21.678284091. Your Python reference should reproduce that.

The C++ side rejects a file at startup, naming the field, unless all of this holds:

- Every key in the example is present except `info`, which is optional. No other top-level key
  is allowed.
- `format`, `version`, and `target` are exactly as above.
- `features` has 1 to 32 distinct names from the feature table in section 3. Identification and
  outcome columns are not allowed.
- `mean` and `scale` have one entry per feature, and every `scale` entry is greater than 0.
- `layers` has 1 to 4 layers. Each layer has exactly the keys `weights`, `bias`, and
  `activation`. Every row of `weights` has the same length, and `bias` has one entry per row.
  Every layer but the last uses `relu`. The last uses `identity` and has exactly one output. No
  layer has more than 64 outputs. The first layer takes `len(features)` inputs, and each later
  layer takes as many inputs as the previous one has outputs.
- `base_feature` is `null` or a name from the feature table. It does not have to be in
  `features`.
- Every number is finite. Write the file with `json.dump(..., allow_nan=False)`.

Two models you can write by hand. Both make good tests.

- Free flow: `"features": ["dist_to_pickup"]`, `"mean": [0]`, `"scale": [1]`, one identity layer
  with `"weights": [[0]]` and `"bias": [0]`, and `"base_feature": "dist_to_pickup"`. Its cost is
  exactly `dist_to_pickup`, so `learned` with this file produces the same runs as `hungarian`,
  down to `log_hash`. The core has a test for this. Your tests should check it too.
- Linear regression: a single identity layer with one output.

### `predict_cost` (Stage 3)

Runs the C++ cost on every row of any CSV that has the columns the model reads (`features`, and
`base_feature` if set). Other columns are ignored.

```bash
./build/predict_cost --model learning/models/cost_model_v1.json --rows path/to/assignments.csv
```

It prints a `row,prediction` header, then one line per data row (`row` counts from 0, empty
lines are skipped, the prediction is printed with `%.17g`). Exit codes: 0 done, 1 bad model or
rows, 2 bad arguments.
Python and C++ must agree within `1e-6` on every row.

### Config (Stage 3)

```yaml
allocation:
  method: learned
  model_file: /absolute/path/to/model.json  # a relative path resolves against the config file's folder
```

`model_file` is required with `learned`. Setting it with any other method is an error.

## 5. Your folder

```
learning/
  README.md          results and how to reproduce them (written last)
  pyproject.toml
  requirements.txt   exact pins
  .gitignore         data/
  fleet_learning/
    __init__.py
    schema.py        assignments.csv v1 columns, dtypes, and checks
    dataset.py       run directories to one table with run metadata; filters; seed splits
    model_io.py      write, read, and check model files; numpy reference prediction
    train.py         fit the candidate models; export the chosen one
    evaluate.py      offline metrics and tables
    collect.py       write sweep configs, build the C++ tools, run run_sweep
    closed_loop.py   closed-loop sweeps and paired comparisons
  models/            committed model files and their metrics files
  figures/           committed figures used in learning/README.md
  tests/
    fixtures/        one tiny run directory
  data/              ignored by git: generated configs, runs, datasets
```

Each module that does work has a command line: `python -m fleet_learning.<module> --help`. Run
every command from the repo root.

## 6. Python rules

- One Python version everywhere: the one on your machine, 3.10 or newer. Put the same version in
  `learning.yml`, so the pins resolve the same way in CI.
- Dependencies: numpy, pandas, scikit-learn, matplotlib, PyYAML; for development, pytest and
  ruff. `requirements.txt` pins every package exactly. No deep learning framework: the deployed
  model is a small MLP, and scikit-learn trains it in seconds.
- Start `pyproject.toml` from this:

  ```toml
  [build-system]
  requires = ["setuptools>=64"]
  build-backend = "setuptools.build_meta"

  [project]
  name = "fleet-learning"
  version = "0.1.0"
  requires-python = ">=3.10"

  [tool.setuptools]
  packages = ["fleet_learning"]

  [tool.ruff]
  line-length = 100
  target-version = "py310"

  [tool.ruff.lint]
  select = ["E", "F", "W", "I", "B", "UP", "NPY"]

  [tool.pytest.ini_options]
  markers = ["core: needs the C++ build; set FLEET_BUILD to its folder"]
  ```

- First setup, once `pyproject.toml` and `fleet_learning/__init__.py` exist:

  ```bash
  python3 -m venv learning/.venv
  learning/.venv/bin/pip install numpy pandas scikit-learn matplotlib pyyaml pytest ruff
  learning/.venv/bin/pip freeze --exclude-editable > learning/requirements.txt
  learning/.venv/bin/pip install -e learning
  ```

- Every later setup, and the checks to run before each push:

  ```bash
  python3 -m venv learning/.venv
  learning/.venv/bin/pip install -r learning/requirements.txt -e learning
  learning/.venv/bin/ruff check learning
  learning/.venv/bin/ruff format --check learning
  learning/.venv/bin/python -m pytest learning/tests -m "not core"
  ```

- Type hints on public functions. Docstrings give units (steps, cells).
- Every random choice takes a seed argument with a fixed default. Record the seeds in the
  metrics file.
- Read CSV files with explicit dtypes. Check the exact column list of `assignments.csv` against
  `schema.py`. Read `summary.csv` and `config.yaml` by name and ignore columns and keys you do
  not use, because they grow between stages.
- Model inputs come only from the feature columns of `assignments.csv`. Never build inputs from
  `positions.csv`, run metadata (fleet size, arrival rate, allocator), or any other source: the
  C++ side cannot compute those at decision time, so the deployed model would see different
  inputs than it trained on. Run metadata is for filtering and grouping only.
- From the Stage 5 rerun on, `order_age` is not a model input. It is the same for every robot
  that could take an order, so it cannot say which robot gets there first. It can only move
  orders up or down the queue, and old orders tend to wait at busy pickups, so a model that reads
  it learns to push them back. The C++ side still accepts it, so Stage 4 models still load.
- No network access in code or tests. The unit tests finish in under 60 seconds.
- Notebooks are fine for exploring, but never the source of a reported number.

## 7. Collecting data

Everything one collection writes goes under `learning/data/<commit>/`, where `<commit>` is the
first 12 characters of `git rev-parse HEAD`. Data from different stages never mixes.

`collect.py` does this, in order:

1. Refuses to run when `git status --porcelain` prints anything, unless `--allow-dirty` is given.
   Records the commit.
2. Builds the C++ tools: `cmake -S fleet_core -B build -G Ninja` and `cmake --build build`.
3. Writes one config per cell of the grid below into `learning/data/<commit>/configs/`. Each
   starts from `benchmarks/configs/default.yaml` and changes only the keys in the grid, plus one
   rewrite: `map.file` becomes an absolute path, because a relative path resolves against the
   folder of the config that holds it. Do the same for `allocation.model_file` in closed-loop
   configs.
4. For each config, runs:

   ```bash
   ./build/run_sweep --config <config> --fleet 5,10,25 --seeds 1000-1039 \
     --out learning/data/<commit>/runs/<run.name> --runs
   ```

   That covers the training, validation, and offline test seeds in one go (see Seeds below).
   `collect.py --fleet` and `--seeds` override those two lists, for plumbing checks. Several
   sweeps can run in parallel, at most one per CPU core. If a sweep exits with anything but 0,
   stop and print its command and output. Each sweep writes `sweep.csv` and `aggregate.csv` into
   its `--out` folder, plus one run directory per run, named `<run.name>_n<fleet size>_s<seed>`.
5. Deletes `positions.csv` and `steps.csv` from the run directories (about 3.4 MB per 25-robot
   run), unless `--keep-traces` is given.
6. Writes `learning/data/<commit>/manifest.json`: commit, configs, seeds, fleet sizes, wall time.

| Key | Values |
|---|---|
| `allocation.method` | `nearest`, `hungarian`, `cost_aware` |
| `planning.method` | `pibt` once Stage 4 lands; `independent_astar` before that, for plumbing only |
| `orders.arrival_rate` | 0.05, 0.1, 0.2 |
| `run.name` | `collect_<method>_r<rate x 100, three digits>`, for example `collect_hungarian_r010` |

Name closed-loop configs the same way: `tune_cost_aware_w050_r020` (weight times 100, at least
three digits), `tune_learned_<model>_r020` (`<model>` is the model file name without `.json`),
`final_learned_r010`.

Logging with three allocators spreads the data over different robot-order choices. The `learned`
cost still has to score pairs that no logging allocator chose. That selection bias is a known
limit: measure it (section 9) and state it in the write-up.

Time one run before collecting everything. If the full grid would take more than an hour on your
machine, drop arrival rate 0.05 first.

### Seeds

| Use | Seeds |
|---|---|
| training data | 1000-1019 |
| validation data: model and hyperparameter choice | 1020-1029 |
| offline test, reported once | 1030-1039 |
| closed-loop tuning: the `cost_aware` weight and the final model | 2000-2009 |
| closed-loop final evaluation, reported once | 1-10 |
| tests and plumbing checks | 3000 and up |

Split by seed, never by row: rows from the same run are correlated. Never use a seed for two
purposes.

## 8. Training and offline evaluation

Fit on the training split, choose on the validation split, report on the offline test split.

| Model | What it is | Deployable |
|---|---|---|
| `free_flow` | predicts `dist_to_pickup`; nothing to fit | yes |
| `linear` | least squares on standardized features, exported as one identity layer | yes |
| `mlp` | scikit-learn `MLPRegressor`, ReLU, at most 3 hidden layers of at most 64 units, fixed `random_state` | yes |
| `gbdt` | scikit-learn `HistGradientBoostingRegressor` | no, offline reference only: format v1 cannot hold trees |

Target: either fit `steps_to_pickup` directly, or fit the excess `steps_to_pickup -
dist_to_pickup` and export with `"base_feature": "dist_to_pickup"`. Try both on validation and
keep the better one.

Export details that are easy to get wrong:

- scikit-learn stores `coefs_[k]` with shape (inputs, outputs). The model file wants
  (outputs, inputs), so transpose.
- `StandardScaler.mean_` and `StandardScaler.scale_` go into `mean` and `scale`.
- If you scale the target, fold that into the last layer before export: for `target = s * y + m`,
  write `s * weights` and `s * bias + m`.
- After export, read the file back. On the test split, the `model_io` cost must equal the
  scikit-learn prediction (plus the base feature, if you used one, and clamped to `[0, 100000]`)
  within `1e-9`.

Report these on the offline test split, for every model, in steps: MAE, RMSE, and mean error
(bias); the same three on rows where `steps_to_pickup > dist_to_pickup`; and MAE by fleet size,
by arrival rate, by logging allocator, and by `dist_to_pickup` bucket (0-5, 6-10, 11-20, 21 and
up).

`train.py` writes `learning/models/<name>.json` and `learning/models/<name>.metrics.json`. The
metrics file records the data commit, the seeds of each split, rows per split after filtering,
the rows each filter dropped, features, hyperparameters, random seeds, and validation and test
metrics.

## 9. Closed-loop evaluation

Offline accuracy is a means. Orders delivered per hour is the result.

1. Tuning, on seeds 2000-2009, fleets 5, 10, and 25, arrival rates 0.1 and 0.2, and
   `planning.method: pibt`:
   - `cost_aware`: try `allocation.congestion_weight` 1, 2, 4, 8, and 16 (the Stage 4 round
     tried 0.25 to 4). Keep the weight
     with the highest mean `orders_per_hour` over all tuning runs. The hand-tuned baseline gets
     the same tuning budget as your model.
   - `learned`: try at most 5 exported models. Keep the one with the highest mean
     `orders_per_hour`.
2. Before the final runs, write the success criterion into `learning/README.md` and commit it:
   at 25 robots and arrival rate 0.2, the per-seed difference in `orders_per_hour` (learned
   minus tuned `cost_aware`, seeds 1-10) has a median whose 95% bootstrap interval lies entirely
   above 0.
3. Final runs, on seeds 1-10 with the same fleets and arrival rates: `hungarian`, tuned
   `cost_aware`, and the chosen `learned`, once each. Change nothing after seeing these numbers.
   If you find a bug, fix it, say so in the write-up, and rerun the whole protocol.
4. Report, per fleet size and arrival rate: mean and median `orders_per_hour`,
   `mean_completion_steps`, and `p95_completion_steps`; runs with a deadlock; and `wall_seconds`.
   Then the per-seed difference, learned minus each baseline: wins, ties, losses, and the median
   difference with a 95% percentile bootstrap interval (resample the 10 differences with
   replacement 10,000 times using `numpy.random.default_rng(0)`; take the 2.5th and 97.5th
   percentiles of the resampled medians).
5. Selection bias check: on the final `learned` runs, predict with `model_io` from the logged
   features of the pairs it chose, and compare with their `steps_to_pickup` (MAE and bias). If
   that is much worse than the offline test, the model is being fooled by pairs it never saw in
   training. Say so.

Every number comes from `sweep.csv` and `assignments.csv`. A seed produces the same orders under
every allocator, so per-seed differences are paired.

`closed_loop.py` writes its configs, runs, and manifest the way `collect.py` does, under
`learning/data/<commit>/closed_loop/`. Its manifest also records the SHA-256 of every model file
it ran, so each result points to the exact model behind it.

## 10. Tests and CI

Unit tests, no C++ build needed:

- schema: the fixture passes; a missing, extra, or renamed column fails with a clear message.
- dataset: each filter drops exactly the expected fixture rows; no seed lands in two splits.
- model_io: write, read, and check round trip; every rule in section 4 rejects a file that breaks
  it; a 2-layer model with small integer weights matches predictions computed by hand.
- train: on synthetic rows where `steps_to_pickup = dist_to_pickup + 3 * pickup_assigned` plus
  small noise, `linear` and `mlp` reach a test MAE below 1 step.
- evaluate: every metric function on tiny inputs with known answers.

Core tests, marked `core`. They run when `FLEET_BUILD` names the C++ build folder, for example
`FLEET_BUILD=build`. When `FLEET_BUILD` is unset they skip; when it is set and a tool is missing
they fail.

- parity: `predict_cost` and `model_io` agree within `1e-6` on the fixture for the free-flow
  model, a linear model, and an MLP.
- free flow equals `hungarian`: `run_sweep` with fleets 5 and 10 and seeds 3000-3001, once with
  `hungarian` and once with `learned` plus the free-flow model, gives the same `log_hash` for
  every run.

Fixture: until Stage 3 lands, write `learning/tests/fixtures/run_tiny/assignments.csv` by hand
from section 3 (about 20 rows), plus a `summary.csv` with the columns you read. When Stage 3
lands, replace them with a real run (5 robots, `run.horizon_steps: 400`, `run.warmup_steps: 100`;
the warmup must be shorter than the horizon) and keep only `assignments.csv`, `summary.csv`, and
`config.yaml`.

CI lives in `.github/workflows/learning.yml`. Start from this:

```yaml
name: learning

on:
  push:
  pull_request:

jobs:
  python:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v5
      - uses: actions/setup-python@v6
        with:
          python-version: "3.12"  # the same version as on your machine
      - name: Install
        run: |
          python -m venv learning/.venv
          learning/.venv/bin/pip install -r learning/requirements.txt -e learning
      - name: Lint
        run: |
          learning/.venv/bin/ruff check learning
          learning/.venv/bin/ruff format --check learning
      - name: Unit tests
        run: learning/.venv/bin/python -m pytest learning/tests -m "not core"
```

When Stage 3 lands, add a `core` job: install the packages and build `fleet_core` the way
`.github/workflows/ci.yml` does, set up Python as above, and run
`FLEET_BUILD=build learning/.venv/bin/python -m pytest learning/tests -m core`.

## 11. Work order

Today:

1. Accept the collaborator invite, clone the repo, and run the build and test commands in
   CONTRIBUTING.md.
2. Read README.md, then `fleet_core/include/fleet_core/allocation/task_allocator.hpp`,
   `fleet_core/include/fleet_core/fleet_manager.hpp` (the tick pipeline),
   `fleet_core/include/fleet_core/simulation.hpp`, and `benchmarks/configs/default.yaml`. Run
   `./build/run_sim --config benchmarks/configs/default.yaml` and look through the run directory
   it writes.
3. On branch `learning/skeleton`: the folder layout, `pyproject.toml`, `requirements.txt`,
   `.gitignore`, `schema.py`, `model_io.py`, their tests, the hand-written fixture, and
   `learning.yml`. Open the pull request.

Before Stage 3 lands: `dataset.py`, `train.py`, `evaluate.py`, and `collect.py`. `collect.py`
can already write every config, but before Stage 3 only `allocation.method: nearest` runs:
`hungarian` and `cost_aware` exit 1 with "not implemented yet". Until Stage 3, loading must fail
with a clear message that `assignments.csv` arrives in Stage 3.

When Stage 3 lands: the real fixture, the `core` CI job, the parity and free-flow tests, and one
plumbing collection with `independent_astar` on seeds 3000-3004.

When Stage 4 lands: collect with `pibt`, train, run the offline evaluation, then closed-loop
tuning and the final runs. Write `learning/README.md`.

When Stage 5 lands: recollect with `pibt` on the new commit, retrain without `order_age`, tune
`cost_aware` on the new grid (section 9), rerun the final evaluation with the same protocol, and
update the write-up. Report the Stage 4 round and this one side by side.

Late-robot round, fixed on Oct 17 after the Stage 5 round: the whole protocol of sections 7 to 9
once more, with every config also setting `execution.delay_probability: 0.2`. Robots then miss
one step in five and wait for each other under the traffic rules, so travel time strays further
from distance than in the rounds before. Everything else stays as it was: the grid, the seeds and
splits, the model families without `order_age`, the `cost_aware` weights, at most 5 learned
models, and the success criterion, now with delay 0.2 in every run. Commit the criterion to
`learning/README.md` before the final runs.

- Keep its data apart: everything goes under `learning/data/<commit>/delay020/`, and run names
  end in `_d020`, for example `collect_hungarian_r010_d020`.
- Report it in its own section of `learning/README.md`, after the main rounds, with the same
  tables. Say that it was chosen after two rounds without a win, so a win here is weaker evidence
  than a win in the first round would have been. Report a loss the same way.

After Stage 6: the same again. Optional, in this order:

1. One more collection round with `learned` added to the allocator grid, then retrain. Data from
   the model's own choices shrinks the selection bias. Report both rounds.
2. Train on fleets 5 and 10, test on 25.
3. Drop the row and column features, and test on `maps/warehouse_large.txt`.
4. Ablate feature groups: distances, local crowding, station load, fleet load.

Dates:

- First offline numbers within two days of Stage 4 landing.
- Oct 26: `learning/README.md` complete, with numbers from the newest stage on `main`.
- Nov 7: rerun on the final code, and the write-up updated with those numbers.

## 12. Done means

- CI passes: lint, unit tests, and core tests.
- `learning/README.md` has the question; the data (runs, rows, seeds); the model; the offline
  table for all four models; the closed-loop table for all three allocators; the paired
  differences with intervals; the selection bias check; the limits; and the exact commands that
  regenerate every number.
- `learning/models/` holds the deployed model and its metrics file.
- Running those commands on a clean checkout reproduces every number except wall time.

## 13. Out of scope

Reinforcement learning. Any deployed model outside format v1. Deep learning frameworks. C++
changes, and any file outside `learning/` and `.github/workflows/learning.yml`. Tuning on seeds
1-10 or 1030-1039. Dashboards, cloud services, and GPUs. Anything not covered here needs an issue
first.

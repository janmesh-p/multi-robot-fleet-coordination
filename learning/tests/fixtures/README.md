# Test fixtures

`run_tiny/` is written by hand from section 3 of `docs/learning_track.md`, because the core
writes `assignments.csv` only from Stage 3 on. It stands for one `hungarian` run: 5 robots on
`maps/warehouse_small.txt`, seed 3000, `run.horizon_steps: 400`, `run.warmup_steps: 100`.

- The distance columns (`dist_to_pickup`, `manhattan_to_pickup`, `dist_pickup_to_dropoff`,
  `box_free_cells`, `robot_on_endpoint`) are computed from the map with the contract's rule:
  a path may start or end on a station or parking cell but never pass through one. The same
  computation gives 14 and 39 for the example row in the contract.
- Each robot starts on a parking cell and stays on the dropoff of its last delivered order.
  `available_robots` and `busy_robots` follow from that timeline.
- The crowding columns, `order_age`, and the delays over `dist_to_pickup` are made up.
- Rows cover each filter in section 3: `steps_to_pickup` = -1 inside the end margin (order 9),
  and rows with `t > steps_run - 300` (orders 10 to 15), some of them -1 too.
- `summary.csv` and `config.yaml` match those rows; values the rows do not determine are
  placeholders.

When Stage 3 lands, replace `run_tiny/` with a real run (see section 10).

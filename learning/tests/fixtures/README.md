# Test fixtures

`run_tiny/` is a real run of the Stage 3 core, as section 10 of `docs/learning_track.md` asks:
`hungarian` allocation and `independent_astar` planning on `maps/warehouse_small.txt`, 5 robots,
seed 3113, arrival rate 0.1, `run.horizon_steps: 400`, and `run.warmup_steps: 100`. It keeps
only `assignments.csv`, `summary.csv`, and `config.yaml`.

- `config.yaml` is the config the core wrote for the run, with one edit: `map.file` is relative
  to this folder instead of absolute, so the config works on any checkout.
- Seed 3113 was picked from seeds 3000-3199 because its 15 rows cover every case the tests need:
  rows past the end margin (`t > steps_run - 300`), rows that never reach the pickup both inside
  and past the margin, delayed rows (`steps_to_pickup > dist_to_pickup`), and the end statuses
  `delivered`, `picked_up`, and `assigned`.
- `test_fixture.py`, a core test, checks that the config still reproduces `assignments.csv` byte
  for byte, and `summary.csv` except the columns that measure wall time.

When a core change breaks that test, regenerate the run from the repo root:

```bash
./build/run_sim --config learning/tests/fixtures/run_tiny/config.yaml
cp build/run_tiny/assignments.csv build/run_tiny/summary.csv learning/tests/fixtures/run_tiny/
```

Then update the tests that name fixture rows, in `test_dataset.py`.

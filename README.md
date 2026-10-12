# multi-robot-fleet-coordination

Warehouse multi-agent pickup and delivery (MAPD): task allocation, lifelong multi-agent path
finding, traffic management, and fault recovery for up to 100 robots. Grid simulator for scale,
Isaac Sim for physics, desk hardware in the loop.

**Status:** Stage 2 (uncoordinated baseline) done. The baseline gives every robot its own
shortest path and ignores the others. In all 15 baseline runs, two robots soon meet head-on in a
one-cell aisle and the fleet freezes within a few simulated minutes ([numbers](#baseline-stage-2)).
Stage 4 adds coordinated planners. Every number in this README comes from measured runs.

## Architecture

```
            reports (may be lost)                       commands (may be lost)
 robots  ------------------------->  FleetManager  ------------------------->  robots
   ^                                 |  WorldModel     belief, not truth
   |                                 |  FaultManager   leases, requeue, false deaths
   |                                 |  TaskManager    order lifecycle
   |                                 |  TaskAllocator  nearest | hungarian | cost_aware
   |                                 |  PathPlanner    independent A* ... CBS, ECBS, PIBT
   |                                 |  TrafficManager permits, dependency graph, deadlocks
   |
 RobotExecutor backends: grid simulator | Isaac Sim + Nav2 | STM32 over WiFi
```

- FleetManager is pure input and output, so one coordinator drives every backend.
- Ground truth stays inside the executor. The coordinator only knows what reports tell it.
- Invariant checks, the progress monitor, and metrics live outside the coordinator, so it never
  grades itself.
- Every random stream has its own seed, so all configs compared under one seed see the same
  orders. Same seed and config give the same trajectory hash.
- Robots rest only on endpoints (stations, parking), and paths never pass through one
  (well-formed MAPD), so a parked or servicing robot never blocks traffic.
- One robot per station at a time; the rest wait their turn on spare parking cells.
- Robot safety in the simulator stops a robot before contact and records why. With
  coordinated planners every such stop counts as a violation.

## Simulator

Each step: new orders arrive (Poisson), robots report, the coordinator ticks, robots move one
cell along their newest command, invariants are checked, and metrics are recorded. 1 step is
1 simulated second.

A progress monitor reads the same ground truth. A robot stopped by robot safety waits for the
robot standing in the cell it wanted. Each robot wants one cell, so these waits form chains; a
chain that loops back on itself for 10 steps is a deadlock, and robots whose chain runs into it
are blocked behind it. A robot with work that makes no progress for 30 steps is stalled. A run is
frozen when no robot makes progress for the rest of the run while orders remain.

A run directory holds `config.yaml` (loading it reproduces the run), `map.txt`, `steps.csv`,
`positions.csv`, `summary.csv`, `violations.csv`, and `deadlocks.csv`. Replay one as a GIF
(robots on a deadlock get a thick ring):

```bash
pip install matplotlib
python3 benchmarks/scripts/replay_gif.py benchmarks/results/default --end 300
```

## Baseline (Stage 2)

Nearest allocation and independent A* on `warehouse_small`: 5, 10 and 25 robots, seeds 1 to 5,
one simulated hour per run (about 360 orders each).

| Robots | Runs that deadlocked | First deadlock, median step | Runs that froze | Frozen from, median step | Orders delivered per run, mean |
|---:|---:|---:|---:|---:|---:|
| 5 | 5 of 5 | 89 | 5 of 5 | 190 | 4.4 |
| 10 | 5 of 5 | 58 | 5 of 5 | 179 | 5.4 |
| 25 | 5 of 5 | 58 | 5 of 5 | 171 | 5.2 |

- All 25 deadlocks are two robots head-on in a one-cell aisle.
- Five broke later, each because one robot's goal changed: a station turn came up (four times)
  or a new order arrived (once). The other 20 never broke.
- Every run froze before the 600-step warmup ended, so steady-state throughput is zero at every
  fleet size. Even 5 robots are frozen by step 190 in the median run, about three minutes in.

Reproduce, and check that nothing has moved:

```bash
./build/run_sweep --config benchmarks/configs/baseline.yaml --out benchmarks/results/baseline \
  --check benchmarks/reference/baseline_sweep.csv
```

`benchmarks/reference/baseline_sweep.csv` holds every run's numbers and trajectory hash. CI runs
this check on every push, on every column except timing, so later stages cannot quietly change
the baseline they are measured against. `--runs` also writes each run directory.

## Build and test (fleet_core)

```bash
sudo apt install cmake ninja-build g++ libyaml-cpp-dev libgtest-dev
cmake -S fleet_core -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
./build/run_sim --config benchmarks/configs/default.yaml
./build/run_sweep --config benchmarks/configs/baseline.yaml --fleet 5,10,25 --seeds 1-5
```

Options: `-DFLEET_SANITIZE=ON` (ASan and UBSan), `-DFLEET_WERROR=OFF`.

## Formatting

```bash
python3 -m venv .venv && .venv/bin/pip install clang-format==18.1.3
git ls-files '*.cpp' '*.hpp' | xargs .venv/bin/clang-format -i
```

## Build (ROS 2 Humble)

```bash
rosdep install --from-paths fleet_msgs fleet_ros -y --ignore-src
colcon build --packages-select fleet_msgs fleet_ros
source install/setup.bash
ros2 launch fleet_ros fleet.launch.py
```

## Configuration

One YAML file per run; [benchmarks/configs/default.yaml](benchmarks/configs/default.yaml)
documents every key. Unknown or duplicate keys, wrong types, and out-of-range values are errors
with the key and line number, so a typo never silently falls back to a default. Each feature has
a toggle under `features:` for ablations.

## Layout

```
fleet_core/        C++17 core, no ROS dependency
  include/fleet_core/
    common/        types, config, rng
    world/         grid map, map generator, distance oracle, world model (coordinator belief)
    tasks/         order generator, task manager, station queues, parking
    allocation/    task allocators
    planning/      multi-agent planners
    traffic/       motion permits, deadlock handling
    faults/        fault injector (ground truth), fault manager (coordinator)
    execution/     robot executor interface, grid simulator robots
    verification/  invariant checker, progress monitor (stalls, deadlocks)
    metrics/       run statistics, run directory writer
    fleet_manager.hpp
    simulation.hpp
    sweep.hpp      fleet sizes x seeds, aggregation, reference check
  apps/            run_sim, run_sweep
  tests/           GoogleTest suite
fleet_msgs/        ROS 2 messages
fleet_ros/         ROS 2 adapter node
sim_isaac/         Isaac Sim scene and bridge (Stage 8)
firmware/          STM32F407 + CC3100 robot client (Stage 9a)
perception/        D435 overhead grid detector (Stage 9b)
benchmarks/        run configs, scripts, results, reference results
maps/              generated warehouse maps: small (15x38, up to 25 robots), large (33x75,
                   up to 100 robots)
docs/              learning track spec: a learned travel-time cost for allocation
```

How to contribute: [CONTRIBUTING.md](CONTRIBUTING.md).

## Stages

| Stage | Scope | Status |
|---|---|---|
| 0 | Foundation: components, config, RNG, tests, CI | done |
| 1 | Simulation core | done |
| 2 | Uncoordinated baseline | done |
| 3 | Task allocation | |
| 4 | Lifelong MAPF, PIBT | |
| 5 | Execution and traffic | |
| 6 | Faults, fuzz harness | |
| 7 | Benchmarks and evidence | |
| 8 | Isaac Sim | |
| 9a | Hardware in the loop | |
| 9b | Overhead perception | |
| 10 | Release | |

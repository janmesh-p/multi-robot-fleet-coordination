# multi-robot-fleet-coordination

Warehouse multi-agent pickup and delivery (MAPD): task allocation, lifelong multi-agent path
finding, traffic management, and fault recovery for up to 100 robots. Grid simulator for scale,
Isaac Sim for physics, desk hardware in the loop.

**Status:** Stage 1 (simulation core) done. The only planner so far is the uncoordinated
baseline, so the default run gridlocks within a few hundred steps: robots meet head-on in
one-cell aisles and stop for good. Stage 2 measures that; Stage 4 adds coordinated planners.
Every number in this README will come from measured runs.

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
- Invariant checks and metrics live outside the coordinator, so it never grades itself.
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

A run directory holds `config.yaml` (loading it reproduces the run), `map.txt`, `steps.csv`,
`positions.csv`, `summary.csv`, and `violations.csv`. Replay one as a GIF:

```bash
pip install matplotlib
python3 benchmarks/scripts/replay_gif.py benchmarks/results/default --end 300
```

## Build and test (fleet_core)

```bash
sudo apt install cmake ninja-build g++ libyaml-cpp-dev libgtest-dev
cmake -S fleet_core -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
./build/run_sim --config benchmarks/configs/default.yaml
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
    verification/  invariant checker
    metrics/       run statistics, run directory writer
    fleet_manager.hpp
    simulation.hpp
  apps/            run_sim
  tests/           GoogleTest suite
fleet_msgs/        ROS 2 messages
fleet_ros/         ROS 2 adapter node
sim_isaac/         Isaac Sim scene and bridge (Stage 8)
firmware/          STM32F407 + CC3100 robot client (Stage 9a)
perception/        D435 overhead grid detector (Stage 9b)
benchmarks/        run configs, scripts, results
maps/              generated warehouse maps: small (15x38, up to 25 robots), large (33x75,
                   up to 100 robots)
```

## Stages

| Stage | Scope | Status |
|---|---|---|
| 0 | Foundation: components, config, RNG, tests, CI | done |
| 1 | Simulation core | done |
| 2 | Uncoordinated baseline | |
| 3 | Task allocation | |
| 4 | Lifelong MAPF, PIBT | |
| 5 | Execution and traffic | |
| 6 | Faults, fuzz harness | |
| 7 | Benchmarks and evidence | |
| 8 | Isaac Sim | |
| 9a | Hardware in the loop | |
| 9b | Overhead perception | |
| 10 | Release | |

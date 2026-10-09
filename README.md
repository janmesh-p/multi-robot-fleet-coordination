# multi-robot-fleet-coordination

Warehouse multi-agent pickup and delivery (MAPD): task allocation, lifelong multi-agent path
finding, traffic management, and fault recovery for up to 100 robots. Grid simulator for scale,
Isaac Sim for physics, desk hardware in the loop.

**Status:** Stage 0 (foundation) done. No results yet; every number in this README will come
from measured runs.

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
  orders.

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
    world/         grid map, world model (coordinator belief)
    tasks/         order generator, task manager
    allocation/    task allocators
    planning/      multi-agent planners
    traffic/       motion permits, deadlock handling
    faults/        fault injector (ground truth), fault manager (coordinator)
    execution/     robot executor interface and backends
    metrics/       run output
    fleet_manager.hpp
  apps/            run_sim
  tests/           GoogleTest suite
fleet_msgs/        ROS 2 messages
fleet_ros/         ROS 2 adapter node
sim_isaac/         Isaac Sim scene and bridge (Stage 8)
firmware/          STM32F407 + CC3100 robot client (Stage 9a)
perception/        D435 overhead grid detector (Stage 9b)
benchmarks/        run configs, scripts, results
maps/              ASCII warehouse maps
```

## Stages

| Stage | Scope | Status |
|---|---|---|
| 0 | Foundation: components, config, RNG, tests, CI | done |
| 1 | Simulation core | |
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

# multi-robot-fleet-coordination

Warehouse multi-agent pickup and delivery (MAPD): task allocation, lifelong multi-agent path
finding, traffic management, and fault recovery for up to 100 robots. Grid simulator for scale,
Isaac Sim for physics, desk hardware in the loop.

**Status:** Stage 5 (execution and traffic) done. Robots that miss up to one step in three ran
320 simulated hours under four coordinated planners without a safety stop. With the traffic rules
off, robot safety stopped them 44,170 times at one missed step in ten
([numbers](#execution-and-traffic-stage-5)). Where independent planning froze every run
([baseline](#baseline-stage-2)), coordinated planners ran 276 simulated hours with up to 100
robots without a safety stop or a frozen run ([numbers](#coordinated-planning-stage-4)). Every
number in this README comes from measured runs.

## Architecture

```
            reports (may be lost)                       commands (may be lost)
 robots  ------------------------->  FleetManager  ------------------------->  robots
   ^                                 |  WorldModel     belief, not truth
   |                                 |  FaultManager   leases, requeue, false deaths
   |                                 |  TaskManager    order lifecycle
   |                                 |  TaskAllocator  nearest | hungarian | cost_aware | learned
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
- One robot per station at a time; the rest wait their turn on spare parking cells, or at home
  when none is free.
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
`positions.csv`, `summary.csv`, `violations.csv`, `deadlocks.csv`, and `assignments.csv` (one row
per assignment: what the coordinator knew about the pair, and how long the robot really took to
reach the pickup). Replay one as a GIF (robots on a deadlock get a thick ring):

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

## Allocation (Stage 3)

When orders wait and robots are free, the allocator decides who fetches what:

| `allocation.method` | Decision |
|---|---|
| `nearest` | oldest order first, to the closest free robot (the Stage 1 baseline) |
| `hungarian` | one minimum-cost matching of all free robots and all waiting orders; cost of a pair = shortest-path distance to the pickup |
| `cost_aware` | the same matching; cost = distance + `congestion_weight` x (robots inside the rectangle between robot and pickup + `service_steps` x robots already heading to that pickup) |
| `learned` | the same matching; cost = a trained model's prediction of the real steps to the pickup ([learning track](docs/learning_track.md)) |

The matching is the Hungarian method in its shortest augmenting path form, O(k^2 n) for k free
robots and n waiting orders, or the other way round. Tests check it against brute force on random
matrices.

Both allocators solved the same random rounds on `warehouse_small`: robots on random free cells,
orders at random stations. The table compares the total distance from robots to their pickups.

| Robots | Waiting orders | Nearest, mean total | Hungarian, mean total | Hungarian lower by | Rounds where Hungarian is strictly better | Hungarian solve, p95 |
|---:|---:|---:|---:|---:|---:|---:|
| 5 | 5 | 116.2 | 109.4 | 5.9% | 72.6% | 0.001 ms |
| 10 | 10 | 223.9 | 210.4 | 6.0% | 95.4% | 0.005 ms |
| 25 | 25 | 543.8 | 516.8 | 5.0% | 99.9% | 0.040 ms |
| 5 | 10 | 114.2 | 101.2 | 11.4% | 98.2% | 0.002 ms |
| 10 | 20 | 224.7 | 201.4 | 10.4% | 100.0% | 0.007 ms |
| 25 | 50 | 540.6 | 499.7 | 7.6% | 100.0% | 0.055 ms |

- 1000 rounds per row, seed 1. Hungarian is never worse; the benchmark stops with an error if it
  ever is.
- With more orders than robots, nearest always serves the oldest orders, while the matching may
  pick closer ones first, so part of that gain is order choice, not only who-goes-where.
- On `warehouse_large`, 100 robots and 100 orders take 1.1 ms at p95 (200 rounds).
- Solve times are wall time on one core of the build machine; every other number depends only
  on the seed.

What this does not show: throughput. With independent planning every run still freezes, whatever
the allocator, so orders per hour cannot separate them until Stage 4. `cost_aware` and `learned`
target traffic and busy stations, which these traffic-free rounds do not have.

```bash
./build/bench_allocation --out benchmarks/results/bench_allocation.csv
./build/bench_allocation --map maps/warehouse_large.txt --sizes 25x25,50x50,100x100,100x200 \
  --rounds 200
```

## Coordinated planning (Stage 4)

Every planner gets the robots' cells and goals and returns timed paths: one cell per step, a
repeated cell is a wait, and a robot holds its last cell when its path ends.

| `planning.method` | How it plans | Looks ahead |
|---|---|---|
| `independent_astar` | each robot alone, ignoring the others (the baseline) | to the goal |
| `prioritized` | one robot at a time, oldest order first, each around the robots planned before it; if one finds no path, start over with it first | to the goal |
| `windowed_coop_astar` | the same, conflict-free only `window` steps ahead (WHCA*) | `window` steps |
| `cbs` | Conflict-Based Search: plan each robot alone, then split on the first conflict, forbidding the cell to one robot or the other; least total cost | `window` steps |
| `ecbs` | bounded-suboptimal CBS: total cost within `ecbs_suboptimality` of the least, preferring plans with fewer conflicts | `window` steps |
| `pibt` | Priority Inheritance with Backtracking: every robot moves one step at a time; a robot that needs an occupied cell lends its priority to the robot standing there | `window` steps |

- Rolling horizon (`features.rolling_replanning`, on by default): every `replan_period` steps,
  every robot except those working at a station gets a new plan, conflict-free for the next
  `window` steps. A robot at its station is an obstacle until its service ends.
- With it off (`prioritized`, `cbs`, and `ecbs` only): a robot whose goal changed gets a path
  all the way there, around the plans the others already have.
- A call that finds no plan within `expansion_budget` keeps every robot on its current plan,
  which stays conflict-free, and runs again the next step.
- One low level for all of them: A* over cells and steps (focal search for ECBS), with the
  distance oracle as the heuristic. No path passes through a station or parking cell.
- Coordinated planners need station queues and parking, so no two robots share a goal. A robot
  whose station has no free waiting cell waits at home, because waiting on a station or in an
  aisle can block the robots it waits for. The baseline still waits where it is, so its
  reference does not move. Robots that run late are covered in
  [Stage 5](#execution-and-traffic-stage-5).
- No path passes through parking, so every parking cell is a dead end, and PIBT's guarantee
  needs a map without one. A robot parked in one whose only exit is held by a robot that wants
  in would wait forever, so in PIBT the robot at the exit steps aside and the parked one leaves
  first.
- With a coordinated planner, every robot-safety stop counts as a violation and stops the run.

`warehouse_small`, optimal matching, one simulated hour per run, seeds 1 to 10. Orders per hour
in steady state, mean over the seeds. 0.1 orders per step offers 360 orders per hour; 0.2 offers
720. Measured again on the Stage 5 code, with the starvation guard on: it changes a run only once
an order has waited 600 steps, so the 25-robot runs at 0.1 orders per step are the same runs.

| Planner | 0.1/step, 5 robots | 10 robots | 25 robots | 0.2/step, 5 robots | 10 robots | 25 robots |
|---|---:|---:|---:|---:|---:|---:|
| `independent_astar` | 0.1 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| `pibt` | 175.4 | 298.9 | 356.5 | 174.6 | 302.4 | 465.1 |
| `windowed_coop_astar` | 184.6 | 346.8 | 359.2 | 184.7 | 349.9 | 602.2 |
| `prioritized` | 185.2 | 346.8 | 358.7 | 185.0 | 351.6 | 611.5 |
| `ecbs` | 185.4 | 348.2 | 359.5 | 185.2 | 352.9 | 653.3 |
| `cbs` | 186.7 | 316.4 | not run | 186.4 | 294.8 | not run |

- `pibt`, `windowed_coop_astar`, `prioritized`, and `ecbs`: 240 runs here and 36 on
  `warehouse_large` below, with no robot-safety stop, no violation, and no frozen run.
  `independent_astar` froze all 60 of its runs.
- At 0.1 orders per step, 25 robots keep up with the 360 orders offered under every coordinated
  planner. With 5 robots the robots themselves are the limit.
- PIBT delivers least: 14% below `windowed_coop_astar` at 10 robots and 0.1 orders per step, 23%
  below at 25 robots and 0.2. In a one-cell aisle the robot that gives way backs up to the next
  junction and then comes back, where the other planners wait or take another aisle. It is the
  fastest planner: 0.18 ms per call at p95 with 25 robots and 0.2 orders per step, against
  0.27 ms for `windowed_coop_astar`, 0.38 ms for `prioritized`, and 2.6 ms for `ecbs`.
- ECBS delivers most at 25 robots and 0.2 orders per step, ahead of `windowed_coop_astar` in all
  10 seeds.
- CBS finds the least total cost in each window, but two robots meeting head-on in a one-cell
  aisle make it split on the same conflict again and again, one step further each time. At 10
  robots and 0.1 orders per step, 2,994 of its 9,598 calls ran out of budget, and 2 of the 10
  runs froze: once every robot has stopped, the same failing call repeats. At 0.2, 3 of 10 runs
  froze (1 of 10 before the starvation guard). Not run at 25 robots.
- Planning times are wall time on one core of the build machine, the mean of each run's p95;
  every other number depends only on the seed. Two benchmark processes shared the machine's two
  cores, so the times run higher than in Stage 4.

`warehouse_large`, 0.5 orders per step (1,800 per hour offered), seeds 1 to 3:

| Planner | 25 robots | 50 robots | 100 robots | Planning p95 at 100 robots |
|---|---:|---:|---:|---:|
| `pibt` | 399.2 | 561.6 | 501.6 | 0.70 ms |
| `windowed_coop_astar` | 434.8 | 649.6 | 574.8 | 0.75 ms |
| `prioritized` | 441.2 | 665.6 | 576.4 | 1.95 ms |
| `ecbs` | 430.8 | 650.4 | 554.0 | 9.27 ms |

Every planner delivers less with 100 robots than with 50. The map has 150 parking cells and 100
are homes, which leaves 50 to wait on near its 20 stations; robots that find none wait at home
and drive over when their turn comes. Stage 7 measures where the time goes.

What this does not show: robots that run late and closed aisles
([Stage 5](#execution-and-traffic-stage-5)), lost messages and dead robots (Stage 6), and the
learned allocator (the learning track).

```bash
./build/bench_planning --config benchmarks/configs/planning.yaml --fleet 5,10,25 --seeds 1-10
./build/bench_planning --config benchmarks/configs/planning.yaml --fleet 5,10,25 --seeds 1-10 \
  --rate 0.2
./build/bench_planning --config benchmarks/configs/planning.yaml --planners cbs \
  --fleet 5,10 --seeds 1-10   # and --rate 0.2
./build/bench_planning --config benchmarks/configs/planning.yaml \
  --planners independent_astar --fleet 5,10,25 --seeds 1-10   # and --rate 0.2
./build/bench_planning --config benchmarks/configs/planning.yaml --fleet 25,50,100 \
  --seeds 1-3 --rate 0.5 --map maps/warehouse_large.txt
```

## Execution and traffic (Stage 5)

Plans are conflict-free only while robots keep to time. Stage 5 makes robots run late and adds
the rules that keep them apart anyway.

- Late robots: with `execution.delay_probability` p, a robot misses each step it could take with
  probability p, drawn from its own seeded stream.
- Permits: every tick each robot gets its plan and how far along it may go. A robot may be told to
  follow another out of its cell; if that robot stays after all, it waits behind it, and robot
  safety never has to stop it. The invariant checker reports any move past a permit.
- Dependency graph (`features.dependency_graph`, Hönig et al. 2019): for every cell, the order in
  which the plans visit it. A robot enters a cell once the robot planned there before it has left,
  or as it leaves. It needs plans made together, so it applies with rolling replanning.
- Reservations (`features.intersection_reservations`): a robot enters a cell only while it is
  empty or its robot leaves in the same step, one robot per cell per step, the older order first.
  They need no shared plan order, so they also cover planning to goals.
- Deadlock recovery (`features.deadlock_recovery`): held robots that wait for each other in a
  cycle get new plans. Rolling replanning moves its next call to the next step; planning to goals
  replans the whole fleet. Replanning only the robots on the cycle froze one benchmark run: they
  found no way out around the others' plans.
- Starvation guard (`features.starvation_guard`): the matching allocators serve orders that have
  waited more than 600 steps first, oldest first. The cost still decides which robot takes each.
- Aisle closures: `block_cells` and `unblock_cells` events close and reopen aisle cells. A closure
  takes effect once no robot stands on any of its cells, on the floor and in the coordinator. Then
  the distance table is rebuilt and the fleet replans. A closure that would cut the open aisles
  apart is rejected at startup.
- All of these are on by default. The traffic rules apply to coordinated planners only, and the
  baseline reference did not move.

`warehouse_small`, optimal matching, 0.1 orders per step, rolling replanning, one simulated hour
per run, seeds 1 to 10. Orders per hour in steady state, mean over the seeds, by the chance that a
robot misses a step. Column 0 is the Stage 4 table.

| Robots | Planner | 0 | 0.05 | 0.1 | 0.2 | 0.3 |
|---:|---|---:|---:|---:|---:|---:|
| 10 | `pibt` | 298.9 | 281.0 | 261.8 | 229.0 | 199.7 |
| 10 | `windowed_coop_astar` | 346.8 | 336.5 | 319.2 | 282.5 | 247.7 |
| 10 | `prioritized` | 346.8 | 336.8 | 321.1 | 284.2 | 247.8 |
| 10 | `ecbs` | 348.2 | 338.9 | 323.4 | 286.7 | 250.4 |
| 25 | `pibt` | 356.5 | 355.7 | 357.8 | 343.7 | 285.1 |
| 25 | `windowed_coop_astar` | 359.2 | 358.8 | 358.7 | 358.6 | 357.5 |
| 25 | `prioritized` | 358.7 | 359.2 | 358.4 | 357.4 | 357.0 |
| 25 | `ecbs` | 359.5 | 359.0 | 359.2 | 359.3 | 358.0 |

- No safety stop and no violation in the 320 runs with late robots.
- With 10 robots the robots are the limit, so missed steps cost deliveries: 7% to 12% fewer at
  0.1, 18% to 23% fewer at 0.2. With 25 robots there is slack: `windowed_coop_astar`,
  `prioritized`, and `ecbs` lose under 1% even at 0.3. PIBT loses 4% at 0.2 and 20% at 0.3.

Which rule does the work, at one missed step in ten: the four planners above with 10 and 25
robots, 80 runs per row.

| Rules on | Robot-safety stops | Cycles replanned per run, 10 / 25 robots |
|---|---:|---:|
| both (the default) | 0 | 0 / 0 |
| dependency graph only | 0 | 0 / 0 |
| reservations only | 0 | 13.5 / 27.9 |
| neither | 44,170 | 0 / 0 |

- With the dependency graph on, reservations never changed a decision: the same 80 runs, down to
  the log hash. They matter when plans are made apart (next table).
- Reservations alone keep late robots apart too, but not in the planned order, so robots meet
  head-on and wait for each other. PIBT needed 75 replans per hour with 25 robots.
- With neither rule, robot safety stopped robots 44,170 times. Each stop is a violation for a
  coordinated planner; these runs used `--record`, where a normal run aborts at the first.

Planning to goals (rolling replanning off) with one missed step in ten: `prioritized` and `ecbs`
with 10 and 25 robots and `cbs` with 10, 50 runs per row.

| Rules | Frozen runs | Robot-safety stops | Orders per hour |
|---|---:|---:|---:|
| reservations and recovery (the default) | 0 | 0 | 334 to 359 |
| the same robots on time | 0 | 0 | 356 to 359 |
| no deadlock recovery | 50 | 0 | 0.4 to 17 |
| no reservations | 50 | 1,052,387 | 0.6 to 12.5 |

One `cbs` run with recovery had a cycle that lasted at least 10 steps, so the stall monitor counts
a deadlock. It broke later, and no run froze.

Starvation guard, mean over the four planners and 10 seeds, guard on / off. Only 0.1 orders per
step with 25 robots keeps up with every order.

| Orders per step | Robots | Oldest open order at the end, steps | Slowest delivery, steps | Orders per hour |
|---:|---:|---:|---:|---:|
| 0.1 | 5 | 1,819 / 2,143 | 1,818 / 2,121 | 182.6 / 186.7 |
| 0.1 | 10 | 417 / 480 | 537 / 586 | 335.2 / 335.4 |
| 0.1 | 25 | 79 / 79 | 154 / 154 | 358.5 / 358.5 |
| 0.2 | 5 | 2,719 / 2,880 | 2,707 / 2,829 | 182.4 / 186.5 |
| 0.2 | 10 | 1,952 / 2,233 | 1,949 / 2,209 | 339.2 / 343.9 |
| 0.2 | 25 | 853 / 938 | 885 / 983 | 583.0 / 587.2 |

- Where robots keep up, no order waits 600 steps and nothing changes.
- Under overload the oldest open order is 6% to 15% younger with the guard, and up to 2.2% fewer
  orders are delivered: robots walk farther to old orders.
- It sets no limit under real overload. When every waiting order is older than 600 steps, the guard
  only serves them oldest first.

Closed aisle: `benchmarks/configs/closure.yaml` closes six cells of the middle aisle from step 900
to step 2700. In 120 runs (the four planners above with rolling replanning, and `prioritized` and
`ecbs` planning to goals, each with 10 and 25 robots) there was no safety stop and no violation.
Every planner and fleet size delivered within 0.7% of its orders per hour with the aisle open,
mean over the seeds. In a first
version every cell closed as soon as it was empty, so a robot inside the six cells was trapped
when the cells around it closed, and `windowed_coop_astar` found no plan for the whole half hour
in 2 of 10 runs. A test now covers that run.

What this does not show: slow robots or bursts of delay (each missed step is drawn on its own),
more than one closure scenario, and lost messages and dead robots (Stage 6).

```bash
for p in 0.05 0.1 0.2 0.3; do
  ./build/bench_planning --config benchmarks/configs/planning.yaml --fleet 10,25 --seeds 1-10 \
    --delay $p
done
./build/bench_planning --config benchmarks/configs/planning.yaml --fleet 10,25 --seeds 1-10 \
  --delay 0.1 --disable intersection_reservations
  # also: --disable dependency_graph, and --disable dependency_graph,intersection_reservations --record
./build/bench_planning --config benchmarks/configs/planning.yaml --planners prioritized,ecbs \
  --fleet 10,25 --seeds 1-10 --delay 0.1 --disable rolling_replanning
  # also: --planners cbs --fleet 10; without --delay; and with deadlock_recovery, or with
  # intersection_reservations and --record, added to --disable
./build/bench_planning --config benchmarks/configs/planning.yaml --fleet 5,10,25 --seeds 1-10 \
  --disable starvation_guard   # and --rate 0.2
./build/bench_planning --config benchmarks/configs/closure.yaml --fleet 10,25 --seeds 1-10
./build/bench_planning --config benchmarks/configs/closure.yaml --planners prioritized,ecbs \
  --fleet 10,25 --seeds 1-10 --disable rolling_replanning
```

## Build and test (fleet_core)

```bash
sudo apt install cmake ninja-build g++ libyaml-cpp-dev libgtest-dev
cmake -S fleet_core -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
./build/run_sim --config benchmarks/configs/default.yaml
./build/run_sweep --config benchmarks/configs/baseline.yaml --fleet 5,10,25 --seeds 1-5
./build/bench_allocation
./build/bench_planning --config benchmarks/configs/planning.yaml
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
    allocation/    task allocators, assignment solver, pair features, learned cost model
    planning/      multi-agent planners
    traffic/       motion permits, deadlock handling
    faults/        fault injector (ground truth), fault manager (coordinator)
    execution/     robot executor interface, grid simulator robots
    verification/  invariant checker, progress monitor (stalls, deadlocks)
    metrics/       run statistics, run directory writer
    fleet_manager.hpp
    simulation.hpp
    sweep.hpp      fleet sizes x seeds, aggregation, reference check
  apps/            run_sim, run_sweep, bench_allocation, bench_planning, predict_cost
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
| 3 | Task allocation | done |
| 4 | Lifelong MAPF, PIBT | done |
| 5 | Execution and traffic | done |
| 6 | Faults, fuzz harness | |
| 7 | Benchmarks and evidence | |
| 8 | Isaac Sim | |
| 9a | Hardware in the loop | |
| 9b | Overhead perception | |
| 10 | Release | |

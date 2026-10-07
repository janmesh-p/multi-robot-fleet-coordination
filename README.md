# multi-robot-fleet-coordination

Warehouse fleet planning: task allocation, multi-agent path finding, traffic management, fault recovery. Grid sim for scale, Isaac Sim for realism, desk-top hardware in the loop.

Status: skeleton. Results will be added as stages land.

## Layout

```
fleet_core/        C++17 core, no ROS dependency (allocators, MAPF, traffic, faults, metrics)
fleet_msgs/        ROS 2 message definitions
fleet_ros/         ROS 2 nodes wrapping fleet_core
sim_isaac/         Isaac Sim warehouse scene and multi-robot bridge
firmware/          STM32F407 + CC3100 robot client (micro-ROS)
perception/        D435 overhead grid detector (Jetson)
benchmarks/        sweep configs, result CSVs, plotting
maps/              ASCII warehouse maps (. free, # shelf, P pickup, D dropoff)
```

## Build (core only)

```bash
cmake -S fleet_core -B build && cmake --build build -j
ctest --test-dir build
./build/run_sim maps/warehouse_small.txt 5
```

## Build (ROS 2)

```bash
colcon build --packages-select fleet_msgs fleet_ros
source install/setup.bash
ros2 launch fleet_ros fleet.launch.py
```

## Stages

1. Grid sim core
2. Independent A* baseline
3. Task allocation: nearest, Hungarian, auction, cost-aware
4. MAPF: prioritized, Cooperative A*, CBS, ECBS
5. Traffic: intersection reservations, deadlock recovery, blocked aisles
6. Faults: robot death, task reassignment, packet loss
7. Benchmarks: 5 to 100 robots
8. Isaac Sim: 5 robots with Nav2
9. Hardware in the loop: Jetson Orin Nano, STM32 + CC3100, D435

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "fleet_core/allocation/task_allocator.hpp"
#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/faults/fault_manager.hpp"
#include "fleet_core/planning/path_planner.hpp"
#include "fleet_core/tasks/task_manager.hpp"
#include "fleet_core/traffic/traffic_manager.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// Everything the coordinator learns in one tick.
struct TickInput {
  Timestep now{0};
  std::vector<RobotReport> reports{};           // may be partial under packet loss (Stage 6)
  std::vector<Task> new_orders{};               // from the order source
  std::vector<CellObservation> observations{};  // blocked or reopened cells
};

// Coordinator-side numbers for one tick, recorded by MetricsCollector.
struct TickStats {
  int32_t assignments{0};
  int32_t robots_planned{0};
  bool planner_called{false};
  bool planner_failed{false};
  double planning_ms{0.0};
  int64_t expansions{0};
  int32_t robots_declared_dead{0};
  int32_t tasks_requeued{0};
  int32_t deadlocks_resolved{0};
};

struct TickOutput {
  std::vector<RobotCommand> commands{};  // one per robot believed alive, every tick
  TickStats stats{};
};

// Central coordinator. Orchestrates the decision components and holds no algorithm itself.
//
// Pure input and output: the same instance runs against the grid simulator, Isaac Sim, or
// hardware. The caller owns the robots (RobotExecutor) and the measurements (MetricsCollector,
// invariant checks), so the system under test never grades itself.
//
// tick() pipeline:
//   1. world       accept reports and cell observations
//   2. faults      expire heartbeat leases, requeue tasks of dead robots, reconcile false deaths
//   3. tasks       add new orders; confirm pickups and deliveries from reports
//   4. allocation  assign pending tasks to available robots
//   5. planning    replan robots with new goals, invalidated paths, or an expiring window
//   6. traffic     turn committed plans into motion permits; resolve deadlocks
//   7. output      one command per robot believed alive
//
// Interface only. Implementation lands in Stage 1.
class FleetManager {
 public:
  FleetManager(const Config& config, WorldModel world);

  // Test seam: inject allocator and planner doubles.
  FleetManager(const Config& config, WorldModel world, std::unique_ptr<TaskAllocator> allocator,
               std::unique_ptr<PathPlanner> planner);

  FleetManager(const FleetManager&) = delete;
  FleetManager& operator=(const FleetManager&) = delete;
  ~FleetManager();

  TickOutput tick(const TickInput& input);

  const WorldModel& world() const { return world_; }
  const TaskManager& tasks() const { return tasks_; }

 private:
  Config config_;
  WorldModel world_;
  TaskManager tasks_{};
  std::unique_ptr<TaskAllocator> allocator_;
  std::unique_ptr<PathPlanner> planner_;
  TrafficManager traffic_;
  FaultManager faults_;
};

}  // namespace fleet_core

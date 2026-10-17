#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fleet_core/allocation/pair_features.hpp"
#include "fleet_core/allocation/task_allocator.hpp"
#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/faults/fault_manager.hpp"
#include "fleet_core/planning/path_planner.hpp"
#include "fleet_core/tasks/parking.hpp"
#include "fleet_core/tasks/station_manager.hpp"
#include "fleet_core/tasks/task_manager.hpp"
#include "fleet_core/traffic/traffic_manager.hpp"
#include "fleet_core/world/distance_oracle.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// Everything the coordinator learns in one tick.
struct TickInput {
  Timestep now{0};
  std::vector<RobotReport> reports{};           // may be partial under packet loss (Stage 6)
  std::vector<Task> new_orders{};               // from the order source
  std::vector<CellObservation> observations{};  // blocked or reopened cells
};

// One assignment made in a tick, with what the coordinator saw when it made it: the features of
// the pair, from its belief before any of that tick's assignments. The runner joins these with
// ground-truth outcomes into assignments.csv; the coordinator never sees the outcomes.
struct AssignmentRecord {
  Timestep t{0};
  TaskId order{-1};
  RobotId robot{-1};
  Cell pickup{};
  PairFeatures features{};
};

struct TickOutput {
  std::vector<RobotCommand> commands{};  // one per robot believed alive, every tick
  std::vector<AssignmentRecord> assignments{};
  std::vector<TaskId> requeued{};  // orders taken back from robots declared dead (Stage 6)
  TickStats stats{};
};

// Why a config cannot run its planner, or nullopt when it can. Coordinated planners need station
// queues and parking, so that no two robots share a goal and idle robots wait off the aisles;
// windowed_coop_astar and pibt plan a window ahead, so they need rolling replanning.
std::optional<std::string> planningRequirementGap(const Config& config);

// Central coordinator. Orchestrates the decision components and holds no algorithm itself.
//
// Pure input and output: the same instance runs against the grid simulator, Isaac Sim, or
// hardware. The caller owns the robots (RobotExecutor) and the measurements (MetricsCollector,
// InvariantChecker), so the system under test never grades itself.
//
// tick() pipeline:
//   1. world       accept reports and cell observations. A closure takes effect once no robot
//                  stands on any of its cells, as on the floor. A closed or reopened cell
//                  rebuilds the distance table and triggers replanning (see onCellsChanged).
//   2. faults      expire heartbeat leases (recovery lands in Stage 6)
//   3. tasks       add new orders; confirm pickups and deliveries from reports
//   4. allocation  assign pending tasks to robots without one
//   5. goals       station (on its turn), station waiting cell, or home parking
//   6. planning    independent_astar: replan every robot whose goal changed, robots already
//                  there just hold. Coordinated planners with rolling replanning: every
//                  planning.replan_period steps, replan every robot not at a station's service,
//                  planning.window steps ahead (rolling horizon). Without it: replan robots
//                  whose goal changed, all the way there, around where the others really are on
//                  their plans.
//   7. traffic     motion permits from the traffic rules (TrafficManager). Held robots that wait
//                  for each other in a cycle get replanned (features.deadlock_recovery).
//   8. output      one command per robot believed alive
//
// A failed coordinated call keeps every committed plan, which stays conflict-free with the
// others, and tries again next tick.
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

  // Goal of the robot's current plan.
  Cell goalOf(RobotId robot) const;

 private:
  struct RobotPlan {
    bool valid{false};  // false: replan next tick even if the goal is unchanged
    Cell goal{};
    int32_t plan_id{-1};
    Path path{};
  };

  bool onClosedCells(const std::vector<Cell>& cells) const;
  void onCellsChanged(Timestep now);
  void confirmProgress(Timestep now);
  void allocate(Timestep now, TickOutput& output);
  std::vector<Cell> computeGoals();
  void plan(Timestep now, const std::vector<Cell>& goals, TickStats& stats);
  void planIndependent(Timestep now, const std::vector<Cell>& goals, TickStats& stats);
  void planRolling(Timestep now, const std::vector<Cell>& goals, TickStats& stats);
  void planToGoals(Timestep now, const std::vector<Cell>& goals, TickStats& stats);
  PlanResult runPlanner(Timestep now, const std::vector<PlanAgent>& agents, std::vector<Path> fixed,
                        int32_t window, TickStats& stats);
  PlanAgent agent(const RobotBelief& robot, const Cell& goal, Timestep now) const;
  void commit(RobotId robot, const Cell& goal, Path path, bool valid,
              std::vector<CommittedPlan>& committed);
  std::vector<Path> fixedPaths(const std::vector<PlanAgent>& agents) const;
  std::vector<int32_t> priorities(Timestep now) const;
  void recover(Timestep now, const std::vector<std::vector<RobotId>>& cycles, TickStats& stats);
  std::vector<RobotCommand> commands(Timestep now, const TrafficDecision& traffic) const;

  Config config_;
  WorldModel world_;
  DistanceOracle distances_;
  StationManager stations_;
  std::optional<ParkingManager> parking_;
  TaskManager tasks_{};
  std::unique_ptr<TaskAllocator> allocator_;
  std::unique_ptr<PathPlanner> planner_;
  TrafficManager traffic_;
  FaultManager faults_;
  std::vector<RobotPlan> plans_;  // index == RobotId
  Timestep next_replan_{0};       // rolling horizon: first step of the next replanning
  std::vector<std::vector<Cell>> pending_closures_{};  // waiting for their cells to empty
};

}  // namespace fleet_core

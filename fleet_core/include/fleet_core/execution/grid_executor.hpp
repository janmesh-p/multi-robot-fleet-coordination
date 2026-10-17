#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "fleet_core/common/rng.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/execution/robot_executor.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// What one robot did in one step.
enum class MotionOutcome : uint8_t {
  kIdle,             // no task and nothing to do
  kMoved,            // advanced one cell along its path
  kPlannedWait,      // its path says wait here this step
  kHold,             // no permitted step left (end of path or permit) while holding a task
  kService,          // dwelling at a station
  kDelayed,          // missed a permitted step (execution noise, Stage 5)
  kBlockedVertex,    // robot safety: lost a same-cell contest to a lower robot id
  kBlockedSwap,      // robot safety: head-on with another robot
  kBlockedOccupied,  // robot safety: next cell held by a robot that did not move
  kBlockedCell,      // next cell is dynamically blocked
  kInvalidStep,      // its path asked for a non-adjacent cell or a shelf
  kDead,
  // Stage 5. Appended so earlier outcomes keep their values (they feed the run fingerprint).
  kYield,     // waited behind the robot it was permitted to follow, which stayed (Stage 5)
  kNoPermit,  // its path goes on, but its permit ends here: held by the traffic rules (Stage 5)
};

std::string_view toString(MotionOutcome outcome);

// Stops caused by another robot. Zero of these is what coordination has to achieve. A yield is
// not one: the coordinator allowed the robot to follow only if the robot ahead moved.
bool isSafetyStop(MotionOutcome outcome);

struct MotionRecord {
  RobotId robot{-1};
  Cell from{};
  Cell to{};
  Cell intended{};  // where its path wanted it this step
  MotionOutcome outcome{MotionOutcome::kIdle};
};

// Ground truth for one step. Read by the invariant checker and metrics, never by the coordinator.
struct StepTruth {
  Timestep t{0};
  std::vector<MotionRecord> motion{};    // index == RobotId
  std::vector<Assignment> pickups{};     // (robot, task) finished picking this step
  std::vector<Assignment> deliveries{};  // (robot, task) finished delivering this step
};

// Ground truth for the fleet at one instant.
struct TruthSnapshot {
  std::vector<Cell> cells{};
  std::vector<bool> alive{};
  std::vector<RobotStatus> status{};
  std::vector<std::optional<TaskId>> holding{};  // task assigned to or carried by each robot
};

// Ground truth for one simulated robot.
struct SimRobot {
  Cell cell{};
  bool alive{true};
  RobotStatus status{RobotStatus::kIdle};
  std::optional<TaskOrder> task{};
  bool carrying{false};
  Timestep service_left{0};
  int32_t plan_id{-1};  // newest command applied
  std::vector<Cell> path{};
  int32_t permitted_until{0};
  RobotId follow{-1};     // its next move may follow this robot out of its cell (Stage 5)
  int32_t path_index{0};  // position along path; path[path_index] == cell while on the plan
  std::optional<TaskId> last_completed{};
  uint32_t seq{0};
};

// Simulated robots on the grid: the ground-truth backend.
//
// Each step every robot advances at most one cell along its newest command, up to the permit.
// Robot safety resolves contact the way a real AMR's safety field would: a robot never enters a
// cell another robot keeps, two robots never swap, and when two want the same cell the lower id
// goes. Following a robot that leaves a cell in the same step is allowed. Every stop caused by
// another robot is recorded, so the checker can tell whether coordination was needed.
//
// A robot starts service only when its path ends on its station, so a robot passing through a
// station never jumps the queue.
//
// Execution noise: with execution.delay_probability p, a robot misses each step it could take
// with probability p. A robot told to follow another (RobotCommand::follow) waits behind it when
// that robot stays.
//
// Closed cells: a closure closes all its cells at once, as soon as no robot stands on any of
// them. Until then they stay open, so a robot inside a closing aisle segment drives out through
// it. No robot enters a closed cell. Packet loss and robot deaths arrive in Stage 6.
class GridExecutor final : public RobotExecutor {
 public:
  // spawn[i] is robot i's start cell; cells must be free and distinct.
  // delay_seed: deriveSeed(master, rng_stream::kDelays).
  GridExecutor(GridMap map, const std::vector<Cell>& spawn, Timestep service_steps,
               double delay_probability, uint64_t delay_seed);

  std::vector<RobotReport> collectReports(Timestep now) override;
  void dispatch(Timestep now, const std::vector<RobotCommand>& commands) override;
  void step(Timestep now) override;

  // Ground-truth closures. A closure waits while any robot stands on any of its cells, then they
  // all close together. Reopening a cell also takes it out of a closure still waiting. Throws
  // std::out_of_range for cells outside the map.
  void block(const std::vector<Cell>& cells);
  void unblock(const std::vector<Cell>& cells);
  bool closing(const Cell& c) const;  // closed, or in a closure still waiting for its robots

  const StepTruth& lastStep() const { return last_; }
  TruthSnapshot snapshot() const;
  const std::vector<SimRobot>& robots() const { return robots_; }
  const GridMap& map() const { return map_; }

 private:
  void finishService(RobotId id);

  GridMap map_;
  std::vector<SimRobot> robots_;
  bool occupied(const std::vector<int32_t>& cells) const;

  std::vector<uint8_t> pending_block_;           // per cell: in a closure still waiting
  std::vector<std::vector<int32_t>> pending_{};  // closures waiting for their cells to empty
  Timestep service_steps_;
  double delay_probability_;
  Rng delay_rng_;
  StepTruth last_{};
};

}  // namespace fleet_core

#pragma once

#include <cstdint>
#include <limits>
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
  // Stage 6.
  kNoCommand,  // no command arrived for this step, so no permit: it waits (lost link)
  kSafeStop,   // no command for longer than the timeout: holding, station service paused
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
  std::vector<RobotId> removed{};        // dead robots taken off the floor this step (Stage 6)
};

// Ground truth for the fleet at one instant.
struct TruthSnapshot {
  std::vector<Cell> cells{};
  std::vector<bool> alive{};
  std::vector<bool> on_floor{};  // false once a dead robot has been taken away (Stage 6)
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
  // Stage 6.
  Timestep last_command{-1};  // step of the newest command that reached it
  bool safe_stop{false};      // no command for longer than the timeout
  Timestep died_at{-1};
  bool removed{false};  // dead and taken off the floor
};

inline constexpr Timestep kNever = std::numeric_limits<Timestep>::max();

// Link and failure behavior (Stage 6). The defaults turn all of it off.
struct ExecutorFaults {
  double packet_loss{0.0};           // drop probability per report and per command
  uint64_t link_seed{0};             // deriveSeed(master, rng_stream::kLink)
  Timestep safe_stop_after{kNever};  // no command for longer than this: safe stop
  Timestep rescue_after{kNever};     // a dead robot is taken off the floor this long after it dies
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
// it. No robot enters a closed cell.
//
// The link (Stage 6): every report and every command is lost with probability packet_loss, and
// all of a robot's messages are lost during a comm outage. One loss draw per robot per message
// round, used or not, so the loss stream never shifts. A permit is good only for the step its
// command arrives in: a robot that hears nothing waits, which the traffic rules already treat as
// a missed step. Commands also say which order a robot holds: a robot drops an order its command
// no longer names, and never takes back an order it has delivered. After safe_stop_after steps
// without a command, a robot holds and pauses station service until a command arrives.
//
// Deaths (Stage 6): a dead robot stops for good, reports nothing, and holds no order. It stays on
// its cell until rescue_after steps later, when it is taken off the floor.
class GridExecutor final : public RobotExecutor {
 public:
  // spawn[i] is robot i's start cell; cells must be free and distinct.
  // delay_seed: deriveSeed(master, rng_stream::kDelays).
  GridExecutor(GridMap map, const std::vector<Cell>& spawn, Timestep service_steps,
               double delay_probability, uint64_t delay_seed, ExecutorFaults faults = {});

  std::vector<RobotReport> collectReports(Timestep now) override;
  void dispatch(Timestep now, const std::vector<RobotCommand>& commands) override;
  void step(Timestep now) override;

  // Ground-truth closures. A closure waits while any robot stands on any of its cells, then they
  // all close together. Reopening a cell also takes it out of a closure still waiting. Throws
  // std::out_of_range for cells outside the map.
  void block(const std::vector<Cell>& cells);
  void unblock(const std::vector<Cell>& cells);
  bool closing(const Cell& c) const;  // closed, or in a closure still waiting for its robots

  // Ground-truth faults (Stage 6). Throw std::out_of_range for an unknown robot.
  void kill(RobotId robot, Timestep now);                       // stops for good where it stands
  void outage(RobotId robot, Timestep now, Timestep duration);  // its messages are lost meanwhile

  int64_t reportsLost() const { return reports_lost_; }
  int64_t commandsLost() const { return commands_lost_; }

  const StepTruth& lastStep() const { return last_; }
  TruthSnapshot snapshot() const;
  const std::vector<SimRobot>& robots() const { return robots_; }
  const GridMap& map() const { return map_; }

 private:
  void finishService(RobotId id);
  SimRobot& robotAt(RobotId id);
  std::vector<uint8_t> drawLosses(Timestep now);  // per robot: is its message lost this round

  GridMap map_;
  std::vector<SimRobot> robots_;
  bool occupied(const std::vector<int32_t>& cells) const;

  std::vector<uint8_t> pending_block_;           // per cell: in a closure still waiting
  std::vector<std::vector<int32_t>> pending_{};  // closures waiting for their cells to empty
  Timestep service_steps_;
  double delay_probability_;
  Rng delay_rng_;
  ExecutorFaults faults_;
  Rng link_rng_;
  std::vector<Timestep> silent_until_;  // per robot: its messages are lost while now < this
  int64_t reports_lost_{0};
  int64_t commands_lost_{0};
  StepTruth last_{};
};

}  // namespace fleet_core

#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/tasks/task_manager.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

enum class ViolationKind : uint8_t {
  // Physical, from ground truth. Robot safety should make these impossible; they guard the
  // executor itself.
  kCellShared,      // two robots in one cell
  kSwap,            // two robots exchanged cells in one step
  kTeleport,        // a robot moved more than one cell in one step
  kOffFreeCell,     // a robot stands on a shelf or a blocked cell
  kDeadRobotMoved,  // a dead robot changed cells
  // Coordination, from what the coordinator sent.
  kInvalidPlan,           // a path step to a non-adjacent cell or a shelf
  kCoordinationConflict,  // robot safety had to stop a permitted move (conflict-free planners)
  // Tasks.
  kTaskHeldTwice,       // two robots hold the same task
  kTaskDeliveredTwice,  // a task was delivered more than once
  kPhantomDelivery,     // the coordinator marked done a task no robot delivered
  kTaskLost,            // the coordinator does not track every order issued
};
// Stage 5 adds a check for illegal entry into reserved intersections.

std::string_view toString(ViolationKind kind);

struct Violation {
  Timestep t{0};
  ViolationKind kind{ViolationKind::kCellShared};
  std::string detail{};
};

// Checks every step of every run. It reads ground truth and the coordinator's task table and
// shares no code with the coordinator's decisions, so the system under test never grades itself.
class InvariantChecker {
 public:
  // conflict_free: the planner promises conflict-free paths (every planner except
  // independent_astar). Only then is a safety stop a violation; otherwise it is a conflict that
  // metrics count.
  explicit InvariantChecker(bool conflict_free);

  // Call once per step, after the robots moved and after the coordinator's tick for that step.
  // orders_issued: orders created so far, all of which the coordinator has been given.
  std::vector<Violation> check(const GridMap& map, const StepTruth& step,
                               const TruthSnapshot& after, const TaskManager& tasks,
                               int64_t orders_issued);

 private:
  bool conflict_free_;
  std::set<TaskId> delivered_{};
};

}  // namespace fleet_core

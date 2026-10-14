#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/world/distance_oracle.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// One robot to plan for.
struct PlanAgent {
  RobotId robot{-1};
  Cell start{};
  Cell goal{};
  int32_t priority{0};  // higher plans first in prioritized schemes: the age of its order, -1 idle
};

// Inputs for one planning call.
//
// Paths are timed: cells[k] is the cell at step start + k, and a robot holds its last cell after
// its path ends. A path may start or end on an endpoint (station, parking) but never passes
// through one (well-formed MAPD), so a robot parked on an endpoint is in nobody's way.
struct PlanningRequest {
  const WorldModel& world;
  const DistanceOracle& distances;  // heuristic
  std::vector<PlanAgent> agents{};  // robots to (re)plan, ascending id
  std::vector<Path> fixed{};        // committed paths of robots not replanned: moving obstacles
  Timestep start{0};                // step at which every new path starts
  int32_t window{20};               // >= 1: resolve conflicts this many steps ahead; 0: to the goal
  int64_t expansion_budget{200000};  // deterministic effort limit; exceeding it fails the call
  double suboptimality{1.0};         // ECBS bound w >= 1
};

// Multi-agent path planner (planning.method):
//   independent_astar    each robot alone, shortest path, ignores the others (the baseline)
//   prioritized          robots one at a time, oldest order first; each avoids the paths of those
//                        before it all the way to its goal, whatever the window; restarts with a
//                        failed robot moved to the front
//   windowed_coop_astar  the same, resolved only `window` steps ahead (WHCA*)
//   cbs                  Conflict-Based Search: optimal total cost, within the window
//   ecbs                 bounded-suboptimal CBS: total cost within `suboptimality` of optimal
//   pibt                 Priority Inheritance with Backtracking: one step for every robot at a
//                        time, repeated `window` steps ahead; needs window >= 1
class PathPlanner {
 public:
  virtual ~PathPlanner() = default;

  virtual std::string_view name() const = 0;

  // On success: one path per agent in request order, each starting at the agent's start cell.
  // Paths end on the goal when it is reached in time, and never with a wait: holding the last
  // cell is implied. Coordinated planners return paths free of vertex and swap conflicts with
  // each other and with `fixed`: for all time with window 0 and with prioritized, otherwise up
  // to step start + window, with no path longer than window + 1 cells. independent_astar, the
  // uncoordinated baseline, ignores other robots. No plan within the budget: success = false
  // and no paths.
  virtual PlanResult plan(const PlanningRequest& request) = 0;
};

std::unique_ptr<PathPlanner> makePlanner(const PlanningConfig& config);

// Whether a method resolves conflicts between robots (every method but independent_astar).
bool isCoordinated(PlanningMethod method);

// Where a robot following `path` is at step t: its first cell before the path starts, its last
// cell after the path ends.
Cell positionAt(const Path& path, Timestep t);

// Two robots in one cell at step t (vertex), or exchanging cells during step t - 1 (swap).
struct PathConflict {
  size_t first{0};   // index into the paths checked
  size_t second{0};  // first < second
  Timestep t{0};
  Cell cell{};  // the shared cell; for a swap, where `first` is at step t
  bool swap{false};
};

// The earliest conflict among `paths` from step `from` to step `until`, inclusive, or none.
// Robots hold their last cell after their paths end. Following into a cell another robot leaves
// in the same step is not a conflict.
std::optional<PathConflict> firstConflict(const std::vector<Path>& paths, Timestep from,
                                          Timestep until);

}  // namespace fleet_core

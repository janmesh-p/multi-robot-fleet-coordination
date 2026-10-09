#pragma once

#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// One robot to plan for.
struct PlanAgent {
  RobotId robot{-1};
  Cell start{};
  Cell goal{};
  int32_t priority{0};  // higher plans first in prioritized schemes (task age, starvation guard)
};

// Inputs for one planning call.
struct PlanningRequest {
  const WorldModel& world;
  std::vector<PlanAgent> agents{};   // robots to (re)plan, ascending id
  std::vector<Path> fixed{};         // committed paths of robots not replanned: moving obstacles
  Timestep start{0};                 // step at which every new path starts
  int32_t window{20};                // windowed planners resolve conflicts this far ahead
  int64_t expansion_budget{200000};  // deterministic effort limit; exceeding it fails the call
  double suboptimality{1.0};         // ECBS bound w >= 1
};

// Multi-agent path planner.
// Implementations: independent_astar, prioritized, windowed_coop_astar, cbs, ecbs, pibt
// (selected by planning.method).
class PathPlanner {
 public:
  virtual ~PathPlanner() = default;

  virtual std::string_view name() const = 0;

  // On success: one path per agent in request order, each starting at the agent's start cell.
  // Paths are free of vertex and swap conflicts with each other and with `fixed`, except for
  // independent_astar, the uncoordinated baseline. On budget exhaustion: success = false and
  // no paths; the coordinator keeps the affected robots waiting.
  virtual PlanResult plan(const PlanningRequest& request) = 0;
};

// Throws std::runtime_error for methods that are not implemented yet.
std::unique_ptr<PathPlanner> makePlanner(const PlanningConfig& config);

}  // namespace fleet_core

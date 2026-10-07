#pragma once

#include <memory>
#include <string>
#include <vector>

#include "fleet_core/grid_map.hpp"
#include "fleet_core/types.hpp"

namespace fleet_core {

struct PlanRequest {
  std::vector<RobotId> robots{};
  std::vector<Cell> starts{};
  std::vector<Cell> goals{};
  Timestep start_time{0};
  double time_limit_ms{1000.0};
  double suboptimality{1.0};  // ECBS w; ignored by optimal planners
};

// Stages 2 and 4. Implementations: independent_astar, prioritized,
// cooperative_astar, cbs, ecbs.
class MAPFPlanner {
 public:
  virtual ~MAPFPlanner() = default;
  virtual std::string name() const = 0;
  virtual PlanResult plan(const GridMap& map, const PlanRequest& req) = 0;
};

std::unique_ptr<MAPFPlanner> makePlanner(const std::string& name);

}  // namespace fleet_core

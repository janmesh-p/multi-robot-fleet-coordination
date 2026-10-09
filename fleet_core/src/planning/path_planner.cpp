#include "fleet_core/planning/path_planner.hpp"

#include <stdexcept>
#include <string>

namespace fleet_core {

// Implementations register here as they land (independent_astar first, then Stage 4).
std::unique_ptr<PathPlanner> makePlanner(const PlanningConfig& config) {
  throw std::runtime_error("planning method '" + std::string(toString(config.method)) +
                           "' is not implemented yet");
}

}  // namespace fleet_core

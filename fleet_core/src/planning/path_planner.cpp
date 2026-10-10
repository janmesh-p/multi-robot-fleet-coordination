#include "fleet_core/planning/path_planner.hpp"

#include <stdexcept>
#include <string>

#include "planners.hpp"

namespace fleet_core {

std::unique_ptr<PathPlanner> makePlanner(const PlanningConfig& config) {
  switch (config.method) {
    case PlanningMethod::kIndependentAStar:
      return makeIndependentAStar();
    case PlanningMethod::kPrioritized:
    case PlanningMethod::kWindowedCoopAStar:
    case PlanningMethod::kCbs:
    case PlanningMethod::kEcbs:
    case PlanningMethod::kPibt:
      break;  // Stage 4
  }
  throw std::runtime_error("planning method '" + std::string(toString(config.method)) +
                           "' is not implemented yet");
}

}  // namespace fleet_core

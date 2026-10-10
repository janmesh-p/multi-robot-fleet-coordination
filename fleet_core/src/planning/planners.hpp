#pragma once

// Private to fleet_core: concrete planners, reached through makePlanner().
#include <memory>

#include "fleet_core/planning/path_planner.hpp"

namespace fleet_core {

std::unique_ptr<PathPlanner> makeIndependentAStar();

}  // namespace fleet_core

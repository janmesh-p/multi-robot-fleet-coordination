#pragma once

// Private to fleet_core: concrete planners, reached through makePlanner().
#include <memory>

#include "fleet_core/planning/path_planner.hpp"

namespace fleet_core {

std::unique_ptr<PathPlanner> makeIndependentAStar();
std::unique_ptr<PathPlanner> makePrioritized(bool windowed);  // windowed: WHCA*
std::unique_ptr<PathPlanner> makeCbs(bool bounded);           // bounded: ECBS
std::unique_ptr<PathPlanner> makePibt();

}  // namespace fleet_core

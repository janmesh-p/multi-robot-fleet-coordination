#include "fleet_core/traffic_manager.hpp"

namespace fleet_core {

TrafficManager::TrafficManager(const GridMap& map) : map_(map) {}

bool TrafficManager::reserve(const Path& /*path*/) {
  // TODO(stage5): check and insert (cell, t) reservations.
  return true;
}

void TrafficManager::release(RobotId /*robot*/) {
  // TODO(stage5)
}

std::vector<RobotId> TrafficManager::detectDeadlock(const std::vector<RobotState>& /*robots*/,
                                                    const std::vector<Path>& /*paths*/,
                                                    Timestep /*t*/) const {
  // TODO(stage5): build wait-for graph, return robots on a cycle.
  return {};
}

std::optional<RobotId> TrafficManager::owner(const Cell& /*c*/, Timestep /*t*/) const {
  return std::nullopt;
}

}  // namespace fleet_core

#include "fleet_core/traffic/traffic_manager.hpp"

#include <cstddef>
#include <stdexcept>

namespace fleet_core {

TrafficManager::TrafficManager(const FeatureToggles& features) : features_(features) {}

void TrafficManager::commit(const std::vector<CommittedPlan>& plans) {
  for (const CommittedPlan& plan : plans) plans_[plan.robot] = plan;
}

void TrafficManager::release(RobotId robot) { plans_.erase(robot); }

std::vector<int32_t> TrafficManager::permits(const WorldModel& world) const {
  std::vector<int32_t> out(static_cast<size_t>(world.fleetSize()), -1);
  for (const auto& [robot, plan] : plans_) {
    if (robot < 0 || robot >= world.fleetSize()) continue;
    out[static_cast<size_t>(robot)] = static_cast<int32_t>(plan.path.cells.size()) - 1;
  }
  return out;
}

std::vector<RobotId> TrafficManager::deadlocked(const WorldModel& /*world*/) const {
  throw std::logic_error("TrafficManager::deadlocked is not implemented yet (Stage 5)");
}

}  // namespace fleet_core

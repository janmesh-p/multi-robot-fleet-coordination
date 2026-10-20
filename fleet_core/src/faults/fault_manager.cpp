#include "fleet_core/faults/fault_manager.hpp"

namespace fleet_core {

FaultManager::FaultManager(const FaultsConfig& config) : config_(config) {}

std::vector<RobotId> FaultManager::expired(Timestep now, const WorldModel& world) const {
  std::vector<RobotId> out;
  for (const RobotBelief& robot : world.robots()) {
    if (robot.status == RobotStatus::kDead) continue;
    if (now - robot.last_heard > config_.heartbeat_timeout_steps) out.push_back(robot.id);
  }
  return out;
}

}  // namespace fleet_core

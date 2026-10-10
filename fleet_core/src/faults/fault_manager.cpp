#include "fleet_core/faults/fault_manager.hpp"

#include <stdexcept>
#include <string>

namespace fleet_core {

FaultManager::FaultManager(const FaultsConfig& config) : config_(config) {}

FaultActions FaultManager::update(Timestep now, WorldModel& world, TaskManager& /*tasks*/) {
  for (const RobotBelief& robot : world.robots()) {
    if (robot.status == RobotStatus::kDead) continue;
    const Timestep silent = now - robot.last_heard;
    if (silent > config_.heartbeat_timeout_steps) {
      throw std::logic_error("FaultManager: robot " + std::to_string(robot.id) + " silent for " +
                             std::to_string(silent) +
                             " steps; fault recovery is not implemented yet (Stage 6)");
    }
  }
  return FaultActions{};
}

}  // namespace fleet_core

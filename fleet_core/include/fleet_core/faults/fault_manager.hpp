#pragma once

#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/tasks/task_manager.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// What FaultManager changed in one tick.
struct FaultActions {
  std::vector<RobotId> declared_dead{};
  std::vector<RobotId> reconciled{};  // heard from again after being declared dead
  std::vector<TaskId> requeued{};
};

// Coordinator-side failure handling: heartbeat leases, requeueing the tasks of dead robots,
// reconciling robots that report again after a false death.
//
// Ground-truth faults are injected into the executor backend by FaultInjector. This class only
// sees their effects: missing, late, or surprising reports.
//
// Today it only detects an expired lease and throws, because nothing can cause one yet (no
// packet loss, no robot deaths). Stage 6 replaces the throw with recovery.
class FaultManager {
 public:
  explicit FaultManager(const FaultsConfig& config);

  FaultActions update(Timestep now, WorldModel& world, TaskManager& tasks);

 private:
  FaultsConfig config_;
};

}  // namespace fleet_core

#pragma once

#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// Coordinator-side failure detection: heartbeat leases (Stage 6). A robot believed alive whose
// newest accepted report is more than heartbeat_timeout_steps old is declared dead. It may only be
// out of touch: FleetManager takes it back if it reports again.
//
// Ground-truth faults are injected into the executor backend by FaultInjector. This class only
// sees their effects: missing reports. FleetManager does the recovery.
class FaultManager {
 public:
  explicit FaultManager(const FaultsConfig& config);

  // Robots whose lease ran out at `now`, in id order.
  std::vector<RobotId> expired(Timestep now, const WorldModel& world) const;

 private:
  FaultsConfig config_;
};

}  // namespace fleet_core

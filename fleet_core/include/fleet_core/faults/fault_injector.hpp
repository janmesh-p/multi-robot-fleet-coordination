#pragma once

#include <cstddef>
#include <vector>

#include "fleet_core/common/types.hpp"

namespace fleet_core {

// Replays the scripted ground-truth faults (config faults.events) into the executor backend.
// Detection and recovery on the coordinator side is FaultManager's job.
class FaultInjector {
 public:
  // Events may come in any order.
  explicit FaultInjector(std::vector<FaultEvent> schedule);

  // Events with at <= now that were not returned before, in time order. Ties keep config order.
  std::vector<FaultEvent> due(Timestep now);

 private:
  std::vector<FaultEvent> schedule_;
  size_t cursor_{0};
};

}  // namespace fleet_core

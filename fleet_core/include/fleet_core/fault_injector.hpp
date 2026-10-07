#pragma once

#include <vector>

#include "fleet_core/types.hpp"

namespace fleet_core {

enum class FaultType : uint8_t { kRobotDeath, kBlockAisle, kUnblockAisle, kPacketLoss, kStalePose };

struct FaultEvent {
  Timestep at{0};
  FaultType type{FaultType::kRobotDeath};
  RobotId robot{-1};       // robot faults
  Cell cell{};             // aisle faults
  double probability{0.0}; // packet loss rate
  Timestep duration{0};    // stale pose / packet loss window
};

// Stage 6. Replays a scripted fault schedule (loaded from benchmark config).
class FaultInjector {
 public:
  explicit FaultInjector(std::vector<FaultEvent> schedule);
  std::vector<FaultEvent> due(Timestep t);

 private:
  std::vector<FaultEvent> schedule_;
  size_t cursor_{0};
};

}  // namespace fleet_core

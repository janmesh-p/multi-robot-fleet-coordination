#pragma once

#include <cstdint>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// A plan the coordinator has sent to a robot.
struct CommittedPlan {
  RobotId robot{-1};
  int32_t plan_id{-1};
  Path path{};
};

// Turns committed plans into motion permits, so plans stay collision-free when robots run late.
//
// Stage 5 adds the execution dependency graph, intersection reservations, wait-for deadlock
// detection, and the starvation guard, each behind its feature toggle. With all of them off,
// every planned cell is permitted: the uncoordinated Stage 2 baseline.
//
// Interface only. Implementation lands in Stage 2 (permit all) and Stage 5.
class TrafficManager {
 public:
  explicit TrafficManager(const FeatureToggles& features);

  // Replaces the committed plan of each listed robot.
  void commit(const std::vector<CommittedPlan>& plans);

  // Drops a robot's plan (robot dead or plan finished).
  void release(RobotId robot);

  // permitted_until per robot (index == RobotId); -1 for robots without a plan.
  std::vector<int32_t> permits(const WorldModel& world) const;

  // Robots on a wait-for cycle. Empty when the fleet can make progress.
  std::vector<RobotId> deadlocked(const WorldModel& world) const;

 private:
  FeatureToggles features_;
  std::vector<CommittedPlan> plans_{};
};

}  // namespace fleet_core

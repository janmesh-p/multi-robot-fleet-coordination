#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/tasks/task_manager.hpp"
#include "fleet_core/world/distance_oracle.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// Inputs for one allocation round. FleetManager fills it from its belief state.
struct AllocationContext {
  const WorldModel& world;
  const TaskManager& tasks;
  const DistanceOracle& distances;
  std::vector<RobotId> robots{};  // available: alive and without a task, ascending id
  std::vector<TaskId> pending{};  // pending tasks, oldest first
};

// Starvation guard (features.starvation_guard), for the matching allocators: an order pending
// longer than this many steps is starving. The oldest starving orders, up to one per available
// robot, are matched first; the remaining robots then match the remaining orders as usual.
// nearest needs no guard: it always serves the oldest order first.
inline constexpr Timestep kStarvationSteps = 600;

// Assigns pending tasks to available robots. See allocation.method:
//   nearest     oldest order first, to the closest available robot (Stage 1 baseline)
//   hungarian   one minimum-cost matching of every available robot and pending order; the cost
//               of a pair is the shortest-path distance to the pickup
//   cost_aware  the same matching, plus a congestion penalty per pair
//               (allocation.congestion_weight)
//   learned     the same matching, with a trained model's predicted steps to the pickup as
//               the cost (allocation.model_file, docs/learning_track.md)
class TaskAllocator {
 public:
  virtual ~TaskAllocator() = default;

  virtual std::string_view name() const = 0;

  // Each robot and each task appears at most once; tasks left out stay pending.
  // Must be deterministic: the same context always gives the same result.
  virtual std::vector<Assignment> assign(const AllocationContext& context) = 0;
};

// The allocator config.allocation.method names. learned loads allocation.model_file and throws
// ModelError (cost_model.hpp) when the file breaks the format.
std::unique_ptr<TaskAllocator> makeAllocator(const Config& config);

}  // namespace fleet_core

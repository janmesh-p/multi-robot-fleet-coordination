#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/tasks/task_manager.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// Inputs for one allocation round. FleetManager fills it from its belief state.
struct AllocationContext {
  const WorldModel& world;
  const TaskManager& tasks;
  std::vector<RobotId> robots{};  // available: alive and without a task, ascending id
  std::vector<TaskId> pending{};  // pending tasks, oldest first
};

// Assigns pending tasks to available robots.
// Implementations: nearest, hungarian, cost_aware (selected by allocation.method).
class TaskAllocator {
 public:
  virtual ~TaskAllocator() = default;

  virtual std::string_view name() const = 0;

  // Each robot and each task appears at most once; tasks left out stay pending.
  // Must be deterministic: the same context always gives the same result.
  virtual std::vector<Assignment> assign(const AllocationContext& context) = 0;
};

// Throws std::runtime_error for methods that are not implemented yet.
std::unique_ptr<TaskAllocator> makeAllocator(const AllocationConfig& config,
                                             const FeatureToggles& features);

}  // namespace fleet_core

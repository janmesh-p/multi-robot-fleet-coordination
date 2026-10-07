#pragma once

#include <memory>
#include <string>
#include <vector>

#include "fleet_core/grid_map.hpp"
#include "fleet_core/types.hpp"

namespace fleet_core {

// Stage 3. Implementations: nearest, hungarian, auction, cost_aware.
class TaskAllocator {
 public:
  virtual ~TaskAllocator() = default;
  virtual std::string name() const = 0;

  // Assign pending tasks to idle, alive robots.
  virtual std::vector<Assignment> assign(const std::vector<RobotState>& robots,
                                         const std::vector<Task>& pending,
                                         const GridMap& map) = 0;
};

// Throws std::invalid_argument for unknown or not-yet-implemented names.
std::unique_ptr<TaskAllocator> makeAllocator(const std::string& name);

}  // namespace fleet_core

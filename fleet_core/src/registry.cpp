#include <stdexcept>

#include "fleet_core/mapf_planner.hpp"
#include "fleet_core/task_allocator.hpp"

namespace fleet_core {

// Register implementations here as each stage lands.

std::unique_ptr<TaskAllocator> makeAllocator(const std::string& name) {
  // Stage 3: nearest, hungarian, auction, cost_aware
  throw std::invalid_argument("allocator not implemented yet: " + name);
}

std::unique_ptr<MAPFPlanner> makePlanner(const std::string& name) {
  // Stage 2: independent_astar
  // Stage 4: prioritized, cooperative_astar, cbs, ecbs
  throw std::invalid_argument("planner not implemented yet: " + name);
}

}  // namespace fleet_core

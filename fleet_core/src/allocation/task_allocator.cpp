#include "fleet_core/allocation/task_allocator.hpp"

#include <stdexcept>
#include <string>

namespace fleet_core {

// Implementations register here as they land (nearest, hungarian, cost_aware).
std::unique_ptr<TaskAllocator> makeAllocator(const AllocationConfig& config,
                                             const FeatureToggles& /*features*/) {
  throw std::runtime_error("allocation method '" + std::string(toString(config.method)) +
                           "' is not implemented yet");
}

}  // namespace fleet_core

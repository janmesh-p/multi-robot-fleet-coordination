#include "fleet_core/allocation/task_allocator.hpp"

#include <stdexcept>
#include <string>

#include "allocators.hpp"

namespace fleet_core {

std::unique_ptr<TaskAllocator> makeAllocator(const AllocationConfig& config,
                                             const FeatureToggles& /*features*/) {
  switch (config.method) {
    case AllocationMethod::kNearest:
      return makeNearestAllocator();
    case AllocationMethod::kHungarian:
    case AllocationMethod::kCostAware:
      break;  // Stage 3
  }
  throw std::runtime_error("allocation method '" + std::string(toString(config.method)) +
                           "' is not implemented yet");
}

}  // namespace fleet_core

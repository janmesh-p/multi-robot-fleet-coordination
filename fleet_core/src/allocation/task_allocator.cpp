#include "fleet_core/allocation/task_allocator.hpp"

#include <stdexcept>

#include "allocators.hpp"
#include "fleet_core/allocation/cost_model.hpp"

namespace fleet_core {

std::unique_ptr<TaskAllocator> makeAllocator(const Config& config) {
  const AllocationConfig& a = config.allocation;
  switch (a.method) {
    case AllocationMethod::kNearest:
      return makeNearestAllocator();
    case AllocationMethod::kHungarian:
      return makeHungarianAllocator(config.features.starvation_guard);
    case AllocationMethod::kCostAware:
      return makeCostAwareAllocator(a.congestion_weight, config.features.congestion_cost,
                                    config.orders.service_steps, config.features.starvation_guard);
    case AllocationMethod::kLearned:
      return makeLearnedAllocator(CostModel::load(a.model_file), config.features.starvation_guard);
  }
  throw std::logic_error("makeAllocator: unknown allocation method");
}

}  // namespace fleet_core

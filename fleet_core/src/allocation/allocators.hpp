#pragma once

// Private to fleet_core: concrete allocators, reached through makeAllocator().
#include <memory>

#include "fleet_core/allocation/task_allocator.hpp"
#include "fleet_core/common/types.hpp"

namespace fleet_core {

std::unique_ptr<TaskAllocator> makeNearestAllocator();
std::unique_ptr<TaskAllocator> makeHungarianAllocator(bool starvation_guard);

// cost = distance + weight * (robots in the box + service_steps * robots already assigned to the
// pickup); with the congestion term off (features.congestion_cost) it is plain hungarian.
std::unique_ptr<TaskAllocator> makeCostAwareAllocator(double weight, bool congestion_term,
                                                      Timestep service_steps,
                                                      bool starvation_guard);

}  // namespace fleet_core

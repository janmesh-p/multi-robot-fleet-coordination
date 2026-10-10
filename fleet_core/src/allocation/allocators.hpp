#pragma once

// Private to fleet_core: concrete allocators, reached through makeAllocator().
#include <memory>

#include "fleet_core/allocation/task_allocator.hpp"

namespace fleet_core {

std::unique_ptr<TaskAllocator> makeNearestAllocator();

}  // namespace fleet_core

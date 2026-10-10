#include <cstddef>
#include <limits>

#include "allocators.hpp"

namespace fleet_core {
namespace {

// Greedy baseline: oldest task first, to the available robot closest to its pickup.
// Ties go to the lowest robot id. Cheap, and blind to everything except distance.
class NearestAllocator final : public TaskAllocator {
 public:
  std::string_view name() const override { return "nearest"; }

  std::vector<Assignment> assign(const AllocationContext& context) override {
    std::vector<Assignment> out;
    std::vector<RobotId> free = context.robots;
    for (const TaskId task_id : context.pending) {
      if (free.empty()) break;
      const Cell pickup = context.tasks.get(task_id).order.pickup;
      size_t best = 0;
      int32_t best_distance = std::numeric_limits<int32_t>::max();
      for (size_t i = 0; i < free.size(); ++i) {
        const int32_t d = context.distances.distance(context.world.robot(free[i]).cell, pickup);
        if (d < best_distance) {  // strict: the lower id wins ties
          best = i;
          best_distance = d;
        }
      }
      out.push_back(Assignment{free[best], task_id});
      free.erase(free.begin() + static_cast<std::ptrdiff_t>(best));
    }
    return out;
  }
};

}  // namespace

std::unique_ptr<TaskAllocator> makeNearestAllocator() {
  return std::make_unique<NearestAllocator>();
}

}  // namespace fleet_core

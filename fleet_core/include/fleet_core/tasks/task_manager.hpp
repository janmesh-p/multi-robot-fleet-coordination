#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "fleet_core/common/types.hpp"

namespace fleet_core {

// Order lifecycle: Pending -> Assigned -> PickedUp -> Done, plus requeue back to Pending when a
// robot dies (Stage 6). Any other transition is a coordinator bug and throws std::logic_error.
//
// Interface only. Implementation lands in Stage 1.
class TaskManager {
 public:
  // New order. Must be Pending with an id not seen before.
  void add(const Task& task);

  const Task& get(TaskId id) const;
  const std::vector<Task>& all() const { return tasks_; }  // ascending id

  // Pending tasks, oldest first; ties broken by id.
  std::vector<TaskId> pending() const;

  // Task assigned to or carried by the robot, if any.
  std::optional<TaskId> taskOf(RobotId robot) const;

  void assign(TaskId id, RobotId robot, Timestep now);
  void markPickedUp(TaskId id, Timestep now);
  void markDone(TaskId id, Timestep now);
  void requeue(TaskId id, Timestep now);

  int32_t doneCount() const;

 private:
  std::vector<Task> tasks_{};
};

}  // namespace fleet_core

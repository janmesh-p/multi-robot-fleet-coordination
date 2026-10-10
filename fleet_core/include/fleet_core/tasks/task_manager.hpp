#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "fleet_core/common/types.hpp"

namespace fleet_core {

// Order lifecycle: Pending -> Assigned -> PickedUp -> Done, plus requeue back to Pending when a
// robot dies (Stage 6). Any other transition is a coordinator bug and throws std::logic_error.
class TaskManager {
 public:
  // New order. Must be Pending and unassigned, with an id larger than every id before it.
  void add(const Task& task);

  bool contains(TaskId id) const;
  const Task& get(TaskId id) const;                        // throws std::out_of_range
  const std::vector<Task>& all() const { return tasks_; }  // ascending id

  // Pending tasks, oldest first; ties broken by id.
  std::vector<TaskId> pending() const;
  int32_t pendingCount() const { return static_cast<int32_t>(pending_.size()); }

  // Task assigned to or carried by the robot, if any.
  std::optional<TaskId> taskOf(RobotId robot) const;

  void assign(TaskId id, RobotId robot, Timestep now);
  void markPickedUp(TaskId id, Timestep now);
  void markDone(TaskId id, Timestep now);
  void requeue(TaskId id, Timestep now);

  int32_t doneCount() const { return done_count_; }

 private:
  Task& mutableGet(TaskId id);

  std::vector<Task> tasks_{};                        // ascending id
  std::set<std::pair<Timestep, TaskId>> pending_{};  // (created_at, id): oldest first
  std::map<RobotId, TaskId> active_{};               // robot -> assigned or carried task
  int32_t done_count_{0};
};

}  // namespace fleet_core

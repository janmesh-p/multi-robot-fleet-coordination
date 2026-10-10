#include "fleet_core/tasks/task_manager.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace fleet_core {
namespace {

[[noreturn]] void badTransition(const char* action, const Task& task, const char* expected) {
  throw std::logic_error(std::string("TaskManager::") + action + ": task " +
                         std::to_string(task.order.id) + " is " +
                         std::string(toString(task.status)) + ", expected " + expected);
}

}  // namespace

void TaskManager::add(const Task& task) {
  if (task.status != TaskStatus::kPending || task.robot) {
    throw std::logic_error("TaskManager::add: task " + std::to_string(task.order.id) +
                           " must be pending and unassigned");
  }
  if (task.order.id < 0 || (!tasks_.empty() && task.order.id <= tasks_.back().order.id)) {
    throw std::logic_error("TaskManager::add: task id " + std::to_string(task.order.id) +
                           " must be >= 0 and larger than every earlier id");
  }
  tasks_.push_back(task);
  pending_.insert({task.created_at, task.order.id});
}

bool TaskManager::contains(TaskId id) const {
  const auto it = std::lower_bound(tasks_.begin(), tasks_.end(), id,
                                   [](const Task& t, TaskId v) { return t.order.id < v; });
  return it != tasks_.end() && it->order.id == id;
}

const Task& TaskManager::get(TaskId id) const {
  const auto it = std::lower_bound(tasks_.begin(), tasks_.end(), id,
                                   [](const Task& t, TaskId v) { return t.order.id < v; });
  if (it == tasks_.end() || it->order.id != id) {
    throw std::out_of_range("TaskManager: unknown task " + std::to_string(id));
  }
  return *it;
}

Task& TaskManager::mutableGet(TaskId id) { return const_cast<Task&>(get(id)); }

std::vector<TaskId> TaskManager::pending() const {
  std::vector<TaskId> ids;
  ids.reserve(pending_.size());
  for (const auto& entry : pending_) ids.push_back(entry.second);
  return ids;
}

std::optional<TaskId> TaskManager::taskOf(RobotId robot) const {
  const auto it = active_.find(robot);
  if (it == active_.end()) return std::nullopt;
  return it->second;
}

void TaskManager::assign(TaskId id, RobotId robot, Timestep now) {
  Task& task = mutableGet(id);
  if (task.status != TaskStatus::kPending) badTransition("assign", task, "pending");
  if (active_.count(robot) > 0) {
    throw std::logic_error("TaskManager::assign: robot " + std::to_string(robot) +
                           " already holds task " + std::to_string(active_.at(robot)));
  }
  pending_.erase({task.created_at, id});
  task.status = TaskStatus::kAssigned;
  task.robot = robot;
  task.assigned_at = now;
  active_[robot] = id;
}

void TaskManager::markPickedUp(TaskId id, Timestep now) {
  Task& task = mutableGet(id);
  if (task.status != TaskStatus::kAssigned) badTransition("markPickedUp", task, "assigned");
  task.status = TaskStatus::kPickedUp;
  task.picked_at = now;
}

void TaskManager::markDone(TaskId id, Timestep now) {
  Task& task = mutableGet(id);
  if (task.status != TaskStatus::kPickedUp) badTransition("markDone", task, "picked_up");
  task.status = TaskStatus::kDone;
  task.done_at = now;
  active_.erase(*task.robot);
  ++done_count_;
}

void TaskManager::requeue(TaskId id, Timestep /*now*/) {
  Task& task = mutableGet(id);
  if (task.status != TaskStatus::kAssigned && task.status != TaskStatus::kPickedUp) {
    badTransition("requeue", task, "assigned or picked_up");
  }
  active_.erase(*task.robot);
  task.status = TaskStatus::kPending;
  task.robot.reset();
  task.assigned_at.reset();
  task.picked_at.reset();
  pending_.insert({task.created_at, id});  // keeps its age, so it is first in line again
}

}  // namespace fleet_core

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/tasks/task_manager.hpp"

namespace fleet_core {
namespace {

Task order(TaskId id, Timestep created_at) {
  Task t;
  t.order = TaskOrder{id, {0, 0}, {0, 4}};
  t.created_at = created_at;
  return t;
}

TEST(TaskManager, FullLifecycle) {
  TaskManager tasks;
  tasks.add(order(3, 10));
  EXPECT_TRUE(tasks.contains(3));
  EXPECT_FALSE(tasks.contains(4));
  EXPECT_EQ(tasks.pending(), std::vector<TaskId>{3});

  tasks.assign(3, 1, 11);
  EXPECT_EQ(tasks.get(3).status, TaskStatus::kAssigned);
  EXPECT_EQ(tasks.get(3).robot, std::optional<RobotId>(1));
  EXPECT_EQ(tasks.taskOf(1), std::optional<TaskId>(3));
  EXPECT_TRUE(tasks.pending().empty());

  tasks.markPickedUp(3, 15);
  tasks.markDone(3, 20);
  const Task& done = tasks.get(3);
  EXPECT_EQ(done.status, TaskStatus::kDone);
  EXPECT_EQ(done.assigned_at, std::optional<Timestep>(11));
  EXPECT_EQ(done.picked_at, std::optional<Timestep>(15));
  EXPECT_EQ(done.done_at, std::optional<Timestep>(20));
  EXPECT_FALSE(tasks.taskOf(1).has_value());
  EXPECT_EQ(tasks.doneCount(), 1);
}

TEST(TaskManager, PendingIsOldestFirst) {
  TaskManager tasks;
  tasks.add(order(1, 5));
  tasks.add(order(2, 3));  // newer id, older order
  tasks.add(order(4, 5));
  EXPECT_EQ(tasks.pending(), (std::vector<TaskId>{2, 1, 4}));
}

TEST(TaskManager, RequeueKeepsTheTasksPlaceInLine) {
  TaskManager tasks;
  tasks.add(order(1, 0));
  tasks.add(order(2, 5));
  tasks.assign(1, 0, 6);
  tasks.markPickedUp(1, 8);
  tasks.requeue(1, 9);
  EXPECT_EQ(tasks.pending(), (std::vector<TaskId>{1, 2}));
  EXPECT_EQ(tasks.get(1).status, TaskStatus::kPending);
  EXPECT_FALSE(tasks.get(1).robot.has_value());
  EXPECT_FALSE(tasks.get(1).picked_at.has_value());
  EXPECT_FALSE(tasks.taskOf(0).has_value());
}

TEST(TaskManager, RejectsInvalidTransitions) {
  TaskManager tasks;
  tasks.add(order(1, 0));
  tasks.add(order(2, 0));
  EXPECT_THROW(tasks.markPickedUp(1, 1), std::logic_error);  // not assigned
  EXPECT_THROW(tasks.markDone(1, 1), std::logic_error);
  EXPECT_THROW(tasks.requeue(1, 1), std::logic_error);
  tasks.assign(1, 0, 1);
  EXPECT_THROW(tasks.assign(1, 1, 1), std::logic_error);  // already assigned
  EXPECT_THROW(tasks.assign(2, 0, 1), std::logic_error);  // robot already busy
  EXPECT_THROW(tasks.markDone(1, 1), std::logic_error);   // not picked up
  EXPECT_THROW(tasks.get(9), std::out_of_range);
}

TEST(TaskManager, RejectsBadNewTasks) {
  TaskManager tasks;
  tasks.add(order(5, 0));
  EXPECT_THROW(tasks.add(order(5, 1)), std::logic_error);  // repeated id
  EXPECT_THROW(tasks.add(order(4, 1)), std::logic_error);  // ids must grow
  Task assigned = order(6, 1);
  assigned.status = TaskStatus::kAssigned;
  EXPECT_THROW(tasks.add(assigned), std::logic_error);
}

}  // namespace
}  // namespace fleet_core

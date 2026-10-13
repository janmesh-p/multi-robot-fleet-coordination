#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/allocation/pair_features.hpp"
#include "fleet_core/world/distance_oracle.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

Task order(TaskId id, Cell pickup, Cell dropoff, Timestep created_at) {
  Task t;
  t.order = TaskOrder{id, pickup, dropoff};
  t.created_at = created_at;
  return t;
}

std::vector<int32_t> values(const PairFeatures& f) {
  std::vector<int32_t> out;
  for (size_t i = 0; i < kPairFeatureCount; ++i) out.push_back(pairFeature(f, i));
  return out;
}

// The example row of docs/learning_track.md (section 3), rebuilt on maps/warehouse_small.txt.
// Robot 3 on parking cell (0, 9) is offered order 57 at pickup (5, 0), dropoff (5, 37).
TEST(PairFeatures, ReproducesTheContractExampleRow) {
  const GridMap map = GridMap::fromFile((kRepo / "maps" / "warehouse_small.txt").string());
  const std::vector<Cell> cells{{14, 20}, {14, 24},  // 0, 1: far away, busy
                                {0, 8},   // 2: next to robot 3 and inside the box, available
                                {0, 9},   // 3: the robot
                                {1, 11},  // 4: near robot 3 but outside the box, available
                                {4, 1},   // 5: inside the box and near the pickup, available
                                {14, 28}, {14, 30}, {14, 32}, {14, 34}};  // 6 to 9: far away, busy
  WorldModel world(map, cells);
  world.setNow(612);
  const DistanceOracle d(map, true);
  TaskManager tasks;
  // Six busy robots: one heading to pickup (5, 0), two to (2, 0), three already carrying.
  tasks.add(order(50, {5, 0}, {2, 37}, 590));
  tasks.add(order(51, {2, 0}, {9, 37}, 591));
  tasks.add(order(52, {2, 0}, {12, 37}, 592));
  tasks.add(order(53, {9, 0}, {5, 37}, 593));
  tasks.add(order(54, {12, 0}, {2, 37}, 594));
  tasks.add(order(55, {9, 0}, {9, 37}, 595));
  // Three pending orders, 57 included.
  tasks.add(order(56, {2, 0}, {2, 37}, 600));
  tasks.add(order(57, {5, 0}, {5, 37}, 607));
  tasks.add(order(58, {9, 0}, {12, 37}, 609));
  const std::vector<std::pair<TaskId, RobotId>> busy{{50, 0}, {51, 1}, {52, 6},
                                                     {53, 7}, {54, 8}, {55, 9}};
  for (const auto& [task, robot] : busy) tasks.assign(task, robot, 600);
  for (const TaskId carried : {53, 54, 55}) tasks.markPickedUp(carried, 605);

  const AllocationContext context{world, tasks, d, {2, 3, 4, 5}, tasks.pending()};
  const PairFeatureTable table(context);
  // 612,57,3,14,14,39,0,9,5,0,1,2,1,2,39,1,0,3,4,6,10,5,...
  EXPECT_EQ(values(table.pair(3, 57)),
            (std::vector<int32_t>{14, 14, 39, 0, 9, 5, 0, 1, 2, 1, 2, 39, 1, 0, 3, 4, 6, 10, 5}));
}

TEST(PairFeatures, NamesMatchTheContractColumns) {
  const std::vector<std::string> expected{
      "dist_to_pickup",     "manhattan_to_pickup", "dist_pickup_to_dropoff",
      "robot_row",          "robot_col",           "pickup_row",
      "pickup_col",         "robot_on_endpoint",   "robots_near_robot",
      "robots_near_pickup", "robots_in_box",       "box_free_cells",
      "pickup_assigned",    "pickup_pending",      "pending_orders",
      "available_robots",   "busy_robots",         "alive_robots",
      "order_age"};
  ASSERT_EQ(pairFeatureNames().size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(pairFeatureNames()[i], expected[i]);
    EXPECT_EQ(pairFeatureIndex(expected[i]), i);
  }
  EXPECT_FALSE(pairFeatureIndex("steps_to_pickup"));
  EXPECT_FALSE(pairFeatureIndex("robot"));
  EXPECT_THROW(pairFeature(PairFeatures{}, kPairFeatureCount), std::out_of_range);
}

TEST(PairFeatures, DeadRobotsAreNotCountedAndManhattanWithoutTheOracle) {
  const GridMap map = GridMap::fromString(
      "........\n"
      ".######.\n"
      "P......D\n");
  WorldModel world(map, {{0, 0}, {0, 1}, {2, 3}});
  world.markDead(1);  // next to robot 0, but dead
  world.setNow(40);
  const DistanceOracle manhattan_only(map, false);
  TaskManager tasks;
  tasks.add(order(0, {2, 0}, {2, 7}, 30));
  const AllocationContext context{world, tasks, manhattan_only, {0, 2}, tasks.pending()};
  const PairFeatures f = PairFeatureTable(context).pair(0, 0);
  EXPECT_EQ(f.dist_to_pickup, 2);
  EXPECT_EQ(f.manhattan_to_pickup, 2);
  EXPECT_EQ(f.robots_near_robot, 0);   // robot 1 is 1 away but dead; robot 2 is 5 away
  EXPECT_EQ(f.robots_near_pickup, 1);  // robot 2 is 3 away from (2, 0)
  EXPECT_EQ(f.robots_in_box, 0);
  EXPECT_EQ(f.alive_robots, 2);
  EXPECT_EQ(f.available_robots, 2);
  EXPECT_EQ(f.busy_robots, 0);
  EXPECT_EQ(f.order_age, 10);
  EXPECT_EQ(f.box_free_cells, 3);  // (0,0), (1,0), (2,0)
}

TEST(PairFeatures, RobotAlreadyOnThePickup) {
  const GridMap map = GridMap::fromString("P....D\n");
  WorldModel world(map, {{0, 0}});
  const DistanceOracle d(map, true);
  TaskManager tasks;
  tasks.add(order(0, {0, 0}, {0, 5}, 0));
  const AllocationContext context{world, tasks, d, {0}, tasks.pending()};
  const PairFeatures f = PairFeatureTable(context).pair(0, 0);
  EXPECT_EQ(f.dist_to_pickup, 0);
  EXPECT_EQ(f.robot_on_endpoint, 1);
  EXPECT_EQ(f.box_free_cells, 1);
  EXPECT_EQ(f.dist_pickup_to_dropoff, 5);
  EXPECT_EQ(f.pending_orders, 1);
  EXPECT_EQ(f.pickup_pending, 0);
}

}  // namespace
}  // namespace fleet_core

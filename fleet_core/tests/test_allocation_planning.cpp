#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/allocation/task_allocator.hpp"
#include "fleet_core/common/rng.hpp"
#include "fleet_core/planning/path_planner.hpp"
#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

Task order(TaskId id, Cell pickup, Timestep created_at = 0) {
  Task t;
  t.order = TaskOrder{id, pickup, {0, 9}};
  t.created_at = created_at;
  return t;
}

class Nearest : public ::testing::Test {
 protected:
  Nearest()
      : world_(GridMap::fromString("..........\n"), {{0, 0}, {0, 5}, {0, 9}}),
        d_(world_.map(), true) {}

  std::vector<Assignment> run(const std::vector<RobotId>& robots) {
    const auto allocator = makeAllocator(Config{});
    const AllocationContext context{world_, tasks_, d_, robots, tasks_.pending()};
    return allocator->assign(context);
  }

  WorldModel world_;
  DistanceOracle d_;
  TaskManager tasks_;
};

TEST_F(Nearest, OldestTaskGetsTheClosestRobot) {
  tasks_.add(order(0, {0, 8}, 0));  // oldest: robot 2 at col 9 is closest
  tasks_.add(order(1, {0, 1}, 1));  // then robot 0 at col 0
  const std::vector<Assignment> a = run({0, 1, 2});
  ASSERT_EQ(a.size(), 2U);
  EXPECT_EQ(a[0].task, 0);
  EXPECT_EQ(a[0].robot, 2);
  EXPECT_EQ(a[1].task, 1);
  EXPECT_EQ(a[1].robot, 0);
}

TEST_F(Nearest, TiesGoToTheLowerIdAndEachRobotOnce) {
  tasks_.add(order(0, {0, 7}));  // robots at col 5 and col 9 are both 2 away
  tasks_.add(order(1, {0, 7}));
  tasks_.add(order(2, {0, 7}));
  tasks_.add(order(3, {0, 7}));
  const std::vector<Assignment> a = run({0, 1, 2});
  ASSERT_EQ(a.size(), 3U);
  EXPECT_EQ(a[0].robot, 1);
  EXPECT_EQ(a[1].robot, 2);
  EXPECT_EQ(a[2].robot, 0);
}

PlanResult planOne(const WorldModel& world, const DistanceOracle& d, Cell start, Cell goal,
                   int64_t budget = 200000) {
  const auto planner = makePlanner(PlanningConfig{});
  PlanningRequest request{world, d, {PlanAgent{0, start, goal, 0}}, {}, 0, 20, budget, 1.0};
  return planner->plan(request);
}

// Independent A* must return shortest paths that obey the endpoint rule, so its length must
// equal the distance oracle's: two separate implementations checking each other.
TEST(IndependentAStar, ShortestValidPathsOnAGeneratedMap) {
  const GridMap map = generateWarehouse(MapGeneratorConfig{3, 3, 4, 2, 3, 1});
  const WorldModel world(map, {});
  const DistanceOracle d(map, true);
  std::vector<Cell> cells;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    if (map.isStaticFree(map.cellAt(i))) cells.push_back(map.cellAt(i));
  }
  Rng rng(11);
  for (int trial = 0; trial < 300; ++trial) {
    const Cell start =
        cells[static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(cells.size()) - 1))];
    const Cell goal =
        cells[static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(cells.size()) - 1))];
    const PlanResult result = planOne(world, d, start, goal);
    const int32_t expected = d.distance(start, goal);
    if (expected == DistanceOracle::kUnreachable) {
      EXPECT_FALSE(result.success);
      continue;
    }
    ASSERT_TRUE(result.success);
    const std::vector<Cell>& path = result.paths.at(0).cells;
    ASSERT_EQ(static_cast<int32_t>(path.size()) - 1, expected);
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), goal);
    for (size_t k = 1; k < path.size(); ++k) {
      EXPECT_EQ(std::abs(path[k].row - path[k - 1].row) + std::abs(path[k].col - path[k - 1].col),
                1);
      EXPECT_TRUE(map.isFree(path[k]));
      if (k + 1 < path.size()) {
        EXPECT_FALSE(map.isEndpoint(path[k]));
      }
    }
  }
}

TEST(IndependentAStar, TrivialUnreachableAndBudget) {
  GridMap map = GridMap::fromString("......\n");
  WorldModel world(map, {});
  const DistanceOracle d(map, true);
  const std::vector<Cell> trivial = planOne(world, d, {0, 2}, {0, 2}).paths.at(0).cells;
  EXPECT_EQ(trivial, (std::vector<Cell>{{0, 2}}));
  EXPECT_FALSE(planOne(world, d, {0, 0}, {0, 5}, 3).success);  // budget too small
  world.applyObservation(CellObservation{0, {{0, 3}}, {}});    // wall in the only corridor
  EXPECT_FALSE(planOne(world, d, {0, 0}, {0, 5}).success);
}

}  // namespace
}  // namespace fleet_core

#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/allocation/cost_model.hpp"
#include "fleet_core/allocation/task_allocator.hpp"
#include "fleet_core/common/rng.hpp"
#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

const std::string kFreeFlow = R"({"format": "fleet_cost_model", "version": 1,
  "target": "steps_to_pickup", "features": ["dist_to_pickup"], "mean": [0], "scale": [1],
  "layers": [{"weights": [[0]], "bias": [0], "activation": "identity"}],
  "base_feature": "dist_to_pickup"})";

Task order(TaskId id, Cell pickup, Timestep created_at = 0, Cell dropoff = {0, 9}) {
  Task t;
  t.order = TaskOrder{id, pickup, dropoff};
  t.created_at = created_at;
  return t;
}

Config configFor(AllocationMethod method) {
  Config c;
  c.allocation.method = method;
  return c;
}

struct Scene {
  WorldModel world;
  DistanceOracle d;
  TaskManager tasks{};

  Scene(const std::string& map, const std::vector<Cell>& robots)
      : world(GridMap::fromString(map), robots), d(world.map(), true) {}

  std::vector<Assignment> run(TaskAllocator& allocator, const std::vector<RobotId>& robots) {
    const AllocationContext context{world, tasks, d, robots, tasks.pending()};
    return allocator.assign(context);
  }

  int32_t total(const std::vector<Assignment>& assignments) const {
    int32_t sum = 0;
    for (const Assignment& a : assignments) {
      sum += d.distance(world.robot(a.robot).cell, tasks.get(a.task).order.pickup);
    }
    return sum;
  }
};

const char* const kCorridor = "..........\n";

// The greedy trap: nearest gives the older order to the first robot that is closest, then the
// other robot has to cross the corridor.
TEST(Hungarian, AvoidsTheGreedyTrap) {
  Scene s(kCorridor, {{0, 2}, {0, 6}});
  s.tasks.add(order(0, {0, 4}, 0));  // older: both robots are 2 away, robot 0 wins the tie
  s.tasks.add(order(1, {0, 0}, 1));
  const auto nearest = makeAllocator(configFor(AllocationMethod::kNearest));
  const auto hungarian = makeAllocator(configFor(AllocationMethod::kHungarian));
  EXPECT_EQ(s.total(s.run(*nearest, {0, 1})), 2 + 6);
  const std::vector<Assignment> best = s.run(*hungarian, {0, 1});
  ASSERT_EQ(best.size(), 2U);
  EXPECT_EQ(s.total(best), 2 + 2);
  EXPECT_EQ(best[0].robot, 0);
  EXPECT_EQ(best[0].task, 1);
  EXPECT_EQ(best[1].robot, 1);
  EXPECT_EQ(best[1].task, 0);
}

TEST(Hungarian, MoreRobotsThanOrdersSendsTheClosest) {
  Scene s(kCorridor, {{0, 0}, {0, 5}, {0, 9}});
  s.tasks.add(order(0, {0, 8}));  // 8, 3, and 1 away
  const auto hungarian = makeAllocator(configFor(AllocationMethod::kHungarian));
  const std::vector<Assignment> a = s.run(*hungarian, {0, 1, 2});
  ASSERT_EQ(a.size(), 1U);
  EXPECT_EQ(a[0].robot, 2);
}

TEST(Hungarian, NeverPairsAnUnreachablePickup) {
  Scene s("..#..\n", {{0, 0}});
  s.tasks.add(order(0, {0, 4}, 0, {0, 3}));
  const auto hungarian = makeAllocator(configFor(AllocationMethod::kHungarian));
  EXPECT_TRUE(s.run(*hungarian, {0}).empty());
}

// On random rounds the optimal matching's total distance is never above nearest's.
TEST(Hungarian, OptimalAndNeverWorseThanNearestOnRandomRounds) {
  const GridMap map = generateWarehouse(MapGeneratorConfig{3, 2, 3, 2, 2, 1});
  std::vector<Cell> free;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    if (map.isStaticFree(map.cellAt(i))) free.push_back(map.cellAt(i));
  }
  Rng rng(3);
  const auto nearest = makeAllocator(configFor(AllocationMethod::kNearest));
  const auto hungarian = makeAllocator(configFor(AllocationMethod::kHungarian));
  for (int round = 0; round < 200; ++round) {
    const auto robots = static_cast<size_t>(rng.uniformInt(1, 5));
    std::vector<Cell> cells;
    while (cells.size() < robots) {
      const Cell c =
          free[static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(free.size()) - 1))];
      if (std::find(cells.begin(), cells.end(), c) == cells.end()) cells.push_back(c);
    }
    Scene s(map.toString(), cells);
    const auto orders = rng.uniformInt(1, 5);
    for (TaskId id = 0; id < orders; ++id) {
      const Cell pickup = map.pickups()[static_cast<size_t>(
          rng.uniformInt(0, static_cast<int64_t>(map.pickups().size()) - 1))];
      s.tasks.add(order(id, pickup, id, map.dropoffs()[0]));
    }
    std::vector<RobotId> ids;
    for (size_t i = 0; i < robots; ++i) ids.push_back(static_cast<RobotId>(i));
    const std::vector<Assignment> h = s.run(*hungarian, ids);
    const std::vector<Assignment> g = s.run(*nearest, ids);
    EXPECT_EQ(h.size(), g.size());
    EXPECT_LE(s.total(h), s.total(g)) << "round " << round;
  }
}

// One robot, two orders. The closer pickup already has a robot heading to it, so cost_aware
// pays service_steps for the queue and takes the other order; hungarian does not look.
TEST(CostAware, SteersAwayFromABusyPickup) {
  Scene s(kCorridor, {{0, 5}, {0, 0}});
  s.tasks.add(order(0, {0, 3}, 0));  // robot 1 is on its way here
  s.tasks.add(order(1, {0, 3}, 1));  // 2 away from robot 0, but the station is busy
  s.tasks.add(order(2, {0, 8}, 2));  // 3 away
  s.tasks.assign(0, 1, 0);
  Config c = configFor(AllocationMethod::kCostAware);
  const auto cost_aware = makeAllocator(c);
  const auto hungarian = makeAllocator(configFor(AllocationMethod::kHungarian));
  const std::vector<Assignment> picked = s.run(*cost_aware, {0});
  ASSERT_EQ(picked.size(), 1U);
  EXPECT_EQ(picked[0].task, 2);  // 3 + 0 beats 2 + 1 * (0 robots in box + 2 * 1 queued)
  EXPECT_EQ(s.run(*hungarian, {0})[0].task, 1);

  c.features.congestion_cost = false;  // the ablation: plain distance again
  EXPECT_EQ(s.run(*makeAllocator(c), {0})[0].task, 1);
  c.features.congestion_cost = true;
  c.allocation.congestion_weight = 0.0;
  EXPECT_EQ(s.run(*makeAllocator(c), {0})[0].task, 1);
}

TEST(Learned, FreeFlowModelMatchesHungarianExactly) {
  const auto learned = makeLearnedAllocator(CostModel::parse(kFreeFlow));
  const auto hungarian = makeAllocator(configFor(AllocationMethod::kHungarian));
  EXPECT_EQ(learned->name(), "learned");
  const GridMap map = generateWarehouse(MapGeneratorConfig{3, 2, 3, 2, 2, 1});
  Rng rng(5);
  for (int round = 0; round < 100; ++round) {
    std::vector<Cell> cells;
    for (const Cell& c : map.parking()) {
      if (rng.bernoulli(0.3)) cells.push_back(c);
    }
    if (cells.empty()) cells.push_back(map.parking()[0]);
    Scene s(map.toString(), cells);
    for (TaskId id = 0; id < 4; ++id) {
      s.tasks.add(order(id, map.pickups()[static_cast<size_t>(id) % map.pickups().size()], id,
                        map.dropoffs()[0]));
    }
    std::vector<RobotId> ids;
    for (size_t i = 0; i < cells.size(); ++i) ids.push_back(static_cast<RobotId>(i));
    const std::vector<Assignment> a = s.run(*learned, ids);
    const std::vector<Assignment> b = s.run(*hungarian, ids);
    ASSERT_EQ(a.size(), b.size());
    for (size_t k = 0; k < a.size(); ++k) {
      EXPECT_EQ(a[k].robot, b[k].robot);
      EXPECT_EQ(a[k].task, b[k].task);
    }
  }
}

TEST(AllocatorFactory, NamesAndModelLoading) {
  EXPECT_EQ(makeAllocator(configFor(AllocationMethod::kNearest))->name(), "nearest");
  EXPECT_EQ(makeAllocator(configFor(AllocationMethod::kHungarian))->name(), "hungarian");
  EXPECT_EQ(makeAllocator(configFor(AllocationMethod::kCostAware))->name(), "cost_aware");

  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / ("fleet_core_model_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "free_flow.json") << kFreeFlow;
  std::ofstream(dir / "broken.json") << "{\"format\": \"fleet_cost_model\"}";
  Config c = configFor(AllocationMethod::kLearned);
  c.allocation.model_file = dir / "free_flow.json";
  EXPECT_EQ(makeAllocator(c)->name(), "learned");
  c.allocation.model_file = dir / "broken.json";
  EXPECT_THROW(makeAllocator(c), ModelError);
  c.allocation.model_file = dir / "missing.json";
  EXPECT_THROW(makeAllocator(c), ModelError);
  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace fleet_core

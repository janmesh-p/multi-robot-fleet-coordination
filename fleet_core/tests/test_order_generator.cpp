#include <algorithm>
#include <cstddef>
#include <set>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/tasks/order_generator.hpp"

namespace fleet_core {
namespace {

GridMap stationMap() {
  return GridMap::fromString(
      "P...D\n"
      ".#.#.\n"
      "P...D\n");
}

bool contains(const std::vector<Cell>& cells, const Cell& cell) {
  return std::find(cells.begin(), cells.end(), cell) != cells.end();
}

TEST(OrderGenerator, SameSeedSameOrders) {
  const GridMap map = stationMap();
  OrderGenerator a(map, 0.7, 11);
  OrderGenerator b(map, 0.7, 11);
  for (Timestep t = 0; t < 500; ++t) {
    const std::vector<Task> x = a.step(t);
    const std::vector<Task> y = b.step(t);
    ASSERT_EQ(x.size(), y.size());
    for (size_t i = 0; i < x.size(); ++i) {
      ASSERT_EQ(x[i].order.id, y[i].order.id);
      ASSERT_TRUE(x[i].order.pickup == y[i].order.pickup);
      ASSERT_TRUE(x[i].order.dropoff == y[i].order.dropoff);
    }
  }
}

TEST(OrderGenerator, MatchesArrivalRate) {
  const GridMap map = stationMap();
  OrderGenerator gen(map, 0.4, 3);
  constexpr int kSteps = 20000;
  size_t total = 0;
  for (Timestep t = 0; t < kSteps; ++t) total += gen.step(t).size();
  EXPECT_NEAR(static_cast<double>(total) / kSteps, 0.4, 0.02);  // about 4.5 sigma
}

TEST(OrderGenerator, OrdersAreWellFormed) {
  const GridMap map = stationMap();
  OrderGenerator gen(map, 1.5, 5);
  TaskId expected_id = 0;
  std::set<Cell> pickups_seen;
  for (Timestep t = 0; t < 200; ++t) {
    for (const Task& task : gen.step(t)) {
      EXPECT_EQ(task.order.id, expected_id++);
      EXPECT_EQ(task.created_at, t);
      EXPECT_EQ(task.status, TaskStatus::kPending);
      EXPECT_FALSE(task.robot.has_value());
      EXPECT_TRUE(contains(map.pickups(), task.order.pickup));
      EXPECT_TRUE(contains(map.dropoffs(), task.order.dropoff));
      pickups_seen.insert(task.order.pickup);
    }
  }
  EXPECT_EQ(pickups_seen.size(), map.pickups().size());
}

TEST(OrderGenerator, ZeroRateProducesNothing) {
  const GridMap map = stationMap();
  OrderGenerator gen(map, 0.0, 1);
  for (Timestep t = 0; t < 1000; ++t) ASSERT_TRUE(gen.step(t).empty());
}

TEST(OrderGenerator, RejectsBadInput) {
  const GridMap no_stations = GridMap::fromString("...\n...\n");
  EXPECT_THROW(OrderGenerator(no_stations, 0.1, 1), std::invalid_argument);
  const GridMap map = stationMap();
  EXPECT_THROW(OrderGenerator(map, -0.1, 1), std::invalid_argument);
}

}  // namespace
}  // namespace fleet_core

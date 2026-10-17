#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "fleet_core/common/rng.hpp"
#include "fleet_core/world/distance_oracle.hpp"
#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

TEST(DistanceOracle, ExactDistances) {
  const GridMap m = GridMap::fromString(
      "....\n"
      ".##.\n"
      "....\n");
  const DistanceOracle d(m, true);
  EXPECT_EQ(d.distance({0, 0}, {0, 0}), 0);
  EXPECT_EQ(d.distance({0, 0}, {0, 3}), 3);
  EXPECT_EQ(d.distance({1, 0}, {1, 3}), 5);  // around the shelves
  EXPECT_EQ(d.distance({2, 1}, {0, 2}), 5);  // Manhattan says 3; the shelves say 5
}

// Paths may start or end on a station or parking cell but never pass through one.
TEST(DistanceOracle, NeverPassesThroughEndpoints) {
  const GridMap m = GridMap::fromString(
      ".P.\n"
      "...\n");
  const DistanceOracle d(m, true);
  EXPECT_EQ(d.distance({0, 0}, {0, 2}), 4);  // around the station, not through it
  EXPECT_EQ(d.distance({0, 0}, {0, 1}), 1);  // into the station as the goal
  EXPECT_EQ(d.distance({0, 1}, {0, 2}), 1);  // out of the station as the start
  const DistanceOracle manhattan(m, false);
  EXPECT_EQ(manhattan.distance({0, 0}, {0, 2}), 2);

  const GridMap parking = GridMap::fromString(
      "RRR\n"
      "...\n");
  const DistanceOracle p(parking, true);
  EXPECT_EQ(p.distance({0, 0}, {0, 2}), 4);  // down the aisle, not along the parking row
}

TEST(DistanceOracle, UnreachableAndOffMap) {
  const GridMap m = GridMap::fromString("..#..\n");
  const DistanceOracle d(m, true);
  EXPECT_EQ(d.distance({0, 0}, {0, 4}), DistanceOracle::kUnreachable);
  EXPECT_EQ(d.distance({0, 0}, {0, 2}), DistanceOracle::kUnreachable);  // shelf
  EXPECT_EQ(d.distance({0, 0}, {5, 5}), DistanceOracle::kUnreachable);  // outside
}

TEST(DistanceOracle, SymmetricOnAGeneratedMap) {
  const GridMap m = generateWarehouse(MapGeneratorConfig{3, 3, 4, 2, 3, 1});
  const DistanceOracle d(m, true);
  Rng rng(5);
  for (int i = 0; i < 2000; ++i) {
    const Cell a = m.cellAt(static_cast<int32_t>(rng.uniformInt(0, m.cellCount() - 1)));
    const Cell b = m.cellAt(static_cast<int32_t>(rng.uniformInt(0, m.cellCount() - 1)));
    ASSERT_EQ(d.distance(a, b), d.distance(b, a));
  }
}

// A closed cell is never entered, though a path may start on one (a robot leaving a cell that
// closed under it). Rebuilt around closures, the table stays exact.
TEST(DistanceOracle, ClosedCellsCanBeLeftButNeverEntered) {
  GridMap m = GridMap::fromString(
      ".....\n"
      ".....\n");
  m.block({0, 2});
  const DistanceOracle d(m, true);
  EXPECT_EQ(d.distance({0, 0}, {0, 4}), 6);  // around through row 1
  EXPECT_EQ(d.distance({0, 2}, {0, 4}), 2);
  EXPECT_EQ(d.distance({0, 0}, {0, 2}), DistanceOracle::kUnreachable);
  EXPECT_EQ(d.distance({0, 2}, {0, 2}), 0);
  const DistanceOracle manhattan(m, false);  // the ablation ignores closures
  EXPECT_EQ(manhattan.distance({0, 0}, {0, 4}), 4);
}

TEST(DistanceOracle, RefusesMapsTooLargeForTheTable) {
  std::string open;
  for (int r = 0; r < 70; ++r) open += std::string(70, '.') + "\n";
  const GridMap m = GridMap::fromString(open);
  EXPECT_THROW(DistanceOracle(m, true), std::invalid_argument);
  EXPECT_NO_THROW(DistanceOracle(m, false));
}

}  // namespace
}  // namespace fleet_core

#include <filesystem>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

GridMap smallMap() {
  return GridMap::fromString(
      "; test map\n"
      "P...D\n"
      ".#.#.\n"
      ".....\n");
}

TEST(GridMap, ParsesStationsAndObstacles) {
  const GridMap m = smallMap();
  EXPECT_EQ(m.rows(), 3);
  EXPECT_EQ(m.cols(), 5);
  EXPECT_EQ(m.freeCellCount(), 13);
  ASSERT_EQ(m.pickups().size(), 1U);
  EXPECT_TRUE(m.pickups()[0] == (Cell{0, 0}));
  ASSERT_EQ(m.dropoffs().size(), 1U);
  EXPECT_TRUE(m.dropoffs()[0] == (Cell{0, 4}));
  EXPECT_TRUE(m.isFree({0, 0}));
  EXPECT_FALSE(m.isFree({1, 1}));
  EXPECT_FALSE(m.isFree({-1, 0}));
  EXPECT_FALSE(m.isFree({3, 0}));
}

TEST(GridMap, NeighborsComeInFixedOrder) {
  const GridMap m = smallMap();
  EXPECT_TRUE(m.neighbors({2, 2}) == (std::vector<Cell>{{1, 2}, {2, 1}, {2, 3}}));
  EXPECT_TRUE(m.neighbors({0, 0}) == (std::vector<Cell>{{1, 0}, {0, 1}}));
}

TEST(GridMap, DynamicBlocks) {
  GridMap m = smallMap();
  m.block({0, 1});
  EXPECT_TRUE(m.isBlocked({0, 1}));
  EXPECT_FALSE(m.isFree({0, 1}));
  EXPECT_TRUE(m.isStaticFree({0, 1}));
  EXPECT_EQ(m.neighbors({0, 0}).size(), 1U);
  EXPECT_EQ(m.freeCellCount(), 13);
  m.unblock({0, 1});
  EXPECT_TRUE(m.isFree({0, 1}));
  EXPECT_THROW(m.block({9, 9}), std::out_of_range);
}

TEST(GridMap, RejectsBadInput) {
  EXPECT_THROW(GridMap::fromString(""), std::runtime_error);
  EXPECT_THROW(GridMap::fromString("; only a comment\n"), std::runtime_error);
  EXPECT_THROW(GridMap::fromString("..\n...\n"), std::runtime_error);
  EXPECT_THROW(GridMap::fromString("..x\n"), std::runtime_error);
  EXPECT_THROW(GridMap::fromFile("/no/such/map.txt"), std::runtime_error);
}

TEST(GridMap, HandlesWindowsLineEndings) {
  const GridMap m = GridMap::fromString("P.D\r\n...\r\n");
  EXPECT_EQ(m.cols(), 3);
  EXPECT_EQ(m.rows(), 2);
}

TEST(GridMap, ParkingAndEndpoints) {
  const GridMap m = GridMap::fromString("RR.P\n..#D\n");
  EXPECT_EQ(m.parking().size(), 2U);
  EXPECT_EQ(m.freeCellCount(), 7);
  EXPECT_TRUE(m.isEndpoint({0, 0}));
  EXPECT_TRUE(m.isEndpoint({0, 3}));
  EXPECT_TRUE(m.isEndpoint({1, 3}));
  EXPECT_FALSE(m.isEndpoint({0, 2}));
  EXPECT_FALSE(m.isEndpoint({1, 2}));  // shelf
  EXPECT_FALSE(m.isEndpoint({5, 5}));  // outside
  EXPECT_EQ(m.kind({1, 2}), '#');
  EXPECT_EQ(m.kind({-1, 0}), '#');
}

TEST(GridMap, IndexRoundTrips) {
  const GridMap m = smallMap();
  for (int32_t i = 0; i < m.cellCount(); ++i) EXPECT_EQ(m.index(m.cellAt(i)), i);
}

TEST(GridMap, ToStringRoundTrips) {
  const std::string text = "RRRR\nP..D\n.#..\n";
  const GridMap m = GridMap::fromString("; comment\n" + text);
  EXPECT_EQ(m.toString(), text);
  EXPECT_EQ(GridMap::fromString(m.toString()).toString(), text);
}

TEST(GridMap, LoadsShippedMaps) {
  const GridMap small = GridMap::fromFile((kRepo / "maps" / "warehouse_small.txt").string());
  EXPECT_EQ(small.rows(), 15);
  EXPECT_EQ(small.cols(), 38);
  EXPECT_EQ(small.pickups().size(), 4U);
  EXPECT_EQ(small.dropoffs().size(), 4U);
  EXPECT_EQ(small.parking().size(), 76U);
  const GridMap large = GridMap::fromFile((kRepo / "maps" / "warehouse_large.txt").string());
  EXPECT_EQ(large.rows(), 33);
  EXPECT_EQ(large.cols(), 75);
  EXPECT_EQ(large.pickups().size(), 10U);
  EXPECT_EQ(large.parking().size(), 150U);
}

}  // namespace
}  // namespace fleet_core

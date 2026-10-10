#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

// The map file without its comment lines.
std::string shippedMap(const std::string& name) {
  std::ifstream in(kRepo / "maps" / name);
  std::string line;
  std::string out;
  while (std::getline(in, line)) {
    if (!line.empty() && line[0] != ';') out += line + "\n";
  }
  return out;
}

// The shipped maps are generator output; this keeps the files and the generator in sync.
TEST(MapGenerator, ShippedMapsMatchTheGenerator) {
  EXPECT_EQ(generateWarehouse(MapGeneratorConfig{5, 4, 6, 2, 4, 1}).toString(),
            shippedMap("warehouse_small.txt"));
  EXPECT_EQ(generateWarehouse(MapGeneratorConfig{8, 10, 8, 2, 10, 1}).toString(),
            shippedMap("warehouse_large.txt"));
}

TEST(MapGenerator, LayoutsAreBridgelessConnectedAndWellPlaced) {
  const std::vector<MapGeneratorConfig> layouts = {
      {5, 4, 6, 2, 4, 1}, {8, 10, 8, 2, 10, 1}, {1, 1, 1, 1, 1, 0},
      {2, 3, 3, 1, 3, 2}, {4, 2, 5, 3, 4, 1},   {3, 6, 2, 2, 9, 0},
  };
  for (const MapGeneratorConfig& g : layouts) {
    SCOPED_TRACE(::testing::Message()
                 << g.blocks_x << "x" << g.blocks_y << " blocks, " << g.stations << " stations");
    const GridMap m = generateWarehouse(g);
    EXPECT_EQ(m.rows(), generatedRows(g));
    EXPECT_EQ(m.cols(), generatedCols(g));
    EXPECT_TRUE(findBridges(m).empty());
    EXPECT_TRUE(isConnected(m));
    ASSERT_EQ(m.pickups().size(), static_cast<size_t>(g.stations));
    ASSERT_EQ(m.dropoffs().size(), static_cast<size_t>(g.stations));
    EXPECT_EQ(m.parking().size(), static_cast<size_t>(2 * g.parking_rows * m.cols()));
    for (size_t k = 0; k < m.pickups().size(); ++k) {
      const Cell p = m.pickups()[k];
      EXPECT_EQ(p.col, 0);
      EXPECT_EQ(m.dropoffs()[k], (Cell{p.row, m.cols() - 1}));
      EXPECT_GT(p.row, g.parking_rows);                 // below the top aisle row
      EXPECT_LT(p.row, m.rows() - 1 - g.parking_rows);  // above the bottom aisle row
      if (k > 0) {
        EXPECT_GE(p.row - m.pickups()[k - 1].row, 2);
      }
    }
  }
}

TEST(MapGenerator, IsDeterministic) {
  const MapGeneratorConfig g{3, 3, 4, 2, 3, 1};
  EXPECT_EQ(generateWarehouse(g).toString(), generateWarehouse(g).toString());
}

TEST(MapGenerator, RejectsImpossibleLayouts) {
  EXPECT_THROW(generateWarehouse(MapGeneratorConfig{0, 1, 1, 1, 1, 0}), ConfigError);
  EXPECT_THROW(generateWarehouse(MapGeneratorConfig{1, 1, 1, 1, 2, 0}), ConfigError);  // stations
}

TEST(MapGenerator, FindBridgesAndConnectivity) {
  const GridMap ring = GridMap::fromString(
      "...\n"
      ".#.\n"
      "...\n");
  EXPECT_TRUE(findBridges(ring).empty());
  EXPECT_TRUE(isConnected(ring));

  const GridMap spur = GridMap::fromString(
      "...#\n"
      ".#..\n"
      "...#\n");
  const auto bridges = findBridges(spur);
  ASSERT_EQ(bridges.size(), 1U);
  EXPECT_EQ(bridges[0].first, (Cell{1, 2}));
  EXPECT_EQ(bridges[0].second, (Cell{1, 3}));

  EXPECT_EQ(findBridges(GridMap::fromString("...\n")).size(), 2U);
  EXPECT_FALSE(isConnected(GridMap::fromString("..#..\n")));
}

}  // namespace
}  // namespace fleet_core

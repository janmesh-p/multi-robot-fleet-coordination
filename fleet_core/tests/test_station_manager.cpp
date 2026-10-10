#include <set>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/tasks/station_manager.hpp"

namespace fleet_core {
namespace {

GridMap stationMap() {
  return GridMap::fromString(
      "RRRRR\n"
      "P...D\n"
      ".....\n"
      "RRRRR\n");
}

const Cell kPickup{1, 0};
const std::vector<Cell> kHomes = {{0, 0}, {0, 1}, {0, 2}};

TEST(StationManager, WaitingCellsAreSpareParkingNearestFirst) {
  const GridMap m = stationMap();
  const DistanceOracle d(m, true);
  const StationManager stations(m, d, kHomes, true);
  const std::vector<Cell>& cells = stations.waitingCells(kPickup);
  ASSERT_FALSE(cells.empty());
  EXPECT_EQ(cells.front(), (Cell{3, 0}));  // two steps from the station
  const std::set<Cell> homes(kHomes.begin(), kHomes.end());
  for (size_t i = 0; i < cells.size(); ++i) {
    EXPECT_EQ(m.kind(cells[i]), 'R');
    EXPECT_EQ(homes.count(cells[i]), 0U);
    if (i > 0) {
      EXPECT_LE(d.distance(cells[i - 1], kPickup), d.distance(cells[i], kPickup));
    }
  }
  EXPECT_THROW(stations.waitingCells({2, 2}), std::out_of_range);
}

TEST(StationManager, OneRobotAtATimeInArrivalOrder) {
  const GridMap m = stationMap();
  const DistanceOracle d(m, true);
  StationManager stations(m, d, kHomes, true);
  stations.request(0, kPickup);
  stations.request(1, kPickup);
  stations.request(2, kPickup);
  stations.request(1, kPickup);  // repeated request: no effect

  EXPECT_TRUE(stations.hasTurn(0, kPickup));
  EXPECT_EQ(stations.goalFor(0, kPickup, {2, 2}), kPickup);
  const Cell wait1 = stations.goalFor(1, kPickup, {2, 2});
  const Cell wait2 = stations.goalFor(2, kPickup, {2, 2});
  EXPECT_EQ(wait1, (Cell{3, 0}));
  EXPECT_NE(wait1, wait2);
  EXPECT_EQ(m.kind(wait2), 'R');

  stations.release(0, kPickup);
  EXPECT_TRUE(stations.hasTurn(1, kPickup));
  EXPECT_EQ(stations.goalFor(1, kPickup, {2, 2}), kPickup);
  stations.request(3, kPickup);  // takes the waiting cell robot 1 left
  EXPECT_EQ(stations.goalFor(3, kPickup, {2, 2}), wait1);

  stations.release(2, kPickup);  // leaving from the middle keeps the turn where it is
  EXPECT_TRUE(stations.hasTurn(1, kPickup));
}

TEST(StationManager, QueuesOffSendsEveryoneStraightIn) {
  const GridMap m = stationMap();
  const DistanceOracle d(m, true);
  StationManager stations(m, d, kHomes, false);
  stations.request(0, kPickup);
  stations.request(1, kPickup);
  EXPECT_EQ(stations.goalFor(1, kPickup, {2, 2}), kPickup);
  EXPECT_TRUE(stations.hasTurn(1, kPickup));
}

TEST(StationManager, NoSpareParkingMeansWaitInPlace) {
  const GridMap m = GridMap::fromString("P...D\n.....\n");
  const DistanceOracle d(m, true);
  StationManager stations(m, d, {}, true);
  const Cell pickup{0, 0};
  stations.request(0, pickup);
  stations.request(1, pickup);
  EXPECT_EQ(stations.goalFor(1, pickup, {1, 3}), (Cell{1, 3}));
  EXPECT_THROW(stations.request(2, {1, 1}), std::out_of_range);  // not a station
}

TEST(StationManager, GoalForRequiresARequest) {
  const GridMap m = stationMap();
  const DistanceOracle d(m, true);
  const StationManager stations(m, d, kHomes, true);
  EXPECT_THROW(stations.goalFor(0, kPickup, {2, 2}), std::logic_error);
}

}  // namespace
}  // namespace fleet_core

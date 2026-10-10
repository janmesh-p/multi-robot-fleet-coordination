#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/tasks/parking.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {
namespace {

RobotReport report(RobotId robot, uint32_t seq, Cell cell) {
  RobotReport r;
  r.robot = robot;
  r.seq = seq;
  r.cell = cell;
  r.status = RobotStatus::kToPickup;
  r.task = 4;
  return r;
}

TEST(WorldModel, StartsWhereTheRobotsStart) {
  const WorldModel w(GridMap::fromString("....\n"), {{0, 0}, {0, 3}});
  EXPECT_EQ(w.fleetSize(), 2);
  EXPECT_EQ(w.robot(1).cell, (Cell{0, 3}));
  EXPECT_EQ(w.robot(1).status, RobotStatus::kIdle);
  EXPECT_EQ(w.robot(1).plan_id, -1);
  EXPECT_THROW(w.robot(2), std::out_of_range);
}

TEST(WorldModel, AcceptsOnlyNewerReports) {
  WorldModel w(GridMap::fromString("....\n"), {{0, 0}});
  w.setNow(7);
  EXPECT_TRUE(w.applyReport(report(0, 1, {0, 1})));
  EXPECT_EQ(w.robot(0).cell, (Cell{0, 1}));
  EXPECT_EQ(w.robot(0).task, std::optional<TaskId>(4));
  EXPECT_EQ(w.robot(0).last_heard, 7);
  EXPECT_FALSE(w.applyReport(report(0, 1, {0, 2})));  // duplicate
  EXPECT_FALSE(w.applyReport(report(0, 0, {0, 2})));  // older
  w.setNow(9);
  EXPECT_TRUE(w.applyReport(report(0, 3, {0, 3})));   // gaps are fine
  EXPECT_FALSE(w.applyReport(report(0, 2, {0, 2})));  // arrived late
  EXPECT_EQ(w.robot(0).cell, (Cell{0, 3}));
  EXPECT_EQ(w.robot(0).last_heard, 9);
  EXPECT_THROW(w.applyReport(report(5, 1, {0, 0})), std::out_of_range);
}

TEST(WorldModel, DeadRobotsIgnoreReports) {
  WorldModel w(GridMap::fromString("....\n"), {{0, 0}});
  w.markDead(0);
  EXPECT_FALSE(w.applyReport(report(0, 1, {0, 1})));
  EXPECT_EQ(w.robot(0).status, RobotStatus::kDead);
}

TEST(WorldModel, ObservationsBlockAndClearCells) {
  WorldModel w(GridMap::fromString("....\n"), {{0, 0}});
  w.applyObservation(CellObservation{0, {{0, 2}}, {}});
  EXPECT_FALSE(w.map().isFree({0, 2}));
  w.applyObservation(CellObservation{1, {}, {{0, 2}}});
  EXPECT_TRUE(w.map().isFree({0, 2}));
}

TEST(ParkingManager, HomesAreParkingCellsInRowMajorOrder) {
  const GridMap m = GridMap::fromString("R.R\n...\n.RR\n");
  const ParkingManager parking(m, 3);
  EXPECT_EQ(parking.homes(), (std::vector<Cell>{{0, 0}, {0, 2}, {2, 1}}));
  EXPECT_EQ(parking.home(2), (Cell{2, 1}));
  EXPECT_THROW(parking.home(3), std::out_of_range);
  EXPECT_THROW(ParkingManager(m, 5), std::invalid_argument);
}

}  // namespace
}  // namespace fleet_core

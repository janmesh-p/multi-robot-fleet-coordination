#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/traffic/traffic_manager.hpp"

namespace fleet_core {
namespace {

// A fleet whose believed state the test sets directly, the way reports would.
struct Fleet {
  WorldModel world;
  TrafficManager traffic;
  std::vector<uint32_t> seq;
  std::vector<int32_t> priority;

  Fleet(const std::string& map, const std::vector<Cell>& cells, TrafficRules rules)
      : world(GridMap::fromString(map), cells),
        traffic(rules),
        seq(cells.size(), 0),
        priority(cells.size(), -1) {}

  // Every robot gets plan id 0 with this path, starting at step 0.
  void commit(const std::vector<std::vector<Cell>>& paths) {
    std::vector<CommittedPlan> plans;
    for (size_t r = 0; r < paths.size(); ++r) {
      plans.push_back(
          CommittedPlan{static_cast<RobotId>(r), 0, Path{static_cast<RobotId>(r), 0, paths[r]}});
    }
    traffic.commit(plans);
  }

  // The robot reports that it stands at `index` of its plan.
  void at(RobotId r, const Cell& cell, int32_t index) {
    RobotReport report;
    report.robot = r;
    report.seq = ++seq[static_cast<size_t>(r)];
    report.cell = cell;
    report.status = RobotStatus::kToPickup;
    report.plan_id = 0;
    report.path_index = index;
    ASSERT_TRUE(world.applyReport(report));
  }

  TrafficDecision decide() const { return traffic.decide(world, priority); }
};

const char* const kOpen =
    ".....\n"
    ".....\n"
    ".....\n";

// A crosses (1,2) at step 2; B is planned through it at step 4, after A. A runs two steps late:
// the dependency graph holds B until A has passed, then lets B follow A out of the cell.
TEST(TrafficManager, DependencyGraphKeepsThePlannedOrder) {
  Fleet f(kOpen, {{1, 0}, {0, 2}}, TrafficRules{true, false});
  f.commit(
      {{{1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}}, {{0, 2}, {0, 2}, {0, 2}, {0, 2}, {1, 2}, {2, 2}}});
  f.at(0, {1, 1}, 1);  // late: planned at (1,3) by now
  f.at(1, {0, 2}, 3);  // on time, next: into (1,2)
  TrafficDecision d = f.decide();
  EXPECT_TRUE(d.permits[1].held);
  EXPECT_EQ(d.permits[1].waits_for, 0);
  EXPECT_EQ(d.permits[1].until, 3);
  EXPECT_FALSE(d.permits[0].held);
  EXPECT_EQ(d.permits[0].until, 4);  // nothing before it anywhere: its whole path
  EXPECT_TRUE(d.cycles.empty());

  f.at(0, {1, 2}, 2);  // in the cell, leaving it next
  d = f.decide();
  EXPECT_FALSE(d.permits[1].held);
  EXPECT_EQ(d.permits[1].until, 5);   // into (1,2) behind A, then on: (2,2) waits for no one
  EXPECT_EQ(d.permits[1].follow, 0);  // it follows A out

  f.at(0, {1, 3}, 3);
  d = f.decide();
  EXPECT_EQ(d.permits[1].until, 5);
  EXPECT_EQ(d.permits[1].follow, -1);  // the cell is empty now
}

// Reservations look at where robots are, so they let whoever asks first, older order first,
// take a cell. With the same scene they let B, with the older order, go ahead of A.
TEST(TrafficManager, ReservationsGoByOrderAgeNotByPlan) {
  Fleet f(kOpen, {{1, 0}, {0, 2}}, TrafficRules{false, true});
  f.commit(
      {{{1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}}, {{0, 2}, {0, 2}, {0, 2}, {0, 2}, {1, 2}, {2, 2}}});
  f.priority = {3, 9};
  f.at(0, {1, 1}, 1);
  f.at(1, {0, 2}, 3);
  const TrafficDecision d = f.decide();
  EXPECT_FALSE(d.permits[1].held);
  EXPECT_EQ(d.permits[1].until, 4);  // one step at a time
  EXPECT_TRUE(d.permits[0].held);
  EXPECT_EQ(d.permits[0].waits_for, 1);
}

TEST(TrafficManager, RobotsEnteringEachOthersCellsInARingMoveTogether) {
  for (const TrafficRules rules :
       {TrafficRules{true, false}, TrafficRules{false, true}, TrafficRules{true, true}}) {
    Fleet f(kOpen, {{0, 0}, {0, 1}, {1, 1}, {1, 0}}, rules);
    f.commit({{{0, 0}, {0, 1}}, {{0, 1}, {1, 1}}, {{1, 1}, {1, 0}}, {{1, 0}, {0, 0}}});
    const TrafficDecision d = f.decide();
    for (RobotId r = 0; r < 4; ++r) {
      EXPECT_FALSE(d.permits[static_cast<size_t>(r)].held) << r;
      EXPECT_EQ(d.permits[static_cast<size_t>(r)].until, 1) << r;
      EXPECT_EQ(d.permits[static_cast<size_t>(r)].follow, (r + 1) % 4) << r;
    }
  }
}

// Plans made apart can send two robots into each other. Reservations hold both, and the two
// waiting on each other are reported as a cycle.
TEST(TrafficManager, HeadOnRobotsAreHeldAndReportedAsACycle) {
  Fleet f("......\n", {{0, 1}, {0, 2}, {0, 4}}, TrafficRules{false, true});
  f.commit({{{0, 1}, {0, 2}, {0, 3}}, {{0, 2}, {0, 1}, {0, 0}}, {{0, 4}, {0, 3}}});
  const TrafficDecision d = f.decide();
  EXPECT_TRUE(d.permits[0].held);
  EXPECT_TRUE(d.permits[1].held);
  EXPECT_EQ(d.permits[0].waits_for, 1);
  EXPECT_EQ(d.permits[1].waits_for, 0);
  EXPECT_FALSE(d.permits[2].held);
  EXPECT_EQ(d.cycles, (std::vector<std::vector<RobotId>>{{0, 1}}));
}

TEST(TrafficManager, AContestedCellGoesToTheOlderOrder) {
  Fleet f(kOpen, {{0, 1}, {1, 0}, {1, 2}}, TrafficRules{false, true});
  f.commit({{{0, 1}, {1, 1}}, {{1, 0}, {1, 1}}, {{1, 2}, {1, 1}}});
  f.priority = {5, 8, 8};  // two tie on age: the lower id wins
  const TrafficDecision d = f.decide();
  EXPECT_TRUE(d.permits[0].held);
  EXPECT_FALSE(d.permits[1].held);
  EXPECT_TRUE(d.permits[2].held);
  EXPECT_EQ(d.permits[0].waits_for, 1);
  EXPECT_EQ(d.permits[2].waits_for, 1);
}

// Without rules the permit covers the whole path, except that no robot is sent into a closed
// cell: the permit stops before it, and a robot whose next cell is closed waits for no one.
TEST(TrafficManager, PermitsStopBeforeClosedCells) {
  Fleet f(kOpen, {{0, 0}, {2, 0}}, TrafficRules{});
  f.commit({{{0, 0}, {0, 1}, {0, 2}, {0, 3}}, {{2, 0}, {2, 1}, {2, 2}}});
  f.world.applyObservation(CellObservation{0, {{0, 2}, {2, 1}}, {}});
  const TrafficDecision d = f.decide();
  EXPECT_EQ(d.permits[0].until, 1);
  EXPECT_FALSE(d.permits[0].held);
  EXPECT_TRUE(d.permits[1].held);
  EXPECT_EQ(d.permits[1].waits_for, -1);
  EXPECT_TRUE(d.cycles.empty());
}

// The rules never hold up plans executed on time: here two robots follow each other down a
// corridor and a third waits for them, as planned. Every robot may take its next step.
TEST(TrafficManager, OnTimePlansAreNeverHeld) {
  for (const TrafficRules rules :
       {TrafficRules{true, false}, TrafficRules{false, true}, TrafficRules{true, true}}) {
    Fleet f(kOpen, {{1, 1}, {1, 0}, {0, 2}}, rules);
    f.commit({{{1, 1}, {1, 2}, {1, 3}, {1, 4}},
              {{1, 0}, {1, 1}, {1, 2}, {1, 3}},
              {{0, 2}, {0, 2}, {0, 2}, {1, 2}, {2, 2}}});
    for (int32_t t = 0; t < 4; ++t) {
      const TrafficDecision d = f.decide();
      for (size_t r = 0; r < 3; ++r) EXPECT_FALSE(d.permits[r].held) << "step " << t;
      f.at(0, {1, std::min(1 + t + 1, 4)}, std::min(t + 1, 3));
      f.at(1, {1, std::min(t + 1, 3)}, std::min(t + 1, 3));
      f.at(2, t + 1 < 3 ? Cell{0, 2} : (t + 1 == 3 ? Cell{1, 2} : Cell{2, 2}), std::min(t + 1, 4));
    }
  }
}

}  // namespace
}  // namespace fleet_core

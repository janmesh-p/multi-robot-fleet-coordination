#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/verification/progress_monitor.hpp"
#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

// Simulated robots on fixed paths, observed by a monitor every step.
struct Bench {
  GridExecutor robots;
  ProgressMonitor monitor;
  std::map<RobotId, std::vector<Cell>> paths;
  std::map<RobotId, int32_t> plan_ids;
  std::map<RobotId, TaskOrder> tasks;
  Timestep t{0};

  Bench(const std::string& map, const std::vector<Cell>& spawn)
      : robots(GridMap::fromString(map), spawn, 2, 0.0, 1),
        monitor(static_cast<int32_t>(spawn.size())) {}

  void plan(RobotId robot, std::vector<Cell> cells) {
    paths[robot] = std::move(cells);
    plan_ids[robot] = plan_ids.count(robot) != 0 ? plan_ids[robot] + 1 : 0;
  }

  const ProgressStep& step() {
    std::vector<RobotCommand> commands;
    for (const auto& [robot, cells] : paths) {
      RobotCommand c;
      c.robot = robot;
      c.plan_id = plan_ids[robot];
      c.issued_at = t;
      c.path = cells;
      c.permitted_until = static_cast<int32_t>(cells.size()) - 1;
      if (tasks.count(robot) != 0) c.task = tasks[robot];
      commands.push_back(c);
    }
    robots.dispatch(t, commands);
    robots.step(t);
    const ProgressStep& seen = monitor.observe(robots.lastStep(), robots.snapshot());
    ++t;
    return seen;
  }
};

const char* const kCorridor =
    "#######\n"
    ".......\n"
    "#######\n";

// One-cell corridor with a pocket below column 3.
const char* const kPocket =
    "#######\n"
    ".......\n"
    "###.###\n";

TEST(ProgressMonitor, HeadOnPairBecomesADeadlockAfterTheConfirmWindow) {
  Bench b(kCorridor, {{1, 2}, {1, 3}});
  b.plan(0, {{1, 2}, {1, 3}, {1, 4}, {1, 5}});
  b.plan(1, {{1, 3}, {1, 2}, {1, 1}, {1, 0}});
  for (Timestep k = 0; k < ProgressMonitor::kConfirmSteps - 1; ++k) {
    const ProgressStep& p = b.step();
    EXPECT_EQ(p.waits_for, (std::vector<RobotId>{1, 0}));
    EXPECT_EQ(p.on_cycle_count, 2);
    EXPECT_TRUE(b.monitor.deadlocks().empty()) << "too young at step " << k;
  }
  b.step();
  ASSERT_EQ(b.monitor.deadlocks().size(), 1U);
  EXPECT_EQ(b.monitor.deadlocks()[0].formed, 0);
  EXPECT_EQ(b.monitor.deadlocks()[0].robots, (std::vector<RobotId>{0, 1}));
  EXPECT_FALSE(b.monitor.deadlocks()[0].broken.has_value());

  const ProgressSummary s = b.monitor.summarize(b.t);
  EXPECT_EQ(s.first_deadlock_step, 0);
  EXPECT_EQ(s.deadlocks, 1);
  EXPECT_EQ(s.deadlocked_robots, 2);
  EXPECT_EQ(s.blocked_behind, 0);
}

TEST(ProgressMonitor, QueueBehindARobotThatStaysIsNotADeadlock) {
  // R3 has no path and stays; R2, R1 and R0 line up behind it.
  Bench b(
      "########\n"
      "........\n"
      "########\n",
      {{1, 1}, {1, 2}, {1, 3}, {1, 4}});
  b.plan(0, {{1, 1}, {1, 2}, {1, 3}, {1, 4}});
  b.plan(1, {{1, 2}, {1, 3}, {1, 4}, {1, 5}});
  b.plan(2, {{1, 3}, {1, 4}, {1, 5}, {1, 6}});
  const ProgressStep& p = b.step();
  EXPECT_EQ(p.waits_for, (std::vector<RobotId>{1, 2, 3, -1}));
  EXPECT_EQ(p.on_cycle_count, 0);
  EXPECT_EQ(p.behind_count, 0);
  for (int k = 0; k < 40; ++k) b.step();
  EXPECT_TRUE(b.monitor.deadlocks().empty());
}

TEST(ProgressMonitor, StallNeedsThirtyStepsWithoutProgress) {
  Bench b(
      "########\n"
      "........\n"
      "########\n",
      {{1, 1}, {1, 2}});
  b.plan(0, {{1, 1}, {1, 2}, {1, 3}});  // R1 has no path and stays: R0 never gets through
  for (Timestep k = 0; k < ProgressMonitor::kStallSteps - 1; ++k) {
    EXPECT_EQ(b.step().stalled_count, 0) << "step " << k;
  }
  const ProgressStep& p = b.step();
  EXPECT_EQ(p.stalled, (std::vector<uint8_t>{1, 0}));  // idle R1 has nothing to do
  b.step();
  EXPECT_EQ(b.monitor.summarize(b.t).stalled_robot_steps, 2);
}

TEST(ProgressMonitor, LoserOfASameCellContestWaitsForTheWinner) {
  Bench b(
      "##.##\n"
      ".....\n"
      "##.##\n",
      {{0, 2}, {1, 1}});
  b.plan(0, {{0, 2}, {1, 2}, {2, 2}});
  b.plan(1, {{1, 1}, {1, 2}, {1, 3}});
  const ProgressStep& first = b.step();  // R0 wins (1,2), R1 stops
  EXPECT_EQ(first.waits_for, (std::vector<RobotId>{-1, 0}));
  EXPECT_EQ(first.on_cycle_count, 0);
  const ProgressStep& second = b.step();  // R0 leaves, R1 follows
  EXPECT_EQ(second.waits_for, (std::vector<RobotId>{-1, -1}));
}

TEST(ProgressMonitor, GridlockLeavesTheFollowerBehindTheCycle) {
  // The explainer's gridlock: R0 east and R1 west in one aisle, R2 following R1.
  const GridMap map = generateWarehouse(MapGeneratorConfig{3, 2, 3, 2, 2, 1});
  const std::string text = map.toString();
  Bench b(text, {{4, 2}, {4, 10}, {4, 12}});
  std::vector<Cell> east, west, follow;
  for (int32_t c = 2; c <= 12; ++c) east.push_back({4, c});
  for (int32_t c = 10; c >= 0; --c) west.push_back({4, c});
  for (int32_t c = 12; c >= 1; --c) follow.push_back({4, c});
  b.plan(0, east);
  b.plan(1, west);
  b.plan(2, follow);
  for (int k = 0; k < 4; ++k) b.step();  // three free steps, then R0 wins (4,6)
  const ProgressStep& p = b.step();      // step 4: head-on
  EXPECT_EQ(p.waits_for, (std::vector<RobotId>{1, 0, 1}));
  EXPECT_EQ(p.on_cycle, (std::vector<uint8_t>{1, 1, 0}));
  EXPECT_EQ(p.behind, (std::vector<uint8_t>{0, 0, 1}));
  for (int k = 0; k < 12; ++k) b.step();
  const ProgressSummary s = b.monitor.summarize(b.t);
  EXPECT_EQ(s.first_deadlock_step, 4);
  EXPECT_EQ(s.deadlocked_robots, 2);
  EXPECT_EQ(s.blocked_behind, 1);
}

TEST(ProgressMonitor, ShortCycleThatBreaksIsNotADeadlock) {
  Bench b(kPocket, {{1, 2}, {1, 3}});
  b.plan(0, {{1, 2}, {1, 3}, {1, 4}, {1, 5}});
  b.plan(1, {{1, 3}, {1, 2}, {1, 1}});
  for (int k = 0; k < 5; ++k) EXPECT_EQ(b.step().on_cycle_count, 2);
  b.plan(1, {{1, 3}, {2, 3}});  // a new plan: R1 ducks into the pocket
  const ProgressStep& p = b.step();
  EXPECT_EQ(p.on_cycle_count, 0);
  for (int k = 0; k < 20; ++k) b.step();
  EXPECT_TRUE(b.monitor.deadlocks().empty());
  EXPECT_EQ(b.monitor.summarize(b.t).first_deadlock_step, -1);
}

TEST(ProgressMonitor, ConfirmedDeadlockRecordsTheStepItBroke) {
  Bench b(kPocket, {{1, 2}, {1, 3}});
  b.plan(0, {{1, 2}, {1, 3}, {1, 4}, {1, 5}});
  b.plan(1, {{1, 3}, {1, 2}, {1, 1}});
  for (int k = 0; k < 12; ++k) b.step();
  b.plan(1, {{1, 3}, {2, 3}});
  b.step();  // step 12: R1 ducks into the pocket, the cycle is gone
  ASSERT_EQ(b.monitor.deadlocks().size(), 1U);
  EXPECT_EQ(b.monitor.deadlocks()[0].broken, std::optional<Timestep>(12));
  const ProgressSummary s = b.monitor.summarize(b.t);
  EXPECT_EQ(s.deadlocks, 1);
  EXPECT_EQ(s.deadlocked_robots, 0);  // nothing is locked at the end
}

TEST(ProgressMonitor, FrozenNeedsWorkLeftAndAQuietTail) {
  Bench b(kCorridor, {{1, 2}, {1, 3}});
  b.plan(0, {{1, 2}, {1, 3}, {1, 4}, {1, 5}});
  b.plan(1, {{1, 3}, {1, 2}, {1, 1}, {1, 0}});
  for (int k = 0; k < 40; ++k) b.step();
  EXPECT_EQ(b.monitor.summarize(b.t).frozen_at, -1);  // locked, but nobody holds an order

  Bench w(kCorridor, {{1, 2}, {1, 3}});
  w.plan(0, {{1, 2}, {1, 3}, {1, 4}, {1, 5}});
  w.plan(1, {{1, 3}, {1, 2}, {1, 1}, {1, 0}});
  w.tasks[0] = TaskOrder{7, {1, 5}, {1, 6}};
  for (int k = 0; k < 20; ++k) w.step();
  EXPECT_EQ(w.monitor.summarize(w.t).frozen_at, -1);  // quiet for only 20 steps
  for (int k = 0; k < 20; ++k) w.step();
  EXPECT_EQ(w.monitor.summarize(w.t).frozen_at, 0);
}

TEST(ProgressMonitor, RejectsAStepForADifferentFleet) {
  ProgressMonitor monitor(3);
  StepTruth step;
  step.motion.resize(2);
  TruthSnapshot after;
  after.cells.resize(2);
  after.alive = {true, true};
  after.holding.resize(2);
  EXPECT_THROW(monitor.observe(step, after), std::invalid_argument);
  EXPECT_THROW(ProgressMonitor(-1), std::invalid_argument);
}

}  // namespace
}  // namespace fleet_core

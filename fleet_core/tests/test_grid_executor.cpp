#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/execution/grid_executor.hpp"

namespace fleet_core {
namespace {

GridMap openMap() {
  return GridMap::fromString(
      ".....\n"
      ".....\n"
      ".....\n"
      ".....\n");
}

RobotCommand command(RobotId robot, int32_t plan_id, std::vector<Cell> path,
                     std::optional<TaskOrder> task = std::nullopt, int32_t permitted = -1) {
  RobotCommand c;
  c.robot = robot;
  c.plan_id = plan_id;
  c.task = task;
  c.permitted_until = permitted >= 0 ? permitted : static_cast<int32_t>(path.size()) - 1;
  c.path = std::move(path);
  return c;
}

MotionOutcome outcome(const GridExecutor& e, RobotId robot) {
  return e.lastStep().motion.at(static_cast<size_t>(robot)).outcome;
}

Cell cellOf(const GridExecutor& e, RobotId robot) {
  return e.robots().at(static_cast<size_t>(robot)).cell;
}

TEST(GridExecutor, FollowingIntoAVacatedCellIsAllowed) {
  GridExecutor e(openMap(), {{2, 0}, {2, 1}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{2, 0}, {2, 1}}), command(1, 0, {{2, 1}, {2, 2}})});
  e.step(0);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kMoved);
  EXPECT_EQ(outcome(e, 1), MotionOutcome::kMoved);
  EXPECT_EQ(cellOf(e, 0), (Cell{2, 1}));
  EXPECT_EQ(cellOf(e, 1), (Cell{2, 2}));
}

TEST(GridExecutor, SameCellContestGoesToTheLowerId) {
  GridExecutor e(openMap(), {{3, 1}, {1, 1}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{3, 1}, {2, 1}}), command(1, 0, {{1, 1}, {2, 1}})});
  e.step(0);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kMoved);
  EXPECT_EQ(outcome(e, 1), MotionOutcome::kBlockedVertex);
  EXPECT_EQ(cellOf(e, 1), (Cell{1, 1}));
  EXPECT_TRUE(isSafetyStop(outcome(e, 1)));
}

TEST(GridExecutor, HeadOnRobotsBothStop) {
  GridExecutor e(openMap(), {{2, 1}, {2, 2}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{2, 1}, {2, 2}}), command(1, 0, {{2, 2}, {2, 1}})});
  e.step(0);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kBlockedSwap);
  EXPECT_EQ(outcome(e, 1), MotionOutcome::kBlockedSwap);
  EXPECT_EQ(cellOf(e, 0), (Cell{2, 1}));
}

TEST(GridExecutor, AStopPropagatesDownTheLine) {
  GridExecutor e(openMap(), {{2, 0}, {2, 1}, {2, 2}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{2, 0}, {2, 1}}), command(1, 0, {{2, 1}, {2, 2}})});
  e.step(0);  // robot 2 has no plan and stays
  EXPECT_EQ(outcome(e, 2), MotionOutcome::kIdle);
  EXPECT_EQ(outcome(e, 1), MotionOutcome::kBlockedOccupied);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kBlockedOccupied);
}

TEST(GridExecutor, ARotatingCycleMovesTogether) {
  GridExecutor e(openMap(), {{1, 1}, {1, 2}, {2, 2}, {2, 1}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{1, 1}, {1, 2}}), command(1, 0, {{1, 2}, {2, 2}}),
                 command(2, 0, {{2, 2}, {2, 1}}), command(3, 0, {{2, 1}, {1, 1}})});
  e.step(0);
  for (RobotId r = 0; r < 4; ++r) EXPECT_EQ(outcome(e, r), MotionOutcome::kMoved);
  EXPECT_EQ(cellOf(e, 3), (Cell{1, 1}));
}

TEST(GridExecutor, WaitsPermitsAndHolds) {
  const TaskOrder task{1, {3, 4}, {0, 4}};
  GridExecutor e(openMap(), {{2, 0}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{2, 0}, {2, 0}, {2, 1}, {2, 2}}, task, 2)});
  e.step(0);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kPlannedWait);
  e.step(1);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kMoved);
  e.step(2);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kHold);  // permit ends at index 2
  EXPECT_EQ(cellOf(e, 0), (Cell{2, 1}));
  e.dispatch(3,
             {command(0, 0, {{2, 0}, {2, 0}, {2, 1}, {2, 2}}, task, 3)});  // same plan, more permit
  e.step(3);
  EXPECT_EQ(cellOf(e, 0), (Cell{2, 2}));
}

TEST(GridExecutor, RefusesInvalidStepsAndBlockedCells) {
  GridMap map = GridMap::fromString(
      ".#...\n"
      ".....\n");
  map.block({1, 4});
  GridExecutor e(map, {{0, 0}, {1, 2}, {1, 3}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{0, 0}, {0, 1}}),    // shelf
                 command(1, 0, {{1, 2}, {0, 3}}),    // diagonal
                 command(2, 0, {{1, 3}, {1, 4}})});  // blocked
  e.step(0);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kInvalidStep);
  EXPECT_EQ(outcome(e, 1), MotionOutcome::kInvalidStep);
  EXPECT_EQ(outcome(e, 2), MotionOutcome::kBlockedCell);
  EXPECT_EQ(cellOf(e, 0), (Cell{0, 0}));
}

TEST(GridExecutor, ServiceAndDeliveryTiming) {
  const GridMap map = GridMap::fromString(
      "P...D\n"
      ".....\n");
  const TaskOrder task{5, {0, 0}, {0, 4}};
  GridExecutor e(map, {{0, 1}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{0, 1}, {0, 0}}, task)});
  e.step(0);  // arrives and starts picking
  EXPECT_EQ(e.robots()[0].status, RobotStatus::kPicking);
  e.step(1);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kService);
  EXPECT_TRUE(e.lastStep().pickups.empty());
  e.step(2);  // second dwell step ends the pick
  ASSERT_EQ(e.lastStep().pickups.size(), 1U);
  EXPECT_EQ(e.lastStep().pickups[0].task, 5);
  EXPECT_EQ(e.robots()[0].status, RobotStatus::kToDropoff);
  e.step(3);
  EXPECT_EQ(outcome(e, 0), MotionOutcome::kHold);  // carrying, waiting for a plan

  e.dispatch(4, {command(0, 1, {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}}, task)});
  for (Timestep t = 4; t <= 7; ++t) e.step(t);  // four moves; arrives at t = 7
  EXPECT_EQ(e.robots()[0].status, RobotStatus::kDropping);
  e.step(8);
  EXPECT_TRUE(e.lastStep().deliveries.empty());
  e.step(9);
  ASSERT_EQ(e.lastStep().deliveries.size(), 1U);
  EXPECT_EQ(e.lastStep().deliveries[0].robot, 0);
  EXPECT_EQ(e.robots()[0].status, RobotStatus::kIdle);
  EXPECT_FALSE(e.robots()[0].task.has_value());

  const std::vector<RobotReport> reports = e.collectReports(10);
  ASSERT_EQ(reports.size(), 1U);
  EXPECT_EQ(reports[0].last_completed, std::optional<TaskId>(5));
  EXPECT_FALSE(reports[0].task.has_value());
  EXPECT_EQ(reports[0].plan_id, 1);
}

TEST(GridExecutor, PassingThroughItsStationDoesNotStartService) {
  const GridMap map = GridMap::fromString(
      "P...D\n"
      ".....\n");
  GridExecutor e(map, {{0, 1}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{0, 1}, {0, 0}, {1, 0}}, TaskOrder{5, {0, 0}, {0, 4}})});
  e.step(0);
  EXPECT_EQ(e.robots()[0].status, RobotStatus::kToPickup);
  e.step(1);
  EXPECT_EQ(cellOf(e, 0), (Cell{1, 0}));
}

TEST(GridExecutor, ZeroServiceStepsFinishOnArrival) {
  const GridMap map = GridMap::fromString("P...D\n");
  GridExecutor e(map, {{0, 1}}, 0, 0.0, 1);
  e.dispatch(0, {command(0, 0, {{0, 1}, {0, 0}}, TaskOrder{5, {0, 0}, {0, 4}})});
  e.step(0);
  EXPECT_EQ(e.lastStep().pickups.size(), 1U);
  EXPECT_EQ(e.robots()[0].status, RobotStatus::kToDropoff);
}

TEST(GridExecutor, StaleCommandsAndPlansThatMissTheRobot) {
  GridExecutor e(openMap(), {{2, 0}}, 2, 0.0, 1);
  e.dispatch(0, {command(0, 3, {{2, 0}, {2, 1}})});
  e.dispatch(0, {command(0, 2, {{2, 0}, {3, 0}})});  // older plan id: ignored
  e.step(0);
  EXPECT_EQ(cellOf(e, 0), (Cell{2, 1}));
  e.dispatch(1, {command(0, 4, {{0, 0}, {0, 1}})});  // does not pass through (2, 1): hold
  e.step(1);
  EXPECT_EQ(cellOf(e, 0), (Cell{2, 1}));
  EXPECT_EQ(e.robots()[0].plan_id, 4);
  e.dispatch(2, {command(0, 5, {{2, 0}, {2, 1}, {2, 2}})});  // joins mid-path at index 1
  e.step(2);
  EXPECT_EQ(cellOf(e, 0), (Cell{2, 2}));
}

TEST(GridExecutor, DelaysAreSeededAndKeepTheirStreamAligned) {
  const auto run = [](bool second_robot_moves) {
    GridExecutor e(openMap(), {{0, 0}, {3, 0}}, 2, 0.5, 42);
    std::vector<Cell> long_path;
    for (int32_t c = 0; c < 5; ++c) long_path.push_back({0, c});
    for (int32_t c = 4; c >= 0; --c) long_path.push_back({1, c});
    std::vector<RobotCommand> commands = {command(0, 0, long_path)};
    if (second_robot_moves) commands.push_back(command(1, 0, {{3, 0}, {3, 1}, {3, 2}}));
    e.dispatch(0, commands);
    std::vector<MotionOutcome> outcomes;
    for (Timestep t = 0; t < 12; ++t) {
      e.step(t);
      outcomes.push_back(outcome(e, 0));
    }
    return outcomes;
  };
  const std::vector<MotionOutcome> a = run(false);
  EXPECT_EQ(a, run(false));  // same seed, same delays
  EXPECT_EQ(a, run(true));   // robot 1's behavior does not shift robot 0's draws
  int delayed = 0;
  for (const MotionOutcome o : a) delayed += o == MotionOutcome::kDelayed ? 1 : 0;
  EXPECT_GT(delayed, 0);
  EXPECT_LT(delayed, 12);
}

TEST(GridExecutor, ReportsCountUpAndSpawnIsChecked) {
  GridExecutor e(openMap(), {{0, 0}, {1, 1}}, 2, 0.0, 1);
  EXPECT_EQ(e.collectReports(0)[1].seq, 1U);
  const std::vector<RobotReport> second = e.collectReports(1);
  EXPECT_EQ(second[1].seq, 2U);
  EXPECT_EQ(second[1].cell, (Cell{1, 1}));
  EXPECT_EQ(second[1].stamp, 1);
  EXPECT_THROW(GridExecutor(openMap(), {{0, 0}, {0, 0}}, 2, 0.0, 1), std::invalid_argument);
  EXPECT_THROW(GridExecutor(GridMap::fromString("#.\n"), {{0, 0}}, 2, 0.0, 1),
               std::invalid_argument);
  EXPECT_THROW(e.dispatch(0, {command(7, 0, {{0, 0}})}), std::out_of_range);
}

}  // namespace
}  // namespace fleet_core

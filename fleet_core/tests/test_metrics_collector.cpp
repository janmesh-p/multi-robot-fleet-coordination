#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/metrics/metrics_collector.hpp"
#include "fleet_core/metrics/run_stats.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

std::string readFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

class MetricsCollectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("fleet_core_metrics_" + std::to_string(::getpid()));
    std::filesystem::remove_all(dir_);
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::filesystem::path dir_;
};

// Two robots, one step: robot 0 moves and delivers task 7, robot 1 is stopped by robot safety.
StepTruth sampleStep() {
  StepTruth step;
  step.t = 3;
  step.motion = {MotionRecord{0, {1, 1}, {1, 2}, {1, 2}, MotionOutcome::kMoved},
                 MotionRecord{1, {2, 2}, {2, 2}, {1, 2}, MotionOutcome::kBlockedVertex}};
  step.deliveries = {Assignment{0, 7}};
  return step;
}

// Robot 1 waits for robot 0, which stands in the cell it wanted.
ProgressStep sampleProgress() {
  ProgressStep p;
  p.waits_for = {-1, 0};
  p.on_cycle = {0, 0};
  p.behind = {0, 0};
  p.stalled = {0, 1};
  p.stalled_count = 1;
  return p;
}

TruthSnapshot sampleTruth() {
  TruthSnapshot s;
  s.cells = {{1, 2}, {2, 2}};
  s.alive = {true, true};
  s.status = {RobotStatus::kIdle, RobotStatus::kToPickup};
  s.holding = {std::nullopt, TaskId{8}};
  return s;
}

TEST_F(MetricsCollectorTest, WritesRunDirectory) {
  Config config;
  config.map.file = (kRepo / "maps" / "warehouse_small.txt").lexically_normal();
  config.run.name = "metrics_test";
  const GridMap map = GridMap::fromString("P..D\n.R..\n");
  {
    MetricsCollector metrics(dir_ / "run");
    metrics.writeResolvedConfig(config);
    metrics.writeMap(map);
    TickStats tick;
    tick.assignments = 1;
    tick.planning_ms = 1.5;
    metrics.recordStep(sampleStep(), sampleTruth(), tick, 2, sampleProgress());
    RunSummary summary;
    summary.run_name = "metrics_test";
    summary.allocator = "nearest";
    summary.planner = "independent_astar";
    summary.log_hash = 0xABCULL;
    metrics.writeSummary(summary);
    metrics.writeViolations({Violation{3, ViolationKind::kSwap, "robot 0, robot 1"}});
    metrics.writeDeadlocks(
        {Deadlock{5, {1, 4}, std::optional<Timestep>(9)}, Deadlock{12, {2, 3}, {}}});
  }
  const std::filesystem::path run = dir_ / "run";

  const std::string written_config = readFile(run / "config.yaml");
  EXPECT_EQ(written_config, dumpConfig(config));
  EXPECT_EQ(parseConfig(written_config, "/").run.name, "metrics_test");
  EXPECT_EQ(readFile(run / "map.txt"), map.toString());

  EXPECT_EQ(readFile(run / "steps.csv"),
            "t,orders_created,deliveries,moved,planned_wait,hold,service,idle,delayed,blocked,"
            "assignments,robots_planned,planning_ms,expansions,on_cycle,behind,stalled,yield,"
            "no_permit,deadlocks_resolved,no_command,safe_stop,declared_dead,requeued\n"
            "3,2,1,1,0,0,0,0,0,1,1,0,1.5,0,0,0,1,0,0,0,0,0,0,0\n");
  EXPECT_EQ(readFile(run / "positions.csv"),
            "t,robot,row,col,status,outcome,waits_for,on_cycle\n"
            "3,0,1,2,idle,moved,-1,0\n"
            "3,1,2,2,to_pickup,blocked_vertex,0,0\n");

  const std::string summary = readFile(run / "summary.csv");
  EXPECT_EQ(summary.rfind("run,allocator,planner,", 0), 0U);
  EXPECT_NE(summary.find("\nmetrics_test,nearest,independent_astar,"), std::string::npos);
  EXPECT_NE(summary.find(",0000000000000abc,"), std::string::npos);
  EXPECT_EQ(readFile(run / "violations.csv"), "t,kind,detail\n3,swap,\"robot 0, robot 1\"\n");
  EXPECT_EQ(readFile(run / "deadlocks.csv"), "formed,broken,robots\n5,9,1 4\n12,,2 3\n");
}

TEST(SummaryCsv, HeaderAndRowHaveTheSameColumns) {
  RunSummary s;
  s.run_name = "a,b";  // quoted, so it stays one column
  s.first_deadlock_step = 135;
  s.frozen_at = 218;
  const auto columns = [](const std::string& line) {
    int count = 1;
    bool quoted = false;
    for (const char c : line) {
      if (c == '"') quoted = !quoted;
      if (c == ',' && !quoted) ++count;
    }
    return count;
  };
  EXPECT_EQ(columns(summaryCsvHeader()), columns(summaryCsvRow(s)));
  EXPECT_NE(summaryCsvRow(s).find(",135,"), std::string::npos);
  EXPECT_EQ(summaryCsvRow(s).rfind("\"a,b\",", 0), 0U);
}

TEST(RunStats, SteadyStateExcludesWarmup) {
  RunStats stats(2, 2);  // two robots, warmup of two steps
  Task early;
  early.order.id = 7;
  early.created_at = 0;
  Task late;
  late.order.id = 9;
  late.created_at = 2;
  stats.recordOrders({early, late});

  TickStats tick;
  tick.planner_called = true;
  tick.planning_ms = 4.0;
  tick.expansions = 10;
  StepTruth step = sampleStep();  // t = 3, delivers task 7, one safety stop
  stats.recordStep(step, sampleTruth(), tick);
  step.t = 5;
  step.deliveries = {Assignment{0, 9}};
  stats.recordStep(step, sampleTruth(), tick);

  Config config;
  const RunSummary s = stats.summarize(config, 6, 0.1, 0, false);
  EXPECT_EQ(s.orders_created, 2);
  EXPECT_EQ(s.orders_delivered, 2);
  EXPECT_EQ(s.completion_samples, 1);  // task 7 was created before the warmup ended
  EXPECT_DOUBLE_EQ(s.mean_completion_steps, 3.0);
  EXPECT_DOUBLE_EQ(s.orders_per_hour, 2.0 * 3600.0 / 4.0);  // 2 deliveries in 4 measured steps
  EXPECT_DOUBLE_EQ(s.utilization, 2.0 / (2.0 * 4.0));       // robot 1 held a task twice
  EXPECT_EQ(s.conflicts, 2);
  EXPECT_EQ(s.congestion_steps, 2);
  EXPECT_EQ(s.planning_calls, 2);
  EXPECT_DOUBLE_EQ(s.planning_p50_ms, 4.0);
  EXPECT_EQ(s.expansions, 20);
}

// Robot i has outcome outcomes[i] at every step. The first three move: robot 0 with no order,
// robot 1 carrying one, robot 2 on its way to a pickup.
TEST(RunStats, TimeBudgetPutsEveryRobotStepInOneClass) {
  const std::vector<MotionOutcome> outcomes = {
      MotionOutcome::kMoved,         MotionOutcome::kMoved,       MotionOutcome::kMoved,
      MotionOutcome::kService,       MotionOutcome::kPlannedWait, MotionOutcome::kNoPermit,
      MotionOutcome::kYield,         MotionOutcome::kHold,        MotionOutcome::kIdle,
      MotionOutcome::kBlockedVertex, MotionOutcome::kBlockedSwap, MotionOutcome::kBlockedOccupied,
      MotionOutcome::kDelayed,       MotionOutcome::kNoCommand,   MotionOutcome::kSafeStop,
      MotionOutcome::kDead,          MotionOutcome::kBlockedCell, MotionOutcome::kInvalidStep};
  const auto fleet = static_cast<int32_t>(outcomes.size());
  RunStats stats(fleet, 2);
  TruthSnapshot after;
  after.cells.assign(outcomes.size(), Cell{0, 0});
  after.alive.assign(outcomes.size(), true);
  after.status.assign(outcomes.size(), RobotStatus::kToPickup);
  after.holding.assign(outcomes.size(), TaskId{1});
  after.holding[0] = std::nullopt;
  after.status[0] = RobotStatus::kIdle;
  after.status[1] = RobotStatus::kToDropoff;
  for (Timestep t = 0; t < 7; ++t) {
    StepTruth step;
    step.t = t;
    for (int32_t i = 0; i < fleet; ++i) {
      step.motion.push_back(MotionRecord{i, {0, 0}, {0, 0}, {0, 0}, outcomes[i]});
    }
    stats.recordStep(step, after, TickStats{});
  }

  const RunSummary s = stats.summarize(Config{}, 7, 0.1, 0, false);
  const int64_t measured = 5;  // steps 2 to 6
  EXPECT_EQ(s.time_drive_free, measured);
  EXPECT_EQ(s.time_drive_loaded, measured);
  EXPECT_EQ(s.time_drive_to_pickup, measured);
  EXPECT_EQ(s.time_service, measured);
  EXPECT_EQ(s.time_traffic, 3 * measured);
  EXPECT_EQ(s.time_station_wait, measured);
  EXPECT_EQ(s.time_idle, measured);
  EXPECT_EQ(s.time_safety_stop, 3 * measured);
  EXPECT_EQ(s.time_lost, 6 * measured);
  EXPECT_EQ(s.time_drive_to_pickup + s.time_drive_loaded + s.time_drive_free + s.time_service +
                s.time_traffic + s.time_station_wait + s.time_idle + s.time_safety_stop +
                s.time_lost,
            fleet * measured);
}

// Order 5: created 2, assigned 3 (and again 4, which does not count), picked up 5, delivered 9.
// Order 6 has no assignment or pickup on record; the phases still add up for it.
TEST(RunStats, OrderPhasesAddUpToTheCompletionTime) {
  RunStats stats(1, 2);
  std::vector<Task> orders(3);
  orders[0].order.id = 4;  // created in the warmup, so not measured
  orders[0].created_at = 1;
  orders[1].order.id = 5;
  orders[1].created_at = 2;
  orders[2].order.id = 6;
  orders[2].created_at = 3;
  stats.recordOrders(orders);
  stats.recordAssignments(1, {4});
  stats.recordAssignments(3, {5});
  stats.recordAssignments(4, {5});
  TruthSnapshot after;
  after.cells = {Cell{0, 0}};
  after.alive = {true};
  after.status = {RobotStatus::kIdle};
  after.holding = {std::nullopt};
  StepTruth step;
  step.motion = {MotionRecord{0, {0, 0}, {0, 0}, {0, 0}, MotionOutcome::kService}};
  step.t = 5;
  step.pickups = {Assignment{0, 5}};
  stats.recordStep(step, after, TickStats{});
  step.t = 9;
  step.pickups.clear();
  step.deliveries = {Assignment{0, 5}, Assignment{0, 6}, Assignment{0, 4}};
  stats.recordStep(step, after, TickStats{});

  const RunSummary s = stats.summarize(Config{}, 10, 0.1, 0, false);
  EXPECT_EQ(s.completion_samples, 2);
  EXPECT_DOUBLE_EQ(s.mean_completion_steps, (7.0 + 6.0) / 2.0);
  EXPECT_DOUBLE_EQ(s.mean_wait_for_robot_steps, (1.0 + 0.0) / 2.0);
  EXPECT_DOUBLE_EQ(s.mean_to_pickup_steps, (2.0 + 0.0) / 2.0);
  EXPECT_DOUBLE_EQ(s.mean_to_dropoff_steps, (4.0 + 6.0) / 2.0);
}

TEST(RunStats, HashDependsOnTrajectoryOnly) {
  const auto run = [](int32_t col, double planning_ms) {
    RunStats stats(2, 0);
    StepTruth step = sampleStep();
    step.motion[0].to.col = col;
    TickStats tick;
    tick.planning_ms = planning_ms;  // wall time must not change the fingerprint
    stats.recordStep(step, sampleTruth(), tick);
    return stats.hash();
  };
  EXPECT_EQ(run(2, 1.0), run(2, 99.0));
  EXPECT_NE(run(2, 1.0), run(3, 1.0));
}

}  // namespace
}  // namespace fleet_core

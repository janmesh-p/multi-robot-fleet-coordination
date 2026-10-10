#include <unistd.h>

#include <filesystem>
#include <fstream>
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
    metrics.recordStep(sampleStep(), sampleTruth(), tick, 2);
    RunSummary summary;
    summary.run_name = "metrics_test";
    summary.allocator = "nearest";
    summary.planner = "independent_astar";
    summary.log_hash = 0xABCULL;
    metrics.writeSummary(summary);
    metrics.writeViolations({Violation{3, ViolationKind::kSwap, "robot 0, robot 1"}});
  }
  const std::filesystem::path run = dir_ / "run";

  const std::string written_config = readFile(run / "config.yaml");
  EXPECT_EQ(written_config, dumpConfig(config));
  EXPECT_EQ(parseConfig(written_config, "/").run.name, "metrics_test");
  EXPECT_EQ(readFile(run / "map.txt"), map.toString());

  EXPECT_EQ(readFile(run / "steps.csv"),
            "t,orders_created,deliveries,moved,planned_wait,hold,service,idle,delayed,blocked,"
            "assignments,robots_planned,planning_ms,expansions\n"
            "3,2,1,1,0,0,0,0,0,1,1,0,1.5,0\n");
  EXPECT_EQ(readFile(run / "positions.csv"),
            "t,robot,row,col,status,outcome\n"
            "3,0,1,2,idle,moved\n"
            "3,1,2,2,to_pickup,blocked_vertex\n");

  const std::string summary = readFile(run / "summary.csv");
  EXPECT_EQ(summary.rfind("run,allocator,planner,", 0), 0U);
  EXPECT_NE(summary.find("\nmetrics_test,nearest,independent_astar,"), std::string::npos);
  EXPECT_NE(summary.find(",0000000000000abc,"), std::string::npos);
  EXPECT_EQ(readFile(run / "violations.csv"), "t,kind,detail\n3,swap,\"robot 0, robot 1\"\n");
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

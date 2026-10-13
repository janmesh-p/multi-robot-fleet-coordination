#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/fleet_manager.hpp"
#include "fleet_core/metrics/assignment_log.hpp"
#include "fleet_core/simulation.hpp"
#include "fleet_core/verification/invariant_checker.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

Config smallConfig() {
  Config c = loadConfig(kRepo / "benchmarks" / "configs" / "default.yaml");
  c.run.horizon_steps = 600;
  c.run.warmup_steps = 100;
  return c;
}

// The coordinator and simulated robots wired by hand, so a test can feed exact orders.
struct Harness {
  Config config;
  GridMap map;
  GridExecutor robots;
  FleetManager fleet;
  InvariantChecker checker{false};
  AssignmentLog log{};
  int64_t issued{0};
  std::vector<Violation> violations{};

  Harness(const Config& c, const std::string& text, const std::vector<Cell>& spawn)
      : config(c),
        map(GridMap::fromString(text)),
        robots(map, spawn, c.orders.service_steps, 0.0, 1),
        fleet(c, WorldModel(map, spawn)) {}

  const StepTruth& step(Timestep t, const std::vector<Task>& orders = {}) {
    TickInput input;
    input.now = t;
    input.new_orders = orders;
    input.reports = robots.collectReports(t);
    issued += static_cast<int64_t>(orders.size());
    const TickOutput output = fleet.tick(input);
    robots.dispatch(t, output.commands);
    robots.step(t);
    log.record(output, robots.lastStep());
    for (Violation& v :
         checker.check(robots.map(), robots.lastStep(), robots.snapshot(), fleet.tasks(), issued)) {
      violations.push_back(std::move(v));
    }
    return robots.lastStep();
  }
};

Task order(TaskId id, Cell pickup, Cell dropoff, Timestep created_at) {
  Task t;
  t.order = TaskOrder{id, pickup, dropoff};
  t.created_at = created_at;
  return t;
}

const char* const kCorridorMap =
    "RRRRRRR\n"
    ".......\n"
    "P.....D\n"
    ".......\n"
    "RRRRRRR\n";

// One robot, one order, every step accounted for:
//   home (0,0) -> pickup (2,0): 2 moves, arriving at step 1; picking at steps 2 and 3
//   pickup -> dropoff (2,6): the coordinator sees the pickup at step 4; 6 moves, arriving at
//   step 9; dropping at steps 10 and 11, so the delivery lands at step 11.
TEST(FleetManager, SingleRobotTimingIsExact) {
  Config c = smallConfig();
  c.fleet.size = 1;
  Harness h(c, kCorridorMap, {{0, 0}});
  std::optional<Timestep> delivered;
  for (Timestep t = 0; t < 30; ++t) {
    const StepTruth& step =
        h.step(t, t == 0 ? std::vector<Task>{order(0, {2, 0}, {2, 6}, 0)} : std::vector<Task>{});
    if (!step.deliveries.empty()) delivered = t;
  }
  ASSERT_TRUE(delivered.has_value());
  EXPECT_EQ(*delivered, 11);
  const Task& task = h.fleet.tasks().get(0);
  EXPECT_EQ(task.status, TaskStatus::kDone);
  EXPECT_EQ(task.assigned_at, std::optional<Timestep>(0));
  EXPECT_EQ(task.picked_at, std::optional<Timestep>(4));  // learned one tick after the pick
  EXPECT_EQ(task.done_at, std::optional<Timestep>(12));
  EXPECT_EQ(h.robots.robots()[0].cell, (Cell{0, 0}));  // back home
  EXPECT_EQ(h.robots.robots()[0].status, RobotStatus::kIdle);
  EXPECT_TRUE(h.violations.empty());

  // Its assignments.csv row: nothing slowed the robot, so steps_to_pickup is the distance.
  const std::vector<AssignmentRow> rows = h.log.rows();
  ASSERT_EQ(rows.size(), 1U);
  EXPECT_EQ(rows[0].record.t, 0);
  EXPECT_EQ(rows[0].record.features.dist_to_pickup, 2);
  EXPECT_EQ(rows[0].record.features.dist_pickup_to_dropoff, 6);
  EXPECT_EQ(rows[0].record.features.robot_on_endpoint, 1);
  EXPECT_EQ(rows[0].arrived_t, 1);
  EXPECT_EQ(rows[0].steps_to_pickup, 2);
  EXPECT_EQ(rows[0].delivered_t, 11);
  EXPECT_EQ(rows[0].end, AssignmentEnd::kDelivered);
}

// Two robots want the same pickup: the second waits on a spare parking cell (not its home,
// not the aisle) and takes the station only after the first has picked.
// (Both orders are not asserted delivered: with the uncoordinated planner the two robots meet
// head-on in row 1 at step 13 and stop for good. That deadlock is the Stage 2 baseline.)
TEST(FleetManager, StationQueueSendsTheSecondRobotToWait) {
  Config c = smallConfig();
  c.fleet.size = 2;
  Harness h(c, kCorridorMap, {{0, 0}, {0, 1}});
  bool saw_waiting = false;
  int pickups = 0;
  int deliveries = 0;
  for (Timestep t = 0; t < 40; ++t) {
    std::vector<Task> orders;
    if (t == 0) orders = {order(0, {2, 0}, {2, 6}, 0), order(1, {2, 0}, {2, 6}, 0)};
    const StepTruth& step = h.step(t, orders);
    pickups += static_cast<int>(step.pickups.size());
    deliveries += static_cast<int>(step.deliveries.size());
    if (h.fleet.goalOf(1) == Cell{4, 0}) saw_waiting = true;  // spare parking next to the pickup
    int on_station = 0;
    for (const SimRobot& r : h.robots.robots()) on_station += r.cell == Cell{2, 0} ? 1 : 0;
    ASSERT_LE(on_station, 1);
  }
  EXPECT_TRUE(saw_waiting);
  EXPECT_EQ(pickups, 2);
  EXPECT_GE(deliveries, 1);
  EXPECT_TRUE(h.violations.empty());
}

TEST(Simulation, SameSeedSameRunDifferentSeedDifferentRun) {
  const Config c = smallConfig();
  const Simulation a(c);
  const RunSummary first = a.run().summary;
  EXPECT_EQ(a.run().summary.log_hash, first.log_hash);              // repeatable
  EXPECT_EQ(Simulation(c).run().summary.log_hash, first.log_hash);  // fresh instance
  Config other = c;
  other.run.seed = c.run.seed + 1;
  EXPECT_NE(Simulation(other).run().summary.log_hash, first.log_hash);
}

TEST(Simulation, DefaultScenarioHasNoViolations) {
  const RunResult result = Simulation(smallConfig()).run();
  EXPECT_EQ(result.summary.violations, 0);
  EXPECT_FALSE(result.summary.aborted);
  EXPECT_EQ(result.summary.steps_run, 600);
  EXPECT_GT(result.summary.orders_created, 0);
  EXPECT_GT(result.summary.orders_delivered, 0);
}

// The shipped config, replayed: the numbers the README and the explainer quote.
//   Steps 113 to 128: R1 and R6 head-on in column 1. It breaks at 129 only because R6's station
//   turn comes up and gives it a new path.
//   Step 135: R8 is handed its station turn and turns back into R9. That cycle never breaks;
//   R0 to R5 queue behind it, and after step 217 no robot moves again.
TEST(Simulation, DefaultRunDeadlocksAndFreezes) {
  const RunResult result = Simulation(smallConfig()).run();
  const RunSummary& s = result.summary;
  EXPECT_EQ(s.first_deadlock_step, 113);
  EXPECT_EQ(s.deadlocks, 2);
  EXPECT_EQ(s.deadlocked_robots, 2);
  EXPECT_EQ(s.blocked_behind, 6);
  EXPECT_EQ(s.frozen_at, 218);
  EXPECT_EQ(s.last_delivery_step, 168);
  EXPECT_EQ(s.stalled_robots, 10);
  ASSERT_EQ(result.deadlocks.size(), 2U);
  EXPECT_EQ(result.deadlocks[0].robots, (std::vector<RobotId>{1, 6}));
  EXPECT_EQ(result.deadlocks[0].broken, std::optional<Timestep>(129));
  EXPECT_EQ(result.deadlocks[1].formed, 135);
  EXPECT_EQ(result.deadlocks[1].robots, (std::vector<RobotId>{8, 9}));
  EXPECT_FALSE(result.deadlocks[1].broken.has_value());
}

TEST(Simulation, RunsAGeneratedMap) {
  Config c = smallConfig();
  c.map.file.clear();
  c.map.generator = MapGeneratorConfig{3, 3, 4, 2, 3, 1};
  c.fleet.size = 4;
  const Simulation sim(c);
  EXPECT_EQ(sim.map().rows(), generatedRows(*c.map.generator));
  EXPECT_EQ(sim.run().summary.violations, 0);
}

TEST(Simulation, RandomSpawnWithoutParking) {
  Config c = smallConfig();
  c.map.file.clear();
  c.map.generator = MapGeneratorConfig{3, 3, 4, 2, 3, 0};  // no parking rows
  c.features.parking = false;
  c.fleet.size = 5;
  const Simulation sim(c);
  const std::set<Cell> distinct(sim.spawn().begin(), sim.spawn().end());
  EXPECT_EQ(distinct.size(), 5U);
  for (const Cell& cell : sim.spawn()) EXPECT_EQ(sim.map().kind(cell), '.');
  EXPECT_EQ(Simulation(c).spawn(), sim.spawn());  // seeded

  c.features.parking = true;  // homes are required for parking
  EXPECT_THROW(Simulation{c}, std::runtime_error);
}

TEST(Simulation, RejectsWhatThisStageCannotRun) {
  const auto expectRejected = [](Config c, const std::string& fragment) {
    try {
      Simulation sim(c);
      ADD_FAILURE() << "expected rejection containing \"" << fragment << "\"";
    } catch (const std::runtime_error& e) {
      EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos) << e.what();
    }
  };
  Config c = smallConfig();
  c.faults.packet_loss = 0.05;
  expectRejected(c, "Stage 6");
  c = smallConfig();
  c.faults.events = {FaultEvent{5, FaultType::kBlockCells, -1, {{1, 1}}, 0}};
  expectRejected(c, "Stage 5");
  c = smallConfig();
  c.faults.events = {FaultEvent{5, FaultType::kRobotDeath, 0, {}, 0}};
  expectRejected(c, "Stage 6");
  c = smallConfig();
  c.planning.method = PlanningMethod::kCbs;
  expectRejected(c, "not implemented yet");
}

TEST(Simulation, WritesTheRunDirectory) {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / ("fleet_core_sim_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  const RunResult result = Simulation(smallConfig()).run(dir);
  for (const char* name : {"config.yaml", "map.txt", "steps.csv", "positions.csv", "summary.csv",
                           "violations.csv", "deadlocks.csv", "assignments.csv"}) {
    EXPECT_TRUE(std::filesystem::is_regular_file(dir / name)) << name;
  }
  std::ifstream summary(dir / "summary.csv");
  std::stringstream text;
  text << summary.rdbuf();
  char hash[17];
  std::snprintf(hash, sizeof(hash), "%016llx",
                static_cast<unsigned long long>(result.summary.log_hash));
  EXPECT_NE(text.str().find(hash), std::string::npos);
  // The written config reproduces the run.
  EXPECT_EQ(Simulation(loadConfig(dir / "config.yaml")).run().summary.log_hash,
            result.summary.log_hash);
  std::filesystem::remove_all(dir);
}

// What docs/learning_track.md promises about assignments.csv, on real runs of every allocator.
TEST(Simulation, AssignmentRowsKeepTheContract) {
  for (const AllocationMethod method :
       {AllocationMethod::kNearest, AllocationMethod::kHungarian, AllocationMethod::kCostAware}) {
    Config c = smallConfig();
    c.allocation.method = method;
    const RunResult result = Simulation(c).run();
    EXPECT_EQ(result.summary.violations, 0) << toString(method);
    ASSERT_FALSE(result.assignments.empty());
    int64_t delivered = 0;
    for (size_t i = 0; i < result.assignments.size(); ++i) {
      const AssignmentRow& row = result.assignments[i];
      const PairFeatures& f = row.record.features;
      EXPECT_EQ(f.available_robots + f.busy_robots, f.alive_robots);
      if (row.steps_to_pickup >= 0) {
        EXPECT_GE(row.steps_to_pickup, f.dist_to_pickup);
        EXPECT_EQ(row.arrived_t, row.record.t + row.steps_to_pickup - 1);
      } else {
        EXPECT_EQ(row.arrived_t, -1);
      }
      if (row.end == AssignmentEnd::kDelivered) {
        ++delivered;
        EXPECT_GT(row.delivered_t, row.arrived_t);
      } else {
        EXPECT_EQ(row.delivered_t, -1);
      }
      if (i > 0) {
        const AssignmentRecord& prev = result.assignments[i - 1].record;
        EXPECT_TRUE(prev.t < row.record.t ||
                    (prev.t == row.record.t && prev.robot < row.record.robot));
      }
    }
    EXPECT_EQ(delivered, result.summary.orders_delivered) << toString(method);
  }
}

// The free-flow model's cost is exactly dist_to_pickup, so learned with it must replay
// hungarian step for step. docs/learning_track.md promises this test.
TEST(Simulation, LearnedWithTheFreeFlowModelReplaysHungarian) {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / ("fleet_core_learned_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "free_flow.json")
      << R"({"format": "fleet_cost_model", "version": 1, "target": "steps_to_pickup",
            "features": ["dist_to_pickup"], "mean": [0], "scale": [1],
            "layers": [{"weights": [[0]], "bias": [0], "activation": "identity"}],
            "base_feature": "dist_to_pickup"})";
  for (const int32_t fleet : {5, 10}) {
    Config h = smallConfig();
    h.fleet.size = fleet;
    h.allocation.method = AllocationMethod::kHungarian;
    Config l = h;
    l.allocation.method = AllocationMethod::kLearned;
    l.allocation.model_file = dir / "free_flow.json";
    const RunResult a = Simulation(h).run();
    const RunResult b = Simulation(l).run();
    EXPECT_EQ(a.summary.log_hash, b.summary.log_hash) << fleet << " robots";
    ASSERT_EQ(a.assignments.size(), b.assignments.size());
    for (size_t i = 0; i < a.assignments.size(); ++i) {
      EXPECT_EQ(assignmentCsvRow(a.assignments[i]), assignmentCsvRow(b.assignments[i]));
    }
  }
  std::filesystem::remove_all(dir);
}

TEST(Simulation, RejectsABadModelFileAtStartup) {
  const std::filesystem::path dir = std::filesystem::temp_directory_path() /
                                    ("fleet_core_badmodel_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "bad.json") << R"({"format": "fleet_cost_model", "version": 2})";
  Config c = smallConfig();
  c.allocation.method = AllocationMethod::kLearned;
  c.allocation.model_file = dir / "bad.json";
  try {
    Simulation sim(c);
    ADD_FAILURE() << "expected ModelError";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("version"), std::string::npos) << e.what();
  }
  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace fleet_core

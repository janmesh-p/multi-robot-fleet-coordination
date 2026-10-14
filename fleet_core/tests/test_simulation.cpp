#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
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
  InvariantChecker checker;
  AssignmentLog log{};
  int64_t issued{0};
  std::vector<Violation> violations{};
  TickOutput output{};

  Harness(const Config& c, const std::string& text, const std::vector<Cell>& spawn,
          std::unique_ptr<PathPlanner> planner = nullptr)
      : config(c),
        map(GridMap::fromString(text)),
        robots(map, spawn, c.orders.service_steps, 0.0, 1),
        fleet(c, WorldModel(map, spawn), makeAllocator(c),
              planner ? std::move(planner) : makePlanner(c.planning)),
        checker(isCoordinated(c.planning.method)) {}

  const StepTruth& step(Timestep t, const std::vector<Task>& orders = {}) {
    TickInput input;
    input.now = t;
    input.new_orders = orders;
    input.reports = robots.collectReports(t);
    issued += static_cast<int64_t>(orders.size());
    output = fleet.tick(input);
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
  // Coordinated planners need queues and parking (unique goals off the aisles), and the
  // windowed ones need rolling replanning. Late robots wait for Stage 5.
  c = smallConfig();
  c.planning.method = PlanningMethod::kCbs;
  c.features.station_queues = false;
  expectRejected(c, "needs features.station_queues");
  c = smallConfig();
  c.planning.method = PlanningMethod::kEcbs;
  c.features.parking = false;
  expectRejected(c, "needs features.parking");
  for (const PlanningMethod m : {PlanningMethod::kWindowedCoopAStar, PlanningMethod::kPibt}) {
    c = smallConfig();
    c.planning.method = m;
    c.features.rolling_replanning = false;
    expectRejected(c, "needs features.rolling_replanning");
  }
  c = smallConfig();
  c.planning.method = PlanningMethod::kPrioritized;
  c.execution.delay_probability = 0.1;
  expectRejected(c, "Stage 5");
  // The coordinator itself refuses the same configs.
  c.execution.delay_probability = 0.0;
  c.features.station_queues = false;
  const GridMap map = GridMap::fromFile((kRepo / "maps" / "warehouse_small.txt").string());
  EXPECT_THROW(FleetManager(c, WorldModel(map, {})), std::invalid_argument);
  // The baseline runs with every toggle: independent_astar plans robots alone.
  c.planning.method = PlanningMethod::kIndependentAStar;
  c.execution.delay_probability = 0.1;
  EXPECT_NO_THROW(FleetManager(c, WorldModel(map, {})));
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

// ---------------------------------------------------------------------------------------------
// Coordinated planning (Stage 4)

// A planner double: fails the calls it is told to, defers the rest to the real planner, and
// keeps what every call asked.
class ScriptedPlanner final : public PathPlanner {
 public:
  struct Seen {
    Timestep start{0};
    int32_t window{0};
    std::vector<PlanAgent> agents{};
    std::vector<Path> fixed{};
  };

  ScriptedPlanner(const PlanningConfig& config, std::set<size_t> failing)
      : inner_(makePlanner(config)), failing_(std::move(failing)) {}

  std::string_view name() const override { return inner_->name(); }

  PlanResult plan(const PlanningRequest& request) override {
    seen.push_back(Seen{request.start, request.window, request.agents, request.fixed});
    if (failing_.count(seen.size() - 1) != 0) return PlanResult{};
    return inner_->plan(request);
  }

  std::vector<Seen> seen{};

 private:
  std::unique_ptr<PathPlanner> inner_;
  std::set<size_t> failing_;
};

// Rolling horizon: a replanning every replan_period steps, `window` steps ahead. A failed call
// keeps every robot on its plan and is retried the next step.
TEST(FleetManager, RollingHorizonReplansOnScheduleAndRetriesFailures) {
  Config c = smallConfig();
  c.fleet.size = 2;
  c.planning.method = PlanningMethod::kWindowedCoopAStar;
  c.planning.window = 8;
  c.planning.replan_period = 4;
  auto planner = std::make_unique<ScriptedPlanner>(c.planning, std::set<size_t>{1, 2});
  const ScriptedPlanner* calls = planner.get();
  Harness h(c, kCorridorMap, {{0, 0}, {0, 1}}, std::move(planner));
  std::vector<int32_t> plan_id;
  std::vector<bool> failed;
  for (Timestep t = 0; t < 20; ++t) {
    h.step(t, t == 0 ? std::vector<Task>{order(0, {2, 0}, {2, 6}, 0)} : std::vector<Task>{});
    plan_id.push_back(h.output.commands.at(0).plan_id);
    failed.push_back(h.output.stats.planner_failed);
  }
  std::vector<Timestep> starts;
  for (const auto& s : calls->seen) {
    starts.push_back(s.start);
    EXPECT_EQ(s.window, 8);
    EXPECT_EQ(s.agents.size() + s.fixed.size(), 2U);
  }
  EXPECT_EQ(starts, (std::vector<Timestep>{0, 4, 5, 6, 10, 14, 18}));
  EXPECT_TRUE(failed[4] && failed[5] && !failed[6]);
  EXPECT_EQ(plan_id[4], plan_id[3]);  // kept through both failures
  EXPECT_EQ(plan_id[5], plan_id[3]);
  EXPECT_EQ(plan_id[6], plan_id[3] + 1);
  EXPECT_EQ(plan_id[9], plan_id[6]);  // between replannings the robots follow their plans
  EXPECT_TRUE(h.violations.empty());
}

// A robot dwelling at a station is no agent: it holds the station as a fixed obstacle until its
// service ends, in rolling and in full-horizon mode.
TEST(FleetManager, RobotsAtTheirServiceAreFixedObstacles) {
  for (const bool rolling : {true, false}) {
    Config c = smallConfig();
    c.fleet.size = 2;
    c.orders.service_steps = 12;
    c.planning.method = PlanningMethod::kEcbs;
    c.planning.replan_period = 3;
    c.features.rolling_replanning = rolling;
    auto planner = std::make_unique<ScriptedPlanner>(c.planning, std::set<size_t>{});
    const ScriptedPlanner* calls = planner.get();
    Harness h(c, kCorridorMap, {{0, 0}, {0, 1}}, std::move(planner));
    std::map<Timestep, Cell> dwelling;  // steps at whose tick robot 0 was at its service
    for (Timestep t = 0; t < 90; ++t) {
      const SimRobot& r = h.robots.robots()[0];
      if (r.status == RobotStatus::kPicking || r.status == RobotStatus::kDropping) {
        dwelling[t] = r.cell;
      }
      h.step(t, t == 0 ? std::vector<Task>{order(0, {2, 0}, {2, 6}, 0), order(1, {2, 0}, {2, 6}, 0)}
                       : std::vector<Task>{});
    }
    int checked = 0;
    for (const auto& s : calls->seen) {
      const auto it = dwelling.find(s.start);
      EXPECT_EQ(s.window, rolling ? c.planning.window : 0);
      if (it == dwelling.end()) continue;
      ++checked;
      for (const PlanAgent& a : s.agents) EXPECT_NE(a.robot, 0) << "step " << s.start;
      bool fixed = false;
      for (const Path& p : s.fixed) {
        if (p.robot == 0) fixed = positionAt(p, s.start + 50) == it->second;
      }
      EXPECT_TRUE(fixed) << "step " << s.start;
    }
    EXPECT_GT(checked, 0) << "rolling " << rolling;
    EXPECT_EQ(h.fleet.tasks().get(0).status, TaskStatus::kDone) << "rolling " << rolling;
    EXPECT_EQ(h.fleet.tasks().get(1).status, TaskStatus::kDone) << "rolling " << rolling;
    EXPECT_TRUE(h.violations.empty()) << "rolling " << rolling;
  }
}

// The two-robot station queue again, now coordinated: the second robot waits on the spare
// parking cell, and both orders are delivered without a single safety stop.
TEST(FleetManager, CoordinatedStationQueueDeliversBoth) {
  for (const PlanningMethod m :
       {PlanningMethod::kPrioritized, PlanningMethod::kWindowedCoopAStar, PlanningMethod::kCbs,
        PlanningMethod::kEcbs, PlanningMethod::kPibt}) {
    Config c = smallConfig();
    c.fleet.size = 2;
    c.planning.method = m;
    Harness h(c, kCorridorMap, {{0, 0}, {0, 1}});
    bool saw_waiting = false;
    int deliveries = 0;
    for (Timestep t = 0; t < 60; ++t) {
      std::vector<Task> orders;
      if (t == 0) {
        orders.push_back(order(0, {2, 0}, {2, 6}, 0));
        orders.push_back(order(1, {2, 0}, {2, 6}, 0));
      }
      deliveries += static_cast<int>(h.step(t, orders).deliveries.size());
      if (h.fleet.goalOf(1) == Cell{4, 0}) saw_waiting = true;
    }
    EXPECT_TRUE(saw_waiting) << toString(m);
    EXPECT_EQ(deliveries, 2) << toString(m);
    EXPECT_TRUE(h.violations.empty()) << toString(m);
  }
}

// With every waiting cell taken, the baseline waits where it is: here, on the pickup it just
// served, while the dropoff it needs is busy. A coordinated fleet sends the robot home instead.
TEST(FleetManager, CoordinatedRobotsWaitAtHomeWithoutAWaitingCell) {
  const char* const two_homes =  // two parking cells, both homes: no waiting cells
      "RR...................\n"
      ".....................\n"
      "P...................D\n"
      ".....................\n";
  for (const PlanningMethod m : {PlanningMethod::kIndependentAStar, PlanningMethod::kEcbs}) {
    Config c = smallConfig();
    c.fleet.size = 2;
    c.orders.service_steps = 30;
    c.planning.method = m;
    Harness h(c, two_homes, {{0, 0}, {0, 1}});
    // Robot 0 takes order 0 first; robot 1 picks order 1 after it and finds the dropoff busy.
    std::set<Cell> goals_while_carrying;
    for (Timestep t = 0; t < 200; ++t) {
      std::vector<Task> orders;
      if (t == 0) {
        orders.push_back(order(0, {2, 0}, {2, 20}, 0));
        orders.push_back(order(1, {2, 0}, {2, 20}, 0));
      }
      h.step(t, orders);
      if (h.fleet.tasks().get(1).status == TaskStatus::kPickedUp &&
          h.fleet.tasks().get(0).status != TaskStatus::kDone) {
        goals_while_carrying.insert(h.fleet.goalOf(1));
      }
    }
    const Cell expected = m == PlanningMethod::kEcbs ? Cell{0, 1} : Cell{2, 0};
    EXPECT_EQ(goals_while_carrying.count(expected), 1U) << toString(m);
    EXPECT_TRUE(h.violations.empty()) << toString(m);
    if (m == PlanningMethod::kEcbs) {
      EXPECT_EQ(h.fleet.tasks().get(1).status, TaskStatus::kDone);
    }
  }
}

// The shipped scenario with every coordinated planner, with rolling replanning and, where the
// planner allows it, without: no robot safety stop, no violation, the fleet keeps delivering,
// and the run repeats bit for bit.
TEST(Simulation, CoordinatedPlannersRunTheShippedScenarioCleanly) {
  const std::vector<std::pair<PlanningMethod, bool>> cases = {
      {PlanningMethod::kPrioritized, true},
      {PlanningMethod::kPrioritized, false},
      {PlanningMethod::kWindowedCoopAStar, true},
      {PlanningMethod::kCbs, true},
      {PlanningMethod::kCbs, false},
      {PlanningMethod::kEcbs, true},
      {PlanningMethod::kEcbs, false},
      {PlanningMethod::kPibt, true}};
  for (const auto& [method, rolling] : cases) {
    Config c = smallConfig();
    c.allocation.method = AllocationMethod::kHungarian;
    c.planning.method = method;
    c.features.rolling_replanning = rolling;
    const Simulation sim(c);
    const RunSummary s = sim.run().summary;
    const std::string what = std::string(toString(method)) + (rolling ? " rolling" : " to goals");
    EXPECT_EQ(s.violations, 0) << what;
    EXPECT_FALSE(s.aborted) << what;
    EXPECT_EQ(s.conflicts, 0) << what;
    EXPECT_EQ(s.deadlocks, 0) << what;
    EXPECT_LT(s.frozen_at, 0) << what;
    EXPECT_GE(s.orders_delivered * 4, s.orders_created * 3) << what;
    if (rolling) {
      EXPECT_LE(s.planning_calls, 600 / c.planning.replan_period + s.planner_failures) << what;
    }
    EXPECT_EQ(sim.run().summary.log_hash, s.log_hash) << what;
  }
}

}  // namespace
}  // namespace fleet_core

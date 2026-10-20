// Stage 6: lost reports and commands, robot deaths, false deaths, and the fuzz harness.

#include <algorithm>
#include <filesystem>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/rng.hpp"
#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/faults/fault_manager.hpp"
#include "fleet_core/fleet_manager.hpp"
#include "fleet_core/fuzz.hpp"
#include "fleet_core/simulation.hpp"
#include "fleet_core/tasks/station_manager.hpp"
#include "fleet_core/tasks/task_manager.hpp"
#include "fleet_core/traffic/traffic_manager.hpp"
#include "fleet_core/verification/invariant_checker.hpp"
#include "fleet_core/world/distance_oracle.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

const char* const kCorridor =
    "RRRRRRR\n"
    ".......\n"
    "P.....D\n"
    ".......\n"
    "RRRRRRR\n";

RobotReport reportFrom(RobotId robot, uint32_t seq, Cell cell) {
  RobotReport r;
  r.robot = robot;
  r.seq = seq;
  r.cell = cell;
  return r;
}

Task order(TaskId id, Cell pickup, Cell dropoff, Timestep created_at = 0) {
  Task t;
  t.order = TaskOrder{id, pickup, dropoff};
  t.created_at = created_at;
  return t;
}

TEST(FaultManager, ALeaseRunsOutAfterTheTimeout) {
  WorldModel world(GridMap::fromString("....\n"), {{0, 0}, {0, 3}});
  FaultsConfig config;
  config.heartbeat_timeout_steps = 3;
  const FaultManager faults(config);
  world.setNow(0);
  world.applyReport(reportFrom(0, 1, {0, 0}));
  world.applyReport(reportFrom(1, 1, {0, 3}));
  world.setNow(3);
  world.applyReport(reportFrom(1, 2, {0, 3}));
  EXPECT_TRUE(faults.expired(3, world).empty());  // silent for 3 steps: still fine
  EXPECT_EQ(faults.expired(4, world), std::vector<RobotId>{0});
  world.markDead(0);
  EXPECT_TRUE(faults.expired(5, world).empty());  // already dead
  EXPECT_EQ(faults.expired(7, world), std::vector<RobotId>{1});
}

TEST(WorldModel, ARobotDeclaredDeadCanComeBackUntilItIsTakenAway) {
  WorldModel world(GridMap::fromString("....\n"), {{0, 0}, {0, 3}});
  world.applyReport(reportFrom(0, 1, {0, 0}));
  world.markDead(0);
  EXPECT_FALSE(world.applyReport(reportFrom(0, 2, {0, 1})));
  RobotReport back = reportFrom(0, 3, {0, 1});
  back.status = RobotStatus::kToDropoff;
  back.carrying = true;
  world.revive(back);
  EXPECT_EQ(world.robot(0).status, RobotStatus::kToDropoff);
  EXPECT_EQ(world.robot(0).cell, (Cell{0, 1}));
  EXPECT_TRUE(world.robot(0).carrying);
  world.markRemoved(1);
  world.revive(reportFrom(1, 1, {0, 3}));
  EXPECT_EQ(world.robot(1).status, RobotStatus::kDead);
  EXPECT_TRUE(world.robot(1).removed);
}

TEST(TaskManager, OrdersTakenBackAndDeliveriesHeardLate) {
  TaskManager tasks;
  tasks.add(order(0, {0, 0}, {0, 3}));
  tasks.add(order(1, {0, 0}, {0, 3}));
  tasks.assign(0, 4, 0);
  tasks.requeue(0, 1);
  EXPECT_EQ(tasks.get(0).requeues, 1);
  tasks.assign(0, 5, 2);
  tasks.markDoneBy(0, 4, 3);  // robot 4 delivered it while out of touch
  EXPECT_EQ(tasks.get(0).status, TaskStatus::kDone);
  EXPECT_EQ(tasks.get(0).robot, std::optional<RobotId>(4));
  EXPECT_FALSE(tasks.taskOf(5).has_value());
  tasks.markDoneBy(1, 6, 4);  // straight from pending
  EXPECT_EQ(tasks.pendingCount(), 0);
  EXPECT_EQ(tasks.doneCount(), 2);
  EXPECT_THROW(tasks.markDoneBy(1, 6, 5), std::logic_error);
}

// Waiting cells are spare parking cells. One a dead robot may stand on closes (Stage 6).
TEST(StationManager, AClosedWaitingCellIsNeverOffered) {
  GridMap map = GridMap::fromString(
      "RRRR\n"
      "....\n"
      "P...\n");
  const DistanceOracle d(map, true);
  StationManager stations(map, d, {}, true);
  stations.request(0, {2, 0});
  stations.request(1, {2, 0});
  const Cell first = stations.goalFor(1, {2, 0}, {9, 9});
  ASSERT_NE(first, (Cell{9, 9}));
  map.block(first);
  stations.refresh(map);
  const Cell second = stations.goalFor(1, {2, 0}, {9, 9});
  EXPECT_NE(second, first);
  stations.request(2, {2, 0});
  EXPECT_NE(stations.goalFor(2, {2, 0}, {9, 9}), first);
}

// A robot whose report was lost may have moved anywhere its commands allowed. With the traffic
// rules, no other robot gets a permit into those cells, nor a permit to follow it.
TEST(TrafficManager, CellsASilentRobotMayReachStayClear) {
  WorldModel world(GridMap::fromString(".....\n"), {{0, 1}, {0, 4}});
  TrafficManager traffic(TrafficRules{true, true});
  traffic.commit({CommittedPlan{0, 0, Path{0, 0, {{0, 1}, {0, 2}}}},
                  CommittedPlan{1, 0, Path{1, 0, {{0, 4}, {0, 3}}}}});
  world.setNow(1);
  world.applyReport(reportFrom(1, 1, {0, 4}));  // robot 0 not heard this step
  const std::vector<std::vector<Cell>> reach = {{{0, 1}, {0, 2}, {0, 3}}, {{0, 4}}};
  const TrafficDecision held = traffic.decide(world, {-1, -1}, reach);
  EXPECT_TRUE(held.permits[1].held);
  EXPECT_EQ(held.permits[1].waits_for, 0);
  // Heard again from where it was: its reach is that cell, and the way is open.
  RobotReport heard = reportFrom(0, 1, {0, 1});
  heard.plan_id = 0;
  world.applyReport(heard);
  const TrafficDecision open = traffic.decide(world, {-1, -1}, {{{0, 1}}, {{0, 4}}});
  EXPECT_FALSE(open.permits[1].held);
}

TEST(InvariantChecker, AnOrderTakenBackMayBeHeldTwice) {
  const GridMap map = GridMap::fromString("....\n");
  TaskManager tasks;  // order 1 is robot 1's and was never taken back
  tasks.add(order(0, {0, 0}, {0, 3}));
  tasks.add(order(1, {0, 0}, {0, 3}));
  tasks.assign(1, 1, 0);
  StepTruth step;
  step.motion = {MotionRecord{0, {0, 0}, {0, 0}, {0, 0}, MotionOutcome::kIdle},
                 MotionRecord{1, {0, 2}, {0, 2}, {0, 2}, MotionOutcome::kIdle}};
  TruthSnapshot after;
  after.cells = {{0, 0}, {0, 2}};
  after.alive = {true, true};
  after.on_floor = {true, true};
  after.status = {RobotStatus::kToPickup, RobotStatus::kToPickup};
  after.holding = {TaskId{1}, TaskId{1}};
  InvariantChecker checker(true);
  EXPECT_EQ(checker.check(map, step, after, tasks, 2, {}).size(), 1U);
  TaskManager taken;  // order 1 was robot 0's, then went to robot 1 while robot 0 was silent
  taken.add(order(0, {0, 0}, {0, 3}));
  taken.add(order(1, {0, 0}, {0, 3}));
  taken.assign(1, 0, 0);
  taken.requeue(1, 1);
  taken.assign(1, 1, 1);
  EXPECT_TRUE(InvariantChecker(true).check(map, step, after, taken, 2, {}).empty());
  // A robot taken off the floor no longer occupies its cell.
  after.cells = {{0, 2}, {0, 2}};
  after.alive = {false, true};
  after.on_floor = {false, true};
  after.holding = {std::nullopt, std::nullopt};
  EXPECT_TRUE(InvariantChecker(true).check(map, step, after, taken, 2, {}).empty());
}

// The coordinator and simulated robots wired by hand, with a switch that drops one robot's
// reports, so a test decides exactly when a robot goes silent and when it is heard again.
struct FaultHarness {
  GridExecutor robots;
  FleetManager fleet;
  InvariantChecker checker{true};
  std::set<RobotId> silent{};
  std::vector<RobotId> removed{};
  int64_t issued{0};
  int64_t deliveries{0};
  std::vector<Violation> violations{};
  TickOutput output{};

  FaultHarness(const Config& c, const std::vector<Cell>& spawn, ExecutorFaults faults)
      : robots(GridMap::fromString(kCorridor), spawn, c.orders.service_steps, 0.0, 1, faults),
        fleet(c, WorldModel(GridMap::fromString(kCorridor), spawn)) {}

  void step(Timestep t, const std::vector<Task>& orders = {}) {
    TickInput input;
    input.now = t;
    input.new_orders = orders;
    for (const RobotReport& r : robots.collectReports(t)) {
      if (silent.count(r.robot) == 0) input.reports.push_back(r);
    }
    input.removed = removed;
    issued += static_cast<int64_t>(orders.size());
    output = fleet.tick(input);
    robots.dispatch(t, output.commands);
    robots.step(t);
    removed = robots.lastStep().removed;
    deliveries += static_cast<int64_t>(robots.lastStep().deliveries.size());
    for (Violation& v : checker.check(robots.map(), robots.lastStep(), robots.snapshot(),
                                      fleet.tasks(), issued, output.commands)) {
      violations.push_back(std::move(v));
    }
  }
};

Config faultConfig() {
  Config c = loadConfig(kRepo / "benchmarks" / "configs" / "default.yaml");
  c.planning.method = PlanningMethod::kPibt;
  c.fleet.size = 2;
  c.faults.heartbeat_timeout_steps = 3;
  return c;
}

ExecutorFaults robotSide(const Config& c, Timestep rescue_after = kNever) {
  ExecutorFaults f;
  f.safe_stop_after = c.faults.heartbeat_timeout_steps;
  f.rescue_after = rescue_after;
  return f;
}

// Robot 0 picks the order up, then goes silent: declared dead, it loses the order to robot 1
// and its cell closes. Heard again, it is still carrying, so the order comes back to it and robot
// 1 drops it. The order is delivered once, by robot 0, without a violation.
TEST(FleetManager, ASilentRobotLosesItsOrderAndGetsItBackIfItCarriesIt) {
  const Config c = faultConfig();
  FaultHarness h(c, {{0, 0}, {0, 6}}, robotSide(c));
  Timestep t = 0;
  h.step(t++, {order(0, {2, 0}, {2, 6})});
  while (!h.robots.robots()[0].carrying && t < 20) h.step(t++);
  ASSERT_TRUE(h.robots.robots()[0].carrying);
  const Cell where = h.robots.robots()[0].cell;
  h.silent.insert(0);
  while (h.output.declared_dead.empty() && t < 40) h.step(t++);
  ASSERT_EQ(h.output.declared_dead, std::vector<RobotId>{0});
  EXPECT_EQ(h.output.requeued, std::vector<TaskId>{0});
  EXPECT_EQ(h.fleet.world().robot(0).status, RobotStatus::kDead);
  EXPECT_TRUE(h.fleet.world().map().isBlocked(where));
  EXPECT_TRUE(std::none_of(h.output.commands.begin(), h.output.commands.end(),
                           [](const RobotCommand& cmd) { return cmd.robot == 0; }));
  for (int k = 0; k < 6; ++k) h.step(t++);
  EXPECT_EQ(h.fleet.tasks().get(0).robot, std::optional<RobotId>(1));
  EXPECT_EQ(h.robots.robots()[0].status, RobotStatus::kToDropoff);
  EXPECT_TRUE(h.robots.robots()[0].safe_stop);

  h.silent.clear();
  h.step(t++);
  EXPECT_EQ(h.output.reconciled, std::vector<RobotId>{0});
  EXPECT_EQ(h.fleet.tasks().get(0).robot, std::optional<RobotId>(0));
  EXPECT_EQ(h.fleet.tasks().get(0).status, TaskStatus::kPickedUp);
  EXPECT_FALSE(h.fleet.world().map().isBlocked(where));
  while (h.deliveries == 0 && t < 120) h.step(t++);
  EXPECT_EQ(h.deliveries, 1);
  EXPECT_FALSE(h.robots.robots()[1].task.has_value());
  EXPECT_TRUE(h.violations.empty()) << h.violations.front().detail;
}

// Robot 0 goes silent while dropping off and finishes the delivery unheard. Its order goes to
// robot 1, but when robot 0 is heard again the delivery counts and robot 1 drops the order.
TEST(FleetManager, ADeliveryMadeOutOfTouchCounts) {
  const Config c = faultConfig();
  FaultHarness h(c, {{0, 0}, {0, 6}}, robotSide(c));
  Timestep t = 0;
  h.step(t++, {order(0, {2, 0}, {2, 6})});
  while (h.robots.robots()[0].status != RobotStatus::kDropping && t < 60) h.step(t++);
  ASSERT_EQ(h.robots.robots()[0].status, RobotStatus::kDropping);
  h.silent.insert(0);
  while (h.fleet.tasks().get(0).robot != std::optional<RobotId>(1) && t < 80) h.step(t++);
  ASSERT_EQ(h.fleet.tasks().get(0).robot, std::optional<RobotId>(1));
  EXPECT_EQ(h.deliveries, 1);  // robot 0 finished it unheard
  h.silent.clear();
  h.step(t++);
  EXPECT_EQ(h.output.reconciled, std::vector<RobotId>{0});
  EXPECT_EQ(h.output.requeued, std::vector<TaskId>{0});  // taken back from robot 1
  EXPECT_EQ(h.fleet.tasks().get(0).status, TaskStatus::kDone);
  EXPECT_EQ(h.fleet.tasks().get(0).robot, std::optional<RobotId>(0));
  for (int k = 0; k < 30; ++k) h.step(t++);
  EXPECT_EQ(h.deliveries, 1);
  EXPECT_FALSE(h.robots.robots()[1].task.has_value());
  EXPECT_TRUE(h.violations.empty()) << h.violations.front().detail;
}

// A robot dies for good. Declared dead, its cell stays closed until it is taken off the floor,
// and robot 1 delivers its order.
TEST(FleetManager, ADeadRobotKeepsItsCellClosedUntilItIsTakenAway) {
  const Config c = faultConfig();
  FaultHarness h(c, {{0, 0}, {0, 6}}, robotSide(c, 20));
  Timestep t = 0;
  h.step(t++, {order(0, {2, 0}, {2, 6})});
  while (h.robots.robots()[0].cell == Cell{0, 0} && t < 10) h.step(t++);
  const Cell where = h.robots.robots()[0].cell;
  h.robots.kill(0, t);
  while (h.output.declared_dead.empty() && t < 30) h.step(t++);
  ASSERT_EQ(h.output.declared_dead, std::vector<RobotId>{0});
  EXPECT_TRUE(h.fleet.world().map().isBlocked(where));
  while (!h.fleet.world().robot(0).removed && t < 60) h.step(t++);
  ASSERT_TRUE(h.fleet.world().robot(0).removed);
  EXPECT_FALSE(h.fleet.world().map().isBlocked(where));
  while (h.deliveries == 0 && t < 160) h.step(t++);
  EXPECT_EQ(h.deliveries, 1);
  EXPECT_EQ(h.fleet.tasks().get(0).robot, std::optional<RobotId>(1));
  EXPECT_TRUE(h.violations.empty()) << h.violations.front().detail;
}

Config runConfig() {
  Config c = loadConfig(kRepo / "benchmarks" / "configs" / "planning.yaml");
  c.run.horizon_steps = 900;
  c.run.warmup_steps = 100;
  return c;
}

TEST(Simulation, DeathsAndOutagesRunWithoutAViolation) {
  Config c = runConfig();
  c.faults.events = {FaultEvent{100, FaultType::kRobotDeath, 0, {}, 0},
                     FaultEvent{200, FaultType::kCommOutage, 3, {}, 60},
                     FaultEvent{300, FaultType::kCommOutage, 4, {}, 3}};
  const RunResult r = Simulation(c).run(std::nullopt);
  EXPECT_EQ(r.summary.violations, 0);
  EXPECT_EQ(r.summary.robots_died, 1);
  EXPECT_EQ(r.summary.declared_dead, 2);  // robot 0, and robot 3 out of touch for 60 steps
  EXPECT_EQ(r.summary.false_deaths, 1);
  EXPECT_EQ(r.summary.reconciled, 1);
  EXPECT_EQ(r.summary.rescued, 1);
  EXPECT_EQ(r.summary.reports_lost, 63);
  EXPECT_GT(r.summary.orders_delivered, 50);
}

// Packet loss under every coordinated planner: no violation, and the same run twice.
TEST(Simulation, PacketLossRunsWithoutAViolation) {
  for (const PlanningMethod m : {PlanningMethod::kPibt, PlanningMethod::kWindowedCoopAStar,
                                 PlanningMethod::kPrioritized, PlanningMethod::kEcbs}) {
    Config c = runConfig();
    c.planning.method = m;
    c.faults.packet_loss = 0.2;
    c.execution.delay_probability = 0.1;
    const RunResult a = Simulation(c).run(std::nullopt);
    EXPECT_EQ(a.summary.violations, 0) << toString(m);
    EXPECT_GT(a.summary.reports_lost, 0) << toString(m);
    EXPECT_GT(a.summary.no_command_steps, 0) << toString(m);
    EXPECT_GT(a.summary.orders_delivered, 25) << toString(m);
    EXPECT_EQ(Simulation(c).run(std::nullopt).summary.log_hash, a.summary.log_hash) << toString(m);
  }
}

TEST(Fuzz, ScenariosAreReproducibleAndPass) {
  const Config base = runConfig();
  for (uint64_t seed = 1; seed <= 4; ++seed) {
    const auto a = randomScenario(base, seed, 4);
    const auto b = randomScenario(base, seed, 4);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(dumpConfig(*a), dumpConfig(*b));
    EXPECT_NO_THROW(validateConfig(*a));
    const FuzzResult r = checkScenario(*a);
    EXPECT_TRUE(r.passed) << "seed " << seed << ": " << r.failure;
  }
}

// Turning the traffic rules off with late robots makes safety stops, which a coordinated planner
// counts as violations: the shrunk scenario stops just after the first one and needs no events.
TEST(Fuzz, ShrinkingKeepsTheFailure) {
  Config c = runConfig();
  c.fleet.size = 25;
  c.execution.delay_probability = 0.2;
  c.features.dependency_graph = false;
  c.features.intersection_reservations = false;
  c.verification.on_violation = ViolationMode::kRecord;
  c.faults.events = {FaultEvent{850, FaultType::kCommOutage, 2, {}, 5}};
  const FuzzResult failed = checkScenario(c);
  ASSERT_FALSE(failed.passed);
  ASSERT_GE(failed.failed_at, 0);
  const Config small = shrinkScenario(c);
  EXPECT_EQ(small.run.horizon_steps, failed.failed_at + 1);
  EXPECT_TRUE(small.faults.events.empty());
  EXPECT_FALSE(checkScenario(small).passed);
}

}  // namespace
}  // namespace fleet_core

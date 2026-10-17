// Whole runs with robots that run late and aisles that close (Stage 5).
#include <algorithm>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/rng.hpp"
#include "fleet_core/planning/path_planner.hpp"
#include "fleet_core/simulation.hpp"
#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

// The shipped scenario for 600 steps with the optimal matching: 10 robots on warehouse_small.
Config smallConfig(PlanningMethod method) {
  Config c = loadConfig(kRepo / "benchmarks" / "configs" / "default.yaml");
  c.run.horizon_steps = 600;
  c.run.warmup_steps = 100;
  c.allocation.method = AllocationMethod::kHungarian;
  c.planning.method = method;
  return c;
}

std::string name(PlanningMethod method) { return std::string(toString(method)); }

const PlanningMethod kCoordinated[] = {PlanningMethod::kPrioritized,
                                       PlanningMethod::kWindowedCoopAStar, PlanningMethod::kCbs,
                                       PlanningMethod::kEcbs, PlanningMethod::kPibt};

// With the default traffic rules, robots that miss one step in five never meet: no safety stop,
// no violation. Followers wait behind late robots, and the runs repeat bit for bit.
TEST(LateRobots, NeverMeetUnderTheTrafficRules) {
  for (const PlanningMethod m : kCoordinated) {
    Config c = smallConfig(m);
    c.execution.delay_probability = 0.2;
    const Simulation sim(c);
    const RunSummary s = sim.run().summary;
    EXPECT_EQ(s.violations, 0) << name(m);
    EXPECT_FALSE(s.aborted) << name(m);
    EXPECT_EQ(s.conflicts, 0) << name(m);
    EXPECT_GT(s.delayed_steps, 0) << name(m);
    EXPECT_GT(s.yields + s.traffic_holds, 0) << name(m);
    EXPECT_GT(s.orders_delivered, 10) << name(m);
    EXPECT_EQ(sim.run().summary.log_hash, s.log_hash) << name(m);
  }
}

// The same late robots with both rules off meet: robot safety stops them, and every stop is a
// violation for a coordinated planner. Either rule alone keeps them apart.
TEST(LateRobots, EitherRuleKeepsThemApartAndNeitherDoesNot) {
  for (const PlanningMethod m : {PlanningMethod::kWindowedCoopAStar, PlanningMethod::kPibt}) {
    Config c = smallConfig(m);
    c.execution.delay_probability = 0.2;
    c.verification.on_violation = ViolationMode::kRecord;
    c.features.dependency_graph = false;
    c.features.intersection_reservations = false;
    const RunSummary none = Simulation(c).run().summary;
    EXPECT_GT(none.conflicts, 0) << name(m);
    EXPECT_GT(none.violations, 0) << name(m);
    c.verification.on_violation = ViolationMode::kAbort;
    EXPECT_TRUE(Simulation(c).run().summary.aborted) << name(m);

    c.features.dependency_graph = true;
    EXPECT_EQ(Simulation(c).run().summary.violations, 0) << name(m) << " dependency graph";
    c.features.dependency_graph = false;
    c.features.intersection_reservations = true;
    EXPECT_EQ(Simulation(c).run().summary.violations, 0) << name(m) << " reservations";
  }
}

// Planning to goals replans a few robots at a time, so the dependency graph does not apply.
// Reservations keep late robots apart, but robots can then meet head-on and wait for each other:
// deadlock recovery replans the fleet. Without it the fleet locks up; without reservations it
// collides.
TEST(LateRobots, PlanningToGoalsNeedsReservationsAndRecovery) {
  for (const PlanningMethod m :
       {PlanningMethod::kPrioritized, PlanningMethod::kCbs, PlanningMethod::kEcbs}) {
    Config c = smallConfig(m);
    c.execution.delay_probability = 0.1;
    c.features.rolling_replanning = false;
    const RunSummary full = Simulation(c).run().summary;
    EXPECT_EQ(full.violations, 0) << name(m);
    EXPECT_GT(full.deadlocks_resolved, 0) << name(m);
    EXPECT_LT(full.frozen_at, 0) << name(m);
    EXPECT_EQ(full.deadlocks, 0) << name(m);

    c.features.deadlock_recovery = false;
    const RunSummary stuck = Simulation(c).run().summary;
    EXPECT_EQ(stuck.violations, 0) << name(m);
    EXPECT_GT(stuck.deadlocks, 0) << name(m);

    c.features.deadlock_recovery = true;
    c.features.intersection_reservations = false;
    c.verification.on_violation = ViolationMode::kRecord;
    EXPECT_GT(Simulation(c).run().summary.violations, 0) << name(m);
  }
  // 25 robots, seed 10: replanning only the robots on a cycle found no way out around the others'
  // plans, every tick, and the fleet froze at step 287. Recovery replans the whole fleet instead.
  Config c = smallConfig(PlanningMethod::kPrioritized);
  c.fleet.size = 25;
  c.run.seed = 10;
  c.execution.delay_probability = 0.1;
  c.features.rolling_replanning = false;
  const RunSummary s = Simulation(c).run().summary;
  EXPECT_EQ(s.violations, 0);
  EXPECT_GT(s.deadlocks_resolved, 0);
  EXPECT_EQ(s.planner_failures, 0);
  EXPECT_EQ(s.deadlocks, 0);
  EXPECT_LT(s.frozen_at, 0);
}

// Six cells of the middle aisle close at step 40 and reopen at step 450. Every planner routes
// around them, nobody stands on a closed cell (the checker would report it), and the closure
// changes the run.
TEST(Closures, RobotsRouteAroundAClosedAisle) {
  std::vector<Cell> cells;
  for (int32_t col = 9; col <= 14; ++col) cells.push_back(Cell{7, col});
  for (const PlanningMethod m :
       {PlanningMethod::kIndependentAStar, PlanningMethod::kPrioritized,
        PlanningMethod::kWindowedCoopAStar, PlanningMethod::kEcbs, PlanningMethod::kPibt}) {
    for (const bool rolling : {true, false}) {
      if (!rolling && (m == PlanningMethod::kWindowedCoopAStar || m == PlanningMethod::kPibt)) {
        continue;
      }
      Config c = smallConfig(m);
      c.features.rolling_replanning = rolling;
      const RunSummary open = Simulation(c).run().summary;
      c.faults.events = {FaultEvent{40, FaultType::kBlockCells, -1, cells, 0},
                         FaultEvent{450, FaultType::kUnblockCells, -1, cells, 0}};
      const RunSummary closed = Simulation(c).run().summary;
      const std::string what = name(m) + (rolling ? " rolling" : " to goals");
      EXPECT_EQ(closed.violations, 0) << what;
      EXPECT_NE(closed.log_hash, open.log_hash) << what;
      if (isCoordinated(m)) {
        EXPECT_EQ(closed.conflicts, 0) << what;
        EXPECT_GT(closed.orders_delivered, 10) << what;
      }
    }
  }
}

// Robot 6 stands at (7,11), inside the six closing cells, when the closure arrives at step 900.
// Closing (7,10) and (7,12) at once left it no way out, and windowed_coop_astar found no plan
// for the whole half hour. Now the cells wait until robot 6 has driven out, then close together.
TEST(Closures, ARobotInsideAClosingSegmentDrivesOutFirst) {
  Config c = loadConfig(kRepo / "benchmarks" / "configs" / "closure.yaml");
  c.planning.method = PlanningMethod::kWindowedCoopAStar;
  c.run.seed = 8;
  c.run.horizon_steps = 1200;
  c.faults.events.resize(1);  // the reopening falls after the end
  const RunSummary s = Simulation(c).run().summary;
  EXPECT_EQ(s.violations, 0);
  EXPECT_EQ(s.planner_failures, 0);
  EXPECT_EQ(s.conflicts, 0);
  EXPECT_GT(s.orders_per_hour, 300.0);
}

// Random warehouses, fleets, planners, allocators, loads, delays, and closures: no run may
// break an invariant. One fleet in four takes every parking cell, so no station has a waiting
// cell. Each failure names its scenario.
TEST(Stress, RandomScenariosBreakNoInvariant) {
  Rng rng(20261015);
  const auto pick = [&](int64_t lo, int64_t hi) { return rng.uniformInt(lo, hi); };
  const PlanningMethod planners[] = {PlanningMethod::kIndependentAStar,
                                     PlanningMethod::kPrioritized,
                                     PlanningMethod::kWindowedCoopAStar,
                                     PlanningMethod::kCbs,
                                     PlanningMethod::kEcbs,
                                     PlanningMethod::kPibt};
  const AllocationMethod allocators[] = {AllocationMethod::kNearest, AllocationMethod::kHungarian,
                                         AllocationMethod::kCostAware};
  const double delays[] = {0.0, 0.1, 0.3};
  const double rates[] = {0.05, 0.1, 0.2};
  int closures = 0, late = 0, full = 0;
  for (int trial = 0; trial < 48; ++trial) {
    const PlanningMethod method = planners[trial % 6];
    const bool fill = (trial / 6) % 4 == 1 && method != PlanningMethod::kCbs;  // small, filled
    MapGeneratorConfig g;
    g.blocks_x = fill ? 1 : static_cast<int32_t>(pick(2, 4));
    g.blocks_y = static_cast<int32_t>(pick(1, 3));
    g.block_width = fill ? 2 : static_cast<int32_t>(pick(2, 5));
    g.block_height = static_cast<int32_t>(pick(1, 2));
    g.parking_rows = 1;
    g.stations = static_cast<int32_t>(pick(1, maxStations(g)));
    const GridMap map = generateWarehouse(g);

    Config c = smallConfig(method);
    c.map.file.clear();
    c.map.generator = g;
    c.run.seed = static_cast<uint64_t>(trial + 1);
    c.run.horizon_steps = 300;
    c.run.warmup_steps = 50;
    const int64_t most = method == PlanningMethod::kCbs ? 6 : 12;
    const auto parking = static_cast<int64_t>(map.parking().size());
    if (fill && parking <= 2 * most) {
      c.fleet.size = static_cast<int32_t>(parking);
      ++full;
    } else {
      c.fleet.size =
          static_cast<int32_t>(pick(2, std::max<int64_t>(2, std::min(most, parking / 2))));
    }
    c.allocation.method = allocators[pick(0, 2)];
    c.orders.arrival_rate = rates[pick(0, 2)];
    c.execution.delay_probability = delays[pick(0, 2)];
    const bool windowed =
        method == PlanningMethod::kWindowedCoopAStar || method == PlanningMethod::kPibt;
    c.features.rolling_replanning = windowed || pick(0, 1) == 1;
    std::vector<Cell> aisle;
    for (int32_t i = 0; i < map.cellCount(); ++i) {
      if (map.kind(map.cellAt(i)) == '.') aisle.push_back(map.cellAt(i));
    }
    if (pick(0, 1) == 1) {
      const Cell cell = aisle[static_cast<size_t>(pick(0, static_cast<int64_t>(aisle.size()) - 1))];
      const auto from = static_cast<Timestep>(pick(20, 150));
      c.faults.events = {FaultEvent{from, FaultType::kBlockCells, -1, {cell}, 0},
                         FaultEvent{from + static_cast<Timestep>(pick(10, 120)),
                                    FaultType::kUnblockCells,
                                    -1,
                                    {cell},
                                    0}};
    }
    std::string what =
        "trial " + std::to_string(trial) + ": " + name(method) +
        (c.features.rolling_replanning ? " rolling" : " to goals") + ", " +
        std::string(toString(c.allocation.method)) + ", " + std::to_string(c.fleet.size) +
        " robots on " + std::to_string(parking) + " parking cells, rate " +
        std::to_string(c.orders.arrival_rate) + ", delay " +
        std::to_string(c.execution.delay_probability) + ", map " + std::to_string(g.blocks_x) +
        "x" + std::to_string(g.blocks_y) + " blocks of " + std::to_string(g.block_width) + "x" +
        std::to_string(g.block_height) + ", " + std::to_string(g.stations) + " stations";
    std::optional<Simulation> sim;
    try {
      sim.emplace(c);
    } catch (const std::runtime_error&) {
      c.faults.events.clear();  // that cell would cut the aisles apart
      sim.emplace(c);
    }
    if (!c.faults.events.empty()) {
      what += ", closure of [" + std::to_string(c.faults.events[0].cells[0].row) + ", " +
              std::to_string(c.faults.events[0].cells[0].col) + "]";
      ++closures;
    }
    if (c.execution.delay_probability > 0.0) ++late;
    const RunSummary s = sim->run().summary;
    EXPECT_EQ(s.violations, 0) << what;
    EXPECT_FALSE(s.aborted) << what;
  }
  EXPECT_GT(closures, 5);
  EXPECT_GT(late, 15);
  EXPECT_GT(full, 3);
}

}  // namespace
}  // namespace fleet_core

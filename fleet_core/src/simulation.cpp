#include "fleet_core/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <deque>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "fleet_core/common/rng.hpp"
#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/faults/fault_injector.hpp"
#include "fleet_core/fleet_manager.hpp"
#include "fleet_core/metrics/metrics_collector.hpp"
#include "fleet_core/tasks/order_generator.hpp"
#include "fleet_core/tasks/parking.hpp"
#include "fleet_core/world/map_generator.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {
namespace {

constexpr double kDensityGuideline = 0.10;  // robots per free cell, from the project plan

GridMap buildMap(const MapConfig& map) {
  if (map.generator) return generateWarehouse(*map.generator);
  return GridMap::fromFile(map.file.string());
}

[[noreturn]] void unsupported(const std::string& what) {
  throw std::runtime_error("scenario: " + what);
}

std::string cellText(const Cell& c) {
  return "[" + std::to_string(c.row) + ", " + std::to_string(c.col) + "]";
}

// With `closed` shut, the open aisle cells form one connected region and every station and
// parking cell touches it, so every goal stays reachable.
bool aislesHold(const GridMap& map, const std::set<Cell>& closed) {
  const auto open = [&](const Cell& c) {
    return map.inBounds(c) && map.kind(c) == '.' && closed.count(c) == 0;
  };
  std::vector<Cell> cells;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    if (open(map.cellAt(i))) cells.push_back(map.cellAt(i));
  }
  if (cells.empty()) return false;
  std::set<Cell> seen{cells.front()};
  std::deque<Cell> queue{cells.front()};
  constexpr int32_t kDr[4] = {-1, 1, 0, 0};
  constexpr int32_t kDc[4] = {0, 0, -1, 1};
  while (!queue.empty()) {
    const Cell c = queue.front();
    queue.pop_front();
    for (int dir = 0; dir < 4; ++dir) {
      const Cell next{c.row + kDr[dir], c.col + kDc[dir]};
      if (open(next) && seen.insert(next).second) queue.push_back(next);
    }
  }
  if (seen.size() != cells.size()) return false;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    const Cell c = map.cellAt(i);
    if (!map.isEndpoint(c)) continue;
    bool touches = false;
    for (int dir = 0; dir < 4; ++dir) {
      touches = touches || open(Cell{c.row + kDr[dir], c.col + kDc[dir]});
    }
    if (!touches) return false;
  }
  return true;
}

// Closures (faults.events block_cells and unblock_cells) may close aisle cells only, and after
// every closure the open aisles must still reach every station and parking cell.
std::optional<std::string> closureGap(const GridMap& map, const std::vector<FaultEvent>& events) {
  std::vector<size_t> order(events.size());
  for (size_t i = 0; i < order.size(); ++i) order[i] = i;
  std::stable_sort(order.begin(), order.end(),
                   [&](size_t a, size_t b) { return events[a].at < events[b].at; });
  std::set<Cell> closed;
  for (const size_t i : order) {
    const FaultEvent& e = events[i];
    const bool blocks = e.type == FaultType::kBlockCells;
    if (!blocks && e.type != FaultType::kUnblockCells) continue;
    const std::string what = "faults.events[" + std::to_string(i) + "] (" +
                             std::string(toString(e.type)) + " at step " + std::to_string(e.at) +
                             ")";
    for (const Cell& c : e.cells) {
      if (!map.inBounds(c) || map.kind(c) != '.') {
        return what + ": cell " + cellText(c) + " is not an aisle cell ('.')";
      }
      if (blocks) {
        closed.insert(c);
      } else {
        closed.erase(c);
      }
    }
    if (blocks && !aislesHold(map, closed)) {
      return what + " would cut the open aisles apart or shut a station or parking cell off";
    }
  }
  return std::nullopt;
}

// Robots start at their home parking cells. Without enough parking (only allowed with
// features.parking off) they start on distinct random free cells that are not stations.
std::vector<Cell> placeFleet(const Config& config, const GridMap& map) {
  const int32_t fleet = config.fleet.size;
  if (map.parking().size() >= static_cast<size_t>(fleet)) {
    return ParkingManager(map, fleet).homes();
  }
  if (config.features.parking) {
    unsupported("features.parking needs one parking cell per robot; the map has " +
                std::to_string(map.parking().size()) + " for " + std::to_string(fleet) + " robots");
  }
  std::vector<Cell> candidates;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    const Cell c = map.cellAt(i);
    if (map.kind(c) == '.' || map.kind(c) == 'R') candidates.push_back(c);
  }
  if (candidates.size() < static_cast<size_t>(fleet)) {
    unsupported("not enough free non-station cells to place " + std::to_string(fleet) + " robots");
  }
  Rng rng(deriveSeed(config.run.seed, rng_stream::kSpawn));
  for (size_t k = 0; k < static_cast<size_t>(fleet); ++k) {  // partial Fisher-Yates
    const auto pick = static_cast<size_t>(
        rng.uniformInt(static_cast<int64_t>(k), static_cast<int64_t>(candidates.size()) - 1));
    std::swap(candidates[k], candidates[pick]);
  }
  candidates.resize(static_cast<size_t>(fleet));
  return candidates;
}

}  // namespace

Simulation::Simulation(const Config& config) : config_(config), map_(buildMap(config.map)) {
  validateConfig(config_);

  // What this stage can run, and what the planner needs. A gap that a later stage fills names
  // that stage.
  makeAllocator(config_);  // learned: rejects a bad model file now, naming the field
  if (const auto gap = planningRequirementGap(config_)) unsupported(*gap);
  if (config_.faults.packet_loss > 0.0) unsupported("faults.packet_loss arrives in Stage 6");
  for (const FaultEvent& e : config_.faults.events) {
    const bool cells = e.type == FaultType::kBlockCells || e.type == FaultType::kUnblockCells;
    if (!cells) {
      unsupported(std::string("faults.events type ") + std::string(toString(e.type)) +
                  " arrives in Stage 6");
    }
  }

  if (map_.pickups().empty() || map_.dropoffs().empty()) {
    unsupported("the map needs at least one pickup 'P' and one dropoff 'D' station");
  }
  if (!isConnected(map_)) unsupported("the map's free cells are not all connected");
  if (const auto gap = closureGap(map_, config_.faults.events)) unsupported(*gap);
  spawn_ = placeFleet(config_, map_);

  const auto bridges = findBridges(map_);
  if (!bridges.empty()) {
    notes_.push_back("map has " + std::to_string(bridges.size()) +
                     " bridge edges (dead ends or single corridors); PIBT's guarantee needs none");
  }
  const double density =
      static_cast<double>(config_.fleet.size) / static_cast<double>(map_.freeCellCount());
  if (density > kDensityGuideline) {
    notes_.push_back("fleet density " + std::to_string(static_cast<int>(density * 100.0)) +
                     "% is above the 10% guideline");
  }
  std::vector<std::string> inert;
  const FeatureToggles& f = config_.features;
  const bool coordinated = isCoordinated(config_.planning.method);
  if (f.congestion_cost && config_.allocation.method != AllocationMethod::kCostAware) {
    inert.push_back("congestion_cost (cost_aware only)");
  }
  if (!coordinated) {
    const std::string only = " (coordinated planners only)";
    if (f.rolling_replanning) inert.push_back("rolling_replanning" + only);
    if (f.dependency_graph) inert.push_back("dependency_graph" + only);
    if (f.intersection_reservations) inert.push_back("intersection_reservations" + only);
    if (f.deadlock_recovery) inert.push_back("deadlock_recovery" + only);
  } else if (f.dependency_graph && !f.rolling_replanning) {
    inert.push_back("dependency_graph (rolling replanning only)");
  }
  if (f.starvation_guard && config_.allocation.method == AllocationMethod::kNearest) {
    inert.push_back("starvation_guard (matching allocators only; nearest serves the oldest first)");
  }
  if (!inert.empty()) {
    std::string text = "toggles on but inactive here:";
    for (size_t i = 0; i < inert.size(); ++i) text += (i == 0 ? " " : ", ") + inert[i];
    notes_.push_back(text);
  }
}

RunResult Simulation::run(const std::optional<std::filesystem::path>& out_dir) const {
  const auto wall_start = std::chrono::steady_clock::now();
  const uint64_t seed = config_.run.seed;

  GridExecutor robots(map_, spawn_, config_.orders.service_steps,
                      config_.execution.delay_probability, deriveSeed(seed, rng_stream::kDelays));
  FleetManager fleet(config_, WorldModel(map_, spawn_));
  OrderGenerator orders(map_, config_.orders.arrival_rate, deriveSeed(seed, rng_stream::kOrders));
  InvariantChecker checker(isCoordinated(config_.planning.method));
  FaultInjector faults(config_.faults.events);
  ProgressMonitor progress(config_.fleet.size);
  RunStats stats(config_.fleet.size, config_.run.warmup_steps);
  AssignmentLog assignments;
  std::optional<MetricsCollector> writer;
  if (out_dir) {
    writer.emplace(*out_dir);
    writer->writeResolvedConfig(config_);
    writer->writeMap(map_);
  }

  RunResult result;
  int64_t issued = 0;
  int64_t violation_count = 0;
  bool aborted = false;
  Timestep steps_run = 0;
  for (Timestep t = 0; t < config_.run.horizon_steps; ++t) {
    TickInput input;
    input.now = t;
    input.new_orders = orders.step(t);
    issued += static_cast<int64_t>(input.new_orders.size());
    stats.recordOrders(input.new_orders);
    // Closures reach the robots and, as observations, the coordinator. Both close a closure's
    // cells together once no robot stands on any of them.
    for (const FaultEvent& e : faults.due(t)) {
      CellObservation seen{t, {}, {}};
      if (e.type == FaultType::kBlockCells) {
        robots.block(e.cells);
        seen.blocked = e.cells;
      } else if (e.type == FaultType::kUnblockCells) {
        robots.unblock(e.cells);
        seen.cleared = e.cells;
      }
      input.observations.push_back(std::move(seen));
    }
    input.reports = robots.collectReports(t);

    const TickOutput output = fleet.tick(input);
    robots.dispatch(t, output.commands);
    robots.step(t);
    assignments.record(output, robots.lastStep());

    const TruthSnapshot truth = robots.snapshot();
    std::vector<Violation> found = checker.check(robots.map(), robots.lastStep(), truth,
                                                 fleet.tasks(), issued, output.commands);
    const ProgressStep& seen = progress.observe(robots.lastStep(), truth);
    stats.recordStep(robots.lastStep(), truth, output.stats);
    if (writer) {
      writer->recordStep(robots.lastStep(), truth, output.stats,
                         static_cast<int32_t>(input.new_orders.size()), seen);
    }
    steps_run = t + 1;
    if (!found.empty()) {
      violation_count += static_cast<int64_t>(found.size());
      for (Violation& v : found) {
        if (result.violations.size() < kMaxStoredViolations)
          result.violations.push_back(std::move(v));
      }
      if (config_.verification.on_violation == ViolationMode::kAbort) {
        aborted = true;
        break;
      }
    }
  }

  const double wall =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  result.summary = stats.summarize(config_, steps_run, wall, violation_count, aborted);
  const ProgressSummary p = progress.summarize(steps_run);
  result.summary.first_deadlock_step = p.first_deadlock_step;
  result.summary.deadlocks = p.deadlocks;
  result.summary.deadlocked_robots = p.deadlocked_robots;
  result.summary.blocked_behind = p.blocked_behind;
  result.summary.stalled_robots = p.stalled_robots;
  result.summary.stalled_robot_steps = p.stalled_robot_steps;
  result.summary.frozen_at = p.frozen_at;
  result.summary.last_delivery_step = p.last_delivery_step;
  result.deadlocks = progress.deadlocks();
  result.assignments = assignments.rows();
  if (writer) {
    writer->writeSummary(result.summary);
    writer->writeViolations(result.violations);
    writer->writeDeadlocks(result.deadlocks);
    writer->writeAssignments(result.assignments);
  }
  return result;
}

}  // namespace fleet_core

#include "fleet_core/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include "fleet_core/common/rng.hpp"
#include "fleet_core/execution/grid_executor.hpp"
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

  // What this stage can run. Each gap names the stage that fills it.
  makeAllocator(config_.allocation, config_.features);  // throws if not implemented
  makePlanner(config_.planning);                        // throws if not implemented
  if (config_.faults.packet_loss > 0.0) unsupported("faults.packet_loss arrives in Stage 6");
  for (const FaultEvent& e : config_.faults.events) {
    const bool cells = e.type == FaultType::kBlockCells || e.type == FaultType::kUnblockCells;
    unsupported(std::string("faults.events type ") + std::string(toString(e.type)) +
                (cells ? " arrives in Stage 5" : " arrives in Stage 6"));
  }

  if (map_.pickups().empty() || map_.dropoffs().empty()) {
    unsupported("the map needs at least one pickup 'P' and one dropoff 'D' station");
  }
  if (!isConnected(map_)) unsupported("the map's free cells are not all connected");
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
  if (f.congestion_cost && config_.allocation.method != AllocationMethod::kCostAware) {
    inert.push_back("congestion_cost (cost_aware only)");
  }
  if (f.rolling_replanning) inert.push_back("rolling_replanning (Stage 4)");
  if (f.dependency_graph) inert.push_back("dependency_graph (Stage 5)");
  if (f.intersection_reservations) inert.push_back("intersection_reservations (Stage 5)");
  if (f.deadlock_recovery) inert.push_back("deadlock_recovery (Stage 5)");
  if (f.starvation_guard) inert.push_back("starvation_guard (Stage 5)");
  if (!inert.empty()) {
    std::string text = "toggles on but not active yet:";
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
  InvariantChecker checker(config_.planning.method != PlanningMethod::kIndependentAStar);
  RunStats stats(config_.fleet.size, config_.run.warmup_steps);
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
    input.reports = robots.collectReports(t);

    const TickOutput output = fleet.tick(input);
    robots.dispatch(t, output.commands);
    robots.step(t);

    const TruthSnapshot truth = robots.snapshot();
    std::vector<Violation> found =
        checker.check(robots.map(), robots.lastStep(), truth, fleet.tasks(), issued);
    stats.recordStep(robots.lastStep(), truth, output.stats);
    if (writer) {
      writer->recordStep(robots.lastStep(), truth, output.stats,
                         static_cast<int32_t>(input.new_orders.size()));
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
  if (writer) {
    writer->writeSummary(result.summary);
    writer->writeViolations(result.violations);
  }
  return result;
}

}  // namespace fleet_core

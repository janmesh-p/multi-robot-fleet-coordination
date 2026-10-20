#include "fleet_core/fuzz.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "fleet_core/common/rng.hpp"
#include "fleet_core/simulation.hpp"
#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

template <typename T>
T pick(Rng& rng, const std::vector<T>& options) {
  return options[static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(options.size()) - 1))];
}

Timestep between(Rng& rng, Timestep lo, Timestep hi) {
  return static_cast<Timestep>(rng.uniformInt(lo, hi));
}

// A straight run of one to three aisle cells.
std::vector<Cell> aisleSegment(Rng& rng, const GridMap& map) {
  std::vector<Cell> aisle;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    if (map.kind(map.cellAt(i)) == '.') aisle.push_back(map.cellAt(i));
  }
  if (aisle.empty()) return {};
  const Cell start =
      aisle[static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(aisle.size()) - 1))];
  const bool across = rng.bernoulli(0.5);
  std::vector<Cell> cells{start};
  for (int64_t k = rng.uniformInt(0, 2); k > 0; --k) {
    const Cell next = across ? Cell{cells.back().row, cells.back().col + 1}
                             : Cell{cells.back().row + 1, cells.back().col};
    if (!map.inBounds(next) || map.kind(next) != '.') break;
    cells.push_back(next);
  }
  return cells;
}

Config draw(const Config& base, uint64_t seed, int32_t max_events) {
  Rng rng(deriveSeed(seed, "fuzz"));
  Config c = base;
  c.run.name = "fuzz_" + std::to_string(seed);
  c.run.seed = seed;
  c.run.horizon_steps = between(rng, 300, 800);
  c.run.warmup_steps = 100;
  c.verification.on_violation = ViolationMode::kRecord;

  MapGeneratorConfig layout;
  layout.blocks_x = between(rng, 2, 5);
  layout.blocks_y = between(rng, 1, 3);
  layout.block_width = between(rng, 2, 6);
  layout.block_height = between(rng, 1, 2);
  layout.parking_rows = between(rng, 1, 2);
  layout.stations = between(rng, 1, std::min(4, maxStations(layout)));
  c.map.file.clear();
  c.map.generator = layout;
  const GridMap map = generateWarehouse(layout);

  const auto room = std::max<int64_t>(1, map.freeCellCount() / 6);
  const auto most = std::min<int64_t>({static_cast<int64_t>(map.parking().size()), room, 20});
  c.fleet.size = static_cast<int32_t>(rng.uniformInt(1, std::max<int64_t>(most, 1)));

  std::vector<PlanningMethod> planners = {
      PlanningMethod::kPibt,        PlanningMethod::kPibt, PlanningMethod::kWindowedCoopAStar,
      PlanningMethod::kPrioritized, PlanningMethod::kEcbs, PlanningMethod::kIndependentAStar};
  if (c.fleet.size <= 6) planners.push_back(PlanningMethod::kCbs);
  c.planning.method = pick(rng, planners);
  const bool windowed = c.planning.method == PlanningMethod::kPibt ||
                        c.planning.method == PlanningMethod::kWindowedCoopAStar;
  c.features.rolling_replanning = windowed || rng.bernoulli(0.5);
  c.allocation.method = pick(
      rng, std::vector<AllocationMethod>{AllocationMethod::kNearest, AllocationMethod::kHungarian,
                                         AllocationMethod::kCostAware});
  c.orders.arrival_rate = pick(rng, std::vector<double>{0.02, 0.05, 0.1, 0.2});
  c.orders.service_steps = between(rng, 0, 4);
  c.execution.delay_probability = pick(rng, std::vector<double>{0.0, 0.0, 0.05, 0.1, 0.2, 0.3});
  c.faults.packet_loss = pick(rng, std::vector<double>{0.0, 0.0, 0.05, 0.1, 0.2, 0.3});
  c.faults.heartbeat_timeout_steps = between(rng, 2, 8);
  c.faults.rescue_steps = pick(rng, std::vector<Timestep>{0, 25, 120, 400});

  c.faults.events.clear();
  for (int64_t k = rng.uniformInt(0, max_events); k > 0; --k) {
    FaultEvent e;
    e.at = between(rng, 0, c.run.horizon_steps - 1);
    const int64_t kind = rng.uniformInt(0, 3);
    if (kind == 0) {
      e.type = FaultType::kRobotDeath;
      e.robot = static_cast<RobotId>(rng.uniformInt(0, c.fleet.size - 1));
    } else if (kind == 1) {
      e.type = FaultType::kCommOutage;
      e.robot = static_cast<RobotId>(rng.uniformInt(0, c.fleet.size - 1));
      e.duration = between(rng, 1, 200);
    } else {
      e.type = FaultType::kBlockCells;
      e.cells = aisleSegment(rng, map);
      if (e.cells.empty()) continue;
      if (kind == 3) {  // reopened later
        FaultEvent open{between(rng, e.at, c.run.horizon_steps - 1), FaultType::kUnblockCells, -1,
                        e.cells, 0};
        c.faults.events.push_back(open);
      }
    }
    c.faults.events.push_back(e);
  }
  return c;
}

bool rejected(const Config& c) {
  try {
    const Simulation sim(c);
    return false;
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()).rfind("scenario:", 0) == 0) return true;
    throw;
  }
}

}  // namespace

std::optional<Config> randomScenario(const Config& base, uint64_t seed, int32_t max_events) {
  Config c = draw(base, seed, max_events);
  validateConfig(c);
  if (!rejected(c)) return c;
  auto& events = c.faults.events;
  events.erase(std::remove_if(events.begin(), events.end(),
                              [](const FaultEvent& e) {
                                return e.type == FaultType::kBlockCells ||
                                       e.type == FaultType::kUnblockCells;
                              }),
               events.end());
  if (!rejected(c)) return c;
  return std::nullopt;
}

FuzzResult checkScenario(const Config& config) {
  FuzzResult r;
  try {
    const RunResult first = Simulation(config).run(std::nullopt);
    r.summary = first.summary;
    if (!first.violations.empty()) {
      const Violation& v = first.violations.front();
      r.passed = false;
      r.failed_at = v.t;
      r.failure =
          "step " + std::to_string(v.t) + ": " + std::string(toString(v.kind)) + ", " + v.detail;
      return r;
    }
    const RunResult second = Simulation(config).run(std::nullopt);
    if (second.summary.log_hash != first.summary.log_hash) {
      r.passed = false;
      r.failure = "two runs of the same config differ";
    }
  } catch (const std::exception& e) {
    r.passed = false;
    r.failure = std::string("exception: ") + e.what();
  }
  return r;
}

Config shrinkScenario(const Config& failing) {
  const auto fails = [](const Config& c) { return !checkScenario(c).passed; };
  Config best = failing;
  const FuzzResult first = checkScenario(best);
  if (first.failed_at >= 0) {
    Config c = best;
    c.run.horizon_steps = first.failed_at + 1;
    c.run.warmup_steps = std::min(c.run.warmup_steps, c.run.horizon_steps - 1);
    if (fails(c)) best = c;
  }
  for (size_t i = 0; i < best.faults.events.size();) {
    Config c = best;
    c.faults.events.erase(c.faults.events.begin() + static_cast<std::ptrdiff_t>(i));
    if (!rejected(c) && fails(c)) {
      best = c;
    } else {
      ++i;
    }
  }
  if (best.faults.packet_loss > 0.0) {
    Config c = best;
    c.faults.packet_loss = 0.0;
    if (fails(c)) best = c;
  }
  if (best.execution.delay_probability > 0.0) {
    Config c = best;
    c.execution.delay_probability = 0.0;
    if (fails(c)) best = c;
  }
  return best;
}

}  // namespace fleet_core

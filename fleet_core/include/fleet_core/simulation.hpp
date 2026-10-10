#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/metrics/run_stats.hpp"
#include "fleet_core/verification/invariant_checker.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

struct RunResult {
  RunSummary summary{};
  std::vector<Violation> violations{};  // the first kMaxStoredViolations
};

// One grid-simulator run, wired in the order a real deployment sees it each step:
//   new orders -> robot reports -> FleetManager::tick -> commands -> robots move
//   -> invariant checks -> metrics
// The coordinator only sees reports; ground truth stays in the executor.
class Simulation {
 public:
  static constexpr size_t kMaxStoredViolations = 1000;

  // Builds the map, validates the scenario against it, and places the fleet.
  // Throws std::runtime_error for scenarios this stage cannot run, naming the stage that adds
  // the missing piece.
  explicit Simulation(const Config& config);

  // Runs to the horizon, or to the first violation when verification.on_violation is abort.
  // Each call starts from the initial state, so two calls give the same result.
  // Writes config.yaml, map.txt, steps.csv, positions.csv, summary.csv, violations.csv to
  // out_dir when given.
  RunResult run(const std::optional<std::filesystem::path>& out_dir = std::nullopt) const;

  const Config& config() const { return config_; }
  const GridMap& map() const { return map_; }
  const std::vector<Cell>& spawn() const { return spawn_; }

  // Things worth knowing that are not errors: density, map bridges, inert feature toggles.
  const std::vector<std::string>& notes() const { return notes_; }

 private:
  Config config_;
  GridMap map_;
  std::vector<Cell> spawn_{};
  std::vector<std::string> notes_{};
};

}  // namespace fleet_core

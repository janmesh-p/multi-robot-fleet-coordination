#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/metrics/run_stats.hpp"
#include "fleet_core/verification/invariant_checker.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// Writes a run's output directory:
//   config.yaml     resolved config; loading it reproduces the run
//   map.txt         the map the run used
//   steps.csv       one row per step: orders, deliveries, what the fleet did, planning time
//   positions.csv   one row per robot per step: cell, status, outcome (replay, heatmaps)
//   summary.csv     one row: RunSummary
//   violations.csv  invariant violations, if any
// Owned by the runner, not by FleetManager, because it records ground truth.
class MetricsCollector {
 public:
  // Creates out_dir and opens the per-step files. Throws std::runtime_error if it cannot write.
  explicit MetricsCollector(const std::filesystem::path& out_dir);

  void writeResolvedConfig(const Config& config);
  void writeMap(const GridMap& map);
  void recordStep(const StepTruth& step, const TruthSnapshot& after, const TickStats& tick,
                  int32_t orders_created);
  void writeSummary(const RunSummary& summary);
  void writeViolations(const std::vector<Violation>& violations);

  const std::filesystem::path& outDir() const { return out_dir_; }

 private:
  std::ofstream open(const char* name) const;

  std::filesystem::path out_dir_;
  std::ofstream steps_;
  std::ofstream positions_;
};

}  // namespace fleet_core

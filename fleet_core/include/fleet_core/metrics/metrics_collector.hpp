#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"

namespace fleet_core {

// One row of steps.csv. Columns grow as stages land.
struct StepMetrics {
  Timestep t{0};
  int32_t tasks_completed{0};
  int32_t conflicts{0};
  int32_t robots_waiting{0};
  int32_t robots_alive{0};
  double planning_ms{0.0};
};

// One row of summary.csv, read by the benchmark scripts (Stage 7).
struct RunSummary {
  std::string allocator{};
  std::string planner{};
  int32_t fleet_size{0};
  Timestep horizon{0};
  double orders_per_hour{0.0};
  double mean_completion_steps{0.0};
  double p95_completion_steps{0.0};
  double utilization{0.0};
  int64_t conflicts{0};
  int64_t congestion_steps{0};
  double planning_p50_ms{0.0};
  double planning_p95_ms{0.0};
};

// Writes a run's output directory: config.yaml (resolved config), steps.csv, summary.csv.
// Owned by the runner, not by FleetManager, because it also records ground truth.
class MetricsCollector {
 public:
  // Creates out_dir and opens steps.csv. Throws std::runtime_error if it cannot write.
  explicit MetricsCollector(const std::filesystem::path& out_dir);

  void writeResolvedConfig(const Config& config);
  void recordStep(const StepMetrics& metrics);
  void writeSummary(const RunSummary& summary);

  const std::filesystem::path& outDir() const { return out_dir_; }

 private:
  std::filesystem::path out_dir_;
  std::ofstream steps_;
};

}  // namespace fleet_core

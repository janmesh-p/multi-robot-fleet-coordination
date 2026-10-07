#pragma once

#include <fstream>
#include <string>

#include "fleet_core/types.hpp"

namespace fleet_core {

struct StepMetrics {
  Timestep t{0};
  int32_t tasks_completed{0};
  int32_t conflicts{0};
  int32_t robots_waiting{0};
  int32_t robots_alive{0};
  double replan_latency_ms{0.0};
};

struct RunSummary {
  std::string allocator{};
  std::string planner{};
  int32_t fleet_size{0};
  Timestep horizon{0};
  double orders_per_hour{0.0};
  double mean_completion_steps{0.0};
  double utilization{0.0};
  int64_t total_conflicts{0};
  int64_t congestion_steps{0};
  double replan_latency_p50_ms{0.0};
  double replan_latency_p95_ms{0.0};
};

// Writes per-step CSV and a one-line summary CSV for benchmarks/scripts.
class MetricsLogger {
 public:
  explicit MetricsLogger(const std::string& out_dir);
  void logStep(const StepMetrics& m);
  void writeSummary(const RunSummary& s);

 private:
  std::ofstream steps_;
  std::string out_dir_;
};

}  // namespace fleet_core

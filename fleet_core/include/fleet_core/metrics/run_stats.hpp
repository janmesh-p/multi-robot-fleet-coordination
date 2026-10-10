#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/execution/grid_executor.hpp"

namespace fleet_core {

// One run's results. Steady-state numbers count only steps at or after the warmup.
struct RunSummary {
  std::string run_name{};
  std::string allocator{};
  std::string planner{};
  int32_t fleet_size{0};
  uint64_t seed{0};
  Timestep horizon{0};
  Timestep warmup{0};
  Timestep steps_run{0};  // less than horizon when a violation aborted the run

  int64_t orders_created{0};    // whole run
  int64_t orders_delivered{0};  // whole run

  double orders_per_hour{0.0};        // steady state
  double mean_completion_steps{0.0};  // created after warmup and delivered before the end
  double p95_completion_steps{0.0};
  int64_t completion_samples{0};
  double utilization{0.0};      // share of robot-steps spent holding a task
  int64_t conflicts{0};         // robot-steps stopped by robot safety
  int64_t congestion_steps{0};  // robot-steps stopped by safety or waiting in a plan

  int64_t planning_calls{0};    // whole run: planning latency is a compute cost, not a
  double planning_p50_ms{0.0};  // steady-state number, so warmup ticks count too
  double planning_p95_ms{0.0};
  int64_t expansions{0};  // whole run

  int64_t violations{0};
  bool aborted{false};
  uint64_t log_hash{0};  // fingerprint of the trajectory; same seed and config, same hash
  double wall_seconds{0.0};
};

// Accumulates a run in memory and fingerprints it. The fingerprint covers every robot's cell,
// status, and outcome at every step, plus every pickup and delivery; it excludes wall time.
class RunStats {
 public:
  RunStats(int32_t fleet_size, Timestep warmup);

  void recordOrders(const std::vector<Task>& orders);
  void recordStep(const StepTruth& step, const TruthSnapshot& after, const TickStats& tick);

  RunSummary summarize(const Config& config, Timestep steps_run, double wall_seconds,
                       int64_t violations, bool aborted) const;

  uint64_t hash() const { return hash_; }

 private:
  void mix(int64_t value);

  int32_t fleet_size_;
  Timestep warmup_;
  uint64_t hash_{0xCBF29CE484222325ULL};
  std::map<TaskId, Timestep> created_at_{};
  int64_t orders_created_{0};
  int64_t delivered_total_{0};
  int64_t delivered_steady_{0};
  std::vector<double> completion_steps_{};
  int64_t holding_robot_steps_{0};
  int64_t conflicts_{0};
  int64_t congestion_steps_{0};
  std::vector<double> planning_ms_{};
  int64_t expansions_{0};
};

}  // namespace fleet_core

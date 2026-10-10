#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/metrics/run_stats.hpp"

namespace fleet_core {

// Parses "5,10,25", "1-5" (inclusive), or a mix such as "1-3,7". Throws std::invalid_argument.
std::vector<int64_t> parseIntList(const std::string& text);

// Runs the base config once per fleet size and seed: fleet size outer, seed inner. Each run is
// named <run.name>_n<size>_s<seed>. With runs_dir, every run also writes its run directory
// there. Every config compared under one seed sees the same orders, so seeds pair up across
// fleet sizes.
std::vector<RunSummary> runSweep(
    const Config& base, const std::vector<int32_t>& fleet_sizes, const std::vector<uint64_t>& seeds,
    const std::optional<std::filesystem::path>& runs_dir = std::nullopt);

// sweep.csv: summary.csv's columns, one row per run.
std::string sweepCsv(const std::vector<RunSummary>& runs);

// One fleet size, summarized over its seeds. Medians skip runs where the event never happened;
// -1 when no run had it.
struct SweepGroup {
  int32_t fleet_size{0};
  int32_t runs{0};
  int32_t deadlocked_runs{0};          // at least one deadlock
  int32_t frozen_runs{0};              // frozen before the end
  double median_first_deadlock{-1.0};  // step
  double median_frozen_at{-1.0};       // step
  double mean_delivered{0.0};
  double mean_orders_per_hour{0.0};  // steady state
  double mean_conflicts{0.0};
};

std::vector<SweepGroup> groupByFleetSize(const std::vector<RunSummary>& runs);

// aggregate.csv: one row per SweepGroup.
std::string aggregateCsv(const std::vector<SweepGroup>& groups);

// Compares runs with a reference sweep.csv on every column that does not depend on the machine
// (timing columns are skipped). Returns one line per difference; empty means identical.
std::vector<std::string> compareWithReference(const std::vector<RunSummary>& runs,
                                              const std::string& reference_csv);

}  // namespace fleet_core

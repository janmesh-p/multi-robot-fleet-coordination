#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/metrics/metrics_collector.hpp"
#include "fleet_core/simulation.hpp"
#include "fleet_core/sweep.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

Config shortConfig() {
  Config c = loadConfig(kRepo / "benchmarks" / "configs" / "default.yaml");
  c.run.horizon_steps = 200;
  c.run.warmup_steps = 50;
  return c;
}

TEST(Sweep, ParsesListsAndRanges) {
  EXPECT_EQ(parseIntList("5,10,25"), (std::vector<int64_t>{5, 10, 25}));
  EXPECT_EQ(parseIntList("1-5"), (std::vector<int64_t>{1, 2, 3, 4, 5}));
  EXPECT_EQ(parseIntList("1-3,7"), (std::vector<int64_t>{1, 2, 3, 7}));
  EXPECT_EQ(parseIntList("4"), (std::vector<int64_t>{4}));
  for (const char* bad : {"", "a", "5-1", "1,,2", "2-", "1.5", "3x"}) {
    EXPECT_THROW(parseIntList(bad), std::invalid_argument) << bad;
  }
}

TEST(Sweep, RunsEverySizeAndSeedLikeASingleRun) {
  const Config base = shortConfig();
  const std::vector<RunSummary> runs = runSweep(base, {2, 3}, {1, 2});
  ASSERT_EQ(runs.size(), 4U);
  EXPECT_EQ(runs[0].run_name, "default_n2_s1");
  EXPECT_EQ(runs[1].run_name, "default_n2_s2");
  EXPECT_EQ(runs[2].run_name, "default_n3_s1");
  EXPECT_EQ(runs[3].fleet_size, 3);
  EXPECT_EQ(runs[3].seed, 2U);

  Config single = base;
  single.fleet.size = 3;
  single.run.seed = 2;
  EXPECT_EQ(Simulation(single).run().summary.log_hash, runs[3].log_hash);
  EXPECT_EQ(runs[0].orders_created, runs[2].orders_created);  // one seed, one order stream
}

TEST(Sweep, GroupsByFleetSize) {
  std::vector<RunSummary> runs(4);
  runs[0].fleet_size = runs[1].fleet_size = runs[2].fleet_size = 5;
  runs[3].fleet_size = 10;
  runs[0].first_deadlock_step = 30;
  runs[1].first_deadlock_step = 90;  // runs[2] never deadlocks
  runs[0].frozen_at = 100;
  runs[0].orders_delivered = 3;
  runs[1].orders_delivered = 6;
  runs[2].orders_delivered = 9;
  runs[3].orders_per_hour = 12.5;
  const std::vector<SweepGroup> groups = groupByFleetSize(runs);
  ASSERT_EQ(groups.size(), 2U);
  EXPECT_EQ(groups[0].fleet_size, 5);
  EXPECT_EQ(groups[0].runs, 3);
  EXPECT_EQ(groups[0].deadlocked_runs, 2);
  EXPECT_DOUBLE_EQ(groups[0].median_first_deadlock, 60.0);
  EXPECT_EQ(groups[0].frozen_runs, 1);
  EXPECT_DOUBLE_EQ(groups[0].median_frozen_at, 100.0);
  EXPECT_DOUBLE_EQ(groups[0].mean_delivered, 6.0);
  EXPECT_EQ(groups[1].deadlocked_runs, 0);
  EXPECT_DOUBLE_EQ(groups[1].median_first_deadlock, -1.0);
  EXPECT_DOUBLE_EQ(groups[1].mean_orders_per_hour, 12.5);
  EXPECT_EQ(aggregateCsv(groups).rfind("fleet_size,runs,", 0), 0U);
}

TEST(Sweep, ReferenceCheckIgnoresTimingButNothingElse) {
  const std::vector<RunSummary> runs = runSweep(shortConfig(), {2}, {1, 2});
  const std::string reference = sweepCsv(runs);
  EXPECT_EQ(reference.rfind(summaryCsvHeader() + "\n", 0), 0U);
  EXPECT_TRUE(compareWithReference(runs, reference).empty());

  std::vector<RunSummary> slower = runs;
  slower[0].wall_seconds += 5.0;
  slower[1].planning_p95_ms += 1.0;
  EXPECT_TRUE(compareWithReference(slower, reference).empty());

  std::vector<RunSummary> changed = runs;
  changed[1].orders_delivered += 1;
  const std::vector<std::string> diffs = compareWithReference(changed, reference);
  ASSERT_EQ(diffs.size(), 1U);
  EXPECT_NE(diffs[0].find("default_n2_s2: orders_delivered"), std::string::npos) << diffs[0];

  const std::vector<RunSummary> fewer(runs.begin(), runs.begin() + 1);
  const std::vector<std::string> missing = compareWithReference(fewer, reference);
  ASSERT_EQ(missing.size(), 1U);
  EXPECT_NE(missing[0].find("was not run"), std::string::npos);
  EXPECT_FALSE(compareWithReference(runs, "").empty());
}

// The frozen baseline: its inert toggles are off, and one run still matches the reference row
// (CI checks the whole sweep with run_sweep --check).
TEST(Sweep, BaselineConfigStillMatchesItsReference) {
  const Config base = loadConfig(kRepo / "benchmarks" / "configs" / "baseline.yaml");
  EXPECT_EQ(base.planning.method, PlanningMethod::kIndependentAStar);
  EXPECT_EQ(base.allocation.method, AllocationMethod::kNearest);
  EXPECT_FALSE(base.features.dependency_graph || base.features.intersection_reservations ||
               base.features.deadlock_recovery || base.features.starvation_guard ||
               base.features.rolling_replanning || base.features.congestion_cost);
  std::ifstream in(kRepo / "benchmarks" / "reference" / "baseline_sweep.csv");
  ASSERT_TRUE(in.good());
  std::string header, line, row;
  std::getline(in, header);
  while (std::getline(in, line)) {
    if (line.rfind("baseline_n10_s1,", 0) == 0) row = line;
  }
  ASSERT_FALSE(row.empty());
  const std::vector<RunSummary> runs = runSweep(base, {10}, {1});
  const std::vector<std::string> diffs = compareWithReference(runs, header + "\n" + row + "\n");
  EXPECT_TRUE(diffs.empty()) << diffs.front();
}

}  // namespace
}  // namespace fleet_core

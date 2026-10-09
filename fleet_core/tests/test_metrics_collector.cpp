#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "fleet_core/metrics/metrics_collector.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();

std::string readFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

class MetricsCollectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("fleet_core_metrics_" + std::to_string(::getpid()));
    std::filesystem::remove_all(dir_);
  }
  void TearDown() override { std::filesystem::remove_all(dir_); }

  std::filesystem::path dir_;
};

TEST_F(MetricsCollectorTest, WritesRunDirectory) {
  Config config;
  config.map.file = (kRepo / "maps" / "warehouse_small.txt").lexically_normal();
  config.run.name = "metrics_test";
  {
    MetricsCollector metrics(dir_ / "run");
    metrics.writeResolvedConfig(config);
    metrics.recordStep(StepMetrics{3, 1, 0, 2, 10, 1.5});
    RunSummary summary;
    summary.allocator = "nearest";
    summary.planner = "cbs";
    summary.fleet_size = 10;
    metrics.writeSummary(summary);
  }
  const std::filesystem::path run = dir_ / "run";

  const std::string written_config = readFile(run / "config.yaml");
  EXPECT_EQ(written_config, dumpConfig(config));
  EXPECT_EQ(parseConfig(written_config, "/").run.name, "metrics_test");

  EXPECT_EQ(readFile(run / "steps.csv"),
            "t,tasks_completed,conflicts,robots_waiting,robots_alive,planning_ms\n"
            "3,1,0,2,10,1.5\n");

  const std::string summary = readFile(run / "summary.csv");
  EXPECT_EQ(summary.rfind("allocator,planner,fleet_size,", 0), 0U);
  EXPECT_NE(summary.find("\nnearest,cbs,10,"), std::string::npos);
}

}  // namespace
}  // namespace fleet_core

#include "fleet_core/metrics/metrics_collector.hpp"

#include <iomanip>
#include <stdexcept>

namespace fleet_core {

MetricsCollector::MetricsCollector(const std::filesystem::path& out_dir) : out_dir_(out_dir) {
  std::filesystem::create_directories(out_dir_);
  steps_.open(out_dir_ / "steps.csv");
  if (!steps_) {
    throw std::runtime_error("MetricsCollector: cannot write " + (out_dir_ / "steps.csv").string());
  }
  steps_ << std::setprecision(10)
         << "t,tasks_completed,conflicts,robots_waiting,robots_alive,planning_ms\n";
}

void MetricsCollector::writeResolvedConfig(const Config& config) {
  std::ofstream out(out_dir_ / "config.yaml");
  if (!out) {
    throw std::runtime_error("MetricsCollector: cannot write " +
                             (out_dir_ / "config.yaml").string());
  }
  out << dumpConfig(config);
}

void MetricsCollector::recordStep(const StepMetrics& m) {
  steps_ << m.t << ',' << m.tasks_completed << ',' << m.conflicts << ',' << m.robots_waiting << ','
         << m.robots_alive << ',' << m.planning_ms << '\n';
}

void MetricsCollector::writeSummary(const RunSummary& s) {
  std::ofstream out(out_dir_ / "summary.csv");
  if (!out) {
    throw std::runtime_error("MetricsCollector: cannot write " +
                             (out_dir_ / "summary.csv").string());
  }
  out << std::setprecision(10)
      << "allocator,planner,fleet_size,horizon,orders_per_hour,mean_completion_steps,"
         "p95_completion_steps,utilization,conflicts,congestion_steps,planning_p50_ms,"
         "planning_p95_ms\n"
      << s.allocator << ',' << s.planner << ',' << s.fleet_size << ',' << s.horizon << ','
      << s.orders_per_hour << ',' << s.mean_completion_steps << ',' << s.p95_completion_steps << ','
      << s.utilization << ',' << s.conflicts << ',' << s.congestion_steps << ','
      << s.planning_p50_ms << ',' << s.planning_p95_ms << '\n';
}

}  // namespace fleet_core

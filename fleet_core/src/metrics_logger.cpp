#include "fleet_core/metrics_logger.hpp"

#include <filesystem>
#include <stdexcept>

namespace fleet_core {

MetricsLogger::MetricsLogger(const std::string& out_dir) : out_dir_(out_dir) {
  std::filesystem::create_directories(out_dir_);
  steps_.open(out_dir_ + "/steps.csv");
  if (!steps_) throw std::runtime_error("MetricsLogger: cannot open " + out_dir_ + "/steps.csv");
  steps_ << "t,tasks_completed,conflicts,robots_waiting,robots_alive,replan_latency_ms\n";
}

void MetricsLogger::logStep(const StepMetrics& m) {
  steps_ << m.t << ',' << m.tasks_completed << ',' << m.conflicts << ',' << m.robots_waiting
         << ',' << m.robots_alive << ',' << m.replan_latency_ms << '\n';
}

void MetricsLogger::writeSummary(const RunSummary& s) {
  std::ofstream out(out_dir_ + "/summary.csv");
  out << "allocator,planner,fleet_size,horizon,orders_per_hour,mean_completion_steps,"
         "utilization,total_conflicts,congestion_steps,replan_p50_ms,replan_p95_ms\n";
  out << s.allocator << ',' << s.planner << ',' << s.fleet_size << ',' << s.horizon << ','
      << s.orders_per_hour << ',' << s.mean_completion_steps << ',' << s.utilization << ','
      << s.total_conflicts << ',' << s.congestion_steps << ',' << s.replan_latency_p50_ms << ','
      << s.replan_latency_p95_ms << '\n';
}

}  // namespace fleet_core

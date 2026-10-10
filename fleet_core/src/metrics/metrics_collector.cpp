#include "fleet_core/metrics/metrics_collector.hpp"

#include <array>
#include <cstddef>
#include <cstdio>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace fleet_core {
namespace {

std::string hex16(uint64_t value) {
  std::array<char, 17> buffer{};
  std::snprintf(buffer.data(), buffer.size(), "%016llx", static_cast<unsigned long long>(value));
  return buffer.data();
}

// CSV field: quoted when it holds a comma, quote, or newline.
std::string csv(const std::string& text) {
  if (text.find_first_of(",\"\n") == std::string::npos) return text;
  std::string out = "\"";
  for (const char c : text) out += c == '"' ? std::string("\"\"") : std::string(1, c);
  return out + "\"";
}

}  // namespace

MetricsCollector::MetricsCollector(const std::filesystem::path& out_dir) : out_dir_(out_dir) {
  std::filesystem::create_directories(out_dir_);
  steps_ = open("steps.csv");
  positions_ = open("positions.csv");
  steps_ << std::setprecision(10)
         << "t,orders_created,deliveries,moved,planned_wait,hold,service,idle,delayed,blocked,"
            "assignments,robots_planned,planning_ms,expansions\n";
  positions_ << "t,robot,row,col,status,outcome\n";
}

std::ofstream MetricsCollector::open(const char* name) const {
  std::ofstream out(out_dir_ / name);
  if (!out)
    throw std::runtime_error("MetricsCollector: cannot write " + (out_dir_ / name).string());
  return out;
}

void MetricsCollector::writeResolvedConfig(const Config& config) {
  open("config.yaml") << dumpConfig(config);
}

void MetricsCollector::writeMap(const GridMap& map) { open("map.txt") << map.toString(); }

void MetricsCollector::recordStep(const StepTruth& step, const TruthSnapshot& after,
                                  const TickStats& tick, int32_t orders_created) {
  int32_t moved = 0, planned_wait = 0, hold = 0, service = 0, idle = 0, delayed = 0, blocked = 0;
  for (const MotionRecord& m : step.motion) {
    switch (m.outcome) {
      case MotionOutcome::kMoved:
        ++moved;
        break;
      case MotionOutcome::kPlannedWait:
        ++planned_wait;
        break;
      case MotionOutcome::kHold:
        ++hold;
        break;
      case MotionOutcome::kService:
        ++service;
        break;
      case MotionOutcome::kIdle:
      case MotionOutcome::kDead:
        ++idle;
        break;
      case MotionOutcome::kDelayed:
        ++delayed;
        break;
      case MotionOutcome::kBlockedVertex:
      case MotionOutcome::kBlockedSwap:
      case MotionOutcome::kBlockedOccupied:
      case MotionOutcome::kBlockedCell:
      case MotionOutcome::kInvalidStep:
        ++blocked;
        break;
    }
  }
  steps_ << step.t << ',' << orders_created << ',' << step.deliveries.size() << ',' << moved << ','
         << planned_wait << ',' << hold << ',' << service << ',' << idle << ',' << delayed << ','
         << blocked << ',' << tick.assignments << ',' << tick.robots_planned << ','
         << tick.planning_ms << ',' << tick.expansions << '\n';
  for (size_t i = 0; i < step.motion.size(); ++i) {
    const MotionRecord& m = step.motion[i];
    positions_ << step.t << ',' << i << ',' << m.to.row << ',' << m.to.col << ','
               << toString(after.status[i]) << ',' << toString(m.outcome) << '\n';
  }
}

void MetricsCollector::writeSummary(const RunSummary& s) {
  std::ofstream out = open("summary.csv");
  out << std::setprecision(10)
      << "run,allocator,planner,fleet_size,seed,horizon,warmup,steps_run,orders_created,"
         "orders_delivered,orders_per_hour,mean_completion_steps,p95_completion_steps,"
         "completion_samples,utilization,conflicts,congestion_steps,planning_calls,planning_p50_ms,"
         "planning_p95_ms,expansions,violations,aborted,log_hash,wall_seconds\n"
      << csv(s.run_name) << ',' << s.allocator << ',' << s.planner << ',' << s.fleet_size << ','
      << s.seed << ',' << s.horizon << ',' << s.warmup << ',' << s.steps_run << ','
      << s.orders_created << ',' << s.orders_delivered << ',' << s.orders_per_hour << ','
      << s.mean_completion_steps << ',' << s.p95_completion_steps << ',' << s.completion_samples
      << ',' << s.utilization << ',' << s.conflicts << ',' << s.congestion_steps << ','
      << s.planning_calls << ',' << s.planning_p50_ms << ',' << s.planning_p95_ms << ','
      << s.expansions << ',' << s.violations << ',' << (s.aborted ? "true" : "false") << ','
      << hex16(s.log_hash) << ',' << s.wall_seconds << '\n';
}

void MetricsCollector::writeViolations(const std::vector<Violation>& violations) {
  std::ofstream out = open("violations.csv");
  out << "t,kind,detail\n";
  for (const Violation& v : violations) {
    out << v.t << ',' << toString(v.kind) << ',' << csv(v.detail) << '\n';
  }
}

}  // namespace fleet_core

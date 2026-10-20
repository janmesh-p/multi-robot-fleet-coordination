#include "fleet_core/metrics/metrics_collector.hpp"

#include <array>
#include <cstddef>
#include <cstdio>
#include <iomanip>
#include <sstream>
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

std::string summaryCsvHeader() {
  return "run,allocator,planner,fleet_size,seed,horizon,warmup,steps_run,orders_created,"
         "orders_delivered,orders_per_hour,mean_completion_steps,p95_completion_steps,"
         "completion_samples,utilization,conflicts,congestion_steps,planning_calls,"
         "planning_p50_ms,planning_p95_ms,expansions,violations,aborted,first_deadlock_step,"
         "deadlocks,deadlocked_robots,blocked_behind,stalled_robots,stalled_robot_steps,frozen_at,"
         "last_delivery_step,log_hash,wall_seconds";
}

std::string summaryCsvRow(const RunSummary& s) {
  std::ostringstream out;
  out << std::setprecision(10) << csv(s.run_name) << ',' << s.allocator << ',' << s.planner << ','
      << s.fleet_size << ',' << s.seed << ',' << s.horizon << ',' << s.warmup << ',' << s.steps_run
      << ',' << s.orders_created << ',' << s.orders_delivered << ',' << s.orders_per_hour << ','
      << s.mean_completion_steps << ',' << s.p95_completion_steps << ',' << s.completion_samples
      << ',' << s.utilization << ',' << s.conflicts << ',' << s.congestion_steps << ','
      << s.planning_calls << ',' << s.planning_p50_ms << ',' << s.planning_p95_ms << ','
      << s.expansions << ',' << s.violations << ',' << (s.aborted ? "true" : "false") << ','
      << s.first_deadlock_step << ',' << s.deadlocks << ',' << s.deadlocked_robots << ','
      << s.blocked_behind << ',' << s.stalled_robots << ',' << s.stalled_robot_steps << ','
      << s.frozen_at << ',' << s.last_delivery_step << ',' << hex16(s.log_hash) << ','
      << s.wall_seconds;
  return out.str();
}

MetricsCollector::MetricsCollector(const std::filesystem::path& out_dir) : out_dir_(out_dir) {
  std::filesystem::create_directories(out_dir_);
  steps_ = open("steps.csv");
  positions_ = open("positions.csv");
  steps_ << std::setprecision(10)
         << "t,orders_created,deliveries,moved,planned_wait,hold,service,idle,delayed,blocked,"
            "assignments,robots_planned,planning_ms,expansions,on_cycle,behind,stalled,yield,"
            "no_permit,deadlocks_resolved,no_command,safe_stop,declared_dead,requeued\n";
  positions_ << "t,robot,row,col,status,outcome,waits_for,on_cycle\n";
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
                                  const TickStats& tick, int32_t orders_created,
                                  const ProgressStep& progress) {
  int32_t moved = 0, planned_wait = 0, hold = 0, service = 0, idle = 0, delayed = 0, blocked = 0,
          yielded = 0, no_permit = 0, no_command = 0, safe_stop = 0;
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
      case MotionOutcome::kYield:
        ++yielded;
        break;
      case MotionOutcome::kNoPermit:
        ++no_permit;
        break;
      case MotionOutcome::kNoCommand:
        ++no_command;
        break;
      case MotionOutcome::kSafeStop:
        ++safe_stop;
        break;
    }
  }
  steps_ << step.t << ',' << orders_created << ',' << step.deliveries.size() << ',' << moved << ','
         << planned_wait << ',' << hold << ',' << service << ',' << idle << ',' << delayed << ','
         << blocked << ',' << tick.assignments << ',' << tick.robots_planned << ','
         << tick.planning_ms << ',' << tick.expansions << ',' << progress.on_cycle_count << ','
         << progress.behind_count << ',' << progress.stalled_count << ',' << yielded << ','
         << no_permit << ',' << tick.deadlocks_resolved << ',' << no_command << ',' << safe_stop
         << ',' << tick.robots_declared_dead << ',' << tick.tasks_requeued << '\n';
  for (size_t i = 0; i < step.motion.size(); ++i) {
    const MotionRecord& m = step.motion[i];
    positions_ << step.t << ',' << i << ',' << m.to.row << ',' << m.to.col << ','
               << toString(after.status[i]) << ',' << toString(m.outcome) << ','
               << progress.waits_for[i] << ',' << static_cast<int>(progress.on_cycle[i]) << '\n';
  }
}

void MetricsCollector::writeSummary(const RunSummary& s) {
  open("summary.csv") << summaryCsvHeader() << '\n' << summaryCsvRow(s) << '\n';
}

void MetricsCollector::writeViolations(const std::vector<Violation>& violations) {
  std::ofstream out = open("violations.csv");
  out << "t,kind,detail\n";
  for (const Violation& v : violations) {
    out << v.t << ',' << toString(v.kind) << ',' << csv(v.detail) << '\n';
  }
}

void MetricsCollector::writeDeadlocks(const std::vector<Deadlock>& deadlocks) {
  std::ofstream out = open("deadlocks.csv");
  out << "formed,broken,robots\n";
  for (const Deadlock& d : deadlocks) {
    out << d.formed << ',';
    if (d.broken) out << *d.broken;
    out << ',';
    for (size_t i = 0; i < d.robots.size(); ++i) out << (i == 0 ? "" : " ") << d.robots[i];
    out << '\n';
  }
}

void MetricsCollector::writeAssignments(const std::vector<AssignmentRow>& rows) {
  std::ofstream out = open("assignments.csv");
  out << assignmentCsvHeader() << '\n';
  for (const AssignmentRow& row : rows) out << assignmentCsvRow(row) << '\n';
}

}  // namespace fleet_core

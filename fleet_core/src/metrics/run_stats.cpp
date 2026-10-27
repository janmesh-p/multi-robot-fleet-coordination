#include "fleet_core/metrics/run_stats.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>

namespace fleet_core {
namespace {

// Nearest-rank percentile of an unsorted sample; 0 for an empty one.
double percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto rank = static_cast<size_t>(std::ceil(p * static_cast<double>(values.size())));
  return values[std::max<size_t>(rank, 1) - 1];
}

double mean(const std::vector<double>& values) {
  if (values.empty()) return 0.0;
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

}  // namespace

RunStats::RunStats(int32_t fleet_size, Timestep warmup)
    : fleet_size_(fleet_size), warmup_(warmup) {}

void RunStats::mix(int64_t value) {
  // FNV-1a over the eight bytes of value, low byte first: the same on every platform.
  auto v = static_cast<uint64_t>(value);
  for (int i = 0; i < 8; ++i) {
    hash_ ^= v & 0xFFU;
    hash_ *= 0x100000001B3ULL;
    v >>= 8U;
  }
}

void RunStats::recordOrders(const std::vector<Task>& orders) {
  for (const Task& task : orders) {
    created_at_[task.order.id] = task.created_at;
    open_[task.order.id] = task.created_at;
    ++orders_created_;
  }
}

void RunStats::recordAssignments(Timestep t, const std::vector<TaskId>& orders) {
  for (const TaskId id : orders) assigned_at_.emplace(id, t);
}

void RunStats::recordStep(const StepTruth& step, const TruthSnapshot& after,
                          const TickStats& tick) {
  mix(step.t);
  for (size_t i = 0; i < step.motion.size(); ++i) {
    const MotionRecord& m = step.motion[i];
    mix(m.to.row);
    mix(m.to.col);
    mix(static_cast<int64_t>(after.status[i]));
    mix(static_cast<int64_t>(m.outcome));
  }
  for (const Assignment& p : step.pickups) {
    mix(p.robot);
    mix(p.task);
  }
  for (const Assignment& d : step.deliveries) {
    mix(d.robot);
    mix(d.task);
  }

  // An order a robot that was out of touch delivered too counts once (Stage 6).
  std::vector<Assignment> first;
  for (const Assignment& d : step.deliveries) {
    if (delivered_.insert(d.task).second) {
      first.push_back(d);
    } else {
      ++duplicate_deliveries_;
    }
  }
  for (const Assignment& p : step.pickups) picked_at_.emplace(p.task, step.t);
  delivered_total_ += static_cast<int64_t>(first.size());
  for (const Assignment& d : first) open_.erase(d.task);
  for (const MotionRecord& m : step.motion) {
    if (m.outcome == MotionOutcome::kNoCommand) ++no_command_steps_;
    if (m.outcome == MotionOutcome::kSafeStop) ++safe_stop_steps_;
  }
  expansions_ += tick.expansions;
  if (tick.planner_called) planning_ms_.push_back(tick.planning_ms);
  if (tick.planner_failed) ++planner_failures_;
  deadlocks_resolved_ += tick.deadlocks_resolved;
  if (step.t < warmup_) return;

  delivered_steady_ += static_cast<int64_t>(first.size());
  for (const Assignment& d : first) {
    const auto it = created_at_.find(d.task);
    if (it != created_at_.end() && it->second >= warmup_) {
      completion_steps_.push_back(static_cast<double>(step.t - it->second));
      // A delivered order was assigned and picked up first; the fallbacks only keep the three
      // phases adding up to the completion time.
      const auto assigned = assigned_at_.find(d.task);
      const auto picked = picked_at_.find(d.task);
      const Timestep a = assigned != assigned_at_.end() ? assigned->second : it->second;
      const Timestep p = picked != picked_at_.end() ? std::max(picked->second, a) : a;
      wait_for_robot_sum_ += static_cast<double>(a - it->second);
      to_pickup_sum_ += static_cast<double>(p - a);
      to_dropoff_sum_ += static_cast<double>(step.t - p);
    }
  }
  for (size_t i = 0; i < step.motion.size(); ++i) {
    if (after.holding[i]) ++holding_robot_steps_;
    const MotionOutcome o = step.motion[i].outcome;
    if (isSafetyStop(o)) ++conflicts_;
    if (isSafetyStop(o) || o == MotionOutcome::kPlannedWait) ++congestion_steps_;
    if (o == MotionOutcome::kDelayed) ++delayed_steps_;
    if (o == MotionOutcome::kYield) ++yields_;
    if (o == MotionOutcome::kNoPermit) ++traffic_holds_;
    switch (o) {  // the time budget: one class per robot-step
      case MotionOutcome::kMoved:
        if (!after.holding[i]) {
          ++drive_free_;
        } else if (after.status[i] == RobotStatus::kToDropoff ||
                   after.status[i] == RobotStatus::kDropping) {
          ++drive_loaded_;
        } else {
          ++drive_to_pickup_;
        }
        break;
      case MotionOutcome::kService:
        ++service_;
        break;
      case MotionOutcome::kPlannedWait:
      case MotionOutcome::kNoPermit:
      case MotionOutcome::kYield:
        ++traffic_;
        break;
      case MotionOutcome::kHold:
        ++station_wait_;
        break;
      case MotionOutcome::kIdle:
        ++idle_;
        break;
      case MotionOutcome::kBlockedVertex:
      case MotionOutcome::kBlockedSwap:
      case MotionOutcome::kBlockedOccupied:
        ++safety_stop_;
        break;
      case MotionOutcome::kDelayed:
      case MotionOutcome::kNoCommand:
      case MotionOutcome::kSafeStop:
      case MotionOutcome::kDead:
      case MotionOutcome::kBlockedCell:
      case MotionOutcome::kInvalidStep:
        ++lost_;
        break;
    }
  }
}

RunSummary RunStats::summarize(const Config& config, Timestep steps_run, double wall_seconds,
                               int64_t violations, bool aborted) const {
  RunSummary s;
  s.run_name = config.run.name;
  s.allocator = std::string(toString(config.allocation.method));
  s.planner = std::string(toString(config.planning.method));
  s.fleet_size = fleet_size_;
  s.seed = config.run.seed;
  s.horizon = config.run.horizon_steps;
  s.warmup = warmup_;
  s.steps_run = steps_run;
  s.orders_created = orders_created_;
  s.orders_delivered = delivered_total_;

  const Timestep measured = std::max<Timestep>(steps_run - warmup_, 0);
  if (measured > 0) {
    s.orders_per_hour = static_cast<double>(delivered_steady_) * 3600.0 / measured;
    s.utilization = static_cast<double>(holding_robot_steps_) /
                    (static_cast<double>(fleet_size_) * static_cast<double>(measured));
  }
  s.mean_completion_steps = mean(completion_steps_);
  s.p95_completion_steps = percentile(completion_steps_, 0.95);
  s.completion_samples = static_cast<int64_t>(completion_steps_.size());
  s.conflicts = conflicts_;
  s.congestion_steps = congestion_steps_;
  s.planning_calls = static_cast<int64_t>(planning_ms_.size());
  s.planning_p50_ms = percentile(planning_ms_, 0.50);
  s.planning_p95_ms = percentile(planning_ms_, 0.95);
  s.expansions = expansions_;
  s.planner_failures = planner_failures_;
  s.delayed_steps = delayed_steps_;
  s.yields = yields_;
  s.traffic_holds = traffic_holds_;
  s.deadlocks_resolved = deadlocks_resolved_;
  s.duplicate_deliveries = duplicate_deliveries_;
  s.no_command_steps = no_command_steps_;
  s.safe_stop_steps = safe_stop_steps_;
  s.time_drive_to_pickup = drive_to_pickup_;
  s.time_drive_loaded = drive_loaded_;
  s.time_drive_free = drive_free_;
  s.time_service = service_;
  s.time_traffic = traffic_;
  s.time_station_wait = station_wait_;
  s.time_idle = idle_;
  s.time_safety_stop = safety_stop_;
  s.time_lost = lost_;
  if (!completion_steps_.empty()) {
    const auto n = static_cast<double>(completion_steps_.size());
    s.mean_wait_for_robot_steps = wait_for_robot_sum_ / n;
    s.mean_to_pickup_steps = to_pickup_sum_ / n;
    s.mean_to_dropoff_steps = to_dropoff_sum_ / n;
  }
  if (!completion_steps_.empty()) {
    s.max_completion_steps = *std::max_element(completion_steps_.begin(), completion_steps_.end());
  }
  for (const auto& [task, created] : open_) {
    s.oldest_open_steps = std::max(s.oldest_open_steps, steps_run - created);
  }
  s.violations = violations;
  s.aborted = aborted;
  s.log_hash = hash_;
  s.wall_seconds = wall_seconds;
  return s;
}

}  // namespace fleet_core

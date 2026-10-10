#include "fleet_core/verification/progress_monitor.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace fleet_core {

ProgressMonitor::ProgressMonitor(int32_t fleet_size) : fleet_size_(fleet_size) {
  if (fleet_size < 0) throw std::invalid_argument("ProgressMonitor: negative fleet size");
  no_progress_.assign(static_cast<size_t>(fleet_size), 0);
}

const ProgressStep& ProgressMonitor::observe(const StepTruth& step, const TruthSnapshot& after) {
  const auto n = static_cast<size_t>(fleet_size_);
  if (step.motion.size() != n || after.cells.size() != n || after.alive.size() != n ||
      after.holding.size() != n) {
    throw std::invalid_argument("ProgressMonitor: step has " + std::to_string(step.motion.size()) +
                                " robots, expected " + std::to_string(n));
  }
  ProgressStep& p = last_;
  p = ProgressStep{};
  p.waits_for.assign(n, -1);
  p.on_cycle.assign(n, 0);
  p.behind.assign(n, 0);
  p.stalled.assign(n, 0);

  // Who stands where after the step. Dead robots still occupy their cells.
  std::map<Cell, RobotId> occupant;
  for (size_t i = 0; i < n; ++i) occupant[after.cells[i]] = static_cast<RobotId>(i);

  bool progress = false;
  work_left_ = false;
  for (size_t i = 0; i < n; ++i) {
    const MotionRecord& m = step.motion[i];
    if (!after.alive[i]) {
      no_progress_[i] = 0;
      continue;
    }
    if (after.holding[i]) work_left_ = true;
    // A safety stop always leaves someone in the wanted cell: the robot that stayed there, or
    // the lower id that won the cell this step.
    if (isSafetyStop(m.outcome)) {
      const auto it = occupant.find(m.intended);
      if (it != occupant.end() && it->second != static_cast<RobotId>(i)) {
        p.waits_for[i] = it->second;
      }
    }
    switch (m.outcome) {
      case MotionOutcome::kMoved:
      case MotionOutcome::kService:
      case MotionOutcome::kPlannedWait:
        no_progress_[i] = 0;
        progress = true;
        break;
      case MotionOutcome::kIdle:  // nothing to do is not a stall
      case MotionOutcome::kDead:
        no_progress_[i] = 0;
        break;
      case MotionOutcome::kHold:
      case MotionOutcome::kDelayed:
      case MotionOutcome::kBlockedVertex:
      case MotionOutcome::kBlockedSwap:
      case MotionOutcome::kBlockedOccupied:
      case MotionOutcome::kBlockedCell:
      case MotionOutcome::kInvalidStep:
        ++no_progress_[i];
        break;
    }
    if (no_progress_[i] >= kStallSteps) {
      p.stalled[i] = 1;
      ++p.stalled_count;
      ++stalled_robot_steps_;
    }
  }
  if (progress) last_progress_ = step.t;
  if (!step.deliveries.empty()) last_delivery_ = step.t;

  // Cycles. Walk each chain until it ends or meets a robot already seen; meeting a robot of the
  // current walk closes a cycle. state: 0 unseen, 1 on the current walk, 2 done.
  std::vector<uint8_t> state(n, 0);
  std::vector<std::vector<RobotId>> cycles;
  std::vector<int32_t> cycle_of(n, -1);  // cycle a robot is on or runs into
  for (size_t s = 0; s < n; ++s) {
    if (state[s] != 0) continue;
    std::vector<RobotId> walk;
    RobotId v = static_cast<RobotId>(s);
    while (v >= 0 && state[static_cast<size_t>(v)] == 0) {
      state[static_cast<size_t>(v)] = 1;
      walk.push_back(v);
      v = p.waits_for[static_cast<size_t>(v)];
    }
    if (v >= 0 && state[static_cast<size_t>(v)] == 1) {
      std::vector<RobotId> cycle(std::find(walk.begin(), walk.end(), v), walk.end());
      for (const RobotId r : cycle) {
        p.on_cycle[static_cast<size_t>(r)] = 1;
        cycle_of[static_cast<size_t>(r)] = static_cast<int32_t>(cycles.size());
      }
      std::sort(cycle.begin(), cycle.end());
      cycles.push_back(std::move(cycle));
    }
    for (const RobotId r : walk) state[static_cast<size_t>(r)] = 2;
  }
  // Behind: follow each chain to its end. Cycle robots already know their cycle, and any loop is
  // a cycle, so every walk stops. known: robot's cycle_of is final.
  std::vector<uint8_t> known(n, 0);
  for (size_t i = 0; i < n; ++i) known[i] = p.on_cycle[i];
  for (size_t s = 0; s < n; ++s) {
    std::vector<RobotId> walk;
    RobotId v = static_cast<RobotId>(s);
    while (v >= 0 && known[static_cast<size_t>(v)] == 0) {
      walk.push_back(v);
      v = p.waits_for[static_cast<size_t>(v)];
    }
    const int32_t reached = v >= 0 ? cycle_of[static_cast<size_t>(v)] : -1;
    for (const RobotId r : walk) {
      cycle_of[static_cast<size_t>(r)] = reached;
      known[static_cast<size_t>(r)] = 1;
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (p.on_cycle[i] != 0) {
      ++p.on_cycle_count;
    } else if (cycle_of[i] >= 0) {
      p.behind[i] = 1;
      ++p.behind_count;
    }
  }

  // Follow cycles across steps: same robots, consecutive steps.
  std::map<std::vector<RobotId>, ActiveCycle> next;
  for (const std::vector<RobotId>& cycle : cycles) {
    const auto it = active_.find(cycle);
    ActiveCycle a = it != active_.end() ? it->second : ActiveCycle{step.t, false, 0};
    if (!a.confirmed && step.t - a.formed + 1 >= kConfirmSteps) {
      a.confirmed = true;
      a.index = deadlocks_.size();
      deadlocks_.push_back(Deadlock{a.formed, cycle, std::nullopt});
    }
    next.emplace(cycle, a);
  }
  for (const auto& [cycle, a] : active_) {
    if (a.confirmed && next.count(cycle) == 0) deadlocks_[a.index].broken = step.t;
  }
  active_ = std::move(next);

  // Last-step view for summarize(): robots on or behind a confirmed cycle.
  final_deadlocked_ = 0;
  final_behind_ = 0;
  for (size_t i = 0; i < n; ++i) {
    if (cycle_of[i] < 0) continue;
    if (!active_.at(cycles[static_cast<size_t>(cycle_of[i])]).confirmed) continue;
    if (p.on_cycle[i] != 0) {
      ++final_deadlocked_;
    } else {
      ++final_behind_;
    }
  }
  return p;
}

ProgressSummary ProgressMonitor::summarize(Timestep steps_run) const {
  ProgressSummary s;
  if (!deadlocks_.empty()) s.first_deadlock_step = deadlocks_.front().formed;
  s.deadlocks = static_cast<int64_t>(deadlocks_.size());
  s.deadlocked_robots = final_deadlocked_;
  s.blocked_behind = final_behind_;
  s.stalled_robots = last_.stalled_count;
  s.stalled_robot_steps = stalled_robot_steps_;
  const Timestep since = last_progress_ + 1;
  if (work_left_ && steps_run - since >= kStallSteps) s.frozen_at = since;
  s.last_delivery_step = last_delivery_;
  return s;
}

}  // namespace fleet_core

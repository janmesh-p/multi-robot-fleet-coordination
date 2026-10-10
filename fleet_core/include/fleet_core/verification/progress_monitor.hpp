#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/execution/grid_executor.hpp"

namespace fleet_core {

// What the progress monitor saw in one step, per robot (index == RobotId).
struct ProgressStep {
  std::vector<RobotId> waits_for{};  // robot standing in the cell it wanted, -1 if none
  std::vector<uint8_t> on_cycle{};   // 1: on a wait-for cycle
  std::vector<uint8_t> behind{};     // 1: its wait chain runs into a cycle
  std::vector<uint8_t> stalled{};    // 1: had work but made no progress for kStallSteps steps
  int32_t on_cycle_count{0};
  int32_t behind_count{0};
  int32_t stalled_count{0};
};

// A wait-for cycle that lasted at least kConfirmSteps consecutive steps.
struct Deadlock {
  Timestep formed{0};                // first step of the cycle
  std::vector<RobotId> robots{};     // ascending
  std::optional<Timestep> broken{};  // first step without the cycle; empty if it never broke
};

// A run's progress in a few numbers. Every step is counted, warmup included: these describe
// failures, and a failure during warmup is still a failure.
struct ProgressSummary {
  Timestep first_deadlock_step{-1};  // formation step of the first deadlock
  int64_t deadlocks{0};
  int32_t deadlocked_robots{0};  // on a deadlock at the last step
  int32_t blocked_behind{0};     // chained into a deadlock at the last step
  int32_t stalled_robots{0};     // stalled at the last step
  int64_t stalled_robot_steps{0};
  Timestep frozen_at{-1};  // no robot made progress from here to the end, work left
  Timestep last_delivery_step{-1};
};

// Watches ground truth for robots that stop making progress, and for groups locked together.
//
// Wait-for graph: a robot stopped by robot safety waits for the robot standing in the cell it
// wanted. Each robot wants one cell, so each robot waits for at most one other, and the graph is
// a set of chains. A chain either ends at a robot that is not waiting (working at a station,
// holding, idle: a queue that clears) or loops back on itself: a cycle, where every robot waits
// for the next and none can move until a plan changes. A cycle that lasts kConfirmSteps steps is
// a deadlock. Robots whose chain runs into a cycle are blocked behind it.
//
// Like the invariant checker, it reads only ground truth, so the coordinator never grades itself.
class ProgressMonitor {
 public:
  // No progress for this long while holding work: stalled. Longer than a station visit plus a
  // short queue, so waiting your turn is not a stall.
  static constexpr Timestep kStallSteps = 30;
  // A cycle must survive this long to count as a deadlock. A new plan can break a cycle in a
  // step or two (a robot gets its station turn and turns around); a deadlock does not break.
  static constexpr Timestep kConfirmSteps = 10;

  explicit ProgressMonitor(int32_t fleet_size);

  // Call once per step, after the robots moved. The reference stays valid until the next call.
  const ProgressStep& observe(const StepTruth& step, const TruthSnapshot& after);

  // Deadlocks so far, in order of formation. A cycle still younger than kConfirmSteps is not
  // listed.
  const std::vector<Deadlock>& deadlocks() const { return deadlocks_; }

  // steps_run: steps observed; frozen_at needs at least kStallSteps frozen steps at the end.
  ProgressSummary summarize(Timestep steps_run) const;

 private:
  struct ActiveCycle {
    Timestep formed{0};
    bool confirmed{false};
    size_t index{0};  // into deadlocks_, when confirmed
  };

  int32_t fleet_size_;
  ProgressStep last_{};
  std::vector<Timestep> no_progress_{};  // consecutive steps without progress, per robot
  std::map<std::vector<RobotId>, ActiveCycle> active_{};
  std::vector<Deadlock> deadlocks_{};
  int64_t stalled_robot_steps_{0};
  Timestep last_progress_{-1};
  Timestep last_delivery_{-1};
  bool work_left_{false};  // some live robot held a task at the last step
  int32_t final_deadlocked_{0};
  int32_t final_behind_{0};
};

}  // namespace fleet_core

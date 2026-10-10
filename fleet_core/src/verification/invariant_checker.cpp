#include "fleet_core/verification/invariant_checker.hpp"

#include <cstddef>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>

namespace fleet_core {
namespace {

std::string cellText(const Cell& c) {
  return "[" + std::to_string(c.row) + ", " + std::to_string(c.col) + "]";
}

std::string robotText(size_t i) { return "robot " + std::to_string(i); }

}  // namespace

std::string_view toString(ViolationKind kind) {
  switch (kind) {
    case ViolationKind::kCellShared:
      return "cell_shared";
    case ViolationKind::kSwap:
      return "swap";
    case ViolationKind::kTeleport:
      return "teleport";
    case ViolationKind::kOffFreeCell:
      return "off_free_cell";
    case ViolationKind::kDeadRobotMoved:
      return "dead_robot_moved";
    case ViolationKind::kInvalidPlan:
      return "invalid_plan";
    case ViolationKind::kCoordinationConflict:
      return "coordination_conflict";
    case ViolationKind::kTaskHeldTwice:
      return "task_held_twice";
    case ViolationKind::kTaskDeliveredTwice:
      return "task_delivered_twice";
    case ViolationKind::kPhantomDelivery:
      return "phantom_delivery";
    case ViolationKind::kTaskLost:
      return "task_lost";
  }
  return "unknown";
}

InvariantChecker::InvariantChecker(bool conflict_free) : conflict_free_(conflict_free) {}

std::vector<Violation> InvariantChecker::check(const GridMap& map, const StepTruth& step,
                                               const TruthSnapshot& after, const TaskManager& tasks,
                                               int64_t orders_issued) {
  std::vector<Violation> out;
  const auto report = [&](ViolationKind kind, std::string detail) {
    out.push_back(Violation{step.t, kind, std::move(detail)});
  };
  const size_t n = after.cells.size();

  // Physical state after the step. Dead robots still occupy their cells.
  std::map<Cell, size_t> occupant;
  for (size_t i = 0; i < n; ++i) {
    const Cell& c = after.cells[i];
    if (!map.isFree(c)) report(ViolationKind::kOffFreeCell, robotText(i) + " on " + cellText(c));
    const auto [it, inserted] = occupant.emplace(c, i);
    if (!inserted) {
      report(ViolationKind::kCellShared,
             robotText(it->second) + " and " + robotText(i) + " on " + cellText(c));
    }
  }

  // Motion during the step.
  std::map<Cell, size_t> started_at;
  for (size_t i = 0; i < step.motion.size(); ++i) started_at.emplace(step.motion[i].from, i);
  for (size_t i = 0; i < step.motion.size(); ++i) {
    const MotionRecord& m = step.motion[i];
    const int32_t moved = std::abs(m.to.row - m.from.row) + std::abs(m.to.col - m.from.col);
    if (moved > 1) {
      report(ViolationKind::kTeleport,
             robotText(i) + " " + cellText(m.from) + " -> " + cellText(m.to));
    }
    if (i < n && !after.alive[i] && moved != 0) {
      report(ViolationKind::kDeadRobotMoved, robotText(i));
    }
    if (moved == 1) {
      const auto other = started_at.find(m.to);
      if (other != started_at.end() && other->second > i) {
        const MotionRecord& o = step.motion[other->second];
        if (o.to == m.from) {
          report(ViolationKind::kSwap, robotText(i) + " and " + robotText(other->second));
        }
      }
    }
    if (m.outcome == MotionOutcome::kInvalidStep) {
      report(ViolationKind::kInvalidPlan,
             robotText(i) + " told to go " + cellText(m.from) + " -> " + cellText(m.intended));
    }
    if (conflict_free_ && isSafetyStop(m.outcome)) {
      report(ViolationKind::kCoordinationConflict, robotText(i) + " " +
                                                       std::string(toString(m.outcome)) +
                                                       " entering " + cellText(m.intended));
    }
  }

  // Tasks: ground truth.
  std::map<TaskId, size_t> holder;
  for (size_t i = 0; i < n; ++i) {
    if (!after.holding[i]) continue;
    const auto [it, inserted] = holder.emplace(*after.holding[i], i);
    if (!inserted) {
      report(ViolationKind::kTaskHeldTwice, "task " + std::to_string(*after.holding[i]) +
                                                " held by " + robotText(it->second) + " and " +
                                                robotText(i));
    }
  }
  for (const Assignment& d : step.deliveries) {
    if (!delivered_.insert(d.task).second) {
      report(ViolationKind::kTaskDeliveredTwice, "task " + std::to_string(d.task) + " again by " +
                                                     robotText(static_cast<size_t>(d.robot)));
    }
  }

  // Tasks: the coordinator's table against the truth.
  if (static_cast<int64_t>(tasks.all().size()) != orders_issued) {
    report(ViolationKind::kTaskLost, "coordinator tracks " + std::to_string(tasks.all().size()) +
                                         " of " + std::to_string(orders_issued) + " orders");
  }
  for (const Task& task : tasks.all()) {
    if (task.status == TaskStatus::kDone && delivered_.count(task.order.id) == 0) {
      report(ViolationKind::kPhantomDelivery, "task " + std::to_string(task.order.id));
    }
  }
  return out;
}

}  // namespace fleet_core

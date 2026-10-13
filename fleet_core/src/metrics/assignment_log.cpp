#include "fleet_core/metrics/assignment_log.hpp"

#include <algorithm>
#include <stdexcept>

namespace fleet_core {

std::string_view toString(AssignmentEnd end) {
  switch (end) {
    case AssignmentEnd::kAssigned:
      return "assigned";
    case AssignmentEnd::kPickedUp:
      return "picked_up";
    case AssignmentEnd::kDelivered:
      return "delivered";
    case AssignmentEnd::kRequeued:
      return "requeued";
  }
  return "unknown";
}

void AssignmentLog::close(TaskId order, AssignmentEnd end) {
  const auto it = open_.find(order);
  if (it == open_.end()) return;
  rows_[it->second].end = end;
  open_.erase(it);
}

void AssignmentLog::record(const TickOutput& tick, const StepTruth& step) {
  const auto motion = [&](RobotId robot) -> const MotionRecord& {
    if (robot < 0 || static_cast<size_t>(robot) >= step.motion.size()) {
      throw std::out_of_range("AssignmentLog: no motion record for robot " + std::to_string(robot));
    }
    return step.motion[static_cast<size_t>(robot)];
  };

  // Requeues happen in the tick before allocation, so a requeued order's new row comes after.
  for (const TaskId order : tick.requeued) close(order, AssignmentEnd::kRequeued);

  for (const AssignmentRecord& r : tick.assignments) {
    close(r.order, AssignmentEnd::kRequeued);  // an order assigned again was taken back first
    AssignmentRow row;
    row.record = r;
    const MotionRecord& m = motion(r.robot);
    if (m.from == r.pickup) {
      row.arrived_t = step.t - 1;
      row.steps_to_pickup = 0;
    }
    open_[r.order] = rows_.size();
    if (row.steps_to_pickup < 0) en_route_.push_back(rows_.size());
    rows_.push_back(row);
  }

  // Arrivals: this step's move ended on the pickup.
  std::vector<size_t> still;
  for (const size_t i : en_route_) {
    AssignmentRow& row = rows_[i];
    if (row.end == AssignmentEnd::kRequeued) continue;
    if (motion(row.record.robot).to == row.record.pickup) {
      row.arrived_t = step.t;
      row.steps_to_pickup = step.t - row.record.t + 1;
    } else {
      still.push_back(i);
    }
  }
  en_route_.swap(still);

  for (const Assignment& p : step.pickups) {
    const auto it = open_.find(p.task);
    if (it != open_.end() && rows_[it->second].record.robot == p.robot) {
      rows_[it->second].end = AssignmentEnd::kPickedUp;
    }
  }
  for (const Assignment& d : step.deliveries) {
    const auto it = open_.find(d.task);
    if (it != open_.end() && rows_[it->second].record.robot == d.robot) {
      rows_[it->second].delivered_t = step.t;
      close(d.task, AssignmentEnd::kDelivered);
    }
  }
}

std::vector<AssignmentRow> AssignmentLog::rows() const {
  std::vector<AssignmentRow> sorted = rows_;
  std::stable_sort(
      sorted.begin(), sorted.end(), [](const AssignmentRow& a, const AssignmentRow& b) {
        return a.record.t != b.record.t ? a.record.t < b.record.t : a.record.robot < b.record.robot;
      });
  return sorted;
}

std::string assignmentCsvHeader() {
  std::string header = "t,order,robot";
  for (const std::string_view name : pairFeatureNames()) header += "," + std::string(name);
  return header + ",arrived_t,steps_to_pickup,delivered_t,end_status";
}

std::string assignmentCsvRow(const AssignmentRow& row) {
  const AssignmentRecord& r = row.record;
  std::string out =
      std::to_string(r.t) + "," + std::to_string(r.order) + "," + std::to_string(r.robot);
  for (size_t i = 0; i < kPairFeatureCount; ++i) {
    out += "," + std::to_string(pairFeature(r.features, i));
  }
  out += "," + std::to_string(row.arrived_t) + "," + std::to_string(row.steps_to_pickup) + "," +
         std::to_string(row.delivered_t) + "," + std::string(toString(row.end));
  return out;
}

}  // namespace fleet_core

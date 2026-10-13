#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/fleet_manager.hpp"

namespace fleet_core {

// How an assignment stood when the run ended.
enum class AssignmentEnd : uint8_t { kAssigned, kPickedUp, kDelivered, kRequeued };

std::string_view toString(AssignmentEnd end);

// One row of assignments.csv: a decision record and what happened next, from ground truth.
struct AssignmentRow {
  AssignmentRecord record{};
  Timestep arrived_t{-1};       // first step at or after t whose move ends on the pickup;
                                // t - 1 if the robot already stood there; -1 if never
  int32_t steps_to_pickup{-1};  // arrived_t - t + 1, or -1
  Timestep delivered_t{-1};     // step whose dropoff service finished the order, or -1
  AssignmentEnd end{AssignmentEnd::kAssigned};
};

// Joins the coordinator's assignment records with ground truth, one step at a time. Owned by the
// runner, like every other measurement, so the coordinator never grades itself.
class AssignmentLog {
 public:
  // Call once per step, after the robots moved: that step's tick output and ground truth.
  void record(const TickOutput& tick, const StepTruth& step);

  // Every assignment so far, sorted by t, then robot.
  std::vector<AssignmentRow> rows() const;

 private:
  void close(TaskId order, AssignmentEnd end);

  std::vector<AssignmentRow> rows_{};
  std::map<TaskId, size_t> open_{};  // order -> its row until delivered or requeued
  std::vector<size_t> en_route_{};   // rows whose robot has not reached the pickup yet
};

// assignments.csv: header and one row, as docs/learning_track.md (section 3) defines them.
std::string assignmentCsvHeader();
std::string assignmentCsvRow(const AssignmentRow& row);

}  // namespace fleet_core

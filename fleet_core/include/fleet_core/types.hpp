#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace fleet_core {

using RobotId = int32_t;
using TaskId = int32_t;
using Timestep = int32_t;

struct Cell {
  int32_t row{0};
  int32_t col{0};

  bool operator==(const Cell& o) const { return row == o.row && col == o.col; }
  bool operator!=(const Cell& o) const { return !(*this == o); }
};

struct CellHash {
  size_t operator()(const Cell& c) const noexcept {
    return std::hash<int64_t>{}((static_cast<int64_t>(c.row) << 32) ^ static_cast<uint32_t>(c.col));
  }
};

enum class RobotStatus : uint8_t { kIdle, kToPickup, kToDropoff, kDead };

struct RobotState {
  RobotId id{-1};
  Cell pose{};
  RobotStatus status{RobotStatus::kIdle};
  std::optional<TaskId> task{};
  double battery{1.0};        // 0..1
  Timestep last_heartbeat{0};  // for fault detection
};

enum class TaskStatus : uint8_t { kPending, kAssigned, kPickedUp, kDone };

struct Task {
  TaskId id{-1};
  Cell pickup{};
  Cell dropoff{};
  Timestep created_at{0};
  TaskStatus status{TaskStatus::kPending};
  std::optional<RobotId> assigned_to{};
};

struct Assignment {
  RobotId robot{-1};
  TaskId task{-1};
};

// Timed path: path[t] is the cell at timestep (start_time + t).
struct Path {
  RobotId robot{-1};
  Timestep start_time{0};
  std::vector<Cell> cells{};
};

struct PlanResult {
  bool success{false};
  std::vector<Path> paths{};
  double planning_time_ms{0.0};
  int64_t nodes_expanded{0};
};

}  // namespace fleet_core

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace fleet_core {

using RobotId = int32_t;
using TaskId = int32_t;
using Timestep = int32_t;  // 1 step = 1 simulated second

// Grid cell. Rows grow downward, columns grow rightward.
struct Cell {
  int32_t row{0};
  int32_t col{0};

  friend bool operator==(const Cell& a, const Cell& b) { return a.row == b.row && a.col == b.col; }
  friend bool operator!=(const Cell& a, const Cell& b) { return !(a == b); }
  // Row-major order, so ordered containers iterate the same way on every run.
  friend bool operator<(const Cell& a, const Cell& b) {
    return a.row != b.row ? a.row < b.row : a.col < b.col;
  }
};

struct CellHash {
  size_t operator()(const Cell& c) const noexcept {
    const uint64_t key =
        (static_cast<uint64_t>(static_cast<uint32_t>(c.row)) << 32U) | static_cast<uint32_t>(c.col);
    return std::hash<uint64_t>{}(key);
  }
};

// Robot mode. Robots report kIdle through kSafeStop. kDead exists only in the coordinator's
// belief, after a robot's heartbeat lease expires.
enum class RobotStatus : uint8_t {
  kIdle,       // no task: parked or heading to parking
  kToPickup,   // heading to the pickup station
  kPicking,    // dwelling at the pickup station
  kToDropoff,  // carrying, heading to the dropoff station
  kDropping,   // dwelling at the dropoff station
  kSafeStop,   // commands stopped arriving; holding position (Stage 6)
  kDead,       // coordinator belief only (Stage 6)
};

enum class TaskStatus : uint8_t { kPending, kAssigned, kPickedUp, kDone };

// What a robot needs to execute one order. Sent inside RobotCommand.
struct TaskOrder {
  TaskId id{-1};
  Cell pickup{};
  Cell dropoff{};
};

// One order and its lifecycle, as tracked by TaskManager.
struct Task {
  TaskOrder order{};
  TaskStatus status{TaskStatus::kPending};
  std::optional<RobotId> robot{};
  Timestep created_at{0};
  std::optional<Timestep> assigned_at{};
  std::optional<Timestep> picked_at{};
  std::optional<Timestep> done_at{};
};

struct Assignment {
  RobotId robot{-1};
  TaskId task{-1};
};

// Timed path: cells[k] is the planned cell at step start + k. A repeated cell is a wait.
struct Path {
  RobotId robot{-1};
  Timestep start{0};
  std::vector<Cell> cells{};
};

struct PlanResult {
  bool success{false};
  std::vector<Path> paths{};  // one per requested agent, in request order
  int64_t expansions{0};      // search effort; deterministic, unlike wall time
  double planning_ms{0.0};    // wall time; measured, never used for decisions
};

// Robot to coordinator. State-based and idempotent: everything a lost report carried is
// repeated by the next one, so pickups and deliveries are never missed under packet loss.
struct RobotReport {
  RobotId robot{-1};
  uint32_t seq{0};    // starts at 1 and grows by 1; stale or reordered reports are dropped
  Timestep stamp{0};  // robot-side step when sent
  Cell cell{};
  RobotStatus status{RobotStatus::kIdle};
  std::optional<TaskId> task{};            // task held: heading to pickup or carrying
  std::optional<TaskId> last_completed{};  // newest delivery, repeated until the next one
  int32_t plan_id{-1};                     // newest command applied
  int32_t path_index{0};                   // progress along that command's path
};

// Coordinator to robot. Sent to every live robot every tick, so a lost command is
// replaced one step later.
struct RobotCommand {
  RobotId robot{-1};
  int32_t plan_id{-1};  // grows per robot; robots ignore older plans
  Timestep issued_at{0};
  std::optional<TaskOrder> task{};  // empty when idle or parking
  std::vector<Cell> path{};         // path[0] is the robot's expected cell
  int32_t permitted_until{0};       // the robot may advance up to path[permitted_until]
};

// Ground-truth faults, scripted in the config and injected into the executor backend.
enum class FaultType : uint8_t { kRobotDeath, kBlockCells, kUnblockCells, kCommOutage };

struct FaultEvent {
  Timestep at{0};
  FaultType type{FaultType::kRobotDeath};
  RobotId robot{-1};          // robot_death, comm_outage
  std::vector<Cell> cells{};  // block_cells, unblock_cells
  Timestep duration{0};       // comm_outage
};

}  // namespace fleet_core

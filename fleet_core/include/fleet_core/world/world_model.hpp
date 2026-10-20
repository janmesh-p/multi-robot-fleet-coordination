#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// What the coordinator believes about one robot, built only from received reports.
struct RobotBelief {
  RobotId id{-1};
  Cell cell{};  // last reported cell
  RobotStatus status{RobotStatus::kIdle};
  std::optional<TaskId> task{};            // task the robot says it holds
  std::optional<TaskId> last_completed{};  // newest delivery the robot reported
  Timestep last_heard{0};                  // step of the newest accepted report (heartbeat lease)
  uint32_t last_seq{0};                    // newest accepted report sequence number
  int32_t plan_id{-1};                     // newest command the robot has applied
  int32_t path_index{0};                   // robot progress along that command's path
  bool carrying{false};                    // has picked up `task`
  bool removed{false};                     // dead and taken off the floor (Stage 6)
};

// Blocked or reopened cells, from the fault script or the overhead camera (Stage 9b).
struct CellObservation {
  Timestep stamp{0};
  std::vector<Cell> blocked{};
  std::vector<Cell> cleared{};
};

// The coordinator's belief about the warehouse: map, dynamic blocks, robots.
//
// Every coordinator component reads this one object, so they never disagree with each other.
// Ground truth lives in the executor backend. With packet loss the two diverge (Stage 6).
class WorldModel {
 public:
  // Robot i starts at initial_cells[i], idle, with no plan.
  WorldModel(GridMap map, const std::vector<Cell>& initial_cells);

  const GridMap& map() const { return map_; }
  Timestep now() const { return now_; }
  void setNow(Timestep now) { now_ = now; }

  int32_t fleetSize() const { return static_cast<int32_t>(robots_.size()); }
  const std::vector<RobotBelief>& robots() const { return robots_; }  // index == RobotId
  const RobotBelief& robot(RobotId id) const;                         // throws std::out_of_range

  // Accepts the report if its sequence number is newer than the last accepted one.
  // Returns false for stale, duplicate, or reordered reports, and for dead robots.
  bool applyReport(const RobotReport& report);

  void applyObservation(const CellObservation& observation);

  // Stage 6. A dead robot keeps its cell until it is removed from the floor. revive() takes back a
  // robot declared dead by mistake: it applies the report if it is newer than the last one.
  void markDead(RobotId id);
  void markRemoved(RobotId id);
  void revive(const RobotReport& report);

 private:
  RobotBelief& mutableRobot(RobotId id);

  GridMap map_;
  std::vector<RobotBelief> robots_{};
  Timestep now_{0};
};

}  // namespace fleet_core

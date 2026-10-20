#pragma once

#include <cstddef>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/world/distance_oracle.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// One robot per station at a time. Robots that need a busy station wait their turn in arrival
// order, so two robots never share a goal: shared goals make CBS unsolvable and pile robots
// onto one cell.
//
// Waiting robots park on spare parking cells (parking cells that are nobody's home), nearest to
// their station first. Like every rest spot they are endpoints off the aisles, so a waiting
// robot never blocks traffic. With no spare cell left a robot waits where the caller says.
class StationManager {
 public:
  static constexpr size_t kWaitingCellsPerStation = 8;

  // reserved: parking cells that are robots' homes, never used for waiting.
  // queues = false (features.station_queues off): every robot heads straight to its station.
  StationManager(const GridMap& map, const DistanceOracle& distances,
                 const std::vector<Cell>& reserved, bool queues);

  // Joins the station's queue. No effect if the robot is already queued there.
  // Throws std::out_of_range if the cell is not a station.
  void request(RobotId robot, const Cell& station);

  // Leaves the station's queue, or gives up its turn so the next robot in line gets it.
  void release(RobotId robot, const Cell& station);

  // Where the robot should go now: the station on its turn, otherwise its waiting cell,
  // otherwise (every waiting cell taken) `fallback`. Throws std::logic_error if the robot never
  // requested the station.
  Cell goalFor(RobotId robot, const Cell& station, const Cell& fallback) const;

  bool hasTurn(RobotId robot, const Cell& station) const;
  // Closed cells (closures, and dead robots in Stage 6) are never offered for waiting. A robot
  // whose waiting cell closes gets another one, or waits at its fallback.
  void refresh(const GridMap& map);
  const std::vector<Cell>& waitingCells(const Cell& station) const;  // throws std::out_of_range

 private:
  void assignWaitingCell(RobotId robot, const Cell& station);
  void freeWaitingCell(RobotId robot);

  bool enabled_;
  std::map<Cell, std::vector<Cell>> candidates_{};  // station -> waiting cells, best first
  std::map<Cell, std::deque<RobotId>> queues_{};    // station -> robots; front has the turn
  std::map<RobotId, Cell> waiting_{};               // robot -> its waiting cell
  std::set<Cell> taken_{};                          // waiting cells in use
  std::set<Cell> closed_{};                         // waiting cells that are closed now
};

}  // namespace fleet_core

#include "fleet_core/tasks/station_manager.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace fleet_core {

StationManager::StationManager(const GridMap& map, const DistanceOracle& distances,
                               const std::vector<Cell>& reserved, bool queues)
    : enabled_(queues) {
  std::vector<Cell> stations = map.pickups();
  stations.insert(stations.end(), map.dropoffs().begin(), map.dropoffs().end());

  const std::set<Cell> homes(reserved.begin(), reserved.end());
  std::vector<Cell> pool;
  for (const Cell& c : map.parking()) {
    if (homes.count(c) == 0) pool.push_back(c);
  }
  for (const Cell& s : stations) {
    std::vector<Cell> ranked;
    for (const Cell& c : pool) {
      if (distances.distance(c, s) < DistanceOracle::kUnreachable) ranked.push_back(c);
    }
    std::sort(ranked.begin(), ranked.end(), [&](const Cell& a, const Cell& b) {
      return std::make_tuple(distances.distance(a, s), a.row, a.col) <
             std::make_tuple(distances.distance(b, s), b.row, b.col);
    });
    if (ranked.size() > kWaitingCellsPerStation) ranked.resize(kWaitingCellsPerStation);
    candidates_[s] = std::move(ranked);
  }
}

void StationManager::request(RobotId robot, const Cell& station) {
  if (!enabled_) return;
  if (candidates_.count(station) == 0) throw std::out_of_range("StationManager: not a station");
  std::deque<RobotId>& queue = queues_[station];
  if (std::find(queue.begin(), queue.end(), robot) != queue.end()) return;
  queue.push_back(robot);
  if (queue.size() > 1) assignWaitingCell(robot, station);
}

void StationManager::release(RobotId robot, const Cell& station) {
  if (!enabled_) return;
  const auto it = queues_.find(station);
  if (it == queues_.end()) return;
  std::deque<RobotId>& queue = it->second;
  const auto pos = std::find(queue.begin(), queue.end(), robot);
  if (pos == queue.end()) return;
  const bool had_turn = pos == queue.begin();
  queue.erase(pos);
  freeWaitingCell(robot);
  if (had_turn && !queue.empty()) freeWaitingCell(queue.front());  // next robot goes in
}

bool StationManager::hasTurn(RobotId robot, const Cell& station) const {
  if (!enabled_) return true;
  const auto it = queues_.find(station);
  return it != queues_.end() && !it->second.empty() && it->second.front() == robot;
}

Cell StationManager::goalFor(RobotId robot, const Cell& station, const Cell& fallback) const {
  if (!enabled_) return station;
  const auto it = queues_.find(station);
  if (it == queues_.end() ||
      std::find(it->second.begin(), it->second.end(), robot) == it->second.end()) {
    throw std::logic_error("StationManager::goalFor: robot " + std::to_string(robot) +
                           " has not requested this station");
  }
  if (it->second.front() == robot) return station;
  const auto waiting = waiting_.find(robot);
  return waiting != waiting_.end() ? waiting->second : fallback;
}

const std::vector<Cell>& StationManager::waitingCells(const Cell& station) const {
  const auto it = candidates_.find(station);
  if (it == candidates_.end()) throw std::out_of_range("StationManager: not a station");
  return it->second;
}

void StationManager::assignWaitingCell(RobotId robot, const Cell& station) {
  const auto it = candidates_.find(station);
  if (it == candidates_.end()) throw std::out_of_range("StationManager: not a station");
  for (const Cell& c : it->second) {
    if (taken_.count(c) > 0 || closed_.count(c) > 0) continue;
    taken_.insert(c);
    waiting_[robot] = c;
    return;
  }
}

void StationManager::refresh(const GridMap& map) {
  if (!enabled_) return;
  closed_.clear();
  for (const auto& [station, cells] : candidates_) {
    for (const Cell& c : cells) {
      if (map.isBlocked(c)) closed_.insert(c);
    }
  }
  std::vector<std::pair<RobotId, Cell>> moved;  // robot, the station it waits for
  for (auto& [station, queue] : queues_) {
    for (size_t k = 1; k < queue.size(); ++k) {
      const auto it = waiting_.find(queue[k]);
      if (it == waiting_.end() || closed_.count(it->second) == 0) continue;
      moved.emplace_back(queue[k], station);
    }
  }
  for (const auto& [robot, station] : moved) freeWaitingCell(robot);
  for (const auto& [robot, station] : moved) assignWaitingCell(robot, station);
}

void StationManager::freeWaitingCell(RobotId robot) {
  const auto it = waiting_.find(robot);
  if (it == waiting_.end()) return;
  taken_.erase(it->second);
  waiting_.erase(it);
}

}  // namespace fleet_core

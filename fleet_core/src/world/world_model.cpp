#include "fleet_core/world/world_model.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace fleet_core {

WorldModel::WorldModel(GridMap map, const std::vector<Cell>& initial_cells) : map_(std::move(map)) {
  robots_.reserve(initial_cells.size());
  for (size_t i = 0; i < initial_cells.size(); ++i) {
    RobotBelief belief;
    belief.id = static_cast<RobotId>(i);
    belief.cell = initial_cells[i];
    robots_.push_back(belief);
  }
}

const RobotBelief& WorldModel::robot(RobotId id) const {
  if (id < 0 || id >= fleetSize()) {
    throw std::out_of_range("WorldModel: unknown robot " + std::to_string(id));
  }
  return robots_[static_cast<size_t>(id)];
}

RobotBelief& WorldModel::mutableRobot(RobotId id) {
  if (id < 0 || id >= fleetSize()) {
    throw std::out_of_range("WorldModel: unknown robot " + std::to_string(id));
  }
  return robots_[static_cast<size_t>(id)];
}

bool WorldModel::applyReport(const RobotReport& report) {
  RobotBelief& b = mutableRobot(report.robot);
  if (b.status == RobotStatus::kDead || report.seq <= b.last_seq) return false;
  b.cell = report.cell;
  b.status = report.status;
  b.task = report.task;
  b.last_completed = report.last_completed;
  b.plan_id = report.plan_id;
  b.path_index = report.path_index;
  b.last_heard = now_;
  b.last_seq = report.seq;
  return true;
}

void WorldModel::applyObservation(const CellObservation& observation) {
  for (const Cell& c : observation.blocked) map_.block(c);
  for (const Cell& c : observation.cleared) map_.unblock(c);
}

void WorldModel::markDead(RobotId id) { mutableRobot(id).status = RobotStatus::kDead; }

}  // namespace fleet_core

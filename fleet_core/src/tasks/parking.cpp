#include "fleet_core/tasks/parking.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace fleet_core {

ParkingManager::ParkingManager(const GridMap& map, int32_t fleet_size) {
  if (fleet_size < 0 || static_cast<size_t>(fleet_size) > map.parking().size()) {
    throw std::invalid_argument("ParkingManager: map has " + std::to_string(map.parking().size()) +
                                " parking cells for " + std::to_string(fleet_size) + " robots");
  }
  homes_.assign(map.parking().begin(), map.parking().begin() + fleet_size);
}

const Cell& ParkingManager::home(RobotId robot) const {
  if (robot < 0 || static_cast<size_t>(robot) >= homes_.size()) {
    throw std::out_of_range("ParkingManager: unknown robot " + std::to_string(robot));
  }
  return homes_[static_cast<size_t>(robot)];
}

}  // namespace fleet_core

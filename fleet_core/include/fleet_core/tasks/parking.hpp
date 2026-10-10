#pragma once

#include <cstdint>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// One home parking cell per robot: robot i parks at the i-th parking cell in row-major order.
// Robots spawn at home and return there when idle (features.parking), so idle robots wait off
// the stations and out of the aisles.
class ParkingManager {
 public:
  // Throws std::invalid_argument when the map has fewer parking cells than robots.
  ParkingManager(const GridMap& map, int32_t fleet_size);

  const Cell& home(RobotId robot) const;  // throws std::out_of_range
  const std::vector<Cell>& homes() const { return homes_; }

 private:
  std::vector<Cell> homes_;
};

}  // namespace fleet_core

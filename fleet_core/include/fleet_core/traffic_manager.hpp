#pragma once

#include <optional>
#include <vector>

#include "fleet_core/grid_map.hpp"
#include "fleet_core/types.hpp"

namespace fleet_core {

// Stage 5. Intersection time-window reservations and deadlock detection.
class TrafficManager {
 public:
  explicit TrafficManager(const GridMap& map);

  // Reserve cells along a path. Returns false on conflict.
  bool reserve(const Path& path);
  void release(RobotId robot);

  // Robots in a wait-for cycle at timestep t. Empty if none.
  std::vector<RobotId> detectDeadlock(const std::vector<RobotState>& robots,
                                      const std::vector<Path>& paths,
                                      Timestep t) const;

  std::optional<RobotId> owner(const Cell& c, Timestep t) const;

 private:
  const GridMap& map_;
  // TODO(stage5): reservation table keyed by (cell, timestep).
};

}  // namespace fleet_core

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// Shortest-path distances on the static map, for allocation costs and planner heuristics.
//
// exact = true (features.distance_oracle): BFS distances between every pair of free cells,
// computed once at startup, following the same rule as the planners: a path may start or end on
// an endpoint (station, parking) but never pass through one. exact = false: Manhattan distance,
// the ablation. Both ignore dynamic blocks, so they never overestimate and stay admissible A*
// heuristics.
class DistanceOracle {
 public:
  static constexpr int32_t kUnreachable = std::numeric_limits<int32_t>::max() / 4;
  static constexpr int32_t kMaxFreeCells = 4000;  // exact table: 2 bytes per pair

  // Throws std::invalid_argument when the exact table would exceed kMaxFreeCells.
  DistanceOracle(const GridMap& map, bool exact);

  // Steps from `from` to `to`. kUnreachable when either cell is not static free (exact mode)
  // or no path exists.
  int32_t distance(const Cell& from, const Cell& to) const;

  bool exact() const { return exact_; }

 private:
  int32_t freeIndex(const Cell& c) const;

  bool exact_;
  int32_t rows_;
  int32_t cols_;
  int32_t free_count_{0};
  std::vector<int32_t> free_index_{};  // cell index -> free-cell index, or -1
  std::vector<uint16_t> table_{};      // [to * free_count_ + from]
};

}  // namespace fleet_core

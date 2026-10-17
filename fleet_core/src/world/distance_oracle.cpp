#include "fleet_core/world/distance_oracle.hpp"

#include <array>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace fleet_core {
namespace {

constexpr uint16_t kNoPath = 0xFFFF;

}  // namespace

DistanceOracle::DistanceOracle(const GridMap& map, bool exact)
    : exact_(exact), rows_(map.rows()), cols_(map.cols()) {
  if (!exact_) return;
  if (map.freeCellCount() > kMaxFreeCells) {
    throw std::invalid_argument("DistanceOracle: " + std::to_string(map.freeCellCount()) +
                                " free cells exceed the exact table limit of " +
                                std::to_string(kMaxFreeCells));
  }
  free_index_.assign(static_cast<size_t>(map.cellCount()), -1);
  std::vector<Cell> free_cells;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    const Cell c = map.cellAt(i);
    if (!map.isStaticFree(c)) continue;
    free_index_[static_cast<size_t>(i)] = static_cast<int32_t>(free_cells.size());
    free_cells.push_back(c);
  }
  free_count_ = static_cast<int32_t>(free_cells.size());
  const auto n = static_cast<size_t>(free_count_);

  // Neighbor lists in free-cell indices. A path may start on an endpoint or a blocked cell, end
  // on an endpoint, and pass through neither: both are leaves of the search.
  std::vector<uint8_t> endpoint(n, 0);
  std::vector<uint8_t> blocked(n, 0);
  for (size_t v = 0; v < n; ++v) {
    endpoint[v] = map.isEndpoint(free_cells[v]) ? 1 : 0;
    blocked[v] = map.isBlocked(free_cells[v]) ? 1 : 0;
  }
  std::vector<std::array<int32_t, 4>> neighbors(n);
  constexpr int32_t kDr[4] = {-1, 1, 0, 0};
  constexpr int32_t kDc[4] = {0, 0, -1, 1};
  for (size_t v = 0; v < n; ++v) {
    for (int dir = 0; dir < 4; ++dir) {
      const Cell next{free_cells[v].row + kDr[dir], free_cells[v].col + kDc[dir]};
      neighbors[v][static_cast<size_t>(dir)] = map.isStaticFree(next) ? freeIndex(next) : -1;
    }
  }

  table_.assign(n * n, kNoPath);
  std::vector<int32_t> queue(n);
  for (size_t source = 0; source < n; ++source) {
    uint16_t* dist = &table_[source * n];
    dist[source] = 0;
    if (blocked[source] != 0) continue;  // no path ends on a blocked cell
    size_t head = 0;
    size_t tail = 0;
    queue[tail++] = static_cast<int32_t>(source);
    while (head < tail) {
      const auto v = static_cast<size_t>(queue[head++]);
      // Endpoints and blocked cells get a distance (a path may start there) but are not passed
      // through.
      if (v != source && (endpoint[v] != 0 || blocked[v] != 0)) continue;
      const auto next_dist = static_cast<uint16_t>(dist[v] + 1);
      for (const int32_t w : neighbors[v]) {
        if (w < 0 || dist[static_cast<size_t>(w)] != kNoPath) continue;
        dist[static_cast<size_t>(w)] = next_dist;
        queue[tail++] = w;
      }
    }
  }
}

int32_t DistanceOracle::freeIndex(const Cell& c) const {
  if (c.row < 0 || c.row >= rows_ || c.col < 0 || c.col >= cols_) return -1;
  return free_index_[static_cast<size_t>(c.row * cols_ + c.col)];
}

int32_t DistanceOracle::distance(const Cell& from, const Cell& to) const {
  if (!exact_) return std::abs(from.row - to.row) + std::abs(from.col - to.col);
  const int32_t a = freeIndex(from);
  const int32_t b = freeIndex(to);
  if (a < 0 || b < 0) return kUnreachable;
  // The BFS row of `to` holds the distance from every cell to `to`: a path reversed is a path,
  // except that one may start on a blocked cell, and the row of a blocked `to` stays empty.
  const uint16_t d =
      table_[static_cast<size_t>(b) * static_cast<size_t>(free_count_) + static_cast<size_t>(a)];
  return d == kNoPath ? kUnreachable : d;
}

}  // namespace fleet_core

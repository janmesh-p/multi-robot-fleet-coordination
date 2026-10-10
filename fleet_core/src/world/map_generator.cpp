#include "fleet_core/world/map_generator.hpp"

#include <algorithm>
#include <cstddef>
#include <string>

namespace fleet_core {
namespace {

constexpr int32_t kDr[4] = {-1, 1, 0, 0};
constexpr int32_t kDc[4] = {0, 0, -1, 1};

// Station rows, spread evenly over the lane between the top and bottom aisle rows.
// Consecutive stations are at least two rows apart, so a free lane cell separates them.
std::vector<int32_t> stationRows(const MapGeneratorConfig& g) {
  const int32_t first = g.parking_rows + 1;
  const int32_t span = g.blocks_y * (g.block_height + 1) - 2;
  const int32_t n = g.stations;
  if (n == 1) return {first + span / 2};
  std::vector<int32_t> rows;
  for (int32_t k = 0; k < n; ++k) {
    rows.push_back(first + (2 * k * span + (n - 1)) / (2 * (n - 1)));  // round(k * span / (n-1))
  }
  return rows;
}

}  // namespace

GridMap generateWarehouse(const MapGeneratorConfig& g) {
  validateMapGenerator(g);

  const int32_t rows = generatedRows(g);
  const int32_t cols = generatedCols(g);
  std::vector<std::string> grid(static_cast<size_t>(rows),
                                std::string(static_cast<size_t>(cols), '.'));
  const auto set = [&grid](int32_t r, int32_t c, char ch) {
    grid[static_cast<size_t>(r)][static_cast<size_t>(c)] = ch;
  };

  for (int32_t p = 0; p < g.parking_rows; ++p) {
    for (int32_t c = 0; c < cols; ++c) {
      set(p, c, 'R');
      set(rows - 1 - p, c, 'R');
    }
  }
  for (int32_t by = 0; by < g.blocks_y; ++by) {
    const int32_t y0 = g.parking_rows + 1 + by * (g.block_height + 1);
    for (int32_t bx = 0; bx < g.blocks_x; ++bx) {
      const int32_t x0 = 2 + bx * (g.block_width + 1);
      for (int32_t r = y0; r < y0 + g.block_height; ++r) {
        for (int32_t c = x0; c < x0 + g.block_width; ++c) set(r, c, '#');
      }
    }
  }
  for (const int32_t r : stationRows(g)) {
    set(r, 0, 'P');
    set(r, cols - 1, 'D');
  }

  std::string text;
  for (const std::string& line : grid) text += line + "\n";
  return GridMap::fromString(text);
}

std::vector<std::pair<Cell, Cell>> findBridges(const GridMap& map) {
  // Tarjan's bridge algorithm, iterative so large maps cannot overflow the call stack.
  const int32_t n = map.cellCount();
  std::vector<int32_t> discovered(static_cast<size_t>(n), -1);
  std::vector<int32_t> low(static_cast<size_t>(n), 0);
  std::vector<std::pair<Cell, Cell>> bridges;
  struct Frame {
    int32_t cell;
    int32_t parent;
    int next_dir;
  };
  std::vector<Frame> stack;
  int32_t timer = 0;
  const auto at = [](std::vector<int32_t>& v, int32_t i) -> int32_t& {
    return v[static_cast<size_t>(i)];
  };

  for (int32_t root = 0; root < n; ++root) {
    if (!map.isStaticFree(map.cellAt(root)) || at(discovered, root) != -1) continue;
    at(discovered, root) = at(low, root) = timer++;
    stack.push_back(Frame{root, -1, 0});
    while (!stack.empty()) {
      Frame& top = stack.back();
      if (top.next_dir < 4) {
        const int dir = top.next_dir++;
        const Cell c = map.cellAt(top.cell);
        const Cell next{c.row + kDr[dir], c.col + kDc[dir]};
        if (!map.isStaticFree(next)) continue;
        const int32_t w = map.index(next);
        if (w == top.parent) continue;  // grid graphs have no parallel edges
        if (at(discovered, w) == -1) {
          at(discovered, w) = at(low, w) = timer++;
          const int32_t parent = top.cell;
          stack.push_back(Frame{w, parent, 0});  // invalidates `top`
        } else {
          at(low, top.cell) = std::min(at(low, top.cell), at(discovered, w));
        }
      } else {
        const int32_t v = top.cell;
        const int32_t parent = top.parent;
        stack.pop_back();
        if (parent >= 0) {
          at(low, parent) = std::min(at(low, parent), at(low, v));
          if (at(low, v) > at(discovered, parent)) {
            bridges.emplace_back(map.cellAt(parent), map.cellAt(v));
          }
        }
      }
    }
  }
  return bridges;
}

bool isConnected(const GridMap& map) {
  const int32_t n = map.cellCount();
  int32_t start = -1;
  for (int32_t i = 0; i < n && start < 0; ++i) {
    if (map.isStaticFree(map.cellAt(i))) start = i;
  }
  if (start < 0) return false;
  std::vector<uint8_t> seen(static_cast<size_t>(n), 0);
  std::vector<int32_t> queue{start};
  seen[static_cast<size_t>(start)] = 1;
  for (size_t head = 0; head < queue.size(); ++head) {
    const Cell c = map.cellAt(queue[head]);
    for (int dir = 0; dir < 4; ++dir) {
      const Cell next{c.row + kDr[dir], c.col + kDc[dir]};
      if (!map.isStaticFree(next)) continue;
      const auto w = static_cast<size_t>(map.index(next));
      if (seen[w] != 0) continue;
      seen[w] = 1;
      queue.push_back(map.index(next));
    }
  }
  return static_cast<int32_t>(queue.size()) == map.freeCellCount();
}

}  // namespace fleet_core

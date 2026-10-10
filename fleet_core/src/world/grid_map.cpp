#include "fleet_core/world/grid_map.hpp"

#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fleet_core {

GridMap GridMap::fromFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("GridMap: cannot open " + path);
  std::stringstream text;
  text << in.rdbuf();
  try {
    return fromString(text.str());
  } catch (const std::runtime_error& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

GridMap GridMap::fromString(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == ';') continue;
    lines.push_back(line);
  }
  if (lines.empty()) throw std::runtime_error("GridMap: empty map");

  GridMap map;
  map.rows_ = static_cast<int32_t>(lines.size());
  map.cols_ = static_cast<int32_t>(lines[0].size());
  map.cells_.reserve(static_cast<size_t>(map.cellCount()));
  for (int32_t r = 0; r < map.rows_; ++r) {
    const std::string& row = lines[static_cast<size_t>(r)];
    if (static_cast<int32_t>(row.size()) != map.cols_) {
      throw std::runtime_error("GridMap: row " + std::to_string(r) + " has " +
                               std::to_string(row.size()) + " columns, expected " +
                               std::to_string(map.cols_));
    }
    for (int32_t c = 0; c < map.cols_; ++c) {
      const char ch = row[static_cast<size_t>(c)];
      const Cell cell{r, c};
      switch (ch) {
        case '#':
        case '.':
          break;
        case 'P':
          map.pickups_.push_back(cell);
          break;
        case 'D':
          map.dropoffs_.push_back(cell);
          break;
        case 'R':
          map.parking_.push_back(cell);
          break;
        default:
          throw std::runtime_error(std::string("GridMap: unknown character '") + ch + "' at row " +
                                   std::to_string(r) + ", col " + std::to_string(c));
      }
      map.cells_.push_back(ch);
      if (ch != '#') ++map.free_count_;
    }
  }
  map.blocked_.assign(static_cast<size_t>(map.cellCount()), 0);
  return map;
}

bool GridMap::inBounds(const Cell& c) const {
  return c.row >= 0 && c.row < rows_ && c.col >= 0 && c.col < cols_;
}

bool GridMap::isStaticFree(const Cell& c) const {
  return inBounds(c) && cells_[static_cast<size_t>(index(c))] != '#';
}

bool GridMap::isFree(const Cell& c) const { return isStaticFree(c) && !isBlocked(c); }

char GridMap::kind(const Cell& c) const {
  return inBounds(c) ? cells_[static_cast<size_t>(index(c))] : '#';
}

bool GridMap::isEndpoint(const Cell& c) const {
  const char k = kind(c);
  return k == 'P' || k == 'D' || k == 'R';
}

void GridMap::block(const Cell& c) {
  if (!inBounds(c)) throw std::out_of_range("GridMap::block: cell outside the map");
  blocked_[static_cast<size_t>(index(c))] = 1;
}

void GridMap::unblock(const Cell& c) {
  if (!inBounds(c)) throw std::out_of_range("GridMap::unblock: cell outside the map");
  blocked_[static_cast<size_t>(index(c))] = 0;
}

bool GridMap::isBlocked(const Cell& c) const {
  return inBounds(c) && blocked_[static_cast<size_t>(index(c))] != 0;
}

std::vector<Cell> GridMap::neighbors(const Cell& c) const {
  static constexpr int32_t kDr[4] = {-1, 1, 0, 0};
  static constexpr int32_t kDc[4] = {0, 0, -1, 1};
  std::vector<Cell> out;
  out.reserve(4);
  for (int i = 0; i < 4; ++i) {
    const Cell n{c.row + kDr[i], c.col + kDc[i]};
    if (isFree(n)) out.push_back(n);
  }
  return out;
}

std::string GridMap::toString() const {
  std::string out;
  out.reserve(static_cast<size_t>(rows_ * (cols_ + 1)));
  for (int32_t r = 0; r < rows_; ++r) {
    out.append(cells_, static_cast<size_t>(r * cols_), static_cast<size_t>(cols_));
    out += '\n';
  }
  return out;
}

}  // namespace fleet_core

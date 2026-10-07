#include "fleet_core/grid_map.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fleet_core {

GridMap GridMap::fromFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("GridMap: cannot open " + path);
  std::stringstream ss;
  ss << in.rdbuf();
  return fromString(ss.str());
}

GridMap GridMap::fromString(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == ';') continue;  // ';' = comment
    lines.push_back(line);
  }
  if (lines.empty()) throw std::runtime_error("GridMap: empty map");

  GridMap m;
  m.rows_ = static_cast<int32_t>(lines.size());
  m.cols_ = static_cast<int32_t>(lines[0].size());
  m.static_free_.assign(static_cast<size_t>(m.rows_) * m.cols_, 0);

  for (int32_t r = 0; r < m.rows_; ++r) {
    if (static_cast<int32_t>(lines[r].size()) != m.cols_)
      throw std::runtime_error("GridMap: ragged row " + std::to_string(r));
    for (int32_t c = 0; c < m.cols_; ++c) {
      const char ch = lines[r][c];
      const Cell cell{r, c};
      switch (ch) {
        case '.': m.static_free_[m.idx(cell)] = 1; break;
        case 'P': m.static_free_[m.idx(cell)] = 1; m.pickups_.push_back(cell); break;
        case 'D': m.static_free_[m.idx(cell)] = 1; m.dropoffs_.push_back(cell); break;
        case '#': break;
        default:
          throw std::runtime_error(std::string("GridMap: bad char '") + ch + "'");
      }
    }
  }
  return m;
}

bool GridMap::inBounds(const Cell& c) const {
  return c.row >= 0 && c.row < rows_ && c.col >= 0 && c.col < cols_;
}

bool GridMap::isFree(const Cell& c) const {
  return inBounds(c) && static_free_[idx(c)] && !isBlocked(c);
}

void GridMap::block(const Cell& c) { dynamic_blocks_.insert(c); }
void GridMap::unblock(const Cell& c) { dynamic_blocks_.erase(c); }
bool GridMap::isBlocked(const Cell& c) const { return dynamic_blocks_.count(c) > 0; }

std::vector<Cell> GridMap::neighbors(const Cell& c) const {
  static constexpr int dr[4] = {-1, 1, 0, 0};
  static constexpr int dc[4] = {0, 0, -1, 1};
  std::vector<Cell> out;
  out.reserve(4);
  for (int i = 0; i < 4; ++i) {
    const Cell n{c.row + dr[i], c.col + dc[i]};
    if (isFree(n)) out.push_back(n);
  }
  return out;
}

}  // namespace fleet_core

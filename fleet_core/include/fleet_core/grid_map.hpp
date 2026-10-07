#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "fleet_core/types.hpp"

namespace fleet_core {

// ASCII map format:
//   '.' free   '#' shelf/wall   'P' pickup station   'D' dropoff station
class GridMap {
 public:
  static GridMap fromFile(const std::string& path);
  static GridMap fromString(const std::string& text);

  int32_t rows() const { return rows_; }
  int32_t cols() const { return cols_; }

  bool inBounds(const Cell& c) const;
  bool isFree(const Cell& c) const;  // static obstacles AND dynamic blocks

  // Dynamic blocking (blocked aisles, D435 detections).
  void block(const Cell& c);
  void unblock(const Cell& c);
  bool isBlocked(const Cell& c) const;

  // 4-connected free neighbors (wait action handled by planners).
  std::vector<Cell> neighbors(const Cell& c) const;

  const std::vector<Cell>& pickups() const { return pickups_; }
  const std::vector<Cell>& dropoffs() const { return dropoffs_; }

 private:
  int32_t rows_{0};
  int32_t cols_{0};
  std::vector<uint8_t> static_free_{};
  std::unordered_set<Cell, CellHash> dynamic_blocks_{};
  std::vector<Cell> pickups_{};
  std::vector<Cell> dropoffs_{};

  size_t idx(const Cell& c) const { return static_cast<size_t>(c.row) * cols_ + c.col; }
};

}  // namespace fleet_core

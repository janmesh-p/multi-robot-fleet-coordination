#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "fleet_core/common/types.hpp"

namespace fleet_core {

// Static warehouse grid plus dynamic blocks (closed aisles, perception detections).
//
// ASCII format, one row per line. Lines starting with ';' are comments.
//   '.' free   '#' shelf or wall   'P' pickup station   'D' dropoff station
class GridMap {
 public:
  static GridMap fromFile(const std::string& path);
  static GridMap fromString(const std::string& text);

  int32_t rows() const { return rows_; }
  int32_t cols() const { return cols_; }
  int32_t freeCellCount() const { return free_count_; }  // static free cells; blocks ignored

  bool inBounds(const Cell& c) const;
  bool isStaticFree(const Cell& c) const;  // inside the map and not a shelf or wall
  bool isFree(const Cell& c) const;        // static free and not dynamically blocked

  // Throws std::out_of_range for cells outside the map.
  void block(const Cell& c);
  void unblock(const Cell& c);
  bool isBlocked(const Cell& c) const;

  // Free 4-connected neighbors in fixed order: up, down, left, right. Waiting is up to planners.
  std::vector<Cell> neighbors(const Cell& c) const;

  const std::vector<Cell>& pickups() const { return pickups_; }    // row-major order
  const std::vector<Cell>& dropoffs() const { return dropoffs_; }  // row-major order

 private:
  size_t index(const Cell& c) const {
    return static_cast<size_t>(c.row) * static_cast<size_t>(cols_) + static_cast<size_t>(c.col);
  }

  int32_t rows_{0};
  int32_t cols_{0};
  int32_t free_count_{0};
  std::vector<uint8_t> static_free_{};
  std::unordered_set<Cell, CellHash> blocked_{};  // lookup only; never iterated
  std::vector<Cell> pickups_{};
  std::vector<Cell> dropoffs_{};
};

}  // namespace fleet_core

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "fleet_core/common/types.hpp"

namespace fleet_core {

// Static warehouse grid plus dynamic blocks (closed aisles, perception detections).
//
// ASCII format, one row per line. Lines starting with ';' are comments.
//   '.' free   '#' shelf or wall   'P' pickup station   'D' dropoff station   'R' parking
class GridMap {
 public:
  static GridMap fromFile(const std::string& path);
  static GridMap fromString(const std::string& text);

  int32_t rows() const { return rows_; }
  int32_t cols() const { return cols_; }
  int32_t cellCount() const { return rows_ * cols_; }
  int32_t freeCellCount() const { return free_count_; }  // static free cells; blocks ignored

  // Row-major index for dense per-cell arrays. Only valid for cells inside the map.
  int32_t index(const Cell& c) const { return c.row * cols_ + c.col; }
  Cell cellAt(int32_t i) const { return Cell{i / cols_, i % cols_}; }

  bool inBounds(const Cell& c) const;
  bool isStaticFree(const Cell& c) const;  // inside the map and not a shelf or wall
  bool isFree(const Cell& c) const;        // static free and not dynamically blocked
  char kind(const Cell& c) const;          // map character; '#' outside the map

  // Stations and parking cells. Paths may start or end on an endpoint but never pass through
  // one (well-formed MAPD), so parked and servicing robots never stand in anyone's way.
  bool isEndpoint(const Cell& c) const;

  // Throws std::out_of_range for cells outside the map.
  void block(const Cell& c);
  void unblock(const Cell& c);
  bool isBlocked(const Cell& c) const;

  // Free 4-connected neighbors in fixed order: up, down, left, right. Waiting is up to planners.
  std::vector<Cell> neighbors(const Cell& c) const;

  const std::vector<Cell>& pickups() const { return pickups_; }    // row-major order
  const std::vector<Cell>& dropoffs() const { return dropoffs_; }  // row-major order
  const std::vector<Cell>& parking() const { return parking_; }    // row-major order

  // Static map in the ASCII format above. Dynamic blocks are not included.
  std::string toString() const;

 private:
  int32_t rows_{0};
  int32_t cols_{0};
  int32_t free_count_{0};
  std::string cells_{};             // one map character per cell, row-major
  std::vector<uint8_t> blocked_{};  // 1 = dynamically blocked, per cell
  std::vector<Cell> pickups_{};
  std::vector<Cell> dropoffs_{};
  std::vector<Cell> parking_{};
};

}  // namespace fleet_core

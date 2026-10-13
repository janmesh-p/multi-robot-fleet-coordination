#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

#include "fleet_core/allocation/task_allocator.hpp"
#include "fleet_core/common/types.hpp"

namespace fleet_core {

// What the coordinator knows about one robot-order pair when it allocates, all from its belief.
// These are the feature columns of assignments.csv, in that order; docs/learning_track.md
// (section 3) defines each one. "Other robots" are robots believed alive, not counting this one;
// "near" is Manhattan distance kNearRadius or less; the box is the rectangle with the robot's
// cell and the pickup as opposite corners, edges included.
struct PairFeatures {
  int32_t dist_to_pickup{0};  // distance oracle: shortest path, or Manhattan when it is off
  int32_t manhattan_to_pickup{0};
  int32_t dist_pickup_to_dropoff{0};
  int32_t robot_row{0};
  int32_t robot_col{0};
  int32_t pickup_row{0};
  int32_t pickup_col{0};
  int32_t robot_on_endpoint{0};   // 1 on a station or parking cell
  int32_t robots_near_robot{0};   // other robots near the robot
  int32_t robots_near_pickup{0};  // other robots near the pickup
  int32_t robots_in_box{0};       // other robots inside the box
  int32_t box_free_cells{0};      // cells inside the box that are not shelves
  int32_t pickup_assigned{0};     // orders at this pickup assigned but not yet picked up
  int32_t pickup_pending{0};      // other unassigned orders at this pickup
  int32_t pending_orders{0};      // unassigned orders, this one included
  int32_t available_robots{0};    // believed alive and holding no order
  int32_t busy_robots{0};         // believed alive and holding an order
  int32_t alive_robots{0};        // believed alive
  int32_t order_age{0};           // steps since the order arrived
};

inline constexpr size_t kPairFeatureCount = 19;
inline constexpr int32_t kNearRadius = 3;

// Column names, in assignments.csv order.
const std::array<std::string_view, kPairFeatureCount>& pairFeatureNames();

// Feature `index` in pairFeatureNames() order. Throws std::out_of_range past the end.
int32_t pairFeature(const PairFeatures& features, size_t index);

// Position of a name in pairFeatureNames(), if it is a feature.
std::optional<size_t> pairFeatureIndex(std::string_view name);

// Features of any robot-order pair in one allocation round, from the context's belief before any
// assignment of that round. Counts that do not depend on the pair are done once here; each pair
// then costs two distance lookups and one pass over the live robots.
class PairFeatureTable {
 public:
  // Keeps a reference to the context, which must outlive the table.
  explicit PairFeatureTable(const AllocationContext& context);

  // Throws std::out_of_range for an unknown robot or task.
  PairFeatures pair(RobotId robot, TaskId task) const;

 private:
  int32_t freeCellsInBox(const Cell& a, const Cell& b) const;

  const AllocationContext& context_;
  int32_t cols_;
  std::vector<int32_t> free_prefix_;  // (rows + 1) x (cols + 1) prefix sums of static free cells
  std::vector<RobotId> alive_{};      // robots believed alive, ascending id
  std::map<Cell, int32_t> assigned_at_pickup_{};
  std::map<Cell, int32_t> pending_at_pickup_{};
  int32_t busy_count_{0};
};

}  // namespace fleet_core

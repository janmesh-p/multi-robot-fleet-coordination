#include "fleet_core/allocation/pair_features.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace fleet_core {
namespace {

constexpr std::array<std::string_view, kPairFeatureCount> kNames{
    "dist_to_pickup",     "manhattan_to_pickup", "dist_pickup_to_dropoff",
    "robot_row",          "robot_col",           "pickup_row",
    "pickup_col",         "robot_on_endpoint",   "robots_near_robot",
    "robots_near_pickup", "robots_in_box",       "box_free_cells",
    "pickup_assigned",    "pickup_pending",      "pending_orders",
    "available_robots",   "busy_robots",         "alive_robots",
    "order_age"};

int32_t manhattan(const Cell& a, const Cell& b) {
  return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

bool inBox(const Cell& c, const Cell& a, const Cell& b) {
  return c.row >= std::min(a.row, b.row) && c.row <= std::max(a.row, b.row) &&
         c.col >= std::min(a.col, b.col) && c.col <= std::max(a.col, b.col);
}

}  // namespace

const std::array<std::string_view, kPairFeatureCount>& pairFeatureNames() { return kNames; }

int32_t pairFeature(const PairFeatures& f, size_t index) {
  switch (index) {
    case 0:
      return f.dist_to_pickup;
    case 1:
      return f.manhattan_to_pickup;
    case 2:
      return f.dist_pickup_to_dropoff;
    case 3:
      return f.robot_row;
    case 4:
      return f.robot_col;
    case 5:
      return f.pickup_row;
    case 6:
      return f.pickup_col;
    case 7:
      return f.robot_on_endpoint;
    case 8:
      return f.robots_near_robot;
    case 9:
      return f.robots_near_pickup;
    case 10:
      return f.robots_in_box;
    case 11:
      return f.box_free_cells;
    case 12:
      return f.pickup_assigned;
    case 13:
      return f.pickup_pending;
    case 14:
      return f.pending_orders;
    case 15:
      return f.available_robots;
    case 16:
      return f.busy_robots;
    case 17:
      return f.alive_robots;
    case 18:
      return f.order_age;
    default:
      break;
  }
  throw std::out_of_range("pairFeature: no feature " + std::to_string(index));
}

std::optional<size_t> pairFeatureIndex(std::string_view name) {
  for (size_t i = 0; i < kNames.size(); ++i) {
    if (kNames[i] == name) return i;
  }
  return std::nullopt;
}

PairFeatureTable::PairFeatureTable(const AllocationContext& context)
    : context_(context), cols_(context.world.map().cols()) {
  const GridMap& map = context.world.map();
  const int32_t rows = map.rows();
  const auto width = static_cast<size_t>(cols_ + 1);
  free_prefix_.assign(static_cast<size_t>(rows + 1) * width, 0);
  for (int32_t r = 0; r < rows; ++r) {
    for (int32_t c = 0; c < cols_; ++c) {
      const auto at = [&](int32_t rr, int32_t cc) {
        return free_prefix_[static_cast<size_t>(rr) * width + static_cast<size_t>(cc)];
      };
      free_prefix_[static_cast<size_t>(r + 1) * width + static_cast<size_t>(c + 1)] =
          at(r, c + 1) + at(r + 1, c) - at(r, c) + (map.isStaticFree(Cell{r, c}) ? 1 : 0);
    }
  }

  for (const RobotBelief& b : context.world.robots()) {
    const std::optional<TaskId> task = context.tasks.taskOf(b.id);
    if (task) {
      const Task& t = context.tasks.get(*task);
      if (t.status == TaskStatus::kAssigned) ++assigned_at_pickup_[t.order.pickup];
    }
    if (b.status == RobotStatus::kDead) continue;
    alive_.push_back(b.id);
    if (task) ++busy_count_;
  }
  for (const TaskId id : context.tasks.pending()) {
    ++pending_at_pickup_[context.tasks.get(id).order.pickup];
  }
}

int32_t PairFeatureTable::freeCellsInBox(const Cell& a, const Cell& b) const {
  const int32_t r0 = std::min(a.row, b.row);
  const int32_t r1 = std::max(a.row, b.row) + 1;
  const int32_t c0 = std::min(a.col, b.col);
  const int32_t c1 = std::max(a.col, b.col) + 1;
  const auto width = static_cast<size_t>(cols_ + 1);
  const auto at = [&](int32_t r, int32_t c) {
    return free_prefix_[static_cast<size_t>(r) * width + static_cast<size_t>(c)];
  };
  return at(r1, c1) - at(r0, c1) - at(r1, c0) + at(r0, c0);
}

PairFeatures PairFeatureTable::pair(RobotId robot, TaskId task) const {
  const RobotBelief& b = context_.world.robot(robot);
  const Task& t = context_.tasks.get(task);
  const Cell& at = b.cell;
  const Cell& pickup = t.order.pickup;
  const GridMap& map = context_.world.map();

  PairFeatures f;
  f.dist_to_pickup = context_.distances.distance(at, pickup);
  f.manhattan_to_pickup = manhattan(at, pickup);
  f.dist_pickup_to_dropoff = context_.distances.distance(pickup, t.order.dropoff);
  f.robot_row = at.row;
  f.robot_col = at.col;
  f.pickup_row = pickup.row;
  f.pickup_col = pickup.col;
  f.robot_on_endpoint = map.isEndpoint(at) ? 1 : 0;
  for (const RobotId other : alive_) {
    if (other == robot) continue;
    const Cell& c = context_.world.robot(other).cell;
    if (manhattan(c, at) <= kNearRadius) ++f.robots_near_robot;
    if (manhattan(c, pickup) <= kNearRadius) ++f.robots_near_pickup;
    if (inBox(c, at, pickup)) ++f.robots_in_box;
  }
  if (map.inBounds(at) && map.inBounds(pickup)) f.box_free_cells = freeCellsInBox(at, pickup);

  const auto count = [](const std::map<Cell, int32_t>& m, const Cell& c) {
    const auto it = m.find(c);
    return it == m.end() ? 0 : it->second;
  };
  f.pickup_assigned = count(assigned_at_pickup_, pickup);
  f.pickup_pending = count(pending_at_pickup_, pickup) - (t.status == TaskStatus::kPending ? 1 : 0);
  f.pending_orders = context_.tasks.pendingCount();
  f.alive_robots = static_cast<int32_t>(alive_.size());
  f.busy_robots = busy_count_;
  f.available_robots = f.alive_robots - f.busy_robots;
  f.order_age = context_.world.now() - t.created_at;
  return f;
}

}  // namespace fleet_core

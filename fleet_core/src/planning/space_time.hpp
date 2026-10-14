#pragma once

// Private to fleet_core: the space-time machinery the coordinated planners share. Cells are
// GridMap indices; times are absolute steps.
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/world/distance_oracle.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core::detail {

inline constexpr Timestep kForever = std::numeric_limits<Timestep>::max() / 4;

// Hard obstacles of one planning call: the paths of robots that are not planned in it, and in
// prioritized schemes the robots planned before. Each robot holds its last cell after its path
// ends, forever.
class ReservationTable {
 public:
  ReservationTable(const GridMap& map, Timestep start);

  void add(const Path& path);

  // No reserved robot at `cell` at step t.
  bool vertexFree(int32_t cell, Timestep t) const;
  // No reserved robot moves from `to` to `from` during step t (from t to t + 1).
  bool edgeFree(int32_t from, int32_t to, Timestep t) const;
  // Latest step a reserved robot is at `cell`: kForever if one ends its path there, -1 if none.
  Timestep lastUse(int32_t cell) const;
  // From this step on no reserved robot moves.
  Timestep settled() const { return settled_; }

 private:
  int32_t occupant(int32_t cell, Timestep t) const;  // reservation index, or -1
  int32_t positionOf(size_t reservation, Timestep t) const;

  const GridMap& map_;
  int32_t n_;
  Timestep start_;
  Timestep settled_;
  std::vector<std::vector<int32_t>> cells_{};  // per reservation: its cell at start_ + k
  std::vector<int32_t> layers_{};              // (t - start_) * n_ + cell -> reservation + 1
  std::vector<Timestep> parked_;               // per cell: step from which a reservation holds it
  std::vector<Timestep> last_;                 // per cell: latest step a path is there, or -1
};

// CBS constraints on one agent.
class Constraints {
 public:
  void addVertex(int32_t cell, Timestep t);
  void addEdge(int32_t from, int32_t to, Timestep t);
  bool vertexBanned(int32_t cell, Timestep t) const { return vertex_.count({t, cell}) != 0; }
  bool edgeBanned(int32_t from, int32_t to, Timestep t) const {
    return edge_.count({t, from, to}) != 0;
  }
  Timestep latest() const { return latest_; }  // -1 without constraints
  Timestep latestAt(int32_t cell) const;       // latest vertex constraint on `cell`, or -1
  size_t size() const { return vertex_.size() + edge_.size(); }

 private:
  std::set<std::pair<Timestep, int32_t>> vertex_{};
  std::set<std::tuple<Timestep, int32_t, int32_t>> edge_{};
  std::unordered_map<int32_t, Timestep> latest_at_{};
  Timestep latest_{-1};
};

// Soft obstacles for ECBS: the other agents' current paths, counted, not avoided.
class ConflictCounter {
 public:
  ConflictCounter(const GridMap& map, Timestep start, Timestep until);
  void add(const std::vector<int32_t>& cells);  // positions at start, start + 1, ...
  // Conflicts of moving from `from` at step t to `to` at step t + 1.
  int32_t count(int32_t from, int32_t to, Timestep t) const;

 private:
  int32_t n_;
  Timestep start_;
  Timestep until_;
  std::vector<int32_t> at_{};  // (t - start_) * n_ + cell -> paths there
  std::unordered_map<uint64_t, int32_t> moves_{};
};

struct SearchInput {
  const GridMap& map;
  const DistanceOracle& distances;
  int32_t start{0};
  int32_t goal{0};
  Timestep t0{0};
  Timestep horizon{kForever};  // last step resolved: t0 + window, or kForever: to the goal
  const ReservationTable& reserved;
  const Constraints* constraints{nullptr};
  const ConflictCounter* soft{nullptr};  // ECBS: prefer fewer conflicts among near-best paths
  double focal{1.0};                     // ECBS low level: suboptimality bound
};

struct SearchOutput {
  bool found{false};
  bool out_of_budget{false};
  std::vector<int32_t> cells{};  // positions at t0, t0 + 1, ...; no trailing waits
  // Path cost in steps: the step it reaches the goal for good, minus t0; or, stopped by the
  // window, window + the distance left to the goal.
  int32_t cost{0};
  int32_t lower_bound{0};  // a bound no path under these constraints beats (focal search)
  int64_t expansions{0};
};

// Space-time A* (focal search when `focal` > 1 or `soft` is set). Moves to a free neighbor or
// waits, never enters an endpoint other than the goal, avoids reservations and constraints, and
// stops at the goal once it can hold it to the horizon, or when it reaches the horizon.
SearchOutput spaceTimeSearch(const SearchInput& in, int64_t budget);

// Path of `robot` starting at t0 from positions (cell indices).
Path toPath(const GridMap& map, RobotId robot, Timestep t0, const std::vector<int32_t>& cells);

// Drops trailing waits: holding the last cell after a path ends is implied.
void trimWaits(std::vector<int32_t>& cells);

}  // namespace fleet_core::detail

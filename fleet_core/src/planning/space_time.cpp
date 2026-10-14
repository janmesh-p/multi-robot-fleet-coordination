#include "space_time.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_set>

#include "fleet_core/planning/path_planner.hpp"

namespace fleet_core::detail {

ReservationTable::ReservationTable(const GridMap& map, Timestep start)
    : map_(map),
      n_(map.cellCount()),
      start_(start),
      settled_(start),
      parked_(static_cast<size_t>(map.cellCount()), kForever),
      last_(static_cast<size_t>(map.cellCount()), -1) {}

void ReservationTable::add(const Path& path) {
  if (path.cells.empty()) return;
  const Timestep end = std::max(start_, path.start + static_cast<Timestep>(path.cells.size()) - 1);
  std::vector<int32_t> positions;
  positions.reserve(static_cast<size_t>(end - start_ + 1));
  for (Timestep t = start_; t <= end; ++t) positions.push_back(map_.index(positionAt(path, t)));

  const size_t need = static_cast<size_t>(end - start_ + 1) * static_cast<size_t>(n_);
  if (layers_.size() < need) layers_.resize(need, 0);
  const auto id = static_cast<int32_t>(cells_.size());
  for (Timestep t = start_; t <= end; ++t) {
    const int32_t cell = positions[static_cast<size_t>(t - start_)];
    layers_[static_cast<size_t>(t - start_) * static_cast<size_t>(n_) + static_cast<size_t>(cell)] =
        id + 1;
    last_[static_cast<size_t>(cell)] = std::max(last_[static_cast<size_t>(cell)], t);
  }
  Timestep& parked = parked_[static_cast<size_t>(positions.back())];
  parked = std::min(parked, end);
  settled_ = std::max(settled_, end);
  cells_.push_back(std::move(positions));
}

int32_t ReservationTable::occupant(int32_t cell, Timestep t) const {
  if (t < start_) return -1;
  const size_t at =
      static_cast<size_t>(t - start_) * static_cast<size_t>(n_) + static_cast<size_t>(cell);
  return at < layers_.size() ? layers_[at] - 1 : -1;
}

int32_t ReservationTable::positionOf(size_t reservation, Timestep t) const {
  const std::vector<int32_t>& cells = cells_[reservation];
  const auto k = static_cast<size_t>(std::max<Timestep>(t - start_, 0));
  return cells[std::min(k, cells.size() - 1)];
}

bool ReservationTable::vertexFree(int32_t cell, Timestep t) const {
  if (t >= parked_[static_cast<size_t>(cell)]) return false;
  return occupant(cell, t) < 0;
}

bool ReservationTable::edgeFree(int32_t from, int32_t to, Timestep t) const {
  const int32_t r = occupant(to, t);  // a robot that holds its last cell never moves
  return r < 0 || positionOf(static_cast<size_t>(r), t + 1) != from;
}

Timestep ReservationTable::lastUse(int32_t cell) const {
  if (parked_[static_cast<size_t>(cell)] != kForever) return kForever;
  return last_[static_cast<size_t>(cell)];
}

void Constraints::addVertex(int32_t cell, Timestep t) {
  vertex_.insert({t, cell});
  Timestep& latest = latest_at_.emplace(cell, -1).first->second;
  latest = std::max(latest, t);
  latest_ = std::max(latest_, t);
}

void Constraints::addEdge(int32_t from, int32_t to, Timestep t) {
  edge_.insert({t, from, to});
  latest_ = std::max(latest_, t + 1);
}

Timestep Constraints::latestAt(int32_t cell) const {
  const auto it = latest_at_.find(cell);
  return it == latest_at_.end() ? -1 : it->second;
}

ConflictCounter::ConflictCounter(const GridMap& map, Timestep start, Timestep until)
    : n_(map.cellCount()),
      start_(start),
      until_(until),
      at_(static_cast<size_t>(until - start + 1) * static_cast<size_t>(map.cellCount()), 0) {}

namespace {

uint64_t moveKey(Timestep offset, int32_t from, int32_t to) {
  return (static_cast<uint64_t>(offset) << 42U) | (static_cast<uint64_t>(from) << 21U) |
         static_cast<uint64_t>(to);
}

}  // namespace

void ConflictCounter::add(const std::vector<int32_t>& cells) {
  if (cells.empty()) return;
  for (Timestep t = start_; t <= until_; ++t) {
    const auto k = std::min(static_cast<size_t>(t - start_), cells.size() - 1);
    ++at_[static_cast<size_t>(t - start_) * static_cast<size_t>(n_) +
          static_cast<size_t>(cells[k])];
  }
  for (size_t k = 0; k + 1 < cells.size(); ++k) {
    if (cells[k] != cells[k + 1])
      ++moves_[moveKey(static_cast<Timestep>(k), cells[k], cells[k + 1])];
  }
}

int32_t ConflictCounter::count(int32_t from, int32_t to, Timestep t) const {
  if (t < start_ || t + 1 > until_) return 0;
  int32_t n =
      at_[static_cast<size_t>(t + 1 - start_) * static_cast<size_t>(n_) + static_cast<size_t>(to)];
  if (from != to) {
    const auto it = moves_.find(moveKey(t - start_, to, from));
    if (it != moves_.end()) n += it->second;
  }
  return n;
}

namespace {

struct Node {
  int32_t cell;
  Timestep t;
  int32_t g;
  int32_t h;
  int32_t conflicts;
  int32_t parent;
};

}  // namespace

SearchOutput spaceTimeSearch(const SearchInput& in, int64_t budget) {
  SearchOutput out;
  const GridMap& map = in.map;
  const Cell goal_cell = map.cellAt(in.goal);
  const auto h = [&](int32_t c) { return in.distances.distance(map.cellAt(c), goal_cell); };
  const int32_t h0 = h(in.start);
  if (h0 >= DistanceOracle::kUnreachable) return out;
  const bool windowed = in.horizon < kForever;
  if (!windowed && in.reserved.lastUse(in.goal) == kForever) return out;  // parked on for good
  const bool focal = in.focal > 1.0 || in.soft != nullptr;
  const Timestep goal_latest = in.constraints ? in.constraints->latestAt(in.goal) : -1;

  // From t_static on nothing around the agent changes. A* folds later steps into one layer for
  // duplicate detection: the earliest arrival at a cell dominates. Focal search keeps steps
  // apart and stops n steps after t_static, when any static path would long have arrived.
  const Timestep t_static =
      windowed ? in.horizon
               : std::max({in.reserved.settled(), in.constraints ? in.constraints->latest() : -1,
                           in.t0}) +
                     1;
  const Timestep t_cap = windowed ? in.horizon : t_static + map.cellCount();
  const int32_t n = map.cellCount();
  const bool dense = windowed || !focal;
  std::vector<uint8_t> closed_dense;
  std::unordered_set<uint64_t> closed_sparse;
  if (dense) {
    closed_dense.assign(static_cast<size_t>(t_static - in.t0 + 1) * static_cast<size_t>(n), 0);
  }
  const auto close = [&](int32_t cell, Timestep t) {  // true if it was open
    if (dense) {
      const size_t k = static_cast<size_t>(std::min(t, t_static) - in.t0) * static_cast<size_t>(n) +
                       static_cast<size_t>(cell);
      if (closed_dense[k] != 0) return false;
      closed_dense[k] = 1;
      return true;
    }
    return closed_sparse
        .insert((static_cast<uint64_t>(t - in.t0) << 32U) | static_cast<uint32_t>(cell))
        .second;
  };
  const auto isClosed = [&](int32_t cell, Timestep t) {
    if (dense) {
      return closed_dense[static_cast<size_t>(std::min(t, t_static) - in.t0) *
                              static_cast<size_t>(n) +
                          static_cast<size_t>(cell)] != 0;
    }
    return closed_sparse.count((static_cast<uint64_t>(t - in.t0) << 32U) |
                               static_cast<uint32_t>(cell)) != 0;
  };
  const auto canHold = [&](Timestep t) {  // at the goal from step t to the horizon
    if (windowed) {
      for (Timestep s = t + 1; s <= in.horizon; ++s) {
        if (!in.reserved.vertexFree(in.goal, s)) return false;
        if (in.constraints && in.constraints->vertexBanned(in.goal, s)) return false;
      }
      return true;
    }
    return goal_latest <= t && in.reserved.lastUse(in.goal) <= t;
  };

  std::vector<Node> nodes;
  nodes.push_back(Node{in.start, in.t0, 0, h0, 0, -1});
  // A*: f, then h (deeper first), then creation order. Focal: conflicts first among nodes whose
  // f is within the bound of the best f.
  using AKey = std::tuple<int32_t, int32_t, int32_t>;
  std::priority_queue<AKey, std::vector<AKey>, std::greater<AKey>> open_a;
  std::set<std::tuple<int32_t, int32_t, int32_t>> open_f;            // f, h, index
  std::set<std::tuple<int32_t, int32_t, int32_t, int32_t>> focal_f;  // conflicts, f, h, index
  int32_t f_min = h0;
  const auto bound = [&]() { return static_cast<int32_t>(in.focal * f_min + 1e-9); };
  const auto push = [&](int32_t index) {
    const Node& v = nodes[static_cast<size_t>(index)];
    const int32_t f = v.g + v.h;
    if (!focal) {
      open_a.emplace(f, v.h, index);
      return;
    }
    open_f.emplace(f, v.h, index);
    if (f <= bound()) focal_f.emplace(v.conflicts, f, v.h, index);
  };
  push(0);

  while (focal ? !open_f.empty() : !open_a.empty()) {
    int32_t index = 0;
    if (!focal) {
      index = std::get<2>(open_a.top());
      open_a.pop();
    } else {
      // Refill FOCAL when the best f in OPEN rose.
      const int32_t best = std::get<0>(*open_f.begin());
      if (best > f_min) {
        const int32_t old_bound = bound();
        f_min = best;
        for (auto it = open_f.upper_bound({old_bound, std::numeric_limits<int32_t>::max(),
                                           std::numeric_limits<int32_t>::max()});
             it != open_f.end() && std::get<0>(*it) <= bound(); ++it) {
          const Node& v = nodes[static_cast<size_t>(std::get<2>(*it))];
          focal_f.emplace(v.conflicts, std::get<0>(*it), v.h, std::get<2>(*it));
        }
      }
      const auto top = *focal_f.begin();
      focal_f.erase(focal_f.begin());
      index = std::get<3>(top);
      open_f.erase({std::get<1>(top), std::get<2>(top), index});
    }
    const Node v = nodes[static_cast<size_t>(index)];
    if (!close(v.cell, v.t)) continue;
    if (++out.expansions > budget) {
      out.out_of_budget = true;
      return out;
    }
    const bool held = v.cell == in.goal && canHold(v.t);
    if (held || (windowed && v.t == in.horizon)) {
      for (int32_t k = index; k != -1; k = nodes[static_cast<size_t>(k)].parent) {
        out.cells.push_back(nodes[static_cast<size_t>(k)].cell);
      }
      std::reverse(out.cells.begin(), out.cells.end());
      trimWaits(out.cells);
      out.found = true;
      out.cost = v.g + v.h;
      out.lower_bound = focal ? std::min(f_min, out.cost) : out.cost;
      return out;
    }
    if (v.t >= t_cap) continue;
    const Timestep nt = v.t + 1;
    const Cell here = map.cellAt(v.cell);
    std::vector<int32_t> next{v.cell};
    for (const Cell& c : map.neighbors(here)) next.push_back(map.index(c));
    for (const int32_t w : next) {
      const bool move = w != v.cell;
      if (move && w != in.goal && map.isEndpoint(map.cellAt(w))) continue;
      if (!in.reserved.vertexFree(w, nt)) continue;
      if (in.constraints && in.constraints->vertexBanned(w, nt)) continue;
      if (move) {
        if (!in.reserved.edgeFree(v.cell, w, v.t)) continue;
        if (in.constraints && in.constraints->edgeBanned(v.cell, w, v.t)) continue;
      }
      if (isClosed(w, nt)) continue;
      const int32_t hw = h(w);
      if (hw >= DistanceOracle::kUnreachable) continue;
      const int32_t conflicts = v.conflicts + (in.soft ? in.soft->count(v.cell, w, v.t) : 0);
      nodes.push_back(Node{w, nt, v.g + 1, hw, conflicts, index});
      push(static_cast<int32_t>(nodes.size()) - 1);
    }
  }
  return out;  // no path under these reservations and constraints
}

Path toPath(const GridMap& map, RobotId robot, Timestep t0, const std::vector<int32_t>& cells) {
  Path p;
  p.robot = robot;
  p.start = t0;
  p.cells.reserve(cells.size());
  for (const int32_t c : cells) p.cells.push_back(map.cellAt(c));
  return p;
}

void trimWaits(std::vector<int32_t>& cells) {
  while (cells.size() >= 2 && cells[cells.size() - 1] == cells[cells.size() - 2]) cells.pop_back();
}

}  // namespace fleet_core::detail

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <vector>

#include "planners.hpp"
#include "space_time.hpp"

namespace fleet_core {
namespace {

// Deterministic stand-in for PIBT's random tie-breaking (splitmix64 finalizer).
uint64_t mix(uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31U);
}

// Priority Inheritance with Backtracking (Okumura et al. 2022), one step for every robot at a
// time, repeated `window` steps ahead.
//
// Each step, robots in priority order pick the free neighbor (or stay) closest to their goal. A
// robot that wants a cell another robot stands on lends it its priority: the other robot must
// move first, anywhere but back into the first robot's cell, or report that it cannot, and then
// the first robot tries its next choice. Priority: robots away from their goal before robots at
// it, then the one that has been away from its goal longest (since it got the goal or last
// stood on it), then a fixed hash of the robot.
//
// PIBT's guarantee that the top robot reaches its goal needs a graph without dead ends. Here
// every parking cell is one, and so is a corner cell between a station and a parking cell,
// because no path passes through an endpoint. A robot in one whose only way out is held by a
// robot that wants in would wait forever, so the robot at the exit steps aside, if it can, and
// the trapped robot moves out behind it in the same step.
class Pibt final : public PathPlanner {
 public:
  std::string_view name() const override { return "pibt"; }

  PlanResult plan(const PlanningRequest& request) override;

 private:
  struct Pursuit {
    Cell goal{};
    Timestep since{0};       // step the robot was given this goal
    Timestep away_since{0};  // since, or the latest planning step that found it on its goal
  };

  std::map<RobotId, Pursuit> pursuits_{};
};

// The state of one lookahead step.
class Step {
 public:
  Step(const GridMap& map, const DistanceOracle& distances, const detail::ReservationTable& fixed,
       const std::vector<int32_t>& goal, const std::vector<uint64_t>& tie,
       std::vector<int32_t>& cur)
      : map_(map),
        distances_(distances),
        fixed_(fixed),
        goal_(goal),
        tie_(tie),
        cur_(cur),
        next_(cur.size(), -1),
        now_(static_cast<size_t>(map.cellCount()), -1),
        taken_(static_cast<size_t>(map.cellCount()), -1) {
    for (size_t a = 0; a < cur_.size(); ++a)
      now_[static_cast<size_t>(cur_[a])] = static_cast<int32_t>(a);
  }

  // Decides robot a's next cell. parent: the robot that needs a's cell, or -1.
  bool push(size_t a, int32_t parent, Timestep t) {
    ++calls_;
    const int32_t here = cur_[a];
    // Nearest to the goal first, staying before moving, then a fixed hash.
    std::vector<std::tuple<int32_t, int32_t, uint64_t, int32_t>> options;
    for (const int32_t w : moves(a, t, true)) {
      if (w < 0) break;
      const int32_t d = distances_.distance(map_.cellAt(w), map_.cellAt(goal_[a]));
      const uint64_t salt =
          mix(tie_[a] ^ (static_cast<uint64_t>(t) << 32U) ^ static_cast<uint64_t>(w));
      options.emplace_back(d, w == here ? 0 : 1, salt, w);
    }
    std::sort(options.begin(), options.end());

    int32_t follower = -1;  // a trapped robot that leaves through `here` once a moves on
    for (const auto& option : options) {
      const int32_t w = std::get<3>(option);
      if (w == here && follower >= 0) continue;
      if (taken_[static_cast<size_t>(w)] >= 0) continue;
      if (parent >= 0 && w == cur_[static_cast<size_t>(parent)]) continue;  // no swap with it
      const int32_t b = now_[static_cast<size_t>(w)];
      const auto bu = static_cast<size_t>(b);
      if (b >= 0 && bu != a && next_[bu] == here) continue;  // no swap with a decided robot
      next_[a] = w;
      taken_[static_cast<size_t>(w)] = static_cast<int32_t>(a);
      if (b >= 0 && bu != a && next_[bu] < 0 && !push(bu, static_cast<int32_t>(a), t)) {
        if (follower < 0 && trapped(bu, here, t)) {
          // b holds w now and would keep a waiting at its only exit. Free b again: a steps
          // aside, and b comes out behind it.
          next_[bu] = -1;
          taken_[static_cast<size_t>(w)] = -1;
          follower = b;
        }
        continue;
      }
      if (follower >= 0 && taken_[static_cast<size_t>(here)] < 0) {
        next_[static_cast<size_t>(follower)] = here;
        taken_[static_cast<size_t>(here)] = follower;
      }
      return true;
    }
    next_[a] = here;
    taken_[static_cast<size_t>(here)] = static_cast<int32_t>(a);
    if (!fixed_.vertexFree(here, t + 1)) clash_ = true;
    return false;
  }

  bool decided(size_t a) const { return next_[a] >= 0; }
  int32_t next(size_t a) const { return next_[a]; }
  int64_t calls() const { return calls_; }
  bool clash() const { return clash_; }

 private:
  // Cells robot a may occupy at t + 1, ignoring the other planned robots: its cell (when
  // `stay`), and free neighbors that are not endpoints other than its goal, unless a fixed path
  // needs them. -1 pads the list.
  std::array<int32_t, 5> moves(size_t a, Timestep t, bool stay) const {
    std::array<int32_t, 5> out{-1, -1, -1, -1, -1};
    size_t count = 0;
    const int32_t here = cur_[a];
    if (stay && fixed_.vertexFree(here, t + 1)) out[count++] = here;
    for (const Cell& c : map_.neighbors(map_.cellAt(here))) {
      const int32_t w = map_.index(c);
      if (w != goal_[a] && map_.isEndpoint(c)) continue;
      if (!fixed_.vertexFree(w, t + 1) || !fixed_.edgeFree(here, w, t)) continue;
      out[count++] = w;
    }
    return out;
  }

  // Robot b, away from its goal, can only leave through `exit`.
  bool trapped(size_t b, int32_t exit, Timestep t) const {
    if (cur_[b] == goal_[b]) return false;
    const std::array<int32_t, 5> out = moves(b, t, false);
    return out[0] == exit && out[1] < 0;
  }

  const GridMap& map_;
  const DistanceOracle& distances_;
  const detail::ReservationTable& fixed_;
  const std::vector<int32_t>& goal_;
  const std::vector<uint64_t>& tie_;
  std::vector<int32_t>& cur_;
  std::vector<int32_t> next_;
  std::vector<int32_t> now_;    // cell -> robot there at t
  std::vector<int32_t> taken_;  // cell -> robot there at t + 1
  int64_t calls_{0};
  bool clash_{false};
};

PlanResult Pibt::plan(const PlanningRequest& request) {
  if (request.window < 1) throw std::invalid_argument("pibt: needs a window of at least 1 step");
  const GridMap& map = request.world.map();
  const size_t n = request.agents.size();
  const Timestep t0 = request.start;
  detail::ReservationTable fixed(map, t0);
  for (const Path& p : request.fixed) fixed.add(p);

  std::vector<int32_t> goal(n);
  std::vector<int32_t> cur(n);
  std::vector<Timestep> away(n);
  std::vector<uint64_t> tie(n);
  for (size_t a = 0; a < n; ++a) {
    const PlanAgent& agent = request.agents[a];
    Pursuit& p = pursuits_.try_emplace(agent.robot, Pursuit{agent.goal, t0, t0}).first->second;
    if (p.goal != agent.goal) p = Pursuit{agent.goal, t0, t0};
    if (agent.start == agent.goal) p.away_since = t0;
    goal[a] = map.index(agent.goal);
    cur[a] = map.index(agent.start);
    away[a] = p.away_since;
    tie[a] = mix((static_cast<uint64_t>(static_cast<uint32_t>(agent.robot)) << 32U) ^
                 static_cast<uint32_t>(p.since));
  }

  PlanResult result;
  std::vector<std::vector<int32_t>> cells(n);
  for (size_t a = 0; a < n; ++a) cells[a].push_back(cur[a]);
  std::vector<size_t> order(n);
  for (Timestep t = t0; t < t0 + request.window; ++t) {
    for (size_t a = 0; a < n; ++a) {
      if (cur[a] == goal[a]) away[a] = t;  // pushed off its goal later, it starts at the back
    }
    Step step(map, request.distances, fixed, goal, tie, cur);
    std::iota(order.begin(), order.end(), size_t{0});
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return std::make_tuple(cur[a] == goal[a], away[a], tie[a], request.agents[a].robot) <
             std::make_tuple(cur[b] == goal[b], away[b], tie[b], request.agents[b].robot);
    });
    for (const size_t a : order) {
      if (!step.decided(a)) step.push(a, -1, t);
    }
    result.expansions += step.calls();
    if (step.clash() || result.expansions > request.expansion_budget) {
      return PlanResult{false, {}, result.expansions, 0.0};
    }
    for (size_t a = 0; a < n; ++a) {
      cur[a] = step.next(a);
      cells[a].push_back(cur[a]);
    }
  }

  result.success = true;
  for (size_t a = 0; a < n; ++a) {
    detail::trimWaits(cells[a]);
    result.paths.push_back(detail::toPath(map, request.agents[a].robot, t0, cells[a]));
  }
  return result;
}

}  // namespace

std::unique_ptr<PathPlanner> makePibt() { return std::make_unique<Pibt>(); }

}  // namespace fleet_core

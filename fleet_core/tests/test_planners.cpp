#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/common/rng.hpp"
#include "fleet_core/planning/path_planner.hpp"
#include "fleet_core/world/map_generator.hpp"

namespace fleet_core {
namespace {

constexpr PlanningMethod kCoordinated[] = {PlanningMethod::kPrioritized,
                                           PlanningMethod::kWindowedCoopAStar, PlanningMethod::kCbs,
                                           PlanningMethod::kEcbs, PlanningMethod::kPibt};

std::unique_ptr<PathPlanner> planner(PlanningMethod method) {
  PlanningConfig config;
  config.method = method;
  return makePlanner(config);
}

PlanningRequest request(const WorldModel& world, const DistanceOracle& d,
                        std::vector<PlanAgent> agents, int32_t window, std::vector<Path> fixed = {},
                        int64_t budget = 200000) {
  return PlanningRequest{world, d, std::move(agents), std::move(fixed), 0, window, budget, 1.5};
}

int32_t sumOfCosts(const PlanResult& result) {
  int32_t total = 0;
  for (const Path& p : result.paths) total += static_cast<int32_t>(p.cells.size()) - 1;
  return total;
}

// Checks every promise of the PathPlanner contract except optimality. `until`: the last step
// checked for conflicts.
void expectValid(const GridMap& map, const PlanningRequest& req, const PlanResult& result,
                 bool windowed, const std::string& what) {
  ASSERT_EQ(result.paths.size(), req.agents.size()) << what;
  std::vector<Path> all = result.paths;
  Timestep last = req.start;
  for (size_t k = 0; k < result.paths.size(); ++k) {
    const Path& p = result.paths[k];
    const PlanAgent& a = req.agents[k];
    ASSERT_FALSE(p.cells.empty()) << what;
    EXPECT_EQ(p.robot, a.robot) << what;
    EXPECT_EQ(p.start, req.start) << what;
    EXPECT_EQ(p.cells.front(), a.start) << what;
    if (p.cells.size() >= 2) {
      EXPECT_NE(p.cells[p.cells.size() - 1], p.cells[p.cells.size() - 2])
          << what << ": trailing wait";
    }
    if (windowed) {
      EXPECT_LE(p.cells.size(), static_cast<size_t>(req.window) + 1) << what;
    } else {
      EXPECT_EQ(p.cells.back(), a.goal) << what << ": full-horizon path must end on its goal";
    }
    for (size_t i = 1; i < p.cells.size(); ++i) {
      const Cell& from = p.cells[i - 1];
      const Cell& to = p.cells[i];
      EXPECT_LE(std::abs(to.row - from.row) + std::abs(to.col - from.col), 1) << what;
      EXPECT_TRUE(map.isFree(to)) << what;
      if (to != from && map.isEndpoint(to)) {
        EXPECT_EQ(to, a.goal) << what << ": robot " << a.robot << " enters an endpoint";
      }
    }
    last = std::max(last, req.start + static_cast<Timestep>(p.cells.size()) - 1);
  }
  for (const Path& f : req.fixed) {
    all.push_back(f);
    last = std::max(last, f.start + static_cast<Timestep>(f.cells.size()) - 1);
  }
  const Timestep until = windowed ? req.start + req.window : last + 1;
  const auto conflict = firstConflict(all, req.start, until);
  EXPECT_FALSE(conflict.has_value())
      << what << ": robots " << all[conflict->first].robot << " and " << all[conflict->second].robot
      << " at step " << conflict->t << (conflict->swap ? " (swap)" : "");
}

// Optimal sum of costs by brute force, for tiny instances: Dijkstra over joint states with the
// planners' rules. A robot's cost is the step from which it stays on its goal; steps spent on
// the goal count only if it leaves again, so they are carried and charged on leaving.
// -1: no solution costs at most `max_cost`.
int32_t optimalSumOfCosts(const GridMap& map, const std::vector<Cell>& starts,
                          const std::vector<Cell>& goals, int32_t max_cost) {
  const size_t n = starts.size();
  using State = std::vector<int32_t>;  // positions, then carried goal steps
  State first;
  for (const Cell& c : starts) first.push_back(map.index(c));
  first.resize(2 * n, 0);
  std::map<State, int32_t> best{{first, 0}};
  using Entry = std::pair<int32_t, State>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  open.emplace(0, first);
  while (!open.empty()) {
    auto [cost, state] = open.top();
    open.pop();
    if (best[state] < cost) continue;
    bool done = true;
    for (size_t i = 0; i < n; ++i) done = done && state[i] == map.index(goals[i]);
    if (done) return cost;

    std::vector<std::vector<int32_t>> options(n);
    for (size_t i = 0; i < n; ++i) {
      options[i].push_back(state[i]);
      for (const Cell& c : map.neighbors(map.cellAt(state[i]))) {
        if (map.isEndpoint(c) && c != goals[i]) continue;
        options[i].push_back(map.index(c));
      }
    }
    std::vector<size_t> pick(n, 0);
    while (true) {
      State next = state;
      int32_t step_cost = 0;
      bool ok = true;
      for (size_t i = 0; i < n && ok; ++i) {
        next[i] = options[i][pick[i]];
        for (size_t j = 0; j < i && ok; ++j) {
          if (next[i] == next[j]) ok = false;
          if (next[i] == state[j] && next[j] == state[i] && next[i] != state[i]) ok = false;
        }
        const int32_t goal = map.index(goals[i]);
        if (next[i] == goal && state[i] == goal) {
          ++next[n + i];
        } else if (next[i] == goal) {
          step_cost += 1;
        } else {
          step_cost += state[n + i] + 1;
          next[n + i] = 0;
        }
      }
      if (ok && cost + step_cost <= max_cost) {
        const auto it = best.find(next);
        if (it == best.end() || it->second > cost + step_cost) {
          best[next] = cost + step_cost;
          open.emplace(cost + step_cost, next);
        }
      }
      size_t i = 0;
      while (i < n && ++pick[i] == options[i].size()) pick[i++] = 0;
      if (i == n) break;
    }
  }
  return -1;
}

TEST(PlannerFactory, EveryMethodBuildsAndNamesItself) {
  for (const PlanningMethod m : {PlanningMethod::kIndependentAStar, PlanningMethod::kPrioritized,
                                 PlanningMethod::kWindowedCoopAStar, PlanningMethod::kCbs,
                                 PlanningMethod::kEcbs, PlanningMethod::kPibt}) {
    EXPECT_EQ(planner(m)->name(), toString(m));
    EXPECT_EQ(isCoordinated(m), m != PlanningMethod::kIndependentAStar);
  }
}

TEST(PathConflicts, VertexSwapAndFollowing) {
  const Path a{0, 0, {{0, 0}, {0, 1}, {0, 2}}};
  const Path follower{1, 0, {{0, 1}, {0, 2}, {0, 3}}};  // `a` enters each cell as it leaves
  EXPECT_FALSE(firstConflict({a, follower}, 0, 10).has_value());
  const Path head_on{1, 0, {{0, 2}, {0, 1}, {0, 0}}};
  const auto swap = firstConflict({a, head_on}, 0, 10);
  ASSERT_TRUE(swap.has_value());
  EXPECT_EQ(swap->t, 1);
  EXPECT_EQ(swap->cell, (Cell{0, 1}));
  EXPECT_FALSE(swap->swap);           // both reach (0, 1) at step 1: a vertex conflict comes first
  const Path parked{1, 0, {{0, 2}}};  // holds (0, 2) forever
  const auto vertex = firstConflict({a, parked}, 0, 10);
  ASSERT_TRUE(vertex.has_value());
  EXPECT_EQ(vertex->t, 2);
  EXPECT_EQ(positionAt(a, 7), (Cell{0, 2}));
  EXPECT_EQ(positionAt(a, -3), (Cell{0, 0}));
  const Path b{1, 0, {{0, 1}, {0, 0}}};
  const auto pure_swap = firstConflict({Path{0, 0, {{0, 0}, {0, 1}}}, b}, 0, 10);
  ASSERT_TRUE(pure_swap.has_value());
  EXPECT_TRUE(pure_swap->swap);
  EXPECT_EQ(pure_swap->t, 1);
}

// Random instances on a generated warehouse: every coordinated planner returns valid,
// conflict-free paths, around fixed paths too, in rolling (windowed) and full-horizon mode.
TEST(CoordinatedPlanners, ValidAndConflictFreeOnRandomInstances) {
  const GridMap map = generateWarehouse(MapGeneratorConfig{3, 2, 4, 2, 2, 1});
  const WorldModel world(map, {});
  const DistanceOracle d(map, true);
  std::vector<Cell> aisles;
  std::vector<Cell> ends;
  for (int32_t i = 0; i < map.cellCount(); ++i) {
    const Cell c = map.cellAt(i);
    if (!map.isStaticFree(c)) continue;
    (map.isEndpoint(c) ? ends : aisles).push_back(c);
  }
  Rng rng(41);
  const auto draw = [&](std::vector<Cell>& pool) {
    const auto k = static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(pool.size()) - 1));
    const Cell c = pool[k];
    pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(k));
    return c;
  };
  int solved = 0;
  int attempts = 0;
  for (int trial = 0; trial < 40; ++trial) {
    std::vector<Cell> starts = aisles;
    starts.insert(starts.end(), ends.begin(), ends.end());
    std::vector<Cell> goals = ends;  // goals are endpoints, as in the fleet
    // Two robots with committed paths: one parked on an endpoint, one on its way.
    const Cell parked = draw(goals);
    starts.erase(std::remove(starts.begin(), starts.end(), parked), starts.end());
    const auto agents = static_cast<int32_t>(rng.uniformInt(2, 8));
    std::vector<PlanAgent> list;
    for (int32_t r = 0; r < agents; ++r) {
      const Cell start = draw(starts);
      const Cell goal = draw(goals);
      list.push_back(PlanAgent{r, start, goal, static_cast<int32_t>(rng.uniformInt(-1, 50))});
    }
    std::vector<Path> fixed{Path{agents, 0, {parked}}};
    {
      const Cell from = draw(starts);
      const Cell to = draw(goals);
      const PlanResult lone =
          planner(PlanningMethod::kPrioritized)
              ->plan(request(world, d, {PlanAgent{agents + 1, from, to, 0}}, 0, {fixed[0]}));
      if (lone.success) fixed.push_back(lone.paths[0]);
    }
    for (const PlanningMethod m : kCoordinated) {
      for (const int32_t window : {0, 6}) {
        if (window == 0 && (m == PlanningMethod::kPibt)) continue;  // pibt needs a window
        const PlanningRequest req = request(world, d, list, window, fixed);
        const PlanResult result = planner(m)->plan(req);
        ++attempts;
        if (!result.success) {
          EXPECT_TRUE(result.paths.empty());
          continue;
        }
        ++solved;
        const bool windowed = window > 0 && m != PlanningMethod::kPrioritized;
        expectValid(map, req, result, windowed,
                    std::string(toString(m)) + " window " + std::to_string(window) + " trial " +
                        std::to_string(trial));
      }
    }
  }
  EXPECT_GE(solved, attempts * 95 / 100);  // the instances are easy; failures would be a bug
}

// Tiny maps where robots must pass each other: CBS matches the brute-force optimum, ECBS stays
// within its bound, prioritized planning never beats the optimum.
TEST(CoordinatedPlanners, OptimalityAgainstBruteForce) {
  const std::vector<std::string> maps = {
      "R..R\n"
      ".#..\n"
      "P..D\n",
      ".....\n"
      "#.#.#\n"
      ".....\n",
      "R...\n"
      ".##.\n"
      "....\n",
  };
  Rng rng(7);
  int compared = 0;
  for (const std::string& text : maps) {
    const GridMap map = GridMap::fromString(text);
    const WorldModel world(map, {});
    const DistanceOracle d(map, true);
    std::vector<Cell> free;
    for (int32_t i = 0; i < map.cellCount(); ++i) {
      if (map.isStaticFree(map.cellAt(i))) free.push_back(map.cellAt(i));
    }
    for (int trial = 0; trial < 40; ++trial) {
      const auto n = static_cast<size_t>(rng.uniformInt(2, 3));
      std::vector<Cell> starts = free;
      std::vector<Cell> goals = free;
      std::vector<PlanAgent> agents;
      std::vector<Cell> s;
      std::vector<Cell> g;
      for (size_t r = 0; r < n; ++r) {
        const auto a =
            static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(starts.size()) - 1));
        const auto b =
            static_cast<size_t>(rng.uniformInt(0, static_cast<int64_t>(goals.size()) - 1));
        s.push_back(starts[a]);
        g.push_back(goals[b]);
        starts.erase(starts.begin() + static_cast<std::ptrdiff_t>(a));
        goals.erase(goals.begin() + static_cast<std::ptrdiff_t>(b));
        agents.push_back(PlanAgent{static_cast<RobotId>(r), s.back(), g.back(), 0});
      }
      const int32_t optimum = optimalSumOfCosts(map, s, g, 40);
      // CBS searches unsolvable instances until the budget runs out, so start small, and give
      // the few hard solvable ones (two robots trading places in a pocket) a large budget.
      PlanningRequest req = request(world, d, agents, 0, {}, 20000);
      PlanResult cbs = planner(PlanningMethod::kCbs)->plan(req);
      PlanResult ecbs = planner(PlanningMethod::kEcbs)->plan(req);
      const PlanResult pp = planner(PlanningMethod::kPrioritized)->plan(req);
      if (optimum >= 0 && (!cbs.success || !ecbs.success)) {
        req.expansion_budget = 5000000;
        cbs = planner(PlanningMethod::kCbs)->plan(req);
        ecbs = planner(PlanningMethod::kEcbs)->plan(req);
      }
      const std::string what =
          "map " + std::to_string(&text - maps.data()) + " trial " + std::to_string(trial);
      if (optimum < 0) {
        EXPECT_FALSE(cbs.success) << what;
        EXPECT_FALSE(ecbs.success) << what;
        EXPECT_FALSE(pp.success) << what;
        continue;
      }
      ++compared;
      ASSERT_TRUE(cbs.success) << what;
      ASSERT_TRUE(ecbs.success) << what;
      expectValid(map, req, cbs, false, "cbs " + what);
      expectValid(map, req, ecbs, false, "ecbs " + what);
      EXPECT_EQ(sumOfCosts(cbs), optimum) << what;
      EXPECT_GE(sumOfCosts(ecbs), optimum) << what;
      EXPECT_LE(sumOfCosts(ecbs), static_cast<int32_t>(1.5 * optimum + 1e-9)) << what;
      if (pp.success) {
        expectValid(map, req, pp, false, "prioritized " + what);
        EXPECT_GE(sumOfCosts(pp), optimum) << what;
      }
    }
  }
  EXPECT_GE(compared, 80);
}

// A robot in a parking pocket whose only exit is held by a robot that wants in. Prioritized
// planning fails with the robot outside first, restarts with the parked one first, and solves
// it. PIBT lets the robot outside step aside so the parked one comes out.
TEST(CoordinatedPlanners, ParkedRobotLeavesBeforeTheOtherEnters) {
  const GridMap map = GridMap::fromString(
      "#R#\n"
      "...\n"
      "#.#\n");
  const WorldModel world(map, {});
  const DistanceOracle d(map, true);
  // Robot 0 waits at the pocket's exit and wants in; robot 1 is inside and wants out.
  const std::vector<PlanAgent> agents{PlanAgent{0, {1, 1}, {0, 1}, 9},
                                      PlanAgent{1, {0, 1}, {2, 1}, 0}};
  for (const PlanningMethod m : kCoordinated) {
    const PlanningRequest req = request(world, d, agents, m == PlanningMethod::kPibt ? 8 : 0);
    const PlanResult result = planner(m)->plan(req);
    ASSERT_TRUE(result.success) << toString(m);
    const bool windowed = m == PlanningMethod::kPibt;
    expectValid(map, req, result, windowed, std::string(toString(m)));
    EXPECT_EQ(result.paths[0].cells.back(), (Cell{0, 1})) << toString(m);
    EXPECT_EQ(result.paths[1].cells.back(), (Cell{2, 1})) << toString(m);
    if (m == PlanningMethod::kCbs) {
      EXPECT_EQ(sumOfCosts(result), 5);  // optimum: 2 + 3
    }
  }
}

// PIBT ranks robots by how long they have pursued their goal, not by id: two robots want the
// middle of a junction at once, and the one whose goal is older gets it. The planner remembers
// when each goal was set across calls.
TEST(Pibt, OlderGoalGetsTheContestedCell) {
  const GridMap map = GridMap::fromString(
      "#.#\n"
      "...\n"
      "#.#\n");
  const WorldModel world(map, {});
  const DistanceOracle d(map, true);
  const PlanAgent across{0, {1, 0}, {1, 2}, 0};
  const PlanAgent down{1, {0, 1}, {2, 1}, 0};
  for (const RobotId older : {0, 1}) {
    const auto pibt = planner(PlanningMethod::kPibt);
    PlanningRequest early = request(world, d, {older == 0 ? across : down}, 1);
    ASSERT_TRUE(pibt->plan(early).success);  // the older robot's goal is set at step 0
    PlanningRequest both = request(world, d, {across, down}, 1);
    both.start = 4;  // the other robot's goal is set at step 4
    const PlanResult result = pibt->plan(both);
    ASSERT_TRUE(result.success);
    expectValid(map, both, result, true, "pibt");
    const auto o = static_cast<size_t>(older);
    EXPECT_EQ(result.paths[o].cells, (std::vector<Cell>{both.agents[o].start, {1, 1}}));
    EXPECT_EQ(result.paths[1 - o].cells, (std::vector<Cell>{both.agents[1 - o].start}));
  }
}

// A robot pushed off its goal ranks behind the robots that have been away from theirs longer, so
// it does not shove its way back at once: R1, parked on its goal in the aisle, steps into the
// side cell, lets R0 pass, and returns.
TEST(Pibt, RobotPushedOffItsGoalGoesBehind) {
  const GridMap map = GridMap::fromString(
      "###.###\n"
      ".......\n"
      "###.###\n");
  const WorldModel world(map, {});
  const DistanceOracle d(map, true);
  const PlanningRequest req =
      request(world, d, {PlanAgent{0, {1, 0}, {1, 6}, 0}, PlanAgent{1, {1, 2}, {1, 2}, 0}}, 8);
  const PlanResult result = planner(PlanningMethod::kPibt)->plan(req);
  ASSERT_TRUE(result.success);
  expectValid(map, req, result, true, "pibt");
  EXPECT_EQ(result.paths[0].cells.back(), (Cell{1, 6}));
  EXPECT_EQ(result.paths[1].cells.back(), (Cell{1, 2}));
  EXPECT_NE(std::find(result.paths[1].cells.begin(), result.paths[1].cells.end(), Cell{0, 3}),
            result.paths[1].cells.end());
}

TEST(Pibt, NeedsAWindow) {
  const GridMap map = GridMap::fromString("...\n");
  const WorldModel world(map, {});
  const DistanceOracle d(map, true);
  EXPECT_THROW(
      planner(PlanningMethod::kPibt)->plan(request(world, d, {PlanAgent{0, {0, 0}, {0, 2}, 0}}, 0)),
      std::invalid_argument);
}

// Every planner stops when the expansion budget runs out, with no paths, and spends the same
// effort on the same request.
TEST(CoordinatedPlanners, BudgetAndDeterminism) {
  const GridMap map = generateWarehouse(MapGeneratorConfig{3, 2, 4, 2, 2, 1});
  const WorldModel world(map, {});
  const DistanceOracle d(map, true);
  std::vector<PlanAgent> agents;
  for (int32_t r = 0; r < 6; ++r) {
    agents.push_back(PlanAgent{r, map.parking()[static_cast<size_t>(r)],
                               map.parking()[map.parking().size() - 1 - static_cast<size_t>(r)],
                               r});
  }
  for (const PlanningMethod m : kCoordinated) {
    PlanningRequest req = request(world, d, agents, 10);
    const PlanResult a = planner(m)->plan(req);
    const PlanResult b = planner(m)->plan(req);
    ASSERT_TRUE(a.success) << toString(m);
    EXPECT_EQ(a.expansions, b.expansions) << toString(m);
    for (size_t k = 0; k < a.paths.size(); ++k) EXPECT_EQ(a.paths[k].cells, b.paths[k].cells);
    req.expansion_budget = 3;
    const PlanResult starved = planner(m)->plan(req);
    EXPECT_FALSE(starved.success) << toString(m);
    EXPECT_TRUE(starved.paths.empty()) << toString(m);
  }
}

}  // namespace
}  // namespace fleet_core

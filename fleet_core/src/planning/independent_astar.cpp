#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "planners.hpp"

namespace fleet_core {
namespace {

// Uncoordinated baseline: every robot gets its own shortest path and ignores the others.
// Spatial A* on the current map (dynamic blocks included) with the distance oracle as the
// heuristic. Like every planner it never passes through a station or parking cell other than
// its start and goal. Paths have no waits; collisions are left to robot safety.
class IndependentAStar final : public PathPlanner {
 public:
  std::string_view name() const override { return "independent_astar"; }

  PlanResult plan(const PlanningRequest& request) override {
    PlanResult result;
    result.success = true;
    for (const PlanAgent& agent : request.agents) {
      std::vector<Cell> cells;
      const int64_t budget = request.expansion_budget - result.expansions;
      if (!search(request, agent, budget, cells, result.expansions)) {
        result.success = false;
        result.paths.clear();
        return result;
      }
      result.paths.push_back(Path{agent.robot, request.start, std::move(cells)});
    }
    return result;
  }

 private:
  // Ties on f go to the smaller h (deeper node), then the smaller cell index: deterministic.
  using Entry = std::tuple<int32_t, int32_t, int32_t>;  // f, h, cell index

  static bool search(const PlanningRequest& request, const PlanAgent& agent, int64_t budget,
                     std::vector<Cell>& out, int64_t& expansions) {
    const GridMap& map = request.world.map();
    if (agent.start == agent.goal) {
      out = {agent.start};
      return true;
    }
    if (!map.inBounds(agent.start) || !map.isFree(agent.goal)) return false;

    const auto n = static_cast<size_t>(map.cellCount());
    std::vector<int32_t> g(n, std::numeric_limits<int32_t>::max());
    std::vector<int32_t> parent(n, -1);
    std::vector<uint8_t> closed(n, 0);
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    const int32_t start = map.index(agent.start);
    const int32_t goal = map.index(agent.goal);
    const int32_t h0 = request.distances.distance(agent.start, agent.goal);
    g[static_cast<size_t>(start)] = 0;
    open.emplace(h0, h0, start);

    int64_t expanded = 0;
    while (!open.empty()) {
      const int32_t v = std::get<2>(open.top());
      open.pop();
      if (closed[static_cast<size_t>(v)] != 0) continue;
      closed[static_cast<size_t>(v)] = 1;
      if (++expanded > budget) {
        expansions += expanded;
        return false;
      }
      if (v == goal) {
        expansions += expanded;
        for (int32_t c = goal; c != -1; c = parent[static_cast<size_t>(c)]) {
          out.push_back(map.cellAt(c));
        }
        std::reverse(out.begin(), out.end());
        return true;
      }
      const int32_t next_g = g[static_cast<size_t>(v)] + 1;
      for (const Cell& next : map.neighbors(map.cellAt(v))) {
        const auto w = static_cast<size_t>(map.index(next));
        if (closed[w] != 0 || next_g >= g[w]) continue;
        if (map.isEndpoint(next) && next != agent.goal) continue;  // no passing through
        g[w] = next_g;
        parent[w] = v;
        const int32_t h = request.distances.distance(next, agent.goal);
        open.emplace(next_g + h, h, map.index(next));
      }
    }
    expansions += expanded;
    return false;  // goal unreachable
  }
};

}  // namespace

std::unique_ptr<PathPlanner> makeIndependentAStar() { return std::make_unique<IndependentAStar>(); }

}  // namespace fleet_core

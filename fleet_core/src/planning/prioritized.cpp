#include <algorithm>
#include <numeric>
#include <string_view>
#include <utility>
#include <vector>

#include "planners.hpp"
#include "space_time.hpp"

namespace fleet_core {
namespace {

// Prioritized planning: robots plan one at a time, oldest order first, each with space-time A*
// around the paths of the robots before it and the fixed paths.
//
// prioritized plans every robot all the way to its goal, where it then holds. windowed_coop_astar
// (WHCA*) resolves only the first `window` steps and relies on replanning before then; past the
// window it follows the distance oracle, the true distance on the static map.
//
// A bad order can leave a robot with no path at all. Then the planner restarts with that robot
// moved to the front, up to once per robot in all, before it gives up.
class Prioritized final : public PathPlanner {
 public:
  explicit Prioritized(bool windowed) : windowed_(windowed) {}

  std::string_view name() const override {
    return windowed_ ? "windowed_coop_astar" : "prioritized";
  }

  PlanResult plan(const PlanningRequest& request) override {
    PlanResult result;
    const GridMap& map = request.world.map();
    const size_t n = request.agents.size();
    std::vector<size_t> order(n);
    std::iota(order.begin(), order.end(), size_t{0});
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      const PlanAgent& x = request.agents[a];
      const PlanAgent& y = request.agents[b];
      return x.priority != y.priority ? x.priority > y.priority : x.robot < y.robot;
    });
    const Timestep horizon =
        windowed_ && request.window > 0 ? request.start + request.window : detail::kForever;

    for (size_t round = 0; round <= n; ++round) {
      detail::ReservationTable table(map, request.start);
      for (const Path& p : request.fixed) table.add(p);
      std::vector<std::vector<int32_t>> cells(n);
      size_t failed = n;
      for (size_t k = 0; k < n; ++k) {
        const PlanAgent& agent = request.agents[order[k]];
        const detail::SearchInput in{map,
                                     request.distances,
                                     map.index(agent.start),
                                     map.index(agent.goal),
                                     request.start,
                                     horizon,
                                     table};
        detail::SearchOutput out =
            detail::spaceTimeSearch(in, request.expansion_budget - result.expansions);
        result.expansions += out.expansions;
        if (out.out_of_budget) return PlanResult{false, {}, result.expansions, 0.0};
        if (!out.found) {
          failed = k;
          break;
        }
        table.add(detail::toPath(map, agent.robot, request.start, out.cells));
        cells[order[k]] = std::move(out.cells);
      }
      if (failed == n) {
        result.success = true;
        for (size_t a = 0; a < n; ++a) {
          result.paths.push_back(
              detail::toPath(map, request.agents[a].robot, request.start, cells[a]));
        }
        return result;
      }
      if (failed == 0) break;  // first in line and still no path: no order helps
      const size_t stuck = order[failed];
      order.erase(order.begin() + static_cast<std::ptrdiff_t>(failed));
      order.insert(order.begin(), stuck);
    }
    return PlanResult{false, {}, result.expansions, 0.0};
  }

 private:
  bool windowed_;
};

}  // namespace

std::unique_ptr<PathPlanner> makePrioritized(bool windowed) {
  return std::make_unique<Prioritized>(windowed);
}

}  // namespace fleet_core

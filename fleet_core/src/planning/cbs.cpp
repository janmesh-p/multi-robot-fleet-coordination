#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "planners.hpp"
#include "space_time.hpp"

namespace fleet_core {
namespace {

// Conflict-Based Search (Sharon et al. 2015) and its bounded-suboptimal variant ECBS (Barer et
// al. 2014), over the window as in rolling-horizon planning (Li et al. 2021).
//
// The high level keeps a tree of constraint sets. Each node plans every robot alone under its
// constraints, finds the earliest conflict between two robots, and branches: in one child the
// first robot may not be there then, in the other the second. CBS expands the cheapest node, so
// the first conflict-free node has the least total cost. ECBS keeps a lower bound on that cost
// and expands, among nodes within `suboptimality` times the bound, the one with the fewest
// conflicts; its low level does the same per robot. Fixed paths are hard obstacles throughout.
class Cbs final : public PathPlanner {
 public:
  explicit Cbs(bool bounded) : bounded_(bounded) {}

  std::string_view name() const override { return bounded_ ? "ecbs" : "cbs"; }

  PlanResult plan(const PlanningRequest& request) override;

 private:
  struct Conflict {
    size_t a{0};
    size_t b{0};
    Timestep t{0};  // the step a vertex conflict happens at, or a swap ends at
    bool swap{false};
  };

  struct Node {
    std::vector<detail::Constraints> constraints;
    std::vector<std::vector<int32_t>> cells;  // positions from the request's start
    std::vector<int32_t> cost;
    std::vector<int32_t> bound;  // per robot: lower bound on its cost under its constraints
    int64_t total{0};
    int64_t lower{0};
    int32_t conflicts{0};  // pairs of robots in conflict
  };

  bool bounded_;
};

int32_t at(const std::vector<int32_t>& cells, Timestep k) {
  return cells[std::min(static_cast<size_t>(k), cells.size() - 1)];
}

PlanResult Cbs::plan(const PlanningRequest& request) {
  PlanResult result;
  const GridMap& map = request.world.map();
  const size_t n = request.agents.size();
  const Timestep t0 = request.start;
  const Timestep horizon = request.window > 0 ? t0 + request.window : detail::kForever;
  const double w = bounded_ ? std::max(1.0, request.suboptimality) : 1.0;
  detail::ReservationTable table(map, t0);
  for (const Path& p : request.fixed) table.add(p);
  bool out_of_budget = false;

  // Steps checked for conflicts: to the window's end, or until every robot holds its goal.
  const auto lastStep = [&](const Node& node) {
    if (horizon != detail::kForever) return horizon;
    Timestep last = t0;
    for (const auto& c : node.cells)
      last = std::max(last, t0 + static_cast<Timestep>(c.size()) - 1);
    return last;
  };
  // Earliest conflict (vertex before swap at the same step, then the lower pair) and the number
  // of robot pairs in conflict.
  const auto inspect = [&](const Node& node, Conflict* first) {
    const Timestep last = lastStep(node);
    std::vector<uint8_t> paired(n * n, 0);
    int32_t pairs = 0;
    bool found = false;
    for (Timestep t = t0 + 1; t <= last; ++t) {
      const Timestep k = t - t0;
      for (size_t i = 0; i < n; ++i) {
        const int32_t ai = at(node.cells[i], k);
        const int32_t pi = at(node.cells[i], k - 1);
        for (size_t j = i + 1; j < n; ++j) {
          const int32_t aj = at(node.cells[j], k);
          const bool vertex = ai == aj;
          const bool swap = !vertex && ai != pi && ai == at(node.cells[j], k - 1) && aj == pi;
          if (!vertex && !swap) continue;
          if (!found && first != nullptr) {
            *first = Conflict{i, j, t, swap};
            found = true;
          }
          if (paired[i * n + j] == 0) {
            paired[i * n + j] = 1;
            ++pairs;
          }
        }
      }
      if (found && first != nullptr && !bounded_) break;  // CBS needs only the first
    }
    return pairs;
  };
  // Plans robot `a` of `node` under its constraints. ECBS counts conflicts with the others.
  const auto replan = [&](Node& node, size_t a, bool has_bound) {
    const PlanAgent& agent = request.agents[a];
    detail::SearchInput in{
        map, request.distances, map.index(agent.start), map.index(agent.goal), t0, horizon, table};
    in.constraints = &node.constraints[a];
    in.focal = w;
    std::optional<detail::ConflictCounter> soft;
    if (bounded_) {
      Timestep until = horizon;
      if (until == detail::kForever) {
        until = t0;
        for (size_t b = 0; b < n; ++b) {
          if (b != a) until = std::max(until, t0 + static_cast<Timestep>(node.cells[b].size()));
        }
      }
      soft.emplace(map, t0, until);
      for (size_t b = 0; b < n; ++b) {
        if (b != a) soft->add(node.cells[b]);
      }
      in.soft = &*soft;
    }
    detail::SearchOutput out =
        detail::spaceTimeSearch(in, request.expansion_budget - result.expansions);
    result.expansions += out.expansions;
    if (out.out_of_budget) out_of_budget = true;
    if (!out.found) return false;
    node.cells[a] = std::move(out.cells);
    node.cost[a] = out.cost;
    node.bound[a] = has_bound ? std::max(node.bound[a], out.lower_bound) : out.lower_bound;
    return true;
  };
  const auto totals = [&](Node& node) {
    node.total = 0;
    node.lower = 0;
    for (size_t a = 0; a < n; ++a) {
      node.total += node.cost[a];
      node.lower += node.bound[a];
    }
    node.conflicts = inspect(node, nullptr);
  };

  std::vector<Node> nodes(1);
  Node& root = nodes[0];
  root.constraints.resize(n);
  root.cells.resize(n);
  root.cost.assign(n, 0);
  root.bound.assign(n, 0);
  for (size_t a = 0; a < n; ++a) {
    root.cells[a] = {map.index(request.agents[a].start)};  // seen by the others while unplanned
  }
  for (size_t a = 0; a < n; ++a) {
    if (!replan(root, a, false)) return PlanResult{false, {}, result.expansions, 0.0};
  }
  totals(root);

  // CBS: cheapest first, then fewer conflicts, then creation order. ECBS: OPEN by lower bound,
  // FOCAL (cost within w times the best bound) by conflicts, then cost, then creation order.
  std::set<std::tuple<int64_t, int32_t, size_t>> open_cbs;
  std::set<std::pair<int64_t, size_t>> open_lb;
  std::set<std::tuple<int32_t, int64_t, size_t>> focal;
  std::vector<uint8_t> in_focal;
  int64_t best_lb = 0;
  const auto eligible = [&](const Node& node) {
    return static_cast<double>(node.total) <= w * static_cast<double>(best_lb) + 1e-9;
  };
  const auto push = [&](size_t index) {
    const Node& node = nodes[index];
    in_focal.push_back(0);
    if (!bounded_) {
      open_cbs.emplace(node.total, node.conflicts, index);
      return;
    }
    open_lb.emplace(node.lower, index);
    if (eligible(node)) {
      focal.emplace(node.conflicts, node.total, index);
      in_focal[index] = 1;
    }
  };
  best_lb = root.lower;
  push(0);

  while (bounded_ ? !open_lb.empty() : !open_cbs.empty()) {
    size_t index = 0;
    if (!bounded_) {
      index = std::get<2>(*open_cbs.begin());
      open_cbs.erase(open_cbs.begin());
    } else {
      const int64_t lb = open_lb.begin()->first;
      if (lb > best_lb) {  // the bound rose: more nodes qualify for FOCAL
        best_lb = lb;
        for (const auto& [lower, i] : open_lb) {
          (void)lower;
          if (in_focal[i] == 0 && eligible(nodes[i])) {
            focal.emplace(nodes[i].conflicts, nodes[i].total, i);
            in_focal[i] = 1;
          }
        }
      }
      index = std::get<2>(*focal.begin());
      focal.erase(focal.begin());
      open_lb.erase({nodes[index].lower, index});
    }

    Conflict conflict;
    if (nodes[index].conflicts == 0 || inspect(nodes[index], &conflict) == 0) {
      result.success = true;
      for (size_t a = 0; a < n; ++a) {
        result.paths.push_back(
            detail::toPath(map, request.agents[a].robot, t0, nodes[index].cells[a]));
      }
      return result;
    }
    if (++result.expansions > request.expansion_budget) break;

    for (const size_t side : {conflict.a, conflict.b}) {
      Node child = nodes[index];
      const std::vector<int32_t>& cells = child.cells[side];
      const Timestep k = conflict.t - t0;
      if (conflict.swap) {
        child.constraints[side].addEdge(at(cells, k - 1), at(cells, k), conflict.t - 1);
      } else {
        child.constraints[side].addVertex(at(cells, k), conflict.t);
      }
      if (!replan(child, side, true)) {
        if (out_of_budget) return PlanResult{false, {}, result.expansions, 0.0};
        continue;
      }
      totals(child);
      nodes.push_back(std::move(child));
      push(nodes.size() - 1);
    }
  }
  return PlanResult{false, {}, result.expansions, 0.0};
}

}  // namespace

std::unique_ptr<PathPlanner> makeCbs(bool bounded) { return std::make_unique<Cbs>(bounded); }

}  // namespace fleet_core

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "allocators.hpp"
#include "fleet_core/allocation/assignment_solver.hpp"
#include "fleet_core/allocation/cost_model.hpp"
#include "fleet_core/allocation/pair_features.hpp"

namespace fleet_core {
namespace {

// Cost of a pair the robot cannot reach. Far above any real total (100 robots at
// CostModel::kMaxCost each is 1e7), so the solver only pairs it when nothing else is left, and
// such pairs are then dropped.
constexpr double kForbidden = 1e9;

// hungarian, cost_aware, and learned: one minimum-cost matching of every available robot with
// every pending order. They differ only in the cost of a pair.
//
// Every pending order is a candidate, not just the oldest ones, so a cost that knows about busy
// stations can steer robots to orders elsewhere. Columns are pending orders oldest first, and the
// solver breaks ties toward lower columns, so between two orders that cost the same the older
// one usually wins.
//
// With the starvation guard, orders pending longer than kStarvationSteps form a prefix of the
// columns. The oldest of them, up to one per robot, are matched first, with the same costs; the
// robots left over then match the orders left over.
class MatchingAllocator final : public TaskAllocator {
 public:
  // cost(table, robot, task, distance); table is empty when needs_features is false.
  using PairCost = std::function<double(const PairFeatureTable*, RobotId, TaskId, int32_t)>;

  MatchingAllocator(std::string name, bool needs_features, bool starvation_guard, PairCost cost)
      : name_(std::move(name)),
        needs_features_(needs_features),
        guard_(starvation_guard),
        cost_(std::move(cost)) {}

  std::string_view name() const override { return name_; }

  std::vector<Assignment> assign(const AllocationContext& context) override {
    const size_t n = context.robots.size();
    const size_t m = context.pending.size();
    if (n == 0 || m == 0) return {};
    std::optional<PairFeatureTable> table;
    if (needs_features_) table.emplace(context);

    std::vector<double> cost(n * m);
    std::vector<char> forbidden(n * m, 0);
    for (size_t i = 0; i < n; ++i) {
      const Cell& at = context.world.robot(context.robots[i]).cell;
      for (size_t j = 0; j < m; ++j) {
        const Cell& pickup = context.tasks.get(context.pending[j]).order.pickup;
        const int32_t d = context.distances.distance(at, pickup);
        if (d >= DistanceOracle::kUnreachable) {
          cost[i * m + j] = kForbidden;
          forbidden[i * m + j] = 1;
          continue;
        }
        cost[i * m + j] =
            cost_(table ? &*table : nullptr, context.robots[i], context.pending[j], d);
      }
    }

    std::vector<size_t> rows(n);
    for (size_t i = 0; i < n; ++i) rows[i] = i;
    std::vector<size_t> columns(m);
    for (size_t j = 0; j < m; ++j) columns[j] = j;
    std::vector<Assignment> out;
    if (guard_) {
      const Timestep now = context.world.now();
      size_t starving = 0;
      while (starving < m && starving < n &&
             now - context.tasks.get(context.pending[starving]).created_at > kStarvationSteps) {
        ++starving;
      }
      if (starving > 0) {
        const std::vector<size_t> oldest(columns.begin(),
                                         columns.begin() + static_cast<std::ptrdiff_t>(starving));
        std::vector<uint8_t> row_used(n, 0);
        std::vector<uint8_t> column_used(m, 0);
        for (const auto& [i, j] : match(cost, forbidden, m, rows, oldest)) {
          out.push_back(Assignment{context.robots[i], context.pending[j]});
          row_used[i] = 1;
          column_used[j] = 1;
        }
        rows.erase(std::remove_if(rows.begin(), rows.end(), [&](size_t i) { return row_used[i]; }),
                   rows.end());
        columns.erase(std::remove_if(columns.begin(), columns.end(),
                                     [&](size_t j) { return column_used[j]; }),
                      columns.end());
      }
    }
    for (const auto& [i, j] : match(cost, forbidden, m, rows, columns)) {
      out.push_back(Assignment{context.robots[i], context.pending[j]});
    }
    return out;
  }

 private:
  // Minimum-cost matching of the given rows and columns of the full n x m table. Pairs the robot
  // cannot reach are dropped.
  static std::vector<std::pair<size_t, size_t>> match(const std::vector<double>& cost,
                                                      const std::vector<char>& forbidden, size_t m,
                                                      const std::vector<size_t>& rows,
                                                      const std::vector<size_t>& columns) {
    std::vector<std::pair<size_t, size_t>> out;
    if (rows.empty() || columns.empty()) return out;
    std::vector<double> sub(rows.size() * columns.size());
    for (size_t a = 0; a < rows.size(); ++a) {
      for (size_t b = 0; b < columns.size(); ++b) {
        sub[a * columns.size() + b] = cost[rows[a] * m + columns[b]];
      }
    }
    const std::vector<int32_t> column_of = solveAssignment(sub, static_cast<int32_t>(rows.size()),
                                                           static_cast<int32_t>(columns.size()));
    for (size_t a = 0; a < rows.size(); ++a) {
      if (column_of[a] < 0) continue;
      const size_t i = rows[a];
      const size_t j = columns[static_cast<size_t>(column_of[a])];
      if (forbidden[i * m + j] != 0) continue;
      out.emplace_back(i, j);
    }
    return out;
  }

  std::string name_;
  bool needs_features_;
  bool guard_;
  PairCost cost_;
};

}  // namespace

std::unique_ptr<TaskAllocator> makeHungarianAllocator(bool starvation_guard) {
  return std::make_unique<MatchingAllocator>(
      "hungarian", false, starvation_guard,
      [](const PairFeatureTable*, RobotId, TaskId, int32_t d) { return static_cast<double>(d); });
}

std::unique_ptr<TaskAllocator> makeCostAwareAllocator(double weight, bool congestion_term,
                                                      Timestep service_steps,
                                                      bool starvation_guard) {
  if (!congestion_term) {
    return std::make_unique<MatchingAllocator>(
        "cost_aware", false, starvation_guard,
        [](const PairFeatureTable*, RobotId, TaskId, int32_t d) { return static_cast<double>(d); });
  }
  return std::make_unique<MatchingAllocator>(
      "cost_aware", true, starvation_guard,
      [weight, service_steps](const PairFeatureTable* table, RobotId robot, TaskId task,
                              int32_t d) {
        const PairFeatures f = table->pair(robot, task);
        const double penalty = static_cast<double>(f.robots_in_box) +
                               static_cast<double>(service_steps) * f.pickup_assigned;
        return static_cast<double>(d) + weight * penalty;
      });
}

std::unique_ptr<TaskAllocator> makeLearnedAllocator(CostModel model, bool starvation_guard) {
  return std::make_unique<MatchingAllocator>(
      "learned", true, starvation_guard,
      [model = std::move(model)](const PairFeatureTable* table, RobotId robot, TaskId task,
                                 int32_t) { return model.cost(table->pair(robot, task)); });
}

}  // namespace fleet_core

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <utility>

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
class MatchingAllocator final : public TaskAllocator {
 public:
  // cost(table, robot, task, distance); table is empty when needs_features is false.
  using PairCost = std::function<double(const PairFeatureTable*, RobotId, TaskId, int32_t)>;

  MatchingAllocator(std::string name, bool needs_features, PairCost cost)
      : name_(std::move(name)), needs_features_(needs_features), cost_(std::move(cost)) {}

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

    const std::vector<int32_t> column_of =
        solveAssignment(cost, static_cast<int32_t>(n), static_cast<int32_t>(m));
    std::vector<Assignment> out;
    for (size_t i = 0; i < n; ++i) {
      if (column_of[i] < 0) continue;
      const auto j = static_cast<size_t>(column_of[i]);
      if (forbidden[i * m + j] != 0) continue;
      out.push_back(Assignment{context.robots[i], context.pending[j]});
    }
    return out;
  }

 private:
  std::string name_;
  bool needs_features_;
  PairCost cost_;
};

}  // namespace

std::unique_ptr<TaskAllocator> makeHungarianAllocator() {
  return std::make_unique<MatchingAllocator>(
      "hungarian", false,
      [](const PairFeatureTable*, RobotId, TaskId, int32_t d) { return static_cast<double>(d); });
}

std::unique_ptr<TaskAllocator> makeCostAwareAllocator(double weight, bool congestion_term,
                                                      Timestep service_steps) {
  if (!congestion_term) {
    return std::make_unique<MatchingAllocator>(
        "cost_aware", false,
        [](const PairFeatureTable*, RobotId, TaskId, int32_t d) { return static_cast<double>(d); });
  }
  return std::make_unique<MatchingAllocator>(
      "cost_aware", true,
      [weight, service_steps](const PairFeatureTable* table, RobotId robot, TaskId task,
                              int32_t d) {
        const PairFeatures f = table->pair(robot, task);
        const double penalty = static_cast<double>(f.robots_in_box) +
                               static_cast<double>(service_steps) * f.pickup_assigned;
        return static_cast<double>(d) + weight * penalty;
      });
}

std::unique_ptr<TaskAllocator> makeLearnedAllocator(CostModel model) {
  return std::make_unique<MatchingAllocator>(
      "learned", true,
      [model = std::move(model)](const PairFeatureTable* table, RobotId robot, TaskId task,
                                 int32_t) { return model.cost(table->pair(robot, task)); });
}

}  // namespace fleet_core

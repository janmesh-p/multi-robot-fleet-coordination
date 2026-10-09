#include "fleet_core/tasks/order_generator.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace fleet_core {

OrderGenerator::OrderGenerator(const GridMap& map, double arrival_rate, uint64_t seed)
    : pickups_(map.pickups()), dropoffs_(map.dropoffs()), arrival_rate_(arrival_rate), rng_(seed) {
  if (pickups_.empty() || dropoffs_.empty()) {
    throw std::invalid_argument("OrderGenerator: map needs at least one 'P' and one 'D' station");
  }
  if (!std::isfinite(arrival_rate_) || arrival_rate_ < 0.0) {
    throw std::invalid_argument("OrderGenerator: arrival_rate must be finite and >= 0");
  }
}

std::vector<Task> OrderGenerator::step(Timestep now) {
  const int32_t count = rng_.poisson(arrival_rate_);
  std::vector<Task> orders;
  orders.reserve(static_cast<size_t>(count));
  const auto last_pickup = static_cast<int64_t>(pickups_.size()) - 1;
  const auto last_dropoff = static_cast<int64_t>(dropoffs_.size()) - 1;
  for (int32_t i = 0; i < count; ++i) {
    Task task;
    task.order.id = next_id_++;
    task.order.pickup = pickups_[static_cast<size_t>(rng_.uniformInt(0, last_pickup))];
    task.order.dropoff = dropoffs_[static_cast<size_t>(rng_.uniformInt(0, last_dropoff))];
    task.created_at = now;
    orders.push_back(task);
  }
  return orders;
}

}  // namespace fleet_core

#include "fleet_core/order_generator.hpp"

#include <stdexcept>

namespace fleet_core {

OrderGenerator::OrderGenerator(const GridMap& map, double orders_per_step, uint32_t seed)
    : map_(map), rng_(seed), arrivals_(orders_per_step) {
  if (map_.pickups().empty() || map_.dropoffs().empty())
    throw std::runtime_error("OrderGenerator: map needs at least one 'P' and one 'D'");
}

std::vector<Task> OrderGenerator::step(Timestep t) {
  std::uniform_int_distribution<size_t> pick(0, map_.pickups().size() - 1);
  std::uniform_int_distribution<size_t> drop(0, map_.dropoffs().size() - 1);
  std::vector<Task> out;
  const int n = arrivals_(rng_);
  for (int i = 0; i < n; ++i) {
    Task task;
    task.id = next_id_++;
    task.pickup = map_.pickups()[pick(rng_)];
    task.dropoff = map_.dropoffs()[drop(rng_)];
    task.created_at = t;
    out.push_back(task);
  }
  return out;
}

}  // namespace fleet_core

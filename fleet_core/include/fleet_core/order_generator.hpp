#pragma once

#include <random>
#include <vector>

#include "fleet_core/grid_map.hpp"
#include "fleet_core/types.hpp"

namespace fleet_core {

// Poisson arrival of pickup/dropoff orders between map stations.
class OrderGenerator {
 public:
  OrderGenerator(const GridMap& map, double orders_per_step, uint32_t seed);

  std::vector<Task> step(Timestep t);

 private:
  const GridMap& map_;
  std::mt19937 rng_;
  std::poisson_distribution<int> arrivals_;
  TaskId next_id_{0};
};

}  // namespace fleet_core

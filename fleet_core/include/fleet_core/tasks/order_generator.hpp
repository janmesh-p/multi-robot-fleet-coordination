#pragma once

#include <cstdint>
#include <vector>

#include "fleet_core/common/rng.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// Poisson order stream between the map's pickup and dropoff stations.
//
// It owns its random stream, so every allocator and planner compared under one master seed
// receives exactly the same orders.
class OrderGenerator {
 public:
  // arrival_rate: mean new orders per step. seed: deriveSeed(master, rng_stream::kOrders).
  // Throws std::invalid_argument without stations or with a negative or non-finite rate.
  OrderGenerator(const GridMap& map, double arrival_rate, uint64_t seed);

  // Orders created at step `now`. Ids are consecutive from 0.
  std::vector<Task> step(Timestep now);

 private:
  std::vector<Cell> pickups_;
  std::vector<Cell> dropoffs_;
  double arrival_rate_;
  Rng rng_;
  TaskId next_id_{0};
};

}  // namespace fleet_core

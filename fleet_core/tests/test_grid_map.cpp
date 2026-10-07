#include <cassert>
#include <iostream>

#include "fleet_core/fault_injector.hpp"
#include "fleet_core/grid_map.hpp"
#include "fleet_core/order_generator.hpp"

using namespace fleet_core;

int main() {
  const auto m = GridMap::fromString(
      "; test map\n"
      "P...D\n"
      ".#.#.\n"
      ".....\n");

  assert(m.rows() == 3 && m.cols() == 5);
  assert(m.pickups().size() == 1 && m.dropoffs().size() == 1);
  assert(m.isFree({0, 0}));
  assert(!m.isFree({1, 1}));
  assert(!m.isFree({-1, 0}));
  assert(m.neighbors({0, 0}).size() == 2);

  auto mm = m;
  mm.block({0, 1});
  assert(!mm.isFree({0, 1}));
  assert(mm.neighbors({0, 0}).size() == 1);
  mm.unblock({0, 1});
  assert(mm.isFree({0, 1}));

  OrderGenerator gen(m, 2.0, 7);
  int total = 0;
  for (Timestep t = 0; t < 100; ++t) total += static_cast<int>(gen.step(t).size());
  assert(total > 100 && total < 300);

  FaultInjector fi({{10, FaultType::kRobotDeath, 3}, {5, FaultType::kBlockAisle, -1, {2, 2}}});
  assert(fi.due(4).empty());
  assert(fi.due(5).size() == 1);
  assert(fi.due(20).size() == 1);

  std::cout << "fleet_core tests passed\n";
  return 0;
}

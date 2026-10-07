// Headless grid-sim runner. Used by benchmarks/scripts/sweep.py.
// Usage: run_sim <map.txt> [fleet_size]
#include <cstdlib>
#include <iostream>

#include "fleet_core/grid_map.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: run_sim <map.txt> [fleet_size]\n";
    return 1;
  }
  try {
    const auto map = fleet_core::GridMap::fromFile(argv[1]);
    const int fleet = argc > 2 ? std::atoi(argv[2]) : 5;
    std::cout << "map " << map.rows() << "x" << map.cols() << ", pickups "
              << map.pickups().size() << ", dropoffs " << map.dropoffs().size()
              << ", fleet " << fleet << "\n";
    // TODO(stage1): FleetManager + sim loop + MetricsLogger.
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

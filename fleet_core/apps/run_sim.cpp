// Headless simulation runner.
//
// Stage 0: loads and validates a run config and its map, then prints what would run.
// The simulation loop replaces the final status line in Stage 1.
#include <cstddef>
#include <cstdio>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "fleet_core/common/config.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace {

using fleet_core::Config;
using fleet_core::GridMap;

constexpr double kDensityGuideline = 0.10;  // robots per free cell, from the project plan

int usage(std::ostream& out, int code) {
  out << "usage: run_sim --config <file.yaml> [--print-config]\n";
  return code;
}

std::string percent(double fraction) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f%%", fraction * 100.0);
  return buffer;
}

std::string features(const fleet_core::FeatureToggles& f) {
  const auto item = [](const char* name, bool on) {
    return std::string(name) + (on ? "=on" : "=off");
  };
  return item("station_queues", f.station_queues) + " " + item("parking", f.parking) + " " +
         item("distance_oracle", f.distance_oracle) + " " +
         item("congestion_cost", f.congestion_cost) + "\n           " +
         item("rolling_replanning", f.rolling_replanning) + " " +
         item("dependency_graph", f.dependency_graph) + " " +
         item("intersection_reservations", f.intersection_reservations) + "\n           " +
         item("deadlock_recovery", f.deadlock_recovery) + " " +
         item("starvation_guard", f.starvation_guard);
}

// Checks that need the map. Config-only checks already ran inside loadConfig.
void checkScenario(const Config& config, const GridMap& map) {
  if (map.pickups().empty() || map.dropoffs().empty()) {
    throw std::runtime_error("map needs at least one pickup 'P' and one dropoff 'D' station");
  }
  if (config.fleet.size > map.freeCellCount()) {
    throw std::runtime_error("fleet.size " + std::to_string(config.fleet.size) +
                             " exceeds the map's " + std::to_string(map.freeCellCount()) +
                             " free cells");
  }
  for (size_t i = 0; i < config.faults.events.size(); ++i) {
    for (const fleet_core::Cell& cell : config.faults.events[i].cells) {
      if (!map.isStaticFree(cell)) {
        throw std::runtime_error("faults.events[" + std::to_string(i) + "]: cell [" +
                                 std::to_string(cell.row) + ", " + std::to_string(cell.col) +
                                 "] is outside the map or not a free cell");
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  bool print_config = false;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--print-config") {
      print_config = true;
    } else if (arg == "--help" || arg == "-h") {
      return usage(std::cout, 0);
    } else {
      return usage(std::cerr, 2);
    }
  }
  if (config_path.empty()) return usage(std::cerr, 2);

  try {
    const Config config = fleet_core::loadConfig(config_path);
    const GridMap map = GridMap::fromFile(config.map.file.string());
    checkScenario(config, map);

    const double density =
        static_cast<double>(config.fleet.size) / static_cast<double>(map.freeCellCount());
    std::cout << "config     " << config_path << " (run \"" << config.run.name << "\", seed "
              << config.run.seed << ")\n"
              << "map        " << config.map.file.string() << "\n"
              << "           " << map.rows() << "x" << map.cols() << ", " << map.freeCellCount()
              << " free cells, " << map.pickups().size() << " pickup and " << map.dropoffs().size()
              << " dropoff stations\n"
              << "fleet      " << config.fleet.size << " robots, " << percent(density)
              << " of free cells\n"
              << "orders     " << config.orders.arrival_rate << " per step, service "
              << config.orders.service_steps << " steps\n"
              << "allocation " << fleet_core::toString(config.allocation.method) << "\n"
              << "planning   " << fleet_core::toString(config.planning.method) << ", window "
              << config.planning.window << ", replan every " << config.planning.replan_period
              << "\n"
              << "horizon    " << config.run.horizon_steps << " steps, warmup "
              << config.run.warmup_steps << "\n"
              << "faults     " << config.faults.events.size() << " scripted events, packet loss "
              << percent(config.faults.packet_loss) << "\n"
              << "features   " << features(config.features) << "\n";
    if (density > kDensityGuideline) {
      std::cout << "warning    fleet density above the " << percent(kDensityGuideline)
                << " guideline\n";
    }
    if (print_config) std::cout << "\n" << fleet_core::dumpConfig(config) << "\n";
    std::cout << "status     config valid; simulation loop not implemented yet (Stage 1)\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

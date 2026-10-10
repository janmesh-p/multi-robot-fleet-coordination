// Headless simulation runner: one config in, one run directory out.
//
//   run_sim --config <file.yaml> [--out <dir>] [--print-config]
//
// Exit codes: 0 run finished, 1 error, 2 bad arguments, 3 aborted on an invariant violation.
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

#include "fleet_core/common/config.hpp"
#include "fleet_core/simulation.hpp"

namespace {

using fleet_core::Config;

int usage(std::ostream& out, int code) {
  out << "usage: run_sim --config <file.yaml> [--out <dir>] [--print-config]\n"
         "  --out           run directory (default: run.output_dir from the config)\n"
         "  --print-config  print the resolved config before running\n";
  return code;
}

std::string fixed(double value, int decimals) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

std::string hex16(uint64_t value) {
  char buffer[17];
  std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(value));
  return buffer;
}

std::string deadlockLine(const fleet_core::RunSummary& s) {
  if (s.deadlocks == 0) return "none";
  return std::to_string(s.deadlocks) + ", the first formed at step " +
         std::to_string(s.first_deadlock_step) + "; at the end " +
         std::to_string(s.deadlocked_robots) + " robots on a deadlock, " +
         std::to_string(s.blocked_behind) + " blocked behind";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  std::string out_arg;
  bool print_config = false;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--out" && i + 1 < argc) {
      out_arg = argv[++i];
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
    if (print_config) std::cout << fleet_core::dumpConfig(config) << "\n";
    const fleet_core::Simulation sim(config);
    const std::filesystem::path out_dir = out_arg.empty() ? config.run.output_dir : out_arg;

    const fleet_core::GridMap& map = sim.map();
    std::cout << "run        " << config.run.name << " (seed " << config.run.seed << ")\n"
              << "map        " << map.rows() << "x" << map.cols() << ", " << map.freeCellCount()
              << " free cells, " << map.pickups().size() << " pickup and " << map.dropoffs().size()
              << " dropoff stations, " << map.parking().size() << " parking\n"
              << "fleet      " << config.fleet.size << " robots, "
              << fleet_core::toString(config.allocation.method) << " allocation, "
              << fleet_core::toString(config.planning.method) << " planning\n"
              << "orders     " << config.orders.arrival_rate << " per step for "
              << config.run.horizon_steps << " steps (warmup " << config.run.warmup_steps << ")\n";
    for (const std::string& note : sim.notes()) std::cout << "note       " << note << "\n";

    const fleet_core::RunResult result = sim.run(out_dir);
    const fleet_core::RunSummary& s = result.summary;
    std::cout << "\nsteps      " << s.steps_run << (s.aborted ? " (aborted)" : "") << "\n"
              << "orders     " << s.orders_delivered << " delivered of " << s.orders_created
              << " created\n"
              << "throughput " << fixed(s.orders_per_hour, 1) << " orders/hour (steady state)\n"
              << "completion " << fixed(s.mean_completion_steps, 1) << " steps mean, "
              << fixed(s.p95_completion_steps, 1) << " p95 (" << s.completion_samples
              << " orders)\n"
              << "utilization " << fixed(s.utilization * 100.0, 1) << "%\n"
              << "conflicts  " << s.conflicts << " robot-steps stopped by robot safety\n"
              << "planning   " << fixed(s.planning_p50_ms, 3) << " ms p50, "
              << fixed(s.planning_p95_ms, 3) << " ms p95 over " << s.planning_calls << " calls\n"
              << "violations " << s.violations << "\n"
              << "deadlocks  " << deadlockLine(s) << "\n"
              << "stalls     " << s.stalled_robots << " robots stalled at the end, "
              << s.stalled_robot_steps << " robot-steps in total\n"
              << "frozen     "
              << (s.frozen_at < 0 ? std::string("no")
                                  : "from step " + std::to_string(s.frozen_at) + " to the end (" +
                                        std::to_string(s.steps_run - s.frozen_at) + " steps)")
              << "\n"
              << "delivered  last at step "
              << (s.last_delivery_step < 0 ? std::string("none")
                                           : std::to_string(s.last_delivery_step))
              << "\n"
              << "hash       " << hex16(s.log_hash) << "\n"
              << "wall       " << fixed(s.wall_seconds, 2) << " s\n"
              << "output     " << out_dir.string() << "\n";
    for (size_t i = 0; i < result.violations.size() && i < 5; ++i) {
      const fleet_core::Violation& v = result.violations[i];
      std::cout << "violation  t=" << v.t << " " << fleet_core::toString(v.kind) << ": " << v.detail
                << "\n";
    }
    return s.aborted ? 3 : 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}

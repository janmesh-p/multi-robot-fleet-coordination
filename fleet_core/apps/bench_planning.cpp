// Planner comparison on full runs: the same scenario, orders, and seeds under each planner.
//
//   bench_planning --config <file.yaml> [--planners pibt,windowed_coop_astar,prioritized,ecbs]
//                  [--fleet 5,10,25] [--seeds 1-5] [--rate R] [--map <map.txt>]
//                  [--out <file.csv>]
//
// Each run is the config with planning.method, fleet.size, and run.seed replaced (and
// orders.arrival_rate and map.file, when given), so per-seed differences between planners are
// paired. Writes one CSV row per run and prints one line per planner and fleet size. Exit codes:
// 0 done, 1 error, 2 bad arguments, 5 a coordinated planner had a violation or an aborted run.
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/planning/path_planner.hpp"
#include "fleet_core/simulation.hpp"
#include "fleet_core/sweep.hpp"

using namespace fleet_core;  // NOLINT: a small app, every name below is fleet_core's

namespace {

int usage(std::ostream& out, int code) {
  out << "usage: bench_planning --config <file.yaml> [--planners a,b,...] [--fleet 5,10,25]\n"
         "                      [--seeds 1-5] [--rate R] [--map <map.txt>] [--out <file.csv>]\n"
         "  --planners  planning methods (default pibt,windowed_coop_astar,prioritized,ecbs)\n"
         "  --fleet     fleet sizes (default 5,10,25)\n"
         "  --seeds     seeds, a list or a range (default 1-5)\n"
         "  --rate      orders per step instead of the config's\n"
         "  --map       map file instead of the config's\n";
  return code;
}

PlanningMethod parseMethod(const std::string& name) {
  for (const PlanningMethod m : {PlanningMethod::kIndependentAStar, PlanningMethod::kPrioritized,
                                 PlanningMethod::kWindowedCoopAStar, PlanningMethod::kCbs,
                                 PlanningMethod::kEcbs, PlanningMethod::kPibt}) {
    if (toString(m) == name) return m;
  }
  throw std::invalid_argument("unknown planner \"" + name + "\"");
}

std::vector<PlanningMethod> parseMethods(const std::string& text) {
  std::vector<PlanningMethod> out;
  std::stringstream items(text);
  std::string item;
  while (std::getline(items, item, ',')) out.push_back(parseMethod(item));
  if (out.empty()) throw std::invalid_argument("no planners");
  return out;
}

double mean(const std::vector<double>& v) {
  double sum = 0.0;
  for (const double x : v) sum += x;
  return v.empty() ? 0.0 : sum / static_cast<double>(v.size());
}

std::string csvRow(const RunSummary& s) {
  char hash[17];
  std::snprintf(hash, sizeof(hash), "%016llx", static_cast<unsigned long long>(s.log_hash));
  std::ostringstream out;
  out.precision(10);
  out << s.planner << ',' << s.fleet_size << ',' << s.seed << ',' << s.orders_delivered << ','
      << s.orders_per_hour << ',' << s.mean_completion_steps << ',' << s.p95_completion_steps << ','
      << s.conflicts << ',' << s.congestion_steps << ',' << s.planning_calls << ','
      << s.planner_failures << ',' << s.expansions << ',' << s.planning_p50_ms << ','
      << s.planning_p95_ms << ',' << s.violations << ',' << (s.aborted ? "true" : "false") << ','
      << s.deadlocks << ',' << s.stalled_robots << ',' << s.frozen_at << ',' << hash << ','
      << s.wall_seconds;
  return out.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path, out_path, map_path, rate_arg;
  std::string planners_arg = "pibt,windowed_coop_astar,prioritized,ecbs";
  std::string fleet_arg = "5,10,25", seeds_arg = "1-5";
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--planners" && i + 1 < argc) {
      planners_arg = argv[++i];
    } else if (arg == "--fleet" && i + 1 < argc) {
      fleet_arg = argv[++i];
    } else if (arg == "--seeds" && i + 1 < argc) {
      seeds_arg = argv[++i];
    } else if (arg == "--rate" && i + 1 < argc) {
      rate_arg = argv[++i];
    } else if (arg == "--map" && i + 1 < argc) {
      map_path = argv[++i];
    } else if (arg == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      return usage(std::cout, 0);
    } else {
      return usage(std::cerr, 2);
    }
  }
  if (config_path.empty()) return usage(std::cerr, 2);

  std::vector<PlanningMethod> methods;
  std::vector<int32_t> fleet_sizes;
  std::vector<uint64_t> seeds;
  std::optional<double> rate;
  try {
    methods = parseMethods(planners_arg);
    if (!rate_arg.empty()) {
      size_t used = 0;
      rate = std::stod(rate_arg, &used);
      if (used != rate_arg.size()) throw std::invalid_argument("bad rate \"" + rate_arg + "\"");
    }
    for (const int64_t v : parseIntList(fleet_arg)) {
      if (v < 1 || v > 100000) throw std::invalid_argument("fleet sizes must be positive");
      fleet_sizes.push_back(static_cast<int32_t>(v));
    }
    for (const int64_t v : parseIntList(seeds_arg)) {
      if (v < 0) throw std::invalid_argument("seeds must not be negative");
      seeds.push_back(static_cast<uint64_t>(v));
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return usage(std::cerr, 2);
  }

  try {
    Config base = loadConfig(config_path);
    if (rate) base.orders.arrival_rate = *rate;
    if (!map_path.empty()) {
      base.map.file = std::filesystem::absolute(map_path);
      base.map.generator.reset();
    }
    validateConfig(base);
    const std::string map_name =
        base.map.generator ? "a generated map" : base.map.file.filename().string();
    std::ostringstream csv;
    csv << "planner,fleet_size,seed,orders_delivered,orders_per_hour,mean_completion_steps,"
           "p95_completion_steps,conflicts,congestion_steps,planning_calls,planner_failures,"
           "expansions,planning_p50_ms,planning_p95_ms,violations,aborted,deadlocks,"
           "stalled_robots,frozen_at,log_hash,wall_seconds\n";
    std::cout << "bench      " << base.run.name << " on " << map_name << ": "
              << toString(base.allocation.method) << " allocation, " << base.orders.arrival_rate
              << " orders per step, " << base.run.horizon_steps << " steps, window "
              << base.planning.window << ", replan every " << base.planning.replan_period
              << " steps, " << seeds.size() << (seeds.size() == 1 ? " seed" : " seeds") << "\n\n"
              << " planner               robots  orders/hour  completion  no plan  plan p95  "
                 "violations  frozen     wall\n"
              << "                                     mean   mean steps    calls  ms, mean  "
                 "           runs  s, mean\n";
    bool clean = true;
    for (const PlanningMethod method : methods) {
      for (const int32_t size : fleet_sizes) {
        std::vector<double> oph, completion, p95_ms, wall;
        int64_t failures = 0, violations = 0;
        int32_t frozen = 0, aborted = 0;
        for (const uint64_t seed : seeds) {
          Config c = base;
          c.planning.method = method;
          c.fleet.size = size;
          c.run.seed = seed;
          c.run.name = base.run.name + "_" + std::string(toString(method)) + "_n" +
                       std::to_string(size) + "_s" + std::to_string(seed);
          const RunSummary s = Simulation(c).run().summary;
          csv << csvRow(s) << "\n";
          oph.push_back(s.orders_per_hour);
          completion.push_back(s.mean_completion_steps);
          p95_ms.push_back(s.planning_p95_ms);
          wall.push_back(s.wall_seconds);
          failures += s.planner_failures;
          violations += s.violations;
          if (s.frozen_at >= 0) ++frozen;
          if (s.aborted) ++aborted;
        }
        if (isCoordinated(method) && (violations > 0 || aborted > 0)) clean = false;
        char line[200];
        std::snprintf(line, sizeof(line),
                      " %-20s %7d  %11.1f  %10.1f  %7lld  %8.3f  %10lld  %6d  %7.2f\n",
                      std::string(toString(method)).c_str(), size, mean(oph), mean(completion),
                      static_cast<long long>(failures), mean(p95_ms),
                      static_cast<long long>(violations), frozen, mean(wall));
        std::cout << line << std::flush;
      }
    }
    if (!out_path.empty()) {
      const std::filesystem::path parent = std::filesystem::path(out_path).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
      std::ofstream out(out_path);
      if (!out) throw std::runtime_error("cannot write " + out_path);
      out << csv.str();
      std::cout << "\noutput     " << out_path << "\n";
    }
    if (!clean) {
      std::cout << "check      a coordinated planner had a violation or an aborted run\n";
      return 5;
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}

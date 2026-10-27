// Planner comparison on full runs: the same scenario, orders, and seeds under each planner.
//
//   bench_planning --config <file.yaml> [--planners pibt,windowed_coop_astar,prioritized,ecbs]
//                  [--fleet 5,10,25] [--seeds 1-5] [--rate R] [--map <map.txt>]
//                  [--delay P] [--loss P] [--disable feature,...] [--record] [--out <file.csv>]
//
// Each run is the config with planning.method, fleet.size, and run.seed replaced (and
// orders.arrival_rate, map.file, execution.delay_probability, faults.packet_loss, and feature
// toggles, when given),
// so per-seed differences between planners are paired. Writes one CSV row per run and prints one
// line per planner and fleet size. Exit codes: 0 done, 1 error, 2 bad arguments, 5 a coordinated
// planner had a violation or an aborted run (not checked with --record, for ablations that
// expect violations).
#include <array>
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
#include "fleet_core/execution/grid_executor.hpp"
#include "fleet_core/planning/path_planner.hpp"
#include "fleet_core/simulation.hpp"
#include "fleet_core/sweep.hpp"

using namespace fleet_core;  // NOLINT: a small app, every name below is fleet_core's

namespace {

int usage(std::ostream& out, int code) {
  out << "usage: bench_planning --config <file.yaml> [--planners a,b,...] [--fleet 5,10,25]\n"
         "                      [--seeds 1-5] [--rate R] [--map <map.txt>] [--delay P]\n"
         "                      [--loss P] [--disable feature,...] [--record] [--out <file.csv>]\n"
         "  --planners  planning methods (default pibt,windowed_coop_astar,prioritized,ecbs)\n"
         "  --fleet     fleet sizes (default 5,10,25)\n"
         "  --seeds     seeds, a list or a range (default 1-5)\n"
         "  --rate      orders per step instead of the config's\n"
         "  --map       map file instead of the config's\n"
         "  --delay     execution.delay_probability instead of the config's\n"
         "  --loss      faults.packet_loss instead of the config's\n"
         "  --disable   turn these features off, for example dependency_graph\n"
         "  --record    keep running after a violation (verification.on_violation: record)\n";
  return code;
}

// Turns one feature toggle off by its YAML name.
void disable(FeatureToggles& f, const std::string& name) {
  bool* const toggles[] = {&f.station_queues,
                           &f.parking,
                           &f.distance_oracle,
                           &f.congestion_cost,
                           &f.rolling_replanning,
                           &f.dependency_graph,
                           &f.intersection_reservations,
                           &f.deadlock_recovery,
                           &f.starvation_guard};
  const char* const names[] = {
      "station_queues",     "parking",          "distance_oracle",           "congestion_cost",
      "rolling_replanning", "dependency_graph", "intersection_reservations", "deadlock_recovery",
      "starvation_guard"};
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
    if (name == names[i]) {
      *toggles[i] = false;
      return;
    }
  }
  throw std::invalid_argument("unknown feature \"" + name + "\"");
}

std::vector<std::string> splitList(const std::string& text) {
  std::vector<std::string> out;
  std::stringstream items(text);
  std::string item;
  while (std::getline(items, item, ',')) {
    if (!item.empty()) out.push_back(item);
  }
  return out;
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

std::string csvRow(const RunSummary& s, double delay, double loss, const std::string& disabled) {
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
      << s.wall_seconds << ',' << delay << ',' << disabled << ',' << s.steps_run << ','
      << s.delayed_steps << ',' << s.yields << ',' << s.traffic_holds << ',' << s.deadlocks_resolved
      << ',' << s.max_completion_steps << ',' << s.oldest_open_steps << ',' << loss << ','
      << s.reports_lost << ',' << s.commands_lost << ',' << s.robots_died << ',' << s.declared_dead
      << ',' << s.false_deaths << ',' << s.reconciled << ',' << s.rescued << ',' << s.requeued
      << ',' << s.duplicate_deliveries << ',' << s.no_command_steps << ',' << s.safe_stop_steps
      << ',' << s.stations << ',' << s.time_drive_to_pickup << ',' << s.time_drive_loaded << ','
      << s.time_drive_free << ',' << s.time_service << ',' << s.time_traffic << ','
      << s.time_station_wait << ',' << s.time_idle << ',' << s.time_safety_stop << ','
      << s.time_lost << ',' << s.mean_wait_for_robot_steps << ',' << s.mean_to_pickup_steps << ','
      << s.mean_to_dropoff_steps;
  return out.str();
}

// Where the robot time went, summed over seeds (Stage 7).
struct TimeBudget {
  std::array<int64_t, 9> steps{};  // in the order of the console table
  void add(const RunSummary& s) {
    const std::array<int64_t, 9> run{s.time_drive_to_pickup,
                                     s.time_drive_loaded,
                                     s.time_drive_free,
                                     s.time_service,
                                     s.time_traffic,
                                     s.time_station_wait,
                                     s.time_idle,
                                     s.time_safety_stop,
                                     s.time_lost};
    for (size_t k = 0; k < steps.size(); ++k) steps[k] += run[k];
  }
  double share(size_t k) const {
    int64_t total = 0;
    for (const int64_t v : steps) total += v;
    return total == 0 ? 0.0 : 100.0 * static_cast<double>(steps[k]) / static_cast<double>(total);
  }
};

}  // namespace

int main(int argc, char** argv) {
  std::string config_path, out_path, map_path, rate_arg, delay_arg, loss_arg, disable_arg;
  std::string planners_arg = "pibt,windowed_coop_astar,prioritized,ecbs";
  std::string fleet_arg = "5,10,25", seeds_arg = "1-5";
  bool record = false;
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
    } else if (arg == "--delay" && i + 1 < argc) {
      delay_arg = argv[++i];
    } else if (arg == "--loss" && i + 1 < argc) {
      loss_arg = argv[++i];
    } else if (arg == "--disable" && i + 1 < argc) {
      disable_arg = argv[++i];
    } else if (arg == "--record") {
      record = true;
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
  std::optional<double> rate, delay;
  std::optional<double> loss;
  const std::vector<std::string> disabled = splitList(disable_arg);
  try {
    methods = parseMethods(planners_arg);
    const auto number = [](const std::string& text, const char* what) {
      size_t used = 0;
      const double v = std::stod(text, &used);
      if (used != text.size()) throw std::invalid_argument(std::string("bad ") + what);
      return v;
    };
    if (!rate_arg.empty()) rate = number(rate_arg, "rate");
    if (!delay_arg.empty()) delay = number(delay_arg, "delay");
    if (!loss_arg.empty()) loss = number(loss_arg, "loss");
    FeatureToggles check;
    for (const std::string& name : disabled) disable(check, name);
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
    if (delay) base.execution.delay_probability = *delay;
    if (loss) base.faults.packet_loss = *loss;
    for (const std::string& name : disabled) disable(base.features, name);
    if (record) base.verification.on_violation = ViolationMode::kRecord;
    validateConfig(base);
    std::string disabled_text;
    for (const std::string& name : disabled) {
      disabled_text += (disabled_text.empty() ? "" : " ") + name;
    }
    const std::string map_name =
        base.map.generator ? "a generated map" : base.map.file.filename().string();
    std::ostringstream csv;
    csv << "planner,fleet_size,seed,orders_delivered,orders_per_hour,mean_completion_steps,"
           "p95_completion_steps,conflicts,congestion_steps,planning_calls,planner_failures,"
           "expansions,planning_p50_ms,planning_p95_ms,violations,aborted,deadlocks,"
           "stalled_robots,frozen_at,log_hash,wall_seconds,delay_probability,disabled,steps_run,"
           "delayed_steps,yields,traffic_holds,deadlocks_resolved,max_completion_steps,"
           "oldest_open_steps,packet_loss,reports_lost,commands_lost,robots_died,declared_dead,"
           "false_deaths,reconciled,rescued,requeued,duplicate_deliveries,no_command_steps,"
           "safe_stop_steps,stations,time_drive_to_pickup,time_drive_loaded,time_drive_free,"
           "time_service,time_traffic,time_station_wait,time_idle,time_safety_stop,time_lost,"
           "mean_wait_for_robot_steps,mean_to_pickup_steps,mean_to_dropoff_steps\n";
    std::cout << "bench      " << base.run.name << " on " << map_name << ": "
              << toString(base.allocation.method) << " allocation, " << base.orders.arrival_rate
              << " orders per step, " << base.run.horizon_steps << " steps, window "
              << base.planning.window << ", replan every " << base.planning.replan_period
              << " steps, " << seeds.size() << (seeds.size() == 1 ? " seed" : " seeds") << "\n";
    if (base.faults.packet_loss > 0.0 || !base.faults.events.empty()) {
      std::cout << "faults     packet loss " << base.faults.packet_loss << ", "
                << base.faults.events.size() << " scripted events, lease "
                << base.faults.heartbeat_timeout_steps << " steps, rescue after "
                << base.faults.rescue_steps << " steps\n";
    }
    if (base.execution.delay_probability > 0.0 || !disabled.empty() || record) {
      std::cout << "execution  delay " << base.execution.delay_probability
                << ", features off: " << (disabled.empty() ? "none" : disabled_text)
                << (record ? ", violations recorded" : "") << "\n";
    }
    std::cout << "\n planner               robots  orders/hour  completion  no plan  plan p95  "
                 "safety  violations  frozen     wall\n"
              << "                                     mean   mean steps    calls  ms, mean  "
                 " stops              runs  s, mean\n";
    bool clean = true;
    std::vector<std::pair<std::string, TimeBudget>> budgets;
    for (const PlanningMethod method : methods) {
      for (const int32_t size : fleet_sizes) {
        TimeBudget budget;
        std::vector<double> oph, completion, p95_ms, wall;
        int64_t failures = 0, violations = 0, stops = 0;
        int32_t frozen = 0, aborted = 0;
        for (const uint64_t seed : seeds) {
          Config c = base;
          c.planning.method = method;
          c.fleet.size = size;
          c.run.seed = seed;
          c.run.name = base.run.name + "_" + std::string(toString(method)) + "_n" +
                       std::to_string(size) + "_s" + std::to_string(seed);
          const RunSummary s = Simulation(c).run().summary;
          csv << csvRow(s, c.execution.delay_probability, c.faults.packet_loss, disabled_text)
              << "\n";
          stops += s.conflicts;
          budget.add(s);
          oph.push_back(s.orders_per_hour);
          completion.push_back(s.mean_completion_steps);
          p95_ms.push_back(s.planning_p95_ms);
          wall.push_back(s.wall_seconds);
          failures += s.planner_failures;
          violations += s.violations;
          if (s.frozen_at >= 0) ++frozen;
          if (s.aborted) ++aborted;
        }
        if (!record && isCoordinated(method) && (violations > 0 || aborted > 0)) clean = false;
        char line[220];
        std::snprintf(line, sizeof(line),
                      " %-20s %7d  %11.1f  %10.1f  %7lld  %8.3f  %6lld  %10lld  %6d  %7.2f\n",
                      std::string(toString(method)).c_str(), size, mean(oph), mean(completion),
                      static_cast<long long>(failures), mean(p95_ms), static_cast<long long>(stops),
                      static_cast<long long>(violations), frozen, mean(wall));
        std::cout << line << std::flush;
        char name[48];
        std::snprintf(name, sizeof(name), " %-20s %7d", std::string(toString(method)).c_str(),
                      size);
        budgets.emplace_back(name, budget);
      }
    }
    std::cout << "\n time, % of robot-steps after the warmup\n"
                 " planner               robots  to pickup  loaded  free  service  traffic  "
                 "station wait  idle  stops  lost\n";
    for (const auto& [name, b] : budgets) {
      char line[200];
      std::snprintf(line, sizeof(line),
                    "%s  %9.1f  %6.1f  %4.1f  %7.1f  %7.1f  %12.1f  %4.1f  %5.1f  %4.1f\n",
                    name.c_str(), b.share(0), b.share(1), b.share(2), b.share(3), b.share(4),
                    b.share(5), b.share(6), b.share(7), b.share(8));
      std::cout << line;
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

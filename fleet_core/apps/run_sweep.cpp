// Runs one config over several fleet sizes and seeds, and summarizes each fleet size.
//
//   run_sweep --config <file.yaml> [--fleet 5,10,25] [--seeds 1-5] [--out <dir>] [--runs]
//             [--check <reference sweep.csv>]
//
// Writes <out>/sweep.csv (one row per run, summary.csv columns) and <out>/aggregate.csv (one row
// per fleet size). Exit codes: 0 done, 1 error, 2 bad arguments, 4 differs from the reference.
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
#include "fleet_core/sweep.hpp"

namespace {

int usage(std::ostream& out, int code) {
  out << "usage: run_sweep --config <file.yaml> [--fleet 5,10,25] [--seeds 1-5] [--out <dir>]\n"
         "                 [--runs] [--check <reference sweep.csv>]\n"
         "  --fleet  fleet sizes (default 5,10,25)\n"
         "  --seeds  seeds, a list or a range (default 1-5)\n"
         "  --out    where sweep.csv and aggregate.csv go (default benchmarks/results/sweep)\n"
         "  --runs   also write every run directory under --out\n"
         "  --check  compare with a reference sweep.csv; exit 4 on any difference\n";
  return code;
}

// A median step: whole, or .5 when two runs share the middle; "-" when no run had the event.
std::string step(double value) {
  if (value < 0) return "-";
  char buffer[32];
  if (value == static_cast<double>(static_cast<long long>(value))) {
    std::snprintf(buffer, sizeof(buffer), "%.0f", value);
  } else {
    std::snprintf(buffer, sizeof(buffer), "%.1f", value);
  }
  return buffer;
}

std::string fixed(double value, int decimals) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

void write(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << text;
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path, fleet_arg = "5,10,25", seeds_arg = "1-5";
  std::string out_arg = "benchmarks/results/sweep", check_path;
  bool write_runs = false;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--fleet" && i + 1 < argc) {
      fleet_arg = argv[++i];
    } else if (arg == "--seeds" && i + 1 < argc) {
      seeds_arg = argv[++i];
    } else if (arg == "--out" && i + 1 < argc) {
      out_arg = argv[++i];
    } else if (arg == "--check" && i + 1 < argc) {
      check_path = argv[++i];
    } else if (arg == "--runs") {
      write_runs = true;
    } else if (arg == "--help" || arg == "-h") {
      return usage(std::cout, 0);
    } else {
      return usage(std::cerr, 2);
    }
  }
  if (config_path.empty()) return usage(std::cerr, 2);

  try {
    std::vector<int32_t> fleet_sizes;
    for (const int64_t v : fleet_core::parseIntList(fleet_arg)) {
      if (v < 1 || v > 100000) throw std::invalid_argument("fleet sizes must be positive");
      fleet_sizes.push_back(static_cast<int32_t>(v));
    }
    std::vector<uint64_t> seeds;
    for (const int64_t v : fleet_core::parseIntList(seeds_arg)) {
      if (v < 0) throw std::invalid_argument("seeds must not be negative");
      seeds.push_back(static_cast<uint64_t>(v));
    }
    const fleet_core::Config config = fleet_core::loadConfig(config_path);
    const std::filesystem::path out_dir(out_arg);
    std::filesystem::create_directories(out_dir);

    std::cout << "sweep      " << config.run.name << ": " << fleet_sizes.size() << " fleet sizes x "
              << seeds.size() << " seeds, " << fleet_core::toString(config.allocation.method)
              << " allocation, " << fleet_core::toString(config.planning.method) << " planning, "
              << config.run.horizon_steps << " steps each\n";
    const std::vector<fleet_core::RunSummary> runs = fleet_core::runSweep(
        config, fleet_sizes, seeds,
        write_runs ? std::optional<std::filesystem::path>(out_dir) : std::nullopt);
    const std::vector<fleet_core::SweepGroup> groups = fleet_core::groupByFleetSize(runs);
    write(out_dir / "sweep.csv", fleet_core::sweepCsv(runs));
    write(out_dir / "aggregate.csv", fleet_core::aggregateCsv(groups));

    std::cout << "\n robots  runs  deadlocked  first deadlock  frozen  frozen from  delivered  "
                 "orders/hour\n"
              << "                            median step          median step       mean  "
                 "       mean\n";
    for (const fleet_core::SweepGroup& g : groups) {
      char line[160];
      std::snprintf(line, sizeof(line), " %6d  %4d  %10d  %14s  %6d  %11s  %9s  %11s\n",
                    g.fleet_size, g.runs, g.deadlocked_runs, step(g.median_first_deadlock).c_str(),
                    g.frozen_runs, step(g.median_frozen_at).c_str(),
                    fixed(g.mean_delivered, 1).c_str(), fixed(g.mean_orders_per_hour, 1).c_str());
      std::cout << line;
    }
    std::cout << "\noutput     " << (out_dir / "sweep.csv").string() << ", "
              << (out_dir / "aggregate.csv").string() << "\n";

    if (!check_path.empty()) {
      std::ifstream in(check_path);
      if (!in) throw std::runtime_error("cannot read " + check_path);
      std::stringstream text;
      text << in.rdbuf();
      const std::vector<std::string> diffs = fleet_core::compareWithReference(runs, text.str());
      if (!diffs.empty()) {
        std::cout << "check      " << diffs.size() << " differences from " << check_path << "\n";
        for (size_t i = 0; i < diffs.size() && i < 20; ++i) std::cout << "  " << diffs[i] << "\n";
        return 4;
      }
      std::cout << "check      identical to " << check_path << "\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}

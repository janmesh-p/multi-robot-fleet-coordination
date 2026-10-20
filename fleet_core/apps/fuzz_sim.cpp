// Fuzz harness: random scenarios with robot deaths, comm outages, packet loss, delays, and
// closures, each checked for violations, exceptions, and determinism (Stage 6).
//
//   fuzz_sim [--cases N] [--seed S] [--events K] [--base <file.yaml>] [--out <dir>]
//
// Case k uses seed S + k, so any case can be rerun alone with --seed S+k --cases 1. A failing
// case is shrunk and written to <dir>/fuzz_<seed>.yaml, with the run_sim command that replays it.
// Exit codes: 0 every case passed, 1 a case failed, 2 bad arguments or base config.

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

#include "fleet_core/common/config.hpp"
#include "fleet_core/fuzz.hpp"

namespace {

int usage(std::ostream& out, int code) {
  out << "usage: fuzz_sim [--cases N] [--seed S] [--events K] [--base <file.yaml>] [--out <dir>]\n"
         "  --cases   scenarios to run (default 100)\n"
         "  --seed    seed of the first scenario (default 1); case k uses seed + k\n"
         "  --events  most fault events per scenario (default 6)\n"
         "  --base    config that supplies what is not drawn\n"
         "            (default benchmarks/configs/planning.yaml)\n"
         "  --out     folder for failing scenarios (default fuzz_failures)\n";
  return code;
}

bool parseCount(std::string_view text, int64_t& out) {
  try {
    size_t used = 0;
    out = std::stoll(std::string(text), &used);
    return used == text.size() && out >= 0;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  int64_t cases = 100, seed = 1, events = 6;
  std::filesystem::path base_path = "benchmarks/configs/planning.yaml";
  std::filesystem::path out = "fuzz_failures";
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    const bool has_value = i + 1 < argc;
    if (arg == "--cases" && has_value) {
      if (!parseCount(argv[++i], cases)) return usage(std::cerr, 2);
    } else if (arg == "--seed" && has_value) {
      if (!parseCount(argv[++i], seed)) return usage(std::cerr, 2);
    } else if (arg == "--events" && has_value) {
      if (!parseCount(argv[++i], events)) return usage(std::cerr, 2);
    } else if (arg == "--base" && has_value) {
      base_path = argv[++i];
    } else if (arg == "--out" && has_value) {
      out = argv[++i];
    } else if (arg == "--help") {
      return usage(std::cout, 0);
    } else {
      return usage(std::cerr, 2);
    }
  }

  fleet_core::Config base;
  try {
    base = fleet_core::loadConfig(base_path);
  } catch (const std::exception& e) {
    std::cerr << "fuzz_sim: " << e.what() << "\n";
    return 2;
  }

  int64_t passed = 0, failed = 0, skipped = 0;
  int64_t deaths = 0, false_deaths = 0, reconciled = 0, rescued = 0, lost = 0, requeued = 0;
  for (int64_t k = 0; k < cases; ++k) {
    const auto case_seed = static_cast<uint64_t>(seed + k);
    const auto scenario = fleet_core::randomScenario(base, case_seed, static_cast<int32_t>(events));
    if (!scenario) {
      ++skipped;
      continue;
    }
    const fleet_core::FuzzResult result = fleet_core::checkScenario(*scenario);
    if (result.passed) {
      ++passed;
      const fleet_core::RunSummary& s = result.summary;
      deaths += s.robots_died;
      false_deaths += s.false_deaths;
      reconciled += s.reconciled;
      rescued += s.rescued;
      lost += s.reports_lost + s.commands_lost;
      requeued += s.requeued;
      continue;
    }
    ++failed;
    const fleet_core::Config small = fleet_core::shrinkScenario(*scenario);
    const fleet_core::FuzzResult again = fleet_core::checkScenario(small);
    std::filesystem::create_directories(out);
    const std::filesystem::path file = out / ("fuzz_" + std::to_string(case_seed) + ".yaml");
    std::ofstream(file) << fleet_core::dumpConfig(small);
    std::cout << "FAIL seed " << case_seed << ": " << result.failure << "\n"
              << "  shrunk:    " << again.failure << "\n"
              << "  reproduce: ./build/run_sim --config " << file.string() << "\n"
              << "  rerun:     ./build/fuzz_sim --seed " << case_seed << " --cases 1\n";
  }
  std::cout << "cases      " << cases << ": " << passed << " passed, " << failed << " failed, "
            << skipped << " skipped (closures that cut the aisles apart)\n"
            << "faults     " << deaths << " robot deaths, " << false_deaths << " false deaths, "
            << reconciled << " robots back after one, " << rescued << " dead robots taken away\n"
            << "link       " << lost << " messages lost, " << requeued << " orders requeued\n";
  return failed == 0 ? 0 : 1;
}

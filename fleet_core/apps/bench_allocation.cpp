// Matching quality without traffic: nearest and hungarian solve the same random allocation
// rounds, and the total distance from robots to their pickups is compared.
//
//   bench_allocation [--map maps/warehouse_small.txt] [--rounds 1000] [--seed 1]
//                    [--sizes 5x5,10x10,25x25,5x10,10x20,25x50] [--out <file.csv>]
//
// A round places R robots on distinct random free cells and O orders at random stations, the
// way the order generator draws them, then both allocators assign. Every number except the
// solve time depends only on the seed. Exit codes: 0 done, 1 error, 2 bad arguments.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "fleet_core/allocation/task_allocator.hpp"
#include "fleet_core/common/rng.hpp"
#include "fleet_core/sweep.hpp"
#include "fleet_core/world/grid_map.hpp"

using namespace fleet_core;  // NOLINT: a small app, every name below is fleet_core's

namespace {

int usage(std::ostream& out, int code) {
  out << "usage: bench_allocation [--map <map.txt>] [--rounds N] [--seed S]\n"
         "                        [--sizes RxO,...] [--out <file.csv>]\n"
         "  --sizes  robots x orders per round (default 5x5,10x10,25x25,5x10,10x20,25x50)\n";
  return code;
}

struct Size {
  int32_t robots{0};
  int32_t orders{0};
};

std::vector<Size> parseSizes(const std::string& text) {
  std::vector<Size> out;
  std::stringstream items(text);
  std::string item;
  while (std::getline(items, item, ',')) {
    const size_t x = item.find('x');
    if (x == std::string::npos) throw std::invalid_argument("bad size \"" + item + "\"");
    const std::vector<int64_t> r = parseIntList(item.substr(0, x));
    const std::vector<int64_t> o = parseIntList(item.substr(x + 1));
    if (r.size() != 1 || o.size() != 1 || r[0] < 1 || o[0] < 1 || r[0] > 1000 || o[0] > 1000) {
      throw std::invalid_argument("bad size \"" + item + "\"");
    }
    out.push_back(Size{static_cast<int32_t>(r[0]), static_cast<int32_t>(o[0])});
  }
  if (out.empty()) throw std::invalid_argument("no sizes");
  return out;
}

struct Totals {
  int64_t nearest{0};
  int64_t hungarian{0};
  int32_t better{0};  // rounds where hungarian is strictly cheaper
  std::vector<double> solve_ms{};
};

int32_t totalDistance(const AllocationContext& c, const std::vector<Assignment>& assignments) {
  int32_t sum = 0;
  for (const Assignment& a : assignments) {
    sum += c.distances.distance(c.world.robot(a.robot).cell, c.tasks.get(a.task).order.pickup);
  }
  return sum;
}

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const auto at = static_cast<size_t>(p * static_cast<double>(v.size() - 1) + 0.5);
  return v[std::min(at, v.size() - 1)];
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_path = "maps/warehouse_small.txt";
  std::string sizes_arg = "5x5,10x10,25x25,5x10,10x20,25x50";
  std::string out_path;
  int64_t rounds = 1000;
  uint64_t seed = 1;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string_view arg(argv[i]);
      if (arg == "--map" && i + 1 < argc) {
        map_path = argv[++i];
      } else if (arg == "--rounds" && i + 1 < argc) {
        rounds = std::stoll(argv[++i]);
        if (rounds < 1 || rounds > 1000000) return usage(std::cerr, 2);
      } else if (arg == "--seed" && i + 1 < argc) {
        seed = std::stoull(argv[++i]);
      } else if (arg == "--sizes" && i + 1 < argc) {
        sizes_arg = argv[++i];
      } else if (arg == "--out" && i + 1 < argc) {
        out_path = argv[++i];
      } else if (arg == "--help" || arg == "-h") {
        return usage(std::cout, 0);
      } else {
        return usage(std::cerr, 2);
      }
    }
  } catch (const std::exception&) {
    return usage(std::cerr, 2);
  }

  try {
    const std::vector<Size> sizes = parseSizes(sizes_arg);
    const GridMap map = GridMap::fromFile(map_path);
    std::vector<Cell> free;
    for (int32_t i = 0; i < map.cellCount(); ++i) {
      if (map.isStaticFree(map.cellAt(i))) free.push_back(map.cellAt(i));
    }
    const DistanceOracle distances(map, true);
    Config nearest_config;
    Config hungarian_config;
    hungarian_config.allocation.method = AllocationMethod::kHungarian;
    const auto nearest = makeAllocator(nearest_config);
    const auto hungarian = makeAllocator(hungarian_config);

    std::ostringstream csv;
    csv << "robots,orders,rounds,nearest_mean_total,hungarian_mean_total,reduction_pct,"
           "rounds_better,hungarian_p50_ms,hungarian_p95_ms\n";
    std::cout << "map        " << map_path << ", " << rounds << " rounds per size, seed " << seed
              << "\n\n robots  orders   nearest  hungarian  lower by  rounds better   solve p95\n"
              << "                   mean total distance                         (hungarian)\n";
    for (const Size& size : sizes) {
      if (static_cast<size_t>(size.robots) > free.size()) {
        throw std::invalid_argument("more robots than free cells");
      }
      Rng rng(deriveSeed(seed, "bench_allocation_" + std::to_string(size.robots) + "x" +
                                   std::to_string(size.orders)));
      Totals totals;
      for (int64_t round = 0; round < rounds; ++round) {
        std::vector<Cell> pool = free;  // partial Fisher-Yates: distinct robot cells
        for (size_t k = 0; k < static_cast<size_t>(size.robots); ++k) {
          const auto pick = static_cast<size_t>(
              rng.uniformInt(static_cast<int64_t>(k), static_cast<int64_t>(pool.size()) - 1));
          std::swap(pool[k], pool[pick]);
        }
        pool.resize(static_cast<size_t>(size.robots));
        WorldModel world(map, pool);
        TaskManager tasks;
        for (TaskId id = 0; id < size.orders; ++id) {
          Task t;
          const auto p = rng.uniformInt(0, static_cast<int64_t>(map.pickups().size()) - 1);
          const auto d = rng.uniformInt(0, static_cast<int64_t>(map.dropoffs().size()) - 1);
          t.order = TaskOrder{id, map.pickups()[static_cast<size_t>(p)],
                              map.dropoffs()[static_cast<size_t>(d)]};
          tasks.add(t);
        }
        std::vector<RobotId> robots;
        for (RobotId r = 0; r < size.robots; ++r) robots.push_back(r);
        const AllocationContext context{world, tasks, distances, robots, tasks.pending()};

        const int32_t greedy = totalDistance(context, nearest->assign(context));
        const auto started = std::chrono::steady_clock::now();
        const std::vector<Assignment> matched = hungarian->assign(context);
        totals.solve_ms.push_back(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
                .count());
        const int32_t optimal = totalDistance(context, matched);
        if (optimal > greedy) throw std::logic_error("hungarian worse than nearest");
        totals.nearest += greedy;
        totals.hungarian += optimal;
        if (optimal < greedy) ++totals.better;
      }
      const double n_mean = static_cast<double>(totals.nearest) / static_cast<double>(rounds);
      const double h_mean = static_cast<double>(totals.hungarian) / static_cast<double>(rounds);
      const double reduction = totals.nearest > 0 ? 100.0 * (1.0 - h_mean / n_mean) : 0.0;
      const double better_pct = 100.0 * totals.better / static_cast<double>(rounds);
      const double p50 = percentile(totals.solve_ms, 0.50);
      const double p95 = percentile(totals.solve_ms, 0.95);
      char line[200];
      std::snprintf(line, sizeof(line), " %6d  %6d  %8.1f  %9.1f  %7.1f%%  %12.1f%%  %8.3f ms\n",
                    size.robots, size.orders, n_mean, h_mean, reduction, better_pct, p95);
      std::cout << line;
      std::snprintf(line, sizeof(line), "%d,%d,%lld,%.3f,%.3f,%.2f,%d,%.4f,%.4f\n", size.robots,
                    size.orders, static_cast<long long>(rounds), n_mean, h_mean, reduction,
                    totals.better, p50, p95);
      csv << line;
    }
    if (!out_path.empty()) {
      const std::filesystem::path parent = std::filesystem::path(out_path).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
      std::ofstream out(out_path);
      if (!out) throw std::runtime_error("cannot write " + out_path);
      out << csv.str();
      std::cout << "\noutput     " << out_path << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

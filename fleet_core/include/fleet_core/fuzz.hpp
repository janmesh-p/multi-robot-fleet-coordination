#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "fleet_core/common/config.hpp"
#include "fleet_core/metrics/run_stats.hpp"

namespace fleet_core {

// Fuzz harness (Stage 6): random small scenarios with every fault the simulator models, checked
// for what must hold in any of them: no invariant violation (with a coordinated planner, no
// safety stop either), no exception, and the same run twice from the same config.

// A random scenario drawn from `seed` on top of `base`, which supplies everything not drawn
// (planning budgets, window, objective). Draws a generated warehouse, fleet size, planner,
// allocator, arrival rate, delays, packet loss, lease and rescue times, and up to `max_events`
// robot deaths, comm outages, and closures. Returns nullopt if the simulator rejects it, which
// only happens when a closure would cut the aisles apart even after dropping every closure.
std::optional<Config> randomScenario(const Config& base, uint64_t seed, int32_t max_events = 6);

struct FuzzResult {
  bool passed{true};
  std::string failure{};   // what went wrong in a failed case
  Timestep failed_at{-1};  // step of the first violation; -1 for none
  RunSummary summary{};    // of the first run
};

// Runs the scenario twice. Fails on a violation, an exception, or two runs that differ.
FuzzResult checkScenario(const Config& config);

// A smaller scenario that still fails: the run stops just after the first violation, events
// that are not needed are dropped one at a time, then the packet loss and the delays if the
// failure does not need them.
Config shrinkScenario(const Config& failing);

}  // namespace fleet_core

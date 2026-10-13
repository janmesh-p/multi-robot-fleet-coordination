#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "fleet_core/common/types.hpp"

namespace fleet_core {

enum class AllocationMethod : uint8_t { kNearest, kHungarian, kCostAware, kLearned };

enum class PlanningMethod : uint8_t {
  kIndependentAStar,
  kPrioritized,
  kWindowedCoopAStar,
  kCbs,
  kEcbs,
  kPibt,
};

enum class ViolationMode : uint8_t { kAbort, kRecord };

// Names used in YAML, logs, and result files.
std::string_view toString(AllocationMethod method);
std::string_view toString(PlanningMethod method);
std::string_view toString(ViolationMode mode);
std::string_view toString(FaultType type);

struct RunConfig {
  std::string name{"default"};
  uint64_t seed{1};  // master seed; every random stream derives from it (deriveSeed)
  Timestep horizon_steps{3600};
  Timestep warmup_steps{600};                            // excluded from steady-state metrics
  std::string output_dir{"benchmarks/results/default"};  // relative to the working directory
};

// Parametric warehouse layout, built by world/map_generator.hpp.
struct MapGeneratorConfig {
  int32_t blocks_x{5};      // shelf blocks per row
  int32_t blocks_y{4};      // rows of shelf blocks
  int32_t block_width{6};   // cells
  int32_t block_height{2};  // cells
  int32_t stations{4};      // pickups on the left lane, as many dropoffs on the right lane
  int32_t parking_rows{1};  // parking rows along the top edge, and as many along the bottom
};

// Size of the generated map and the most stations its lanes can hold (two rows apart).
int32_t generatedRows(const MapGeneratorConfig& layout);
int32_t generatedCols(const MapGeneratorConfig& layout);
int32_t maxStations(const MapGeneratorConfig& layout);

// Range checks for a layout. Throws ConfigError. validateConfig calls it.
void validateMapGenerator(const MapGeneratorConfig& layout);

// Exactly one of file and generator is set.
struct MapConfig {
  std::filesystem::path file{};                   // absolute after loading
  std::optional<MapGeneratorConfig> generator{};  // used instead of a file
};

struct FleetConfig {
  int32_t size{10};
};

struct OrdersConfig {
  double arrival_rate{0.1};   // Poisson mean orders per step
  Timestep service_steps{2};  // dwell at each pickup and dropoff station
};

struct AllocationConfig {
  AllocationMethod method{AllocationMethod::kNearest};
  double congestion_weight{1.0};       // cost_aware only
  std::filesystem::path model_file{};  // learned only, and required there; absolute after loading
};

struct PlanningConfig {
  PlanningMethod method{PlanningMethod::kIndependentAStar};
  int32_t window{20};                // windowed planners: conflict-free steps per plan
  int32_t replan_period{5};          // rolling horizon: replan every k steps, k <= window
  int64_t expansion_budget{200000};  // per planning call; deterministic time limit
  double ecbs_suboptimality{1.2};    // ecbs only; cost bound w >= 1
};

struct ExecutionConfig {
  double delay_probability{0.0};  // chance that a robot misses a step
};

struct FaultsConfig {
  Timestep heartbeat_timeout_steps{5};  // silent this long: robot declared dead
  Timestep rescue_steps{120};           // dead robot removed from the floor after this long
  double packet_loss{0.0};              // drop probability per report and per command
  std::vector<FaultEvent> events{};
};

// One switch per feature, so each one can be ablated.
struct FeatureToggles {
  bool station_queues{true};
  bool parking{true};
  bool distance_oracle{true};  // false: Manhattan distance
  bool congestion_cost{true};
  bool rolling_replanning{true};
  bool dependency_graph{true};
  bool intersection_reservations{true};
  bool deadlock_recovery{true};
  bool starvation_guard{true};
};

struct VerificationConfig {
  ViolationMode on_violation{ViolationMode::kAbort};
};

// J = sum of weight x term, each term normalized by the baseline run (Stage 7).
struct ObjectiveConfig {
  double w_completion{1.0};
  double w_distance{1.0};
  double w_congestion{1.0};
  double w_replanning{1.0};
};

struct Config {
  RunConfig run{};
  MapConfig map{};
  FleetConfig fleet{};
  OrdersConfig orders{};
  AllocationConfig allocation{};
  PlanningConfig planning{};
  ExecutionConfig execution{};
  FaultsConfig faults{};
  FeatureToggles features{};
  VerificationConfig verification{};
  ObjectiveConfig objective{};
};

class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Reads a YAML run config. Unknown or duplicate keys, wrong types, and out-of-range values
// throw ConfigError naming the key and line, so a typo never silently falls back to a default.
// Missing keys keep the defaults above. map.file and allocation.model_file resolve relative to the
// config file and must exist; map.generator describes a generated map instead.
Config loadConfig(const std::filesystem::path& file);

// Same, from text. A relative map.file or allocation.model_file resolves against base_dir.
Config parseConfig(std::string_view yaml, const std::filesystem::path& base_dir);

// Range and cross-field checks. parseConfig calls it; call it on configs built in code.
void validateConfig(const Config& config);

// Canonical YAML of every field. parseConfig(dumpConfig(c), any_dir) gives back c.
std::string dumpConfig(const Config& config);

}  // namespace fleet_core

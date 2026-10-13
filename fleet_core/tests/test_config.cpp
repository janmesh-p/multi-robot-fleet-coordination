#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "fleet_core/common/config.hpp"

namespace fleet_core {
namespace {

const std::filesystem::path kRepo = std::filesystem::path(FLEET_REPO_ROOT).lexically_normal();
const std::filesystem::path kMaps = kRepo / "maps";
const std::filesystem::path kDefaultConfig = kRepo / "benchmarks" / "configs" / "default.yaml";

// Smallest valid config, plus extra YAML.
std::string withMap(const std::string& extra = "") {
  return "map:\n  file: warehouse_small.txt\n" + extra;
}

Config parse(const std::string& yaml) { return parseConfig(yaml, kMaps); }

// Expects a ConfigError whose message contains every fragment.
void expectError(const std::string& yaml, const std::vector<std::string>& fragments) {
  try {
    parse(yaml);
    ADD_FAILURE() << "expected ConfigError for:\n" << yaml;
  } catch (const ConfigError& e) {
    const std::string message = e.what();
    for (const std::string& fragment : fragments) {
      EXPECT_NE(message.find(fragment), std::string::npos)
          << "missing \"" << fragment << "\" in: " << message;
    }
  }
}

TEST(Config, ShippedDefaultConfigLoads) {
  const Config c = loadConfig(kDefaultConfig);
  EXPECT_EQ(c.run.name, "default");
  EXPECT_EQ(c.fleet.size, 10);
  EXPECT_EQ(c.planning.method, PlanningMethod::kIndependentAStar);
  EXPECT_TRUE(c.map.file.is_absolute());
  EXPECT_EQ(c.map.file, (kMaps / "warehouse_small.txt").lexically_normal());
}

// The shipped file documents the defaults; this keeps file and code from drifting apart.
TEST(Config, ShippedDefaultConfigMatchesBuiltInDefaults) {
  const Config loaded = loadConfig(kDefaultConfig);
  Config defaults;
  defaults.map.file = loaded.map.file;
  EXPECT_EQ(dumpConfig(loaded), dumpConfig(defaults));
}

TEST(Config, MissingKeysKeepDefaults) {
  const Config c = parse(withMap());
  Config defaults;
  defaults.map.file = c.map.file;
  EXPECT_EQ(dumpConfig(c), dumpConfig(defaults));
}

TEST(Config, EmptySectionsKeepDefaults) {
  const Config c = parse(withMap("features:\nfaults:\n  events:\n"));
  EXPECT_TRUE(c.features.dependency_graph);
  EXPECT_TRUE(c.faults.events.empty());
}

TEST(Config, MapPathRules) {
  EXPECT_EQ(parse(withMap()).map.file, (kMaps / "warehouse_small.txt").lexically_normal());
  const std::string absolute = (kMaps / "warehouse_small.txt").string();
  EXPECT_EQ(parseConfig("map: {file: \"" + absolute + "\"}\n", "/").map.file,
            std::filesystem::path(absolute).lexically_normal());
  expectError("fleet: {size: 3}\n", {"missing section 'map'"});
  expectError("map: {}\n", {"map", "missing key 'file'"});
  expectError("map: {file: nope.txt}\n", {"map.file", "not found"});
  EXPECT_THROW(loadConfig(kRepo / "no_such_config.yaml"), ConfigError);
}

TEST(Config, UnknownKeysAreRejected) {
  expectError(withMap("planing:\n  window: 5\n"),
              {"planing", "line 3", "unknown key", "known keys", "planning"});
  expectError(withMap("features:\n  dependancy_graph: false\n"),
              {"features.dependancy_graph", "line 4", "unknown key"});
}

TEST(Config, DuplicateKeysAreRejected) {
  expectError(withMap("fleet:\n  size: 5\n  size: 6\n"), {"fleet.size", "duplicate"});
}

TEST(Config, WrongTypesAreRejected) {
  expectError(withMap("fleet:\n  size: ten\n"), {"fleet.size", "line 4", "expected an integer"});
  expectError(withMap("fleet:\n  size: 2.5\n"), {"fleet.size", "expected an integer"});
  expectError(withMap("features:\n  parking: maybe\n"), {"features.parking", "true or false"});
  expectError(withMap("orders:\n  arrival_rate: fast\n"), {"orders.arrival_rate", "number"});
  expectError(withMap("fleet: 5\n"), {"fleet", "expected a mapping"});
  expectError(withMap("run:\n  name: [a, b]\n"), {"run.name", "expected a string"});
  expectError("map: [1, 2\n", {"invalid YAML"});
}

TEST(Config, UnknownEnumValueListsChoices) {
  expectError(withMap("planning:\n  method: astar\n"),
              {"planning.method", "'astar'",
               "independent_astar | prioritized | windowed_coop_astar | cbs | ecbs | pibt"});
  expectError(withMap("allocation:\n  method: auction\n"),
              {"allocation.method", "nearest | hungarian | cost_aware | learned"});
  expectError(withMap("verification:\n  on_violation: ignore\n"), {"abort | record"});
}

TEST(Config, RangeChecks) {
  expectError(withMap("fleet: {size: 0}\n"), {"fleet.size"});
  expectError(withMap("fleet: {size: 100000}\n"), {"fleet.size"});
  expectError(withMap("run: {horizon_steps: 0}\n"), {"run.horizon_steps"});
  expectError(withMap("run: {horizon_steps: 100, warmup_steps: 100}\n"), {"run.warmup_steps"});
  expectError(withMap("run: {seed: -1}\n"), {"run.seed"});
  expectError(withMap("run: {name: \"bad name\"}\n"), {"run.name"});
  expectError(withMap("run: {output_dir: \"\"}\n"), {"run.output_dir"});
  expectError(withMap("orders: {arrival_rate: .inf}\n"), {"orders.arrival_rate"});
  expectError(withMap("orders: {arrival_rate: -0.5}\n"), {"orders.arrival_rate"});
  expectError(withMap("planning: {window: 0}\n"), {"planning.window"});
  expectError(withMap("planning: {window: 4, replan_period: 5}\n"), {"planning.replan_period"});
  expectError(withMap("planning: {expansion_budget: 0}\n"), {"planning.expansion_budget"});
  expectError(withMap("planning: {ecbs_suboptimality: 0.9}\n"), {"planning.ecbs_suboptimality"});
  expectError(withMap("execution: {delay_probability: 1.0}\n"), {"execution.delay_probability"});
  expectError(withMap("faults: {packet_loss: -0.1}\n"), {"faults.packet_loss"});
  expectError(withMap("faults: {heartbeat_timeout_steps: 0}\n"), {"heartbeat_timeout_steps"});
  expectError(withMap("objective: {w_distance: -1}\n"), {"objective"});
}

TEST(Config, FaultEventsParse) {
  const Config c =
      parse(withMap("faults:\n"
                    "  events:\n"
                    "    - {at: 500, type: robot_death, robot: 0}\n"
                    "    - {at: 800, type: block_cells, cells: [[5, 9], [5, 10]]}\n"
                    "    - {at: 900, type: comm_outage, robot: 3, duration: 20}\n"));
  ASSERT_EQ(c.faults.events.size(), 3U);
  EXPECT_EQ(c.faults.events[0].type, FaultType::kRobotDeath);
  EXPECT_EQ(c.faults.events[0].robot, 0);
  EXPECT_EQ(c.faults.events[1].type, FaultType::kBlockCells);
  ASSERT_EQ(c.faults.events[1].cells.size(), 2U);
  EXPECT_TRUE(c.faults.events[1].cells[1] == (Cell{5, 10}));
  EXPECT_EQ(c.faults.events[2].duration, 20);
}

TEST(Config, InvalidFaultEventsAreRejected) {
  const auto event = [](const std::string& body) {
    return withMap("faults:\n  events:\n    - {" + body + "}\n");
  };
  expectError(event("at: 5, type: robot_death, robot: 10"), {"faults.events[0]", "robot"});
  expectError(event("at: 5, type: robot_death"), {"faults.events[0]", "needs robot"});
  expectError(event("at: 5, type: robot_death, robot: 1, cells: [[1, 1]]"),
              {"faults.events[0]", "does not take cells"});
  expectError(event("at: 5, type: comm_outage, robot: 1"), {"faults.events[0]", "duration"});
  expectError(event("at: 5, type: block_cells"), {"faults.events[0]", "at least one cell"});
  expectError(event("at: 5, type: block_cells, cells: [[1, 2]], robot: 0"),
              {"faults.events[0]", "does not take robot"});
  expectError(event("at: 5, type: block_cells, cells: [[1]]"), {"cells[0]", "[row, col]"});
  expectError(event("at: 5000, type: robot_death, robot: 1"), {"faults.events[0].at"});
  expectError(event("type: robot_death, robot: 1"), {"faults.events[0]", "missing key 'at'"});
  expectError(event("at: 5, type: explode, robot: 1"),
              {"faults.events[0].type", "robot_death | block_cells | unblock_cells | comm_outage"});
  expectError(event("at: 5, type: robot_death, robot: 1, when: 3"),
              {"faults.events[0].when", "unknown key"});
}

// Every scalar differs from its default, so a field the parser skips is caught below.
const char* const kEveryFieldChanged = R"(
run:
  name: everything
  seed: 99
  horizon_steps: 1800
  warmup_steps: 300
  output_dir: out/everything
map:
  file: warehouse_small.txt
fleet:
  size: 7
orders:
  arrival_rate: 0.35
  service_steps: 3
allocation:
  method: cost_aware
  congestion_weight: 2.5
planning:
  method: ecbs
  window: 12
  replan_period: 4
  expansion_budget: 5000
  ecbs_suboptimality: 1.5
execution:
  delay_probability: 0.05
faults:
  heartbeat_timeout_steps: 9
  rescue_steps: 60
  packet_loss: 0.02
  events:
    - {at: 10, type: robot_death, robot: 2}
    - {at: 20, type: block_cells, cells: [[2, 4], [2, 5]]}
    - {at: 30, type: unblock_cells, cells: [[2, 4]]}
    - {at: 40, type: comm_outage, robot: 1, duration: 8}
features:
  station_queues: false
  parking: false
  distance_oracle: false
  congestion_cost: false
  rolling_replanning: false
  dependency_graph: false
  intersection_reservations: false
  deadlock_recovery: false
  starvation_guard: false
verification:
  on_violation: record
objective:
  w_completion: 2.0
  w_distance: 0.5
  w_congestion: 0.25
  w_replanning: 0.125
)";

// Paths of scalar leaves that hold the same text in both trees.
void collectEqualLeaves(const YAML::Node& a, const YAML::Node& b, const std::string& path,
                        std::vector<std::string>& equal) {
  if (a.IsMap()) {
    for (const auto& entry : a) {
      const std::string key = entry.first.Scalar();
      const YAML::Node other = b[key];
      if (!other) continue;
      collectEqualLeaves(entry.second, other, path.empty() ? key : path + "." + key, equal);
    }
  } else if (a.IsScalar() && b.IsScalar() && a.Scalar() == b.Scalar()) {
    equal.push_back(path);
  }
}

TEST(Config, EveryFieldIsParsed) {
  const Config custom = parse(kEveryFieldChanged);
  Config defaults;
  defaults.map.file = custom.map.file;  // shared on purpose
  std::vector<std::string> equal;
  collectEqualLeaves(YAML::Load(dumpConfig(defaults)), YAML::Load(dumpConfig(custom)), "", equal);
  EXPECT_EQ(equal, std::vector<std::string>{"map.file"});
  EXPECT_EQ(custom.faults.events.size(), 4U);
}

TEST(Config, DumpRoundTrips) {
  const Config custom = parse(kEveryFieldChanged);
  const std::string dumped = dumpConfig(custom);
  EXPECT_EQ(dumpConfig(parseConfig(dumped, "/")), dumped);
}

TEST(Config, ValidateCatchesConfigsBuiltInCode) {
  Config c;
  c.map.file = kMaps / "warehouse_small.txt";
  EXPECT_NO_THROW(validateConfig(c));
  c.planning.replan_period = c.planning.window + 1;
  EXPECT_THROW(validateConfig(c), ConfigError);
}

TEST(Config, ModelFileRules) {
  expectError(withMap("allocation: {method: learned}\n"),
              {"allocation.model_file is required with allocation.method learned"});
  expectError(withMap("allocation: {method: hungarian, model_file: m.json}\n"),
              {"allocation.model_file is only for allocation.method learned"});
  expectError(withMap("allocation: {method: learned, model_file: \"\"}\n"),
              {"allocation.model_file", "must not be empty"});
  expectError(withMap("allocation: {method: learned, model_file: no_such_model.json}\n"),
              {"allocation.model_file", "file not found"});

  // A relative path resolves against the config file's folder, like map.file.
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / ("fleet_core_cfg_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir / "models");
  std::filesystem::copy_file(kMaps / "warehouse_small.txt", dir / "warehouse_small.txt",
                             std::filesystem::copy_options::overwrite_existing);
  std::ofstream(dir / "models" / "m.json") << "{}";  // the config only checks that it exists
  std::ofstream(dir / "run.yaml") << withMap(
      "allocation: {method: learned, model_file: models/m.json}\n");
  const Config c = loadConfig(dir / "run.yaml");
  EXPECT_EQ(c.allocation.method, AllocationMethod::kLearned);
  EXPECT_EQ(c.allocation.model_file, (dir / "models" / "m.json").lexically_normal());
  const std::string dumped = dumpConfig(c);
  EXPECT_NE(dumped.find("  model_file: "), std::string::npos);
  EXPECT_EQ(dumpConfig(parseConfig(dumped, "/")), dumped);
  EXPECT_EQ(dumpConfig(Config{}).find("model_file"), std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST(Config, MapGeneratorParsesAndRoundTrips) {
  const Config c = parse(
      "map:\n"
      "  generator: {blocks_x: 3, blocks_y: 2, block_width: 4, block_height: 1, stations: 2,"
      " parking_rows: 2}\n");
  ASSERT_TRUE(c.map.generator.has_value());
  EXPECT_TRUE(c.map.file.empty());
  EXPECT_EQ(c.map.generator->blocks_x, 3);
  EXPECT_EQ(c.map.generator->parking_rows, 2);
  const std::string dumped = dumpConfig(c);
  EXPECT_EQ(dumpConfig(parseConfig(dumped, "/")), dumped);

  const Config defaults = parse("map: {generator: {}}\n");
  ASSERT_TRUE(defaults.map.generator.has_value());
  EXPECT_EQ(defaults.map.generator->blocks_x, MapGeneratorConfig{}.blocks_x);
}

TEST(Config, MapGeneratorRules) {
  expectError("map: {file: warehouse_small.txt, generator: {}}\n",
              {"either 'file' or 'generator'"});
  expectError("map: {generator: {stations: 99}}\n", {"map.generator.stations", "for this layout"});
  expectError("map: {generator: {blocks_x: 0}}\n", {"map.generator.blocks_x"});
  expectError("map: {generator: {blcoks_x: 2}}\n", {"map.generator.blcoks_x", "unknown key"});
  Config c;
  c.map.generator = MapGeneratorConfig{};
  c.map.file = kMaps / "warehouse_small.txt";
  EXPECT_THROW(validateConfig(c), ConfigError);  // both set
}

TEST(Config, GeneratedSizeFormulas) {
  const MapGeneratorConfig g{5, 4, 6, 2, 4, 1};
  EXPECT_EQ(generatedRows(g), 15);
  EXPECT_EQ(generatedCols(g), 38);
  EXPECT_EQ(maxStations(g), 6);
}

}  // namespace
}  // namespace fleet_core

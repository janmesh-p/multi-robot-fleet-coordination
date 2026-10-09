#include "fleet_core/common/config.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace fleet_core {

std::string_view toString(AllocationMethod method) {
  switch (method) {
    case AllocationMethod::kNearest:
      return "nearest";
    case AllocationMethod::kHungarian:
      return "hungarian";
    case AllocationMethod::kCostAware:
      return "cost_aware";
  }
  return "unknown";
}

std::string_view toString(PlanningMethod method) {
  switch (method) {
    case PlanningMethod::kIndependentAStar:
      return "independent_astar";
    case PlanningMethod::kPrioritized:
      return "prioritized";
    case PlanningMethod::kWindowedCoopAStar:
      return "windowed_coop_astar";
    case PlanningMethod::kCbs:
      return "cbs";
    case PlanningMethod::kEcbs:
      return "ecbs";
    case PlanningMethod::kPibt:
      return "pibt";
  }
  return "unknown";
}

std::string_view toString(ViolationMode mode) {
  switch (mode) {
    case ViolationMode::kAbort:
      return "abort";
    case ViolationMode::kRecord:
      return "record";
  }
  return "unknown";
}

std::string_view toString(FaultType type) {
  switch (type) {
    case FaultType::kRobotDeath:
      return "robot_death";
    case FaultType::kBlockCells:
      return "block_cells";
    case FaultType::kUnblockCells:
      return "unblock_cells";
    case FaultType::kCommOutage:
      return "comm_outage";
  }
  return "unknown";
}

namespace {

constexpr std::array<AllocationMethod, 3> kAllocationMethods{
    AllocationMethod::kNearest, AllocationMethod::kHungarian, AllocationMethod::kCostAware};
constexpr std::array<PlanningMethod, 6> kPlanningMethods{PlanningMethod::kIndependentAStar,
                                                         PlanningMethod::kPrioritized,
                                                         PlanningMethod::kWindowedCoopAStar,
                                                         PlanningMethod::kCbs,
                                                         PlanningMethod::kEcbs,
                                                         PlanningMethod::kPibt};
constexpr std::array<ViolationMode, 2> kViolationModes{ViolationMode::kAbort,
                                                       ViolationMode::kRecord};
constexpr std::array<FaultType, 4> kFaultTypes{FaultType::kRobotDeath, FaultType::kBlockCells,
                                               FaultType::kUnblockCells, FaultType::kCommOutage};

// Upper bounds that catch typos such as an extra zero, not physical limits.
constexpr int32_t kMaxFleetSize = 10000;
constexpr double kMaxArrivalRate = 1000.0;

// ---------------------------------------------------------------------------------------------
// Formatting

// Shortest text that reads back as exactly the same double.
std::string formatDouble(double value) {
  std::array<char, 32> buffer{};
  for (int precision = 1; precision <= 17; ++precision) {
    std::snprintf(buffer.data(), buffer.size(), "%.*g", precision, value);
    if (std::strtod(buffer.data(), nullptr) == value) break;
  }
  std::string text(buffer.data());
  // Keep a decimal point so every YAML reader sees a float. 'n' covers inf and nan.
  if (text.find_first_of(".eEn") == std::string::npos) text += ".0";
  return text;
}

std::string quote(std::string_view text) {
  std::string out = "\"";
  for (const char c : text) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
    }
  }
  out += '"';
  return out;
}

const char* yamlBool(bool value) { return value ? "true" : "false"; }

template <typename E, size_t N>
std::string choices(const std::array<E, N>& values) {
  std::string out;
  for (const E value : values) {
    if (!out.empty()) out += " | ";
    out += toString(value);
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Reading

[[noreturn]] void fail(const std::string& path, const YAML::Node& node, const std::string& what) {
  std::string where = path.empty() ? std::string("<root>") : path;
  const YAML::Mark mark = node.Mark();
  if (!mark.is_null()) where += " (line " + std::to_string(mark.line + 1) + ")";
  throw ConfigError("config: " + where + ": " + what);
}

// A YAML mapping read key by key. finish() rejects every key nobody asked for.
class Section {
 public:
  Section(const YAML::Node& node, std::string path) : path_(std::move(path)) {
    if (!node || node.IsNull()) return;  // absent or empty section: all defaults
    if (!node.IsMap()) fail(path_, node, "expected a mapping of keys to values");
    for (const auto& entry : node) {
      if (!entry.first.IsScalar()) fail(path_, entry.first, "keys must be plain names");
      const std::string key = entry.first.Scalar();
      for (const Entry& existing : entries_) {
        if (existing.key == key) fail(child(key), entry.first, "duplicate key");
      }
      entries_.push_back(Entry{key, entry.first, entry.second, false});
    }
  }

  // Value for `key`, or nullopt when absent.
  std::optional<YAML::Node> take(std::string_view key) {
    asked_.emplace_back(key);
    for (Entry& entry : entries_) {
      if (entry.key == key) {
        entry.used = true;
        return entry.value;
      }
    }
    return std::nullopt;
  }

  void finish() const {
    for (const Entry& entry : entries_) {
      if (entry.used) continue;
      std::string known;
      for (const std::string& key : asked_) {
        if (!known.empty()) known += ", ";
        known += key;
      }
      fail(child(entry.key), entry.key_node, "unknown key (known keys: " + known + ")");
    }
  }

  std::string child(std::string_view key) const {
    return path_.empty() ? std::string(key) : path_ + "." + std::string(key);
  }

 private:
  struct Entry {
    std::string key;
    YAML::Node key_node;
    YAML::Node value;
    bool used;
  };

  std::string path_;
  std::vector<Entry> entries_{};
  std::vector<std::string> asked_{};
};

int64_t readInt64(const YAML::Node& node, const std::string& path) {
  if (node.IsScalar()) {
    try {
      return node.as<int64_t>();
    } catch (const YAML::BadConversion&) {
      // reported below
    }
  }
  fail(path, node, "expected an integer");
}

int32_t readInt32(const YAML::Node& node, const std::string& path) {
  const int64_t value = readInt64(node, path);
  if (value < std::numeric_limits<int32_t>::min() || value > std::numeric_limits<int32_t>::max()) {
    fail(path, node, "integer out of range");
  }
  return static_cast<int32_t>(value);
}

uint64_t readSeed(const YAML::Node& node, const std::string& path) {
  const int64_t value = readInt64(node, path);
  if (value < 0) fail(path, node, "seed must be >= 0");
  return static_cast<uint64_t>(value);
}

double readDouble(const YAML::Node& node, const std::string& path) {
  if (node.IsScalar()) {
    try {
      return node.as<double>();
    } catch (const YAML::BadConversion&) {
      // reported below
    }
  }
  fail(path, node, "expected a number");
}

bool readBool(const YAML::Node& node, const std::string& path) {
  if (node.IsScalar()) {
    try {
      return node.as<bool>();
    } catch (const YAML::BadConversion&) {
      // reported below
    }
  }
  fail(path, node, "expected true or false");
}

std::string readString(const YAML::Node& node, const std::string& path) {
  if (!node.IsScalar()) fail(path, node, "expected a string");
  return node.Scalar();
}

template <typename E, size_t N>
E readEnum(const YAML::Node& node, const std::string& path, const std::array<E, N>& values) {
  const std::string text = readString(node, path);
  for (const E value : values) {
    if (toString(value) == text) return value;
  }
  fail(path, node, "unknown value '" + text + "' (expected " + choices(values) + ")");
}

Cell readCell(const YAML::Node& node, const std::string& path) {
  if (!node.IsSequence() || node.size() != 2) fail(path, node, "expected [row, col]");
  return Cell{readInt32(node[0], path + "[0]"), readInt32(node[1], path + "[1]")};
}

std::vector<Cell> readCells(const YAML::Node& node, const std::string& path) {
  if (!node.IsSequence()) fail(path, node, "expected a list of [row, col] cells");
  std::vector<Cell> cells;
  for (size_t i = 0; i < node.size(); ++i) {
    cells.push_back(readCell(node[i], path + "[" + std::to_string(i) + "]"));
  }
  return cells;
}

template <typename T, typename Reader>
void field(Section& section, std::string_view key, T& out, Reader read) {
  if (const auto node = section.take(key)) out = read(*node, section.child(key));
}

std::vector<FaultEvent> readEvents(const YAML::Node& node, const std::string& path) {
  std::vector<FaultEvent> events;
  if (node.IsNull()) return events;
  if (!node.IsSequence()) fail(path, node, "expected a list of events");
  for (size_t i = 0; i < node.size(); ++i) {
    const YAML::Node item = node[i];
    const std::string item_path = path + "[" + std::to_string(i) + "]";
    Section section(item, item_path);
    FaultEvent event;
    const auto at = section.take("at");
    const auto type = section.take("type");
    field(section, "robot", event.robot, readInt32);
    field(section, "cells", event.cells, readCells);
    field(section, "duration", event.duration, readInt32);
    section.finish();
    if (!at) fail(item_path, item, "missing key 'at'");
    if (!type) fail(item_path, item, "missing key 'type'");
    event.at = readInt32(*at, item_path + ".at");
    event.type = readEnum(*type, item_path + ".type", kFaultTypes);
    events.push_back(std::move(event));
  }
  return events;
}

void parseRun(const YAML::Node& node, RunConfig& run) {
  Section s(node, "run");
  field(s, "name", run.name, readString);
  field(s, "seed", run.seed, readSeed);
  field(s, "horizon_steps", run.horizon_steps, readInt32);
  field(s, "warmup_steps", run.warmup_steps, readInt32);
  field(s, "output_dir", run.output_dir, readString);
  s.finish();
}

void parseMap(const YAML::Node& node, const std::filesystem::path& base_dir, MapConfig& map) {
  Section s(node, "map");
  const auto file = s.take("file");
  s.finish();
  if (!file) fail("map", node, "missing key 'file'");
  std::filesystem::path path = readString(*file, "map.file");
  if (path.empty()) fail("map.file", *file, "must not be empty");
  if (path.is_relative()) path = std::filesystem::absolute(base_dir) / path;
  map.file = path.lexically_normal();
}

void parseFleet(const YAML::Node& node, FleetConfig& fleet) {
  Section s(node, "fleet");
  field(s, "size", fleet.size, readInt32);
  s.finish();
}

void parseOrders(const YAML::Node& node, OrdersConfig& orders) {
  Section s(node, "orders");
  field(s, "arrival_rate", orders.arrival_rate, readDouble);
  field(s, "service_steps", orders.service_steps, readInt32);
  s.finish();
}

void parseAllocation(const YAML::Node& node, AllocationConfig& allocation) {
  Section s(node, "allocation");
  field(s, "method", allocation.method, [](const YAML::Node& n, const std::string& p) {
    return readEnum(n, p, kAllocationMethods);
  });
  field(s, "congestion_weight", allocation.congestion_weight, readDouble);
  s.finish();
}

void parsePlanning(const YAML::Node& node, PlanningConfig& planning) {
  Section s(node, "planning");
  field(s, "method", planning.method,
        [](const YAML::Node& n, const std::string& p) { return readEnum(n, p, kPlanningMethods); });
  field(s, "window", planning.window, readInt32);
  field(s, "replan_period", planning.replan_period, readInt32);
  field(s, "expansion_budget", planning.expansion_budget, readInt64);
  field(s, "ecbs_suboptimality", planning.ecbs_suboptimality, readDouble);
  s.finish();
}

void parseExecution(const YAML::Node& node, ExecutionConfig& execution) {
  Section s(node, "execution");
  field(s, "delay_probability", execution.delay_probability, readDouble);
  s.finish();
}

void parseFaults(const YAML::Node& node, FaultsConfig& faults) {
  Section s(node, "faults");
  field(s, "heartbeat_timeout_steps", faults.heartbeat_timeout_steps, readInt32);
  field(s, "rescue_steps", faults.rescue_steps, readInt32);
  field(s, "packet_loss", faults.packet_loss, readDouble);
  field(s, "events", faults.events, readEvents);
  s.finish();
}

void parseFeatures(const YAML::Node& node, FeatureToggles& f) {
  Section s(node, "features");
  field(s, "station_queues", f.station_queues, readBool);
  field(s, "parking", f.parking, readBool);
  field(s, "distance_oracle", f.distance_oracle, readBool);
  field(s, "congestion_cost", f.congestion_cost, readBool);
  field(s, "rolling_replanning", f.rolling_replanning, readBool);
  field(s, "dependency_graph", f.dependency_graph, readBool);
  field(s, "intersection_reservations", f.intersection_reservations, readBool);
  field(s, "deadlock_recovery", f.deadlock_recovery, readBool);
  field(s, "starvation_guard", f.starvation_guard, readBool);
  s.finish();
}

void parseVerification(const YAML::Node& node, VerificationConfig& verification) {
  Section s(node, "verification");
  field(s, "on_violation", verification.on_violation,
        [](const YAML::Node& n, const std::string& p) { return readEnum(n, p, kViolationModes); });
  s.finish();
}

void parseObjective(const YAML::Node& node, ObjectiveConfig& objective) {
  Section s(node, "objective");
  field(s, "w_completion", objective.w_completion, readDouble);
  field(s, "w_distance", objective.w_distance, readDouble);
  field(s, "w_congestion", objective.w_congestion, readDouble);
  field(s, "w_replanning", objective.w_replanning, readDouble);
  s.finish();
}

// ---------------------------------------------------------------------------------------------
// Validation helpers

bool isValidName(const std::string& name) {
  if (name.empty() || name.size() > 64) return false;
  for (const char c : name) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                    c == '_' || c == '-' || c == '.';
    if (!ok) return false;
  }
  return true;
}

bool isProbability(double p) { return std::isfinite(p) && p >= 0.0 && p < 1.0; }
bool isNonNegative(double v) { return std::isfinite(v) && v >= 0.0; }

std::string gotInt(int64_t value) { return ", got " + std::to_string(value); }
std::string gotNumber(double value) { return ", got " + formatDouble(value); }

void require(bool ok, const std::string& message) {
  if (!ok) throw ConfigError("config: " + message);
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Public API

void validateConfig(const Config& c) {
  require(isValidName(c.run.name),
          "run.name must be 1 to 64 characters from [A-Za-z0-9_.-], got \"" + c.run.name + "\"");
  require(c.run.horizon_steps >= 1, "run.horizon_steps must be >= 1" + gotInt(c.run.horizon_steps));
  require(c.run.warmup_steps >= 0 && c.run.warmup_steps < c.run.horizon_steps,
          "run.warmup_steps must be in [0, run.horizon_steps)" + gotInt(c.run.warmup_steps));
  require(!c.run.output_dir.empty(), "run.output_dir must not be empty");

  require(!c.map.file.empty(), "map.file is required");

  require(
      c.fleet.size >= 1 && c.fleet.size <= kMaxFleetSize,
      "fleet.size must be in [1, " + std::to_string(kMaxFleetSize) + "]" + gotInt(c.fleet.size));

  require(isNonNegative(c.orders.arrival_rate) && c.orders.arrival_rate <= kMaxArrivalRate,
          "orders.arrival_rate must be in [0, " + formatDouble(kMaxArrivalRate) + "]" +
              gotNumber(c.orders.arrival_rate));
  require(c.orders.service_steps >= 0,
          "orders.service_steps must be >= 0" + gotInt(c.orders.service_steps));

  require(isNonNegative(c.allocation.congestion_weight),
          "allocation.congestion_weight must be >= 0" + gotNumber(c.allocation.congestion_weight));

  require(c.planning.window >= 1, "planning.window must be >= 1" + gotInt(c.planning.window));
  require(
      c.planning.replan_period >= 1 && c.planning.replan_period <= c.planning.window,
      "planning.replan_period must be in [1, planning.window]" + gotInt(c.planning.replan_period));
  require(c.planning.expansion_budget >= 1,
          "planning.expansion_budget must be >= 1" + gotInt(c.planning.expansion_budget));
  require(std::isfinite(c.planning.ecbs_suboptimality) && c.planning.ecbs_suboptimality >= 1.0,
          "planning.ecbs_suboptimality must be >= 1" + gotNumber(c.planning.ecbs_suboptimality));

  require(
      isProbability(c.execution.delay_probability),
      "execution.delay_probability must be in [0, 1)" + gotNumber(c.execution.delay_probability));

  require(c.faults.heartbeat_timeout_steps >= 1,
          "faults.heartbeat_timeout_steps must be >= 1" + gotInt(c.faults.heartbeat_timeout_steps));
  require(c.faults.rescue_steps >= 0,
          "faults.rescue_steps must be >= 0" + gotInt(c.faults.rescue_steps));
  require(isProbability(c.faults.packet_loss),
          "faults.packet_loss must be in [0, 1)" + gotNumber(c.faults.packet_loss));

  for (size_t i = 0; i < c.faults.events.size(); ++i) {
    const FaultEvent& e = c.faults.events[i];
    const std::string p = "faults.events[" + std::to_string(i) + "]";
    const std::string type(toString(e.type));
    require(e.at >= 0 && e.at < c.run.horizon_steps,
            p + ".at must be in [0, run.horizon_steps)" + gotInt(e.at));
    const bool robot_event = e.type == FaultType::kRobotDeath || e.type == FaultType::kCommOutage;
    if (robot_event) {
      require(e.robot >= 0 && e.robot < c.fleet.size,
              p + ": " + type + " needs robot in [0, fleet.size)" + gotInt(e.robot));
      require(e.cells.empty(), p + ": " + type + " does not take cells");
    } else {
      require(!e.cells.empty(), p + ": " + type + " needs at least one cell");
      require(e.robot == -1, p + ": " + type + " does not take robot");
      for (const Cell& cell : e.cells) {
        require(cell.row >= 0 && cell.col >= 0, p + ": cells must have row, col >= 0");
      }
    }
    if (e.type == FaultType::kCommOutage) {
      require(e.duration >= 1, p + ": comm_outage needs duration >= 1" + gotInt(e.duration));
    } else {
      require(e.duration == 0, p + ": " + type + " does not take duration");
    }
  }

  const ObjectiveConfig& o = c.objective;
  require(isNonNegative(o.w_completion) && isNonNegative(o.w_distance) &&
              isNonNegative(o.w_congestion) && isNonNegative(o.w_replanning),
          "objective weights must be >= 0");
}

Config parseConfig(std::string_view yaml, const std::filesystem::path& base_dir) {
  YAML::Node root;
  try {
    root = YAML::Load(std::string(yaml));
  } catch (const YAML::Exception& e) {
    throw ConfigError(std::string("config: invalid YAML: ") + e.what());
  }

  Config config;
  Section top(root, "");
  if (const auto n = top.take("run")) parseRun(*n, config.run);
  const auto map = top.take("map");
  if (const auto n = top.take("fleet")) parseFleet(*n, config.fleet);
  if (const auto n = top.take("orders")) parseOrders(*n, config.orders);
  if (const auto n = top.take("allocation")) parseAllocation(*n, config.allocation);
  if (const auto n = top.take("planning")) parsePlanning(*n, config.planning);
  if (const auto n = top.take("execution")) parseExecution(*n, config.execution);
  if (const auto n = top.take("faults")) parseFaults(*n, config.faults);
  if (const auto n = top.take("features")) parseFeatures(*n, config.features);
  if (const auto n = top.take("verification")) parseVerification(*n, config.verification);
  if (const auto n = top.take("objective")) parseObjective(*n, config.objective);
  top.finish();

  if (!map) throw ConfigError("config: missing section 'map' (map.file is required)");
  parseMap(*map, base_dir, config.map);

  validateConfig(config);
  if (!std::filesystem::is_regular_file(config.map.file)) {
    throw ConfigError("config: map.file: file not found: " + config.map.file.string());
  }
  return config;
}

Config loadConfig(const std::filesystem::path& file) {
  if (!std::filesystem::is_regular_file(file)) {
    throw ConfigError("config: file not found: " + file.string());
  }
  std::ifstream in(file);
  if (!in) throw ConfigError("config: cannot open " + file.string());
  std::ostringstream text;
  text << in.rdbuf();
  try {
    return parseConfig(text.str(), std::filesystem::absolute(file).parent_path());
  } catch (const ConfigError& e) {
    throw ConfigError(file.string() + ": " + e.what());
  }
}

std::string dumpConfig(const Config& c) {
  std::ostringstream o;
  o << "run:\n"
    << "  name: " << quote(c.run.name) << "\n"
    << "  seed: " << c.run.seed << "\n"
    << "  horizon_steps: " << c.run.horizon_steps << "\n"
    << "  warmup_steps: " << c.run.warmup_steps << "\n"
    << "  output_dir: " << quote(c.run.output_dir) << "\n"
    << "map:\n"
    << "  file: " << quote(c.map.file.string()) << "\n"
    << "fleet:\n"
    << "  size: " << c.fleet.size << "\n"
    << "orders:\n"
    << "  arrival_rate: " << formatDouble(c.orders.arrival_rate) << "\n"
    << "  service_steps: " << c.orders.service_steps << "\n"
    << "allocation:\n"
    << "  method: " << toString(c.allocation.method) << "\n"
    << "  congestion_weight: " << formatDouble(c.allocation.congestion_weight) << "\n"
    << "planning:\n"
    << "  method: " << toString(c.planning.method) << "\n"
    << "  window: " << c.planning.window << "\n"
    << "  replan_period: " << c.planning.replan_period << "\n"
    << "  expansion_budget: " << c.planning.expansion_budget << "\n"
    << "  ecbs_suboptimality: " << formatDouble(c.planning.ecbs_suboptimality) << "\n"
    << "execution:\n"
    << "  delay_probability: " << formatDouble(c.execution.delay_probability) << "\n"
    << "faults:\n"
    << "  heartbeat_timeout_steps: " << c.faults.heartbeat_timeout_steps << "\n"
    << "  rescue_steps: " << c.faults.rescue_steps << "\n"
    << "  packet_loss: " << formatDouble(c.faults.packet_loss) << "\n";
  if (c.faults.events.empty()) {
    o << "  events: []\n";
  } else {
    o << "  events:\n";
    for (const FaultEvent& e : c.faults.events) {
      o << "    - {at: " << e.at << ", type: " << toString(e.type);
      if (e.robot >= 0) o << ", robot: " << e.robot;
      if (!e.cells.empty()) {
        o << ", cells: [";
        for (size_t i = 0; i < e.cells.size(); ++i) {
          o << (i > 0 ? ", " : "") << "[" << e.cells[i].row << ", " << e.cells[i].col << "]";
        }
        o << "]";
      }
      if (e.duration > 0) o << ", duration: " << e.duration;
      o << "}\n";
    }
  }
  const FeatureToggles& f = c.features;
  o << "features:\n"
    << "  station_queues: " << yamlBool(f.station_queues) << "\n"
    << "  parking: " << yamlBool(f.parking) << "\n"
    << "  distance_oracle: " << yamlBool(f.distance_oracle) << "\n"
    << "  congestion_cost: " << yamlBool(f.congestion_cost) << "\n"
    << "  rolling_replanning: " << yamlBool(f.rolling_replanning) << "\n"
    << "  dependency_graph: " << yamlBool(f.dependency_graph) << "\n"
    << "  intersection_reservations: " << yamlBool(f.intersection_reservations) << "\n"
    << "  deadlock_recovery: " << yamlBool(f.deadlock_recovery) << "\n"
    << "  starvation_guard: " << yamlBool(f.starvation_guard) << "\n"
    << "verification:\n"
    << "  on_violation: " << toString(c.verification.on_violation) << "\n"
    << "objective:\n"
    << "  w_completion: " << formatDouble(c.objective.w_completion) << "\n"
    << "  w_distance: " << formatDouble(c.objective.w_distance) << "\n"
    << "  w_congestion: " << formatDouble(c.objective.w_congestion) << "\n"
    << "  w_replanning: " << formatDouble(c.objective.w_replanning) << "\n";
  return o.str();
}

}  // namespace fleet_core

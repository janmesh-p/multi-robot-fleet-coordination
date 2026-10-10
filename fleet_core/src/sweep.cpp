#include "fleet_core/sweep.hpp"

#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

#include "fleet_core/metrics/metrics_collector.hpp"
#include "fleet_core/simulation.hpp"

namespace fleet_core {
namespace {

int64_t parseInt(const std::string& text, const std::string& whole) {
  size_t used = 0;
  int64_t value = 0;
  try {
    value = std::stoll(text, &used);
  } catch (const std::exception&) {
    used = 0;
  }
  if (text.empty() || used != text.size()) {
    throw std::invalid_argument("not a number list: \"" + whole + "\"");
  }
  return value;
}

// One CSV line into fields; double quotes protect commas, "" is a quote.
std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields(1);
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted) {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
        fields.back() += '"';
        ++i;
      } else if (c == '"') {
        quoted = false;
      } else {
        fields.back() += c;
      }
    } else if (c == '"') {
      quoted = true;
    } else if (c == ',') {
      fields.emplace_back();
    } else if (c != '\r') {
      fields.back() += c;
    }
  }
  return fields;
}

struct Table {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
};

Table parseCsv(const std::string& text) {
  Table t;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line == "\r") continue;
    if (t.header.empty()) {
      t.header = splitCsv(line);
    } else {
      t.rows.push_back(splitCsv(line));
    }
  }
  return t;
}

double median(std::vector<double> values) {
  if (values.empty()) return -1.0;
  std::sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;
  return values.size() % 2 == 1 ? values[mid] : (values[mid - 1] + values[mid]) / 2.0;
}

}  // namespace

std::vector<int64_t> parseIntList(const std::string& text) {
  std::vector<int64_t> out;
  std::stringstream items(text);
  std::string item;
  while (std::getline(items, item, ',')) {
    const size_t dash = item.find('-', 1);  // a leading '-' would be a sign
    if (dash == std::string::npos) {
      out.push_back(parseInt(item, text));
      continue;
    }
    const int64_t first = parseInt(item.substr(0, dash), text);
    const int64_t last = parseInt(item.substr(dash + 1), text);
    if (last < first || last - first > 100000) {
      throw std::invalid_argument("bad range \"" + item + "\" in \"" + text + "\"");
    }
    for (int64_t v = first; v <= last; ++v) out.push_back(v);
  }
  if (out.empty()) throw std::invalid_argument("empty number list");
  return out;
}

std::vector<RunSummary> runSweep(const Config& base, const std::vector<int32_t>& fleet_sizes,
                                 const std::vector<uint64_t>& seeds,
                                 const std::optional<std::filesystem::path>& runs_dir) {
  std::vector<RunSummary> out;
  for (const int32_t size : fleet_sizes) {
    for (const uint64_t seed : seeds) {
      Config c = base;
      c.fleet.size = size;
      c.run.seed = seed;
      c.run.name = base.run.name + "_n" + std::to_string(size) + "_s" + std::to_string(seed);
      const Simulation sim(c);
      std::optional<std::filesystem::path> dir;
      if (runs_dir) dir = *runs_dir / c.run.name;
      out.push_back(sim.run(dir).summary);
    }
  }
  return out;
}

std::string sweepCsv(const std::vector<RunSummary>& runs) {
  std::string out = summaryCsvHeader() + "\n";
  for (const RunSummary& s : runs) out += summaryCsvRow(s) + "\n";
  return out;
}

std::vector<SweepGroup> groupByFleetSize(const std::vector<RunSummary>& runs) {
  std::map<int32_t, std::vector<const RunSummary*>> by_size;
  for (const RunSummary& s : runs) by_size[s.fleet_size].push_back(&s);
  std::vector<SweepGroup> out;
  for (const auto& [size, group] : by_size) {
    SweepGroup g;
    g.fleet_size = size;
    g.runs = static_cast<int32_t>(group.size());
    std::vector<double> deadlock_steps, frozen_steps;
    for (const RunSummary* s : group) {
      if (s->first_deadlock_step >= 0) {
        ++g.deadlocked_runs;
        deadlock_steps.push_back(static_cast<double>(s->first_deadlock_step));
      }
      if (s->frozen_at >= 0) {
        ++g.frozen_runs;
        frozen_steps.push_back(static_cast<double>(s->frozen_at));
      }
      g.mean_delivered += static_cast<double>(s->orders_delivered);
      g.mean_orders_per_hour += s->orders_per_hour;
      g.mean_conflicts += static_cast<double>(s->conflicts);
    }
    g.mean_delivered /= g.runs;
    g.mean_orders_per_hour /= g.runs;
    g.mean_conflicts /= g.runs;
    g.median_first_deadlock = median(deadlock_steps);
    g.median_frozen_at = median(frozen_steps);
    out.push_back(g);
  }
  return out;
}

std::string aggregateCsv(const std::vector<SweepGroup>& groups) {
  std::ostringstream out;
  out << std::setprecision(10)
      << "fleet_size,runs,deadlocked_runs,frozen_runs,median_first_deadlock,median_frozen_at,"
         "mean_delivered,mean_orders_per_hour,mean_conflicts\n";
  for (const SweepGroup& g : groups) {
    out << g.fleet_size << ',' << g.runs << ',' << g.deadlocked_runs << ',' << g.frozen_runs << ','
        << g.median_first_deadlock << ',' << g.median_frozen_at << ',' << g.mean_delivered << ','
        << g.mean_orders_per_hour << ',' << g.mean_conflicts << '\n';
  }
  return out.str();
}

std::vector<std::string> compareWithReference(const std::vector<RunSummary>& runs,
                                              const std::string& reference_csv) {
  static const std::set<std::string> kMachineDependent = {"planning_p50_ms", "planning_p95_ms",
                                                          "wall_seconds"};
  const Table ours = parseCsv(sweepCsv(runs));
  const Table ref = parseCsv(reference_csv);
  std::vector<std::string> diffs;
  if (ref.header.empty()) return {"the reference is empty"};

  std::map<std::string, size_t> ref_column;
  for (size_t i = 0; i < ref.header.size(); ++i) ref_column[ref.header[i]] = i;
  std::map<std::string, const std::vector<std::string>*> ref_rows;
  for (const auto& row : ref.rows) {
    if (!row.empty()) ref_rows[row[0]] = &row;
  }
  for (const std::string& name : ours.header) {
    if (kMachineDependent.count(name) == 0 && ref_column.count(name) == 0) {
      diffs.push_back("column " + name + " is not in the reference");
    }
  }
  std::set<std::string> seen;
  for (const auto& row : ours.rows) {
    const std::string& run = row[0];
    seen.insert(run);
    const auto it = ref_rows.find(run);
    if (it == ref_rows.end()) {
      diffs.push_back("run " + run + " is not in the reference");
      continue;
    }
    for (size_t i = 0; i < ours.header.size() && i < row.size(); ++i) {
      const std::string& name = ours.header[i];
      const auto col = ref_column.find(name);
      if (kMachineDependent.count(name) != 0 || col == ref_column.end()) continue;
      const std::vector<std::string>& expected = *it->second;
      const std::string want = col->second < expected.size() ? expected[col->second] : "";
      if (row[i] != want) {
        diffs.push_back("run " + run + ": " + name + " is " + row[i] + ", the reference has " +
                        want);
      }
    }
  }
  for (const auto& [run, row] : ref_rows) {
    if (seen.count(run) == 0)
      diffs.push_back("run " + run + " is in the reference but was not run");
  }
  return diffs;
}

}  // namespace fleet_core

#include "fleet_core/allocation/cost_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <utility>

#include "../common/json.hpp"

namespace fleet_core {
namespace {

using json::Value;

constexpr std::string_view kFormat = "fleet_cost_model";
constexpr std::string_view kTarget = "steps_to_pickup";

struct Reader {
  const std::string& source;

  [[noreturn]] void fail(const std::string& field, const std::string& what) const {
    throw ModelError(source + ": " + field + ": " + what);
  }

  // Rejects keys outside `known`.
  void checkKeys(const Value& object, const std::string& field,
                 const std::vector<std::string_view>& known) const {
    std::string list;
    for (size_t i = 0; i < known.size(); ++i) list += (i == 0 ? "" : ", ") + std::string(known[i]);
    for (const auto& member : object.members) {
      if (std::find(known.begin(), known.end(), member.first) == known.end()) {
        fail(field.empty() ? member.first : field + "." + member.first,
             "unknown key (known keys: " + list + ")");
      }
    }
  }

  const Value& require(const Value& object, const std::string& field, std::string_view key) const {
    const Value* v = object.find(key);
    if (v == nullptr) {
      fail(field.empty() ? std::string("<root>") : field, "missing key '" + std::string(key) + "'");
    }
    return *v;
  }

  std::string text(const Value& v, const std::string& field) const {
    if (v.type != Value::Type::kString) fail(field, "expected a string");
    return v.text;
  }

  double number(const Value& v, const std::string& field) const {
    if (v.type != Value::Type::kNumber) fail(field, "expected a number");
    if (!std::isfinite(v.number)) fail(field, "expected a finite number");
    return v.number;
  }

  const std::vector<Value>& list(const Value& v, const std::string& field) const {
    if (v.type != Value::Type::kArray) fail(field, "expected a list");
    return v.items;
  }

  std::vector<double> numbers(const Value& v, const std::string& field, size_t expected) const {
    const std::vector<Value>& items = list(v, field);
    if (items.size() != expected) {
      fail(field, "expected " + std::to_string(expected) + " numbers, got " +
                      std::to_string(items.size()));
    }
    std::vector<double> out;
    for (size_t i = 0; i < items.size(); ++i) {
      out.push_back(number(items[i], field + "[" + std::to_string(i) + "]"));
    }
    return out;
  }

  size_t featureName(const Value& v, const std::string& field) const {
    const std::string name = text(v, field);
    const std::optional<size_t> index = pairFeatureIndex(name);
    if (!index) {
      std::string names;
      for (size_t i = 0; i < kPairFeatureCount; ++i) {
        names += (i == 0 ? "" : ", ") + std::string(pairFeatureNames()[i]);
      }
      fail(field, "'" + name + "' is not a feature column (features: " + names + ")");
    }
    return *index;
  }
};

}  // namespace

CostModel CostModel::load(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) throw ModelError(file.string() + ": cannot read the model file");
  std::ostringstream text;
  text << in.rdbuf();
  return parse(text.str(), file.string());
}

CostModel CostModel::parse(std::string_view json_text, const std::string& source) {
  Value root;
  try {
    root = json::parse(json_text);
  } catch (const json::ParseError& e) {
    throw ModelError(source + ": " + e.what());
  }
  const Reader r{source};
  if (root.type != Value::Type::kObject) r.fail("<root>", "expected a JSON object");
  r.checkKeys(root, "",
              {"format", "version", "target", "features", "mean", "scale", "layers", "base_feature",
               "info"});

  if (r.text(r.require(root, "", "format"), "format") != kFormat) {
    r.fail("format", "expected \"" + std::string(kFormat) + "\"");
  }
  if (r.number(r.require(root, "", "version"), "version") != 1.0) r.fail("version", "expected 1");
  if (r.text(r.require(root, "", "target"), "target") != kTarget) {
    r.fail("target", "expected \"" + std::string(kTarget) + "\"");
  }

  CostModel model;
  const std::vector<Value>& names = r.list(r.require(root, "", "features"), "features");
  if (names.empty() || names.size() > kMaxFeatures) {
    r.fail("features", "expected 1 to " + std::to_string(kMaxFeatures) + " names, got " +
                           std::to_string(names.size()));
  }
  std::set<size_t> seen;
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string field = "features[" + std::to_string(i) + "]";
    const size_t index = r.featureName(names[i], field);
    if (!seen.insert(index).second) r.fail(field, "'" + names[i].text + "' is listed twice");
    model.features_.push_back(names[i].text);
    model.feature_index_.push_back(index);
  }
  const size_t n = model.features_.size();
  model.mean_ = r.numbers(r.require(root, "", "mean"), "mean", n);
  model.scale_ = r.numbers(r.require(root, "", "scale"), "scale", n);
  for (size_t i = 0; i < n; ++i) {
    if (!(model.scale_[i] > 0.0)) r.fail("scale[" + std::to_string(i) + "]", "must be > 0");
  }

  const std::vector<Value>& layers = r.list(r.require(root, "", "layers"), "layers");
  if (layers.empty() || layers.size() > kMaxLayers) {
    r.fail("layers", "expected 1 to " + std::to_string(kMaxLayers) + " layers, got " +
                         std::to_string(layers.size()));
  }
  size_t inputs = n;
  for (size_t k = 0; k < layers.size(); ++k) {
    const std::string field = "layers[" + std::to_string(k) + "]";
    const Value& layer = layers[k];
    if (layer.type != Value::Type::kObject) r.fail(field, "expected an object");
    r.checkKeys(layer, field, {"weights", "bias", "activation"});
    const std::vector<Value>& rows = r.list(r.require(layer, field, "weights"), field + ".weights");
    if (rows.empty() || rows.size() > kMaxUnits) {
      r.fail(field + ".weights", "expected 1 to " + std::to_string(kMaxUnits) +
                                     " rows (one per output), got " + std::to_string(rows.size()));
    }
    Layer out;
    out.inputs = inputs;
    out.outputs = rows.size();
    for (size_t j = 0; j < rows.size(); ++j) {
      const std::vector<double> row =
          r.numbers(rows[j], field + ".weights[" + std::to_string(j) + "]", inputs);
      out.weights.insert(out.weights.end(), row.begin(), row.end());
    }
    out.bias = r.numbers(r.require(layer, field, "bias"), field + ".bias", out.outputs);
    const std::string activation =
        r.text(r.require(layer, field, "activation"), field + ".activation");
    const bool last = k + 1 == layers.size();
    if (activation != "relu" && activation != "identity") {
      r.fail(field + ".activation", "expected \"relu\" or \"identity\"");
    }
    if (!last && activation != "relu") {
      r.fail(field + ".activation", "every layer but the last must use \"relu\"");
    }
    if (last && activation != "identity") {
      r.fail(field + ".activation", "the last layer must use \"identity\"");
    }
    if (last && out.outputs != 1) {
      r.fail(field + ".weights",
             "the last layer must have exactly 1 output, got " + std::to_string(out.outputs));
    }
    out.relu = activation == "relu";
    inputs = out.outputs;
    model.layers_.push_back(std::move(out));
  }

  const Value& base = r.require(root, "", "base_feature");
  if (base.type != Value::Type::kNull) {
    model.base_index_ = r.featureName(base, "base_feature");
    model.base_feature_ = base.text;
  }
  return model;
}

double CostModel::predict(const std::vector<double>& inputs, double base) const {
  if (inputs.size() != features_.size()) {
    throw std::invalid_argument("CostModel::predict: expected " + std::to_string(features_.size()) +
                                " inputs, got " + std::to_string(inputs.size()));
  }
  std::vector<double> h(inputs.size());
  for (size_t i = 0; i < inputs.size(); ++i) h[i] = (inputs[i] - mean_[i]) / scale_[i];
  std::vector<double> next;
  for (const Layer& layer : layers_) {
    next.assign(layer.outputs, 0.0);
    for (size_t j = 0; j < layer.outputs; ++j) {
      double sum = 0.0;
      for (size_t k = 0; k < layer.inputs; ++k) sum += layer.weights[j * layer.inputs + k] * h[k];
      sum += layer.bias[j];
      next[j] = layer.relu ? std::max(sum, 0.0) : sum;
    }
    h.swap(next);
  }
  double y = h[0];
  if (base_index_) y += base;
  if (std::isnan(y)) return kMaxCost;  // only reachable through overflow; never NaN out
  return std::clamp(y, 0.0, kMaxCost);
}

namespace {

std::vector<std::string_view> splitFields(std::string_view line) {
  std::vector<std::string_view> fields;
  size_t start = 0;
  while (true) {
    const size_t comma = line.find(',', start);
    fields.push_back(line.substr(start, comma == std::string_view::npos ? comma : comma - start));
    if (comma == std::string_view::npos) return fields;
    start = comma + 1;
  }
}

}  // namespace

std::string predictCsv(const CostModel& model, std::string_view csv, const std::string& source) {
  std::vector<std::string_view> lines;
  size_t start = 0;
  while (start < csv.size()) {
    const size_t end = csv.find('\n', start);
    std::string_view line = csv.substr(start, end == std::string_view::npos ? end : end - start);
    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
    if (!line.empty()) lines.push_back(line);
    if (end == std::string_view::npos) break;
    start = end + 1;
  }
  if (lines.empty()) throw ModelError(source + ": no header row");

  const std::vector<std::string_view> header = splitFields(lines[0]);
  const auto column = [&](const std::string& name) {
    size_t found = header.size();
    for (size_t i = 0; i < header.size(); ++i) {
      if (header[i] != name) continue;
      if (found != header.size())
        throw ModelError(source + ": column '" + name + "' appears twice");
      found = i;
    }
    if (found == header.size()) throw ModelError(source + ": missing column '" + name + "'");
    return found;
  };
  std::vector<size_t> input_columns;
  for (const std::string& name : model.features()) input_columns.push_back(column(name));
  std::optional<size_t> base_column;
  if (model.baseFeature()) base_column = column(*model.baseFeature());

  std::string out = "row,prediction\n";
  std::vector<double> inputs(input_columns.size());
  for (size_t row = 0; row + 1 < lines.size(); ++row) {
    const std::vector<std::string_view> fields = splitFields(lines[row + 1]);
    if (fields.size() != header.size()) {
      throw ModelError(source + ": row " + std::to_string(row) + ": expected " +
                       std::to_string(header.size()) + " fields, got " +
                       std::to_string(fields.size()));
    }
    const auto value = [&](size_t col) {
      const std::string text(fields[col]);
      char* end = nullptr;
      const double v = std::strtod(text.c_str(), &end);
      if (text.empty() || end != text.c_str() + text.size() || !std::isfinite(v)) {
        throw ModelError(source + ": row " + std::to_string(row) + ", column '" +
                         std::string(header[col]) + "': '" + text + "' is not a number");
      }
      return v;
    };
    for (size_t i = 0; i < input_columns.size(); ++i) inputs[i] = value(input_columns[i]);
    const double base = base_column ? value(*base_column) : 0.0;
    char buffer[40];
    std::snprintf(buffer, sizeof(buffer), "%.17g", model.predict(inputs, base));
    out += std::to_string(row) + "," + buffer + "\n";
  }
  return out;
}

double CostModel::cost(const PairFeatures& features) const {
  std::vector<double> inputs(feature_index_.size());
  for (size_t i = 0; i < feature_index_.size(); ++i) {
    inputs[i] = static_cast<double>(pairFeature(features, feature_index_[i]));
  }
  const double base = base_index_ ? static_cast<double>(pairFeature(features, *base_index_)) : 0.0;
  return predict(inputs, base);
}

}  // namespace fleet_core

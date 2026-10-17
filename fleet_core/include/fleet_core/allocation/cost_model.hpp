#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "fleet_core/allocation/pair_features.hpp"

namespace fleet_core {

class ModelError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// The learned allocator's cost of a robot-order pair: a small multilayer perceptron read from a
// fleet_cost_model version 1 file, the format in docs/learning_track.md (section 4). The file is
// the whole interface between the core and the learning track.
//
// cost = clamp(mlp((features - mean) / scale) + base_feature, 0, kMaxCost), in double precision,
// summing each weighted input in index order, then the bias.
class CostModel {
 public:
  static constexpr double kMaxCost = 100000.0;
  static constexpr size_t kMaxFeatures = 32;
  static constexpr size_t kMaxLayers = 4;
  static constexpr size_t kMaxUnits = 64;

  // Throws ModelError naming the file and the offending field.
  static CostModel load(const std::filesystem::path& file);

  // Same, from JSON text; `source` starts every error message.
  static CostModel parse(std::string_view json, const std::string& source = "model");

  // Cost from raw inputs: inputs[i] is the value of features()[i], base the value of
  // baseFeature() (ignored without one). Throws std::invalid_argument on a size mismatch.
  double predict(const std::vector<double>& inputs, double base) const;

  double cost(const PairFeatures& features) const;

  const std::vector<std::string>& features() const { return features_; }
  const std::optional<std::string>& baseFeature() const { return base_feature_; }

 private:
  struct Layer {
    size_t inputs{0};
    size_t outputs{0};
    std::vector<double> weights{};  // outputs x inputs, row-major
    std::vector<double> bias{};
    bool relu{false};
  };

  CostModel() = default;

  std::vector<std::string> features_{};
  std::vector<size_t> feature_index_{};  // positions in pairFeatureNames()
  std::vector<double> mean_{};
  std::vector<double> scale_{};
  std::vector<Layer> layers_{};
  std::optional<std::string> base_feature_{};
  std::optional<size_t> base_index_{};
};

// allocation.method learned with this model as the pair cost.
std::unique_ptr<TaskAllocator> makeLearnedAllocator(CostModel model, bool starvation_guard = false);

// What predict_cost prints: a "row,prediction" header, then the model's cost of every data row
// of `csv` (row counts from 0, the cost printed with %.17g). The CSV needs a header row and the
// columns the model reads; other columns are ignored. Throws ModelError naming `source`, the row,
// and the column for a missing column or a value that is not a number.
std::string predictCsv(const CostModel& model, std::string_view csv, const std::string& source);

}  // namespace fleet_core

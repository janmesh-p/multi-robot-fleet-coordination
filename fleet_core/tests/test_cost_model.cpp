#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/allocation/cost_model.hpp"

namespace fleet_core {
namespace {

// The example model of docs/learning_track.md (section 4), verbatim.
const std::string kExample = R"({
  "format": "fleet_cost_model",
  "version": 1,
  "target": "steps_to_pickup",
  "features": ["dist_to_pickup", "robots_in_box", "pickup_assigned"],
  "mean": [14.2, 1.3, 0.9],
  "scale": [7.5, 1.6, 1.1],
  "layers": [
    {"weights": [[0.8, 0.1, 0.3], [0.2, -0.4, 0.6]], "bias": [0.1, -0.2], "activation": "relu"},
    {"weights": [[1.5, 0.7]], "bias": [0.4], "activation": "identity"}
  ],
  "base_feature": "dist_to_pickup",
  "info": {"notes": "free-form; the C++ side ignores this key"}
})";

const std::string kFreeFlow = R"({"format": "fleet_cost_model", "version": 1,
  "target": "steps_to_pickup", "features": ["dist_to_pickup"], "mean": [0], "scale": [1],
  "layers": [{"weights": [[0]], "bias": [0], "activation": "identity"}],
  "base_feature": "dist_to_pickup"})";

std::string replaced(std::string text, const std::string& from, const std::string& to) {
  const size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << "fixture text not found: " << from;
  if (at != std::string::npos) text.replace(at, from.size(), to);
  return text;
}

void expectRejected(const std::string& json, const std::vector<std::string>& fragments) {
  try {
    CostModel::parse(json, "m.json");
    ADD_FAILURE() << "expected ModelError for:\n" << json;
  } catch (const ModelError& e) {
    const std::string message = e.what();
    EXPECT_EQ(message.rfind("m.json: ", 0), 0U) << message;
    for (const std::string& fragment : fragments) {
      EXPECT_NE(message.find(fragment), std::string::npos)
          << "missing \"" << fragment << "\" in: " << message;
    }
  }
}

TEST(CostModel, WorkedExample) {
  const CostModel model = CostModel::parse(kExample);
  EXPECT_EQ(model.features(),
            (std::vector<std::string>{"dist_to_pickup", "robots_in_box", "pickup_assigned"}));
  EXPECT_EQ(model.baseFeature(), "dist_to_pickup");
  PairFeatures f;
  f.dist_to_pickup = 20;
  f.robots_in_box = 3;
  f.pickup_assigned = 1;
  EXPECT_NEAR(model.cost(f), 21.678284091, 1e-9);
  EXPECT_DOUBLE_EQ(model.predict({20, 3, 1}, 20), model.cost(f));
}

TEST(CostModel, FreeFlowIsExactlyTheDistance) {
  const CostModel model = CostModel::parse(kFreeFlow);
  for (const int32_t d : {0, 1, 14, 39, 4000}) {
    PairFeatures f;
    f.dist_to_pickup = d;
    EXPECT_EQ(model.cost(f), static_cast<double>(d));
  }
}

TEST(CostModel, ClampsToTheCostRange) {
  const std::string linear =
      replaced(kFreeFlow, R"("weights": [[0]], "bias": [0])", R"("weights": [[-2]], "bias": [-5])");
  const CostModel model = CostModel::parse(
      replaced(linear, R"("base_feature": "dist_to_pickup")", R"("base_feature": null)"));
  EXPECT_FALSE(model.baseFeature());
  EXPECT_EQ(model.predict({10}, 0), 0.0);  // -25 clamps to 0
  const CostModel big =
      CostModel::parse(replaced(kFreeFlow, R"("weights": [[0]])", R"("weights": [[1e9]])"));
  EXPECT_EQ(big.predict({10}, 10), CostModel::kMaxCost);
  EXPECT_THROW(big.predict({1, 2}, 0), std::invalid_argument);
}

TEST(CostModel, ReluZeroesNegativeHiddenUnits) {
  const CostModel model = CostModel::parse(R"({"format": "fleet_cost_model", "version": 1,
    "target": "steps_to_pickup", "features": ["order_age"], "mean": [0], "scale": [1],
    "layers": [{"weights": [[1], [-1]], "bias": [0, 0], "activation": "relu"},
               {"weights": [[2, 3]], "bias": [1], "activation": "identity"}],
    "base_feature": null})");
  EXPECT_EQ(model.predict({4}, 0), 9.0);    // relu(4) = 4, relu(-4) = 0: 2*4 + 1
  EXPECT_EQ(model.predict({-4}, 0), 13.0);  // 3*4 + 1
}

TEST(CostModel, EveryRuleRejectsAFileThatBreaksIt) {
  expectRejected("{\"format\": ", {"invalid JSON", "line 1"});
  expectRejected(replaced(kExample, "[14.2, 1.3, 0.9]", "[NaN, 1.3, 0.9]"), {"invalid JSON"});
  expectRejected(replaced(kExample, "[14.2, 1.3, 0.9]", "[14.2, 1.3, 0.9,]"), {"invalid JSON"});
  expectRejected(replaced(kExample, R"("version": 1,)", R"("version": 1, "version": 1,)"),
                 {"duplicate key \"version\""});
  expectRejected("[1, 2]", {"<root>", "expected a JSON object"});
  expectRejected(replaced(kExample, R"("info")", R"("notes")"), {"notes", "unknown key"});
  expectRejected(replaced(kExample, R"("scale": [7.5, 1.6, 1.1],)", ""),
                 {"<root>", "missing key 'scale'"});
  expectRejected(replaced(kExample, R"("base_feature": "dist_to_pickup",)", ""),
                 {"missing key 'base_feature'"});
  expectRejected(replaced(kExample, "\"fleet_cost_model\"", "\"cost_model\""), {"format"});
  expectRejected(replaced(kExample, R"("version": 1)", R"("version": 2)"), {"version"});
  expectRejected(replaced(kExample, R"("version": 1)", R"("version": true)"),
                 {"version", "expected a number"});
  expectRejected(replaced(kExample, R"("target": "steps_to_pickup")", R"("target": "eta")"),
                 {"target"});
  expectRejected(
      replaced(kExample, R"(["dist_to_pickup", "robots_in_box", "pickup_assigned"])", "[]"),
      {"features", "1 to 32"});
  std::string many = "[";
  for (int i = 0; i < 33; ++i) many += (i == 0 ? "" : ", ") + std::string("\"order_age\"");
  many += "]";
  expectRejected(
      replaced(kExample, R"(["dist_to_pickup", "robots_in_box", "pickup_assigned"])", many),
      {"features", "1 to 32"});
  expectRejected(replaced(kExample, R"("robots_in_box", "pickup_assigned"])",
                          R"("steps_to_pickup", "pickup_assigned"])"),
                 {"features[1]", "'steps_to_pickup' is not a feature column"});
  expectRejected(replaced(kExample, R"("robots_in_box", "pickup_assigned"])",
                          R"("robot", "pickup_assigned"])"),
                 {"features[1]", "'robot' is not a feature column"});
  expectRejected(replaced(kExample, R"("robots_in_box", "pickup_assigned"])",
                          R"("pickup_assigned", "pickup_assigned"])"),
                 {"features[2]", "listed twice"});
  expectRejected(replaced(kExample, "[14.2, 1.3, 0.9]", "[14.2, 1.3]"),
                 {"mean", "expected 3 numbers, got 2"});
  expectRejected(replaced(kExample, "[14.2, 1.3, 0.9]", "[14.2, \"1.3\", 0.9]"),
                 {"mean[1]", "expected a number"});
  expectRejected(replaced(kExample, "[7.5, 1.6, 1.1]", "[7.5, 0, 1.1]"), {"scale[1]", "> 0"});
  expectRejected(replaced(kExample, "[7.5, 1.6, 1.1]", "[7.5, -1.6, 1.1]"), {"scale[1]", "> 0"});
  expectRejected(replaced(kExample, "[0.8, 0.1, 0.3]", "[0.8, 1e400, 0.3]"),
                 {"layers[0].weights[0][1]", "finite"});
  expectRejected(replaced(kExample, "[0.8, 0.1, 0.3]", "[0.8, true, 0.3]"),
                 {"layers[0].weights[0][1]", "expected a number"});
  expectRejected(replaced(kExample, "[0.2, -0.4, 0.6]", "[0.2, -0.4]"),
                 {"layers[0].weights[1]", "expected 3 numbers, got 2"});
  expectRejected(replaced(kExample, "[0.1, -0.2]", "[0.1]"), {"layers[0].bias", "expected 2"});
  expectRejected(
      replaced(kExample, R"("activation": "relu"},)", R"("activation": "relu", "dropout": 0.5},)"),
      {"layers[0].dropout", "unknown key"});
  expectRejected(replaced(kExample, R"("bias": [0.4], )", ""), {"layers[1]", "missing key 'bias'"});
  expectRejected(replaced(kExample, R"("activation": "relu"})", R"("activation": "tanh"})"),
                 {"layers[0].activation", "relu"});
  expectRejected(replaced(kExample, R"("activation": "relu"})", R"("activation": "identity"})"),
                 {"layers[0].activation", "every layer but the last"});
  expectRejected(replaced(kExample, R"("activation": "identity")", R"("activation": "relu")"),
                 {"layers[1].activation", "the last layer"});
  expectRejected(replaced(kExample, R"({"weights": [[1.5, 0.7]], "bias": [0.4])",
                          R"({"weights": [[1.5, 0.7], [1, 1]], "bias": [0.4, 0])"),
                 {"layers[1].weights", "exactly 1 output"});
  expectRejected(replaced(kExample, "[[1.5, 0.7]]", "[[1.5, 0.7, 0.1]]"),
                 {"layers[1].weights[0]", "expected 2 numbers, got 3"});
  expectRejected(replaced(kExample, R"("layers": [)", R"("layers": [], "x": [)"),
                 {"x", "unknown key"});
  std::string wide = "[";
  for (int j = 0; j < 65; ++j) wide += (j == 0 ? "" : ", ") + std::string("[1, 1, 1]");
  wide += "]";
  std::string wide_bias = "[";
  for (int j = 0; j < 65; ++j) wide_bias += (j == 0 ? "" : ", ") + std::string("0");
  wide_bias += "]";
  expectRejected(replaced(replaced(kExample, "[[0.8, 0.1, 0.3], [0.2, -0.4, 0.6]]", wide),
                          "[0.1, -0.2]", wide_bias),
                 {"layers[0].weights", "1 to 64 rows"});
  const std::string one_layer = R"({"weights": [[1, 1, 1]], "bias": [0], "activation": "relu"}, )";
  std::string five = kExample;
  five = replaced(five, R"("layers": [)", "\"layers\": [" + one_layer + one_layer + one_layer);
  expectRejected(five, {"layers", "1 to 4 layers, got 5"});
  expectRejected(replaced(kExample, R"("layers": [
    {"weights": [[0.8, 0.1, 0.3], [0.2, -0.4, 0.6]], "bias": [0.1, -0.2], "activation": "relu"},
    {"weights": [[1.5, 0.7]], "bias": [0.4], "activation": "identity"}
  ])",
                          R"("layers": [])"),
                 {"layers", "1 to 4 layers, got 0"});
  expectRejected(
      replaced(kExample, R"("base_feature": "dist_to_pickup")", R"("base_feature": "delivered_t")"),
      {"base_feature", "not a feature column"});
}

TEST(CostModel, BaseFeatureNeedNotBeAnInput) {
  const CostModel model =
      CostModel::parse(replaced(kFreeFlow, R"("base_feature": "dist_to_pickup")",
                                R"("base_feature": "manhattan_to_pickup")"));
  PairFeatures f;
  f.dist_to_pickup = 9;
  f.manhattan_to_pickup = 7;
  EXPECT_EQ(model.cost(f), 7.0);
}

TEST(PredictCsv, ReadsColumnsByNameAndPrintsEveryDigit) {
  const CostModel model = CostModel::parse(kExample);
  const std::string csv =
      "t,robots_in_box,end_status,pickup_assigned,dist_to_pickup\r\n"
      "612,3,delivered,1,20\r\n"
      "700,0,assigned,0,0\r\n"
      "\n";
  EXPECT_EQ(predictCsv(model, csv, "rows.csv"),
            "row,prediction\n"
            "0,21.678284090909091\n"
            "1,0.40000000000000002\n");
}

TEST(PredictCsv, RejectsMissingColumnsAndNonNumbers) {
  const CostModel model = CostModel::parse(kExample);
  const auto expectError = [&](const std::string& csv, const std::string& fragment) {
    try {
      predictCsv(model, csv, "rows.csv");
      ADD_FAILURE() << "expected ModelError for " << csv;
    } catch (const ModelError& e) {
      EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos) << e.what();
    }
  };
  expectError("", "no header row");
  expectError("dist_to_pickup,robots_in_box\n1,2\n", "missing column 'pickup_assigned'");
  expectError("dist_to_pickup,robots_in_box,pickup_assigned\n1,x,2\n",
              "row 0, column 'robots_in_box': 'x' is not a number");
  expectError("dist_to_pickup,robots_in_box,pickup_assigned\n1,2\n", "row 0: expected 3 fields");
  expectError("dist_to_pickup,robots_in_box,pickup_assigned,dist_to_pickup\n1,2,3,4\n",
              "appears twice");
}

TEST(CostModel, LoadNamesTheFile) {
  try {
    CostModel::load("/nonexistent/model.json");
    ADD_FAILURE() << "expected ModelError";
  } catch (const ModelError& e) {
    EXPECT_NE(std::string(e.what()).find("/nonexistent/model.json"), std::string::npos);
  }
}

}  // namespace
}  // namespace fleet_core

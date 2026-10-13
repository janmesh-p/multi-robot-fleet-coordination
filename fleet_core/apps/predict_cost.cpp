// The learned allocator's cost for every row of a CSV, computed exactly as the allocator does.
// The learning track checks its Python reference against this.
//
//   predict_cost --model <model.json> --rows <file.csv>
//
// Prints a row,prediction header, then one line per data row (row counts from 0, the cost
// printed with %.17g). The CSV needs a header row and the columns the model reads; other
// columns are ignored. Exit codes: 0 done, 1 bad model or rows, 2 bad arguments.
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

#include "fleet_core/allocation/cost_model.hpp"

namespace {

int usage(std::ostream& out, int code) {
  out << "usage: predict_cost --model <model.json> --rows <file.csv>\n";
  return code;
}

}  // namespace

int main(int argc, char** argv) {
  std::string model_path, rows_path;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--model" && i + 1 < argc) {
      model_path = argv[++i];
    } else if (arg == "--rows" && i + 1 < argc) {
      rows_path = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      return usage(std::cout, 0);
    } else {
      return usage(std::cerr, 2);
    }
  }
  if (model_path.empty() || rows_path.empty()) return usage(std::cerr, 2);

  try {
    const fleet_core::CostModel model = fleet_core::CostModel::load(model_path);
    std::ifstream in(rows_path);
    if (!in) throw fleet_core::ModelError(rows_path + ": cannot read the rows file");
    std::ostringstream text;
    text << in.rdbuf();
    std::cout << fleet_core::predictCsv(model, text.str(), rows_path);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

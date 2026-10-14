#include "fleet_core/planning/path_planner.hpp"

#include <stdexcept>

#include "planners.hpp"

namespace fleet_core {

std::unique_ptr<PathPlanner> makePlanner(const PlanningConfig& config) {
  switch (config.method) {
    case PlanningMethod::kIndependentAStar:
      return makeIndependentAStar();
    case PlanningMethod::kPrioritized:
      return makePrioritized(false);
    case PlanningMethod::kWindowedCoopAStar:
      return makePrioritized(true);
    case PlanningMethod::kCbs:
      return makeCbs(false);
    case PlanningMethod::kEcbs:
      return makeCbs(true);
    case PlanningMethod::kPibt:
      return makePibt();
  }
  throw std::invalid_argument("unknown planning method");
}

bool isCoordinated(PlanningMethod method) { return method != PlanningMethod::kIndependentAStar; }

Cell positionAt(const Path& path, Timestep t) {
  if (path.cells.empty()) throw std::invalid_argument("positionAt: empty path");
  if (t <= path.start) return path.cells.front();
  const auto k = static_cast<size_t>(t - path.start);
  return k < path.cells.size() ? path.cells[k] : path.cells.back();
}

std::optional<PathConflict> firstConflict(const std::vector<Path>& paths, Timestep from,
                                          Timestep until) {
  for (Timestep t = from; t <= until; ++t) {
    for (size_t i = 0; i < paths.size(); ++i) {
      const Cell a = positionAt(paths[i], t);
      for (size_t j = i + 1; j < paths.size(); ++j) {
        const Cell b = positionAt(paths[j], t);
        if (a == b) return PathConflict{i, j, t, a, false};
        if (t > from && a == positionAt(paths[j], t - 1) && b == positionAt(paths[i], t - 1)) {
          return PathConflict{i, j, t, a, true};
        }
      }
    }
  }
  return std::nullopt;
}

}  // namespace fleet_core

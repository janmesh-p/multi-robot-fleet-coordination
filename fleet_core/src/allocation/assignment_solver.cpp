#include "fleet_core/allocation/assignment_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace fleet_core {
namespace {

// rows <= cols. Rows are added one at a time; each addition finds the cheapest augmenting path
// in the reduced costs cost[i][j] - u[i] - v[j], Dijkstra style, and keeps every reduced cost
// of a matched pair at zero. Index 0 of u, v, match, and way is a sentinel.
std::vector<int32_t> solveWide(const std::vector<double>& cost, int32_t rows, int32_t cols) {
  const auto n = static_cast<size_t>(rows);
  const auto m = static_cast<size_t>(cols);
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(m + 1, 0.0);
  std::vector<double> min_reduced(m + 1, inf);
  std::vector<size_t> match(m + 1, 0);  // column -> row (1-based), 0 = free
  std::vector<size_t> way(m + 1, 0);    // previous column on the augmenting path
  std::vector<char> used(m + 1, 0);

  for (size_t i = 1; i <= n; ++i) {
    match[0] = i;
    size_t j0 = 0;
    std::fill(min_reduced.begin(), min_reduced.end(), inf);
    std::fill(used.begin(), used.end(), 0);
    do {
      used[j0] = 1;
      const size_t i0 = match[j0];
      double delta = inf;
      size_t j1 = 0;
      for (size_t j = 1; j <= m; ++j) {
        if (used[j] != 0) continue;
        const double reduced = cost[(i0 - 1) * m + (j - 1)] - u[i0] - v[j];
        if (reduced < min_reduced[j]) {
          min_reduced[j] = reduced;
          way[j] = j0;
        }
        if (min_reduced[j] < delta) {  // strict: the lower column wins a tie
          delta = min_reduced[j];
          j1 = j;
        }
      }
      if (j1 == 0) throw std::logic_error("solveAssignment: no augmenting path");
      for (size_t j = 0; j <= m; ++j) {
        if (used[j] != 0) {
          u[match[j]] += delta;
          v[j] -= delta;
        } else {
          min_reduced[j] -= delta;
        }
      }
      j0 = j1;
    } while (match[j0] != 0);
    do {  // flip the path
      const size_t j1 = way[j0];
      match[j0] = match[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int32_t> column_of(n, -1);
  for (size_t j = 1; j <= m; ++j) {
    if (match[j] != 0) column_of[match[j] - 1] = static_cast<int32_t>(j - 1);
  }
  return column_of;
}

}  // namespace

std::vector<int32_t> solveAssignment(const std::vector<double>& cost, int32_t rows, int32_t cols) {
  if (rows < 0 || cols < 0) throw std::invalid_argument("solveAssignment: negative size");
  const auto n = static_cast<size_t>(rows);
  const auto m = static_cast<size_t>(cols);
  if (cost.size() != n * m) {
    throw std::invalid_argument("solveAssignment: expected " + std::to_string(n * m) +
                                " costs, got " + std::to_string(cost.size()));
  }
  for (const double c : cost) {
    if (!std::isfinite(c)) throw std::invalid_argument("solveAssignment: costs must be finite");
  }
  if (n == 0 || m == 0) return std::vector<int32_t>(n, -1);
  if (n <= m) return solveWide(cost, rows, cols);

  // More rows than columns: match every column to a row instead.
  std::vector<double> transposed(n * m);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < m; ++j) transposed[j * n + i] = cost[i * m + j];
  }
  const std::vector<int32_t> row_of = solveWide(transposed, cols, rows);
  std::vector<int32_t> column_of(n, -1);
  for (size_t j = 0; j < m; ++j) {
    if (row_of[j] >= 0) column_of[static_cast<size_t>(row_of[j])] = static_cast<int32_t>(j);
  }
  return column_of;
}

}  // namespace fleet_core

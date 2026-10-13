#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/allocation/assignment_solver.hpp"
#include "fleet_core/common/rng.hpp"

namespace fleet_core {
namespace {

double totalCost(const std::vector<double>& cost, int32_t cols,
                 const std::vector<int32_t>& column_of) {
  double total = 0.0;
  for (size_t i = 0; i < column_of.size(); ++i) {
    if (column_of[i] >= 0) total += cost[i * static_cast<size_t>(cols) + column_of[i]];
  }
  return total;
}

// Cheapest matching of size min(rows, cols), by trying every one.
double bruteForce(const std::vector<double>& cost, int32_t rows, int32_t cols) {
  const int32_t k = std::min(rows, cols);
  double best = std::numeric_limits<double>::infinity();
  std::vector<char> row_used(static_cast<size_t>(rows), 0);
  std::vector<char> col_used(static_cast<size_t>(cols), 0);
  // Matches the smaller side in order; the larger side picks freely.
  std::function<void(int32_t, double)> go = [&](int32_t depth, double sum) {
    if (depth == k) {
      best = std::min(best, sum);
      return;
    }
    if (rows <= cols) {
      for (int32_t j = 0; j < cols; ++j) {
        if (col_used[static_cast<size_t>(j)] != 0) continue;
        col_used[static_cast<size_t>(j)] = 1;
        go(depth + 1, sum + cost[static_cast<size_t>(depth * cols + j)]);
        col_used[static_cast<size_t>(j)] = 0;
      }
    } else {
      for (int32_t i = 0; i < rows; ++i) {
        if (row_used[static_cast<size_t>(i)] != 0) continue;
        row_used[static_cast<size_t>(i)] = 1;
        go(depth + 1, sum + cost[static_cast<size_t>(i * cols + depth)]);
        row_used[static_cast<size_t>(i)] = 0;
      }
    }
  };
  go(0, 0.0);
  return best;
}

void expectValidMatching(const std::vector<int32_t>& column_of, int32_t rows, int32_t cols) {
  ASSERT_EQ(column_of.size(), static_cast<size_t>(rows));
  std::vector<char> seen(static_cast<size_t>(cols), 0);
  int32_t matched = 0;
  for (const int32_t j : column_of) {
    if (j < 0) continue;
    ASSERT_LT(j, cols);
    EXPECT_EQ(seen[static_cast<size_t>(j)], 0) << "column " << j << " used twice";
    seen[static_cast<size_t>(j)] = 1;
    ++matched;
  }
  EXPECT_EQ(matched, std::min(rows, cols));
}

TEST(AssignmentSolver, MatchesBruteForceOnRandomMatrices) {
  Rng rng(7);
  for (int trial = 0; trial < 600; ++trial) {
    const auto rows = static_cast<int32_t>(rng.uniformInt(1, 6));
    const auto cols = static_cast<int32_t>(rng.uniformInt(1, 6));
    std::vector<double> cost(static_cast<size_t>(rows * cols));
    // Small integers make ties common, which is where solvers go wrong.
    for (double& c : cost) c = static_cast<double>(rng.uniformInt(0, 9));
    const std::vector<int32_t> column_of = solveAssignment(cost, rows, cols);
    expectValidMatching(column_of, rows, cols);
    EXPECT_EQ(totalCost(cost, cols, column_of), bruteForce(cost, rows, cols))
        << rows << "x" << cols << " trial " << trial;
  }
}

TEST(AssignmentSolver, FractionalAndNegativeCosts) {
  Rng rng(11);
  for (int trial = 0; trial < 200; ++trial) {
    const auto rows = static_cast<int32_t>(rng.uniformInt(1, 5));
    const auto cols = static_cast<int32_t>(rng.uniformInt(1, 5));
    std::vector<double> cost(static_cast<size_t>(rows * cols));
    for (double& c : cost) c = rng.uniform01() * 200.0 - 50.0;
    const std::vector<int32_t> column_of = solveAssignment(cost, rows, cols);
    expectValidMatching(column_of, rows, cols);
    EXPECT_NEAR(totalCost(cost, cols, column_of), bruteForce(cost, rows, cols), 1e-9);
  }
}

// The greedy trap from the explainer: taking each row's cheapest column first costs 1 + 10,
// the optimum is 2 + 3.
TEST(AssignmentSolver, BeatsGreedy) {
  const std::vector<double> cost{1, 2,  //
                                 3, 10};
  EXPECT_EQ(solveAssignment(cost, 2, 2), (std::vector<int32_t>{1, 0}));
}

TEST(AssignmentSolver, RectangularShapes) {
  // Three rows, one column: the cheapest row takes it.
  EXPECT_EQ(solveAssignment({5, 2, 7}, 3, 1), (std::vector<int32_t>{-1, 0, -1}));
  // One row, three columns.
  EXPECT_EQ(solveAssignment({5, 2, 7}, 1, 3), (std::vector<int32_t>{1}));
  EXPECT_EQ(solveAssignment({}, 0, 4), (std::vector<int32_t>{}));
  EXPECT_EQ(solveAssignment({}, 3, 0), (std::vector<int32_t>{-1, -1, -1}));
}

TEST(AssignmentSolver, SameInputSameAnswer) {
  const std::vector<double> ties(16, 1.0);
  const std::vector<int32_t> first = solveAssignment(ties, 4, 4);
  expectValidMatching(first, 4, 4);
  for (int i = 0; i < 5; ++i) EXPECT_EQ(solveAssignment(ties, 4, 4), first);
}

TEST(AssignmentSolver, RejectsBadInput) {
  EXPECT_THROW(solveAssignment({1, 2, 3}, 2, 2), std::invalid_argument);
  EXPECT_THROW(solveAssignment({}, -1, 2), std::invalid_argument);
  EXPECT_THROW(solveAssignment({1, std::numeric_limits<double>::infinity()}, 1, 2),
               std::invalid_argument);
  EXPECT_THROW(solveAssignment({std::numeric_limits<double>::quiet_NaN()}, 1, 1),
               std::invalid_argument);
}

}  // namespace
}  // namespace fleet_core

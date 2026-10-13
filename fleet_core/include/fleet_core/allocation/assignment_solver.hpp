#pragma once

#include <cstdint>
#include <vector>

namespace fleet_core {

// Minimum-cost assignment: the Hungarian method, in its shortest augmenting path form.
//
// cost is a rows x cols matrix in row-major order, every entry finite. Each row gets at most one
// column and each column at most one row. Exactly min(rows, cols) pairs are matched, with the
// smallest total cost any such matching has. Returns the column of every row, or -1 for a row
// left out. Runs in O(k * k * n) for k = min(rows, cols) and n = max(rows, cols).
//
// Deterministic: the same matrix always gives the same matching, ties included.
// Throws std::invalid_argument for a negative size, a size mismatch, or a non-finite cost.
std::vector<int32_t> solveAssignment(const std::vector<double>& cost, int32_t rows, int32_t cols);

}  // namespace fleet_core

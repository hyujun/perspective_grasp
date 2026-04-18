#include "cross_camera_associator/hungarian_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace perspective_grasp {

std::vector<int> HungarianSolver::solveSquare(const Eigen::MatrixXd& cost) {
  const int n = static_cast<int>(cost.rows());
  if (n == 0) return {};

  // Kuhn-Munkres O(n^3) implementation using potentials.
  // Uses 1-indexed arrays internally for cleaner math.
  const double INF = std::numeric_limits<double>::infinity();

  std::vector<double> u(static_cast<std::size_t>(n + 1), 0.0);
  std::vector<double> v(static_cast<std::size_t>(n + 1), 0.0);
  std::vector<int> p(static_cast<std::size_t>(n + 1), 0);    // p[j] = row assigned to col j
  std::vector<int> way(static_cast<std::size_t>(n + 1), 0);  // way[j] = previous col in shortest path

  for (int i = 1; i <= n; ++i) {
    // Assign row i
    p[0] = i;
    int j0 = 0;  // virtual column
    std::vector<double> minv(static_cast<std::size_t>(n + 1), INF);
    std::vector<bool> used(static_cast<std::size_t>(n + 1), false);

    do {
      used[static_cast<std::size_t>(j0)] = true;
      int i0 = p[static_cast<std::size_t>(j0)];
      double delta = INF;
      int j1 = -1;

      for (int j = 1; j <= n; ++j) {
        auto ju = static_cast<std::size_t>(j);
        if (used[ju]) continue;
        double cur = cost(i0 - 1, j - 1) - u[static_cast<std::size_t>(i0)] - v[ju];
        if (cur < minv[ju]) {
          minv[ju] = cur;
          way[ju] = j0;
        }
        if (minv[ju] < delta) {
          delta = minv[ju];
          j1 = j;
        }
      }

      for (int j = 0; j <= n; ++j) {
        auto ju = static_cast<std::size_t>(j);
        if (used[ju]) {
          u[static_cast<std::size_t>(p[ju])] += delta;
          v[ju] -= delta;
        } else {
          minv[ju] -= delta;
        }
      }

      j0 = j1;
    } while (p[static_cast<std::size_t>(j0)] != 0);

    // Unwind the path
    do {
      auto j0u = static_cast<std::size_t>(j0);
      int j1 = way[j0u];
      p[j0u] = p[static_cast<std::size_t>(j1)];
      j0 = j1;
    } while (j0 != 0);
  }

  // Convert to 0-indexed: result[row] = col
  std::vector<int> result(static_cast<std::size_t>(n), -1);
  for (int j = 1; j <= n; ++j) {
    auto ju = static_cast<std::size_t>(j);
    if (p[ju] != 0) {
      result[static_cast<std::size_t>(p[ju] - 1)] = j - 1;
    }
  }
  return result;
}

std::vector<Assignment> HungarianSolver::solve(
    const Eigen::MatrixXd& cost_matrix, double reject_threshold) {
  if (cost_matrix.rows() == 0 || cost_matrix.cols() == 0) {
    return {};
  }

  const int rows = static_cast<int>(cost_matrix.rows());
  const int cols = static_cast<int>(cost_matrix.cols());
  const int n = std::max(rows, cols);

  // Pad to square with large values so padded entries are never preferred.
  const double PAD_VALUE = reject_threshold + 1.0;
  Eigen::MatrixXd square = Eigen::MatrixXd::Constant(n, n, PAD_VALUE);
  square.block(0, 0, rows, cols) = cost_matrix;

  auto assignment = solveSquare(square);

  std::vector<Assignment> result;
  for (int r = 0; r < rows; ++r) {
    int c = assignment[static_cast<std::size_t>(r)];
    if (c >= 0 && c < cols) {
      double cost = cost_matrix(r, c);
      if (std::isfinite(cost) && cost <= reject_threshold) {
        result.push_back({r, c, cost});
      }
    }
  }
  return result;
}

}  // namespace perspective_grasp

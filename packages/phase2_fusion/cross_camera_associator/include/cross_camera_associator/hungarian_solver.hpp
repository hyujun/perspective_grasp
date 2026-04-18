#pragma once

#include <vector>

#include <Eigen/Dense>

namespace perspective_grasp {

/// Result of a single assignment from the Hungarian solver.
struct Assignment {
  int row;
  int col;
  double cost;
};

/// O(n^3) Kuhn-Munkres (Hungarian) algorithm for optimal assignment.
class HungarianSolver {
 public:
  /// Solve the assignment problem on a cost matrix.
  /// @param cost_matrix  Possibly non-square cost matrix (rows x cols).
  /// @param reject_threshold  Assignments with cost > this value are rejected.
  /// @return Vector of accepted assignments.
  static std::vector<Assignment> solve(const Eigen::MatrixXd& cost_matrix,
                                       double reject_threshold);

 private:
  /// Internal solver on a square matrix. Returns assignment[row] = col.
  static std::vector<int> solveSquare(const Eigen::MatrixXd& cost);
};

}  // namespace perspective_grasp

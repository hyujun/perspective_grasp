#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <set>

#include "cross_camera_associator/hungarian_solver.hpp"

using perspective_grasp::Assignment;
using perspective_grasp::HungarianSolver;

TEST(HungarianSolver, SquareIdentityLikeCostMatrix) {
  Eigen::MatrixXd cost(3, 3);
  cost << 1.0, 10.0, 10.0,
          10.0, 2.0, 10.0,
          10.0, 10.0, 3.0;

  auto result = HungarianSolver::solve(cost, 15.0);
  ASSERT_EQ(result.size(), 3u);

  std::set<std::pair<int, int>> pairs;
  double total_cost = 0.0;
  for (const auto& a : result) {
    pairs.insert({a.row, a.col});
    total_cost += a.cost;
  }

  EXPECT_TRUE(pairs.count({0, 0}));
  EXPECT_TRUE(pairs.count({1, 1}));
  EXPECT_TRUE(pairs.count({2, 2}));
  EXPECT_DOUBLE_EQ(total_cost, 6.0);
}

TEST(HungarianSolver, NonSquareRowsLessThanCols) {
  // 2x3 matrix: rows < cols
  Eigen::MatrixXd cost(2, 3);
  cost << 5.0, 1.0, 3.0,
          2.0, 8.0, 7.0;

  auto result = HungarianSolver::solve(cost, 10.0);
  ASSERT_EQ(result.size(), 2u);

  // Optimal: row 0 -> col 1 (cost 1), row 1 -> col 0 (cost 2)
  std::set<std::pair<int, int>> pairs;
  double total_cost = 0.0;
  for (const auto& a : result) {
    pairs.insert({a.row, a.col});
    total_cost += a.cost;
  }
  EXPECT_TRUE(pairs.count({0, 1}));
  EXPECT_TRUE(pairs.count({1, 0}));
  EXPECT_DOUBLE_EQ(total_cost, 3.0);
}

TEST(HungarianSolver, NonSquareRowsMoreThanCols) {
  // 3x2: one row must go unassigned
  Eigen::MatrixXd cost(3, 2);
  cost << 1.0, 9.0,
          9.0, 2.0,
          5.0, 5.0;

  auto result = HungarianSolver::solve(cost, 10.0);
  // At most min(rows,cols) = 2 assignments
  ASSERT_EQ(result.size(), 2u);

  std::set<int> rows_used, cols_used;
  double total_cost = 0.0;
  for (const auto& a : result) {
    rows_used.insert(a.row);
    cols_used.insert(a.col);
    total_cost += a.cost;
  }
  EXPECT_EQ(cols_used.size(), 2u);
  // Optimal is row0->col0 (1) + row1->col1 (2) = 3
  EXPECT_DOUBLE_EQ(total_cost, 3.0);
}

TEST(HungarianSolver, SingleElementMatrix) {
  Eigen::MatrixXd cost(1, 1);
  cost << 4.0;

  auto accepted = HungarianSolver::solve(cost, 5.0);
  ASSERT_EQ(accepted.size(), 1u);
  EXPECT_EQ(accepted[0].row, 0);
  EXPECT_EQ(accepted[0].col, 0);
  EXPECT_DOUBLE_EQ(accepted[0].cost, 4.0);

  auto rejected = HungarianSolver::solve(cost, 3.0);
  EXPECT_TRUE(rejected.empty());
}

TEST(HungarianSolver, AllCostsAboveThreshold) {
  Eigen::MatrixXd cost(2, 2);
  cost << 10.0, 20.0,
          15.0, 25.0;

  auto result = HungarianSolver::solve(cost, 5.0);
  EXPECT_TRUE(result.empty());
}

TEST(HungarianSolver, InfiniteCostEntriesRejected) {
  // Class-mismatch pattern: infinite cost between incompatible pairs.
  // Optimal: row0 -> col1 (2), row1 -> col0 (3) => total 5
  const double INF = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd cost(2, 2);
  cost << INF, 2.0,
          3.0, INF;

  auto result = HungarianSolver::solve(cost, 10.0);
  ASSERT_EQ(result.size(), 2u);
  for (const auto& a : result) {
    EXPECT_TRUE(std::isfinite(a.cost));
  }
  std::set<std::pair<int, int>> pairs;
  for (const auto& a : result) pairs.insert({a.row, a.col});
  EXPECT_TRUE(pairs.count({0, 1}));
  EXPECT_TRUE(pairs.count({1, 0}));
}

TEST(HungarianSolver, PartialThresholdRejection) {
  // Optimal assignment picks diagonal (1 + 4 = 5); anti-diagonal totals 200.
  // Threshold=3 keeps (0,0) but drops (1,1).
  Eigen::MatrixXd cost(2, 2);
  cost << 1.0, 100.0,
          100.0, 4.0;

  auto result = HungarianSolver::solve(cost, 3.0);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].row, 0);
  EXPECT_EQ(result[0].col, 0);
  EXPECT_DOUBLE_EQ(result[0].cost, 1.0);
}

TEST(HungarianSolver, TiedCostsReturnValidAssignment) {
  // All costs equal — solver must still return a valid permutation.
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(3, 3, 2.0);

  auto result = HungarianSolver::solve(cost, 5.0);
  ASSERT_EQ(result.size(), 3u);

  std::set<int> rows_used, cols_used;
  for (const auto& a : result) {
    rows_used.insert(a.row);
    cols_used.insert(a.col);
    EXPECT_DOUBLE_EQ(a.cost, 2.0);
  }
  EXPECT_EQ(rows_used.size(), 3u);
  EXPECT_EQ(cols_used.size(), 3u);
}

TEST(HungarianSolver, RandomSquareMatrixIsValidPermutation) {
  // Sanity: on any valid instance, the solver returns a permutation
  // where each row and each col appears at most once, with finite costs.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(0.0, 10.0);

  const int N = 6;
  Eigen::MatrixXd cost(N, N);
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      cost(i, j) = dist(rng);
    }
  }

  auto result = HungarianSolver::solve(cost, 100.0);
  ASSERT_EQ(result.size(), static_cast<std::size_t>(N));

  std::set<int> rows_used, cols_used;
  for (const auto& a : result) {
    EXPECT_GE(a.row, 0);
    EXPECT_LT(a.row, N);
    EXPECT_GE(a.col, 0);
    EXPECT_LT(a.col, N);
    rows_used.insert(a.row);
    cols_used.insert(a.col);
  }
  EXPECT_EQ(rows_used.size(), static_cast<std::size_t>(N));
  EXPECT_EQ(cols_used.size(), static_cast<std::size_t>(N));
}

TEST(HungarianSolver, EmptyMatrix) {
  Eigen::MatrixXd cost(0, 0);
  auto result = HungarianSolver::solve(cost, 10.0);
  EXPECT_TRUE(result.empty());
}

TEST(HungarianSolver, DegenerateDimensions) {
  // 3x0 and 0x3 should both return empty.
  Eigen::MatrixXd r0(3, 0);
  EXPECT_TRUE(HungarianSolver::solve(r0, 10.0).empty());
  Eigen::MatrixXd c0(0, 3);
  EXPECT_TRUE(HungarianSolver::solve(c0, 10.0).empty());
}

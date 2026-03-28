#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <set>

#include "cross_camera_associator/hungarian_solver.hpp"

using perspective_grasp::Assignment;
using perspective_grasp::HungarianSolver;

TEST(HungarianSolver, SquareIdentityLikeCostMatrix) {
  // 3x3 matrix where diagonal has low cost, off-diagonal has high cost.
  // Optimal: (0,0)=1, (1,1)=2, (2,2)=3 => total=6
  Eigen::MatrixXd cost(3, 3);
  cost << 1.0, 10.0, 10.0,
          10.0, 2.0, 10.0,
          10.0, 10.0, 3.0;

  auto result = HungarianSolver::solve(cost, 15.0);
  ASSERT_EQ(result.size(), 3u);

  // Collect assignments as (row, col) pairs
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

TEST(HungarianSolver, NonSquareMatrix) {
  // 2x3 matrix: rows < cols
  Eigen::MatrixXd cost(2, 3);
  cost << 5.0, 1.0, 3.0,
          2.0, 8.0, 7.0;

  auto result = HungarianSolver::solve(cost, 10.0);
  ASSERT_EQ(result.size(), 2u);

  // Optimal: row 0 -> col 1 (cost 1), row 1 -> col 0 (cost 2), total=3
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

TEST(HungarianSolver, AllCostsAboveThreshold) {
  Eigen::MatrixXd cost(2, 2);
  cost << 10.0, 20.0,
          15.0, 25.0;

  auto result = HungarianSolver::solve(cost, 5.0);
  EXPECT_TRUE(result.empty());
}

TEST(HungarianSolver, EmptyMatrix) {
  Eigen::MatrixXd cost(0, 0);
  auto result = HungarianSolver::solve(cost, 10.0);
  EXPECT_TRUE(result.empty());
}

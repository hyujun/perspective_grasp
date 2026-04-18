#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "cross_camera_associator/global_id_manager.hpp"

using perspective_grasp::GlobalIdManager;

class GlobalIdManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  rclcpp::Time makeTime(double seconds) {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
  }
};

TEST_F(GlobalIdManagerTest, FirstObjectGetsIdZero) {
  GlobalIdManager mgr(0.05);
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  int id = mgr.getOrAssignId(pos, "cup", makeTime(1.0));
  EXPECT_EQ(id, 0);
  EXPECT_EQ(mgr.size(), 1u);
}

TEST_F(GlobalIdManagerTest, SamePositionReusesId) {
  GlobalIdManager mgr(0.05);
  Eigen::Vector3d pos(1.0, 2.0, 3.0);

  int id1 = mgr.getOrAssignId(pos, "cup", makeTime(1.0));
  Eigen::Vector3d pos2(1.01, 2.01, 3.01);  // ~1.7cm away, within threshold
  int id2 = mgr.getOrAssignId(pos2, "cup", makeTime(2.0));

  EXPECT_EQ(id1, id2);
  EXPECT_EQ(mgr.size(), 1u);
}

TEST_F(GlobalIdManagerTest, DifferentPositionGetsNewId) {
  GlobalIdManager mgr(0.05);
  Eigen::Vector3d pos1(1.0, 2.0, 3.0);
  Eigen::Vector3d pos2(5.0, 6.0, 7.0);

  int id1 = mgr.getOrAssignId(pos1, "cup", makeTime(1.0));
  int id2 = mgr.getOrAssignId(pos2, "cup", makeTime(1.0));

  EXPECT_NE(id1, id2);
  EXPECT_EQ(mgr.size(), 2u);
}

TEST_F(GlobalIdManagerTest, DifferentClassSamePositionGetsNewId) {
  // Class name acts as a hard filter: same position but different class
  // must receive a fresh global ID.
  GlobalIdManager mgr(0.05);
  Eigen::Vector3d pos(1.0, 2.0, 3.0);

  int id_cup = mgr.getOrAssignId(pos, "cup", makeTime(1.0));
  int id_bottle = mgr.getOrAssignId(pos, "bottle", makeTime(1.0));

  EXPECT_NE(id_cup, id_bottle);
  EXPECT_EQ(mgr.size(), 2u);
}

TEST_F(GlobalIdManagerTest, ClosestMatchWinsAmongCandidates) {
  // Register two distinct objects far enough apart to stay separate, then
  // query close to one of them and expect its ID to be returned.
  GlobalIdManager mgr(0.02);  // 2cm threshold
  int id_near = mgr.getOrAssignId({0.00, 0.0, 0.0}, "cup", makeTime(1.0));
  int id_far = mgr.getOrAssignId({0.20, 0.0, 0.0}, "cup", makeTime(1.0));
  ASSERT_NE(id_near, id_far);

  // Query 1cm from id_near (20cm from id_far) → within threshold of near only.
  int hit = mgr.getOrAssignId({0.01, 0.0, 0.0}, "cup", makeTime(2.0));
  EXPECT_EQ(hit, id_near);
}

TEST_F(GlobalIdManagerTest, BeyondThresholdAssignsNewId) {
  GlobalIdManager mgr(0.05);
  int id1 = mgr.getOrAssignId({0.0, 0.0, 0.0}, "cup", makeTime(1.0));
  // 10cm shift — outside 5cm threshold
  int id2 = mgr.getOrAssignId({0.10, 0.0, 0.0}, "cup", makeTime(1.0));

  EXPECT_NE(id1, id2);
  EXPECT_EQ(mgr.size(), 2u);
}

TEST_F(GlobalIdManagerTest, StalePruningRemovesOldObjects) {
  GlobalIdManager mgr(0.05);
  mgr.getOrAssignId({1.0, 2.0, 3.0}, "cup", makeTime(1.0));
  EXPECT_EQ(mgr.size(), 1u);

  // age = 4 - 1 = 3s > 2s timeout
  mgr.pruneStaleObjects(2.0, makeTime(4.0));
  EXPECT_EQ(mgr.size(), 0u);
}

TEST_F(GlobalIdManagerTest, PruneOnlyRemovesStaleObjects) {
  GlobalIdManager mgr(0.05);
  mgr.getOrAssignId({0.0, 0.0, 0.0}, "cup", makeTime(1.0));
  mgr.getOrAssignId({1.0, 0.0, 0.0}, "cup", makeTime(5.0));
  EXPECT_EQ(mgr.size(), 2u);

  // At t=6, timeout=2: first object age=5 → prune; second age=1 → keep.
  mgr.pruneStaleObjects(2.0, makeTime(6.0));
  EXPECT_EQ(mgr.size(), 1u);
}

TEST_F(GlobalIdManagerTest, IdsAreMonotonicAcrossPrune) {
  // next_id_ is monotonic: after pruning, a new object still gets a
  // higher ID than the pruned one.
  GlobalIdManager mgr(0.05);
  int id_old = mgr.getOrAssignId({0.0, 0.0, 0.0}, "cup", makeTime(1.0));
  mgr.pruneStaleObjects(0.5, makeTime(5.0));
  EXPECT_EQ(mgr.size(), 0u);

  int id_new = mgr.getOrAssignId({0.0, 0.0, 0.0}, "cup", makeTime(5.0));
  EXPECT_GT(id_new, id_old);
}

TEST_F(GlobalIdManagerTest, UpdatingReusedObjectRefreshesLastSeen) {
  // When an existing object is re-observed, last_seen should update
  // so that pruning respects the new timestamp.
  GlobalIdManager mgr(0.05);
  int first = mgr.getOrAssignId({0.0, 0.0, 0.0}, "cup", makeTime(1.0));
  // Re-observe at t=5
  int second = mgr.getOrAssignId({0.01, 0.0, 0.0}, "cup", makeTime(5.0));
  EXPECT_EQ(first, second);

  // Prune with timeout=2 at t=6: age since last_seen = 6-5 = 1 < 2 → keep.
  mgr.pruneStaleObjects(2.0, makeTime(6.0));
  EXPECT_EQ(mgr.size(), 1u);
}

TEST_F(GlobalIdManagerTest, ResetClearsStateAndIdCounter) {
  GlobalIdManager mgr(0.05);
  mgr.getOrAssignId({0.0, 0.0, 0.0}, "cup", makeTime(1.0));
  mgr.getOrAssignId({1.0, 0.0, 0.0}, "cup", makeTime(1.0));
  EXPECT_EQ(mgr.size(), 2u);

  mgr.reset();
  EXPECT_EQ(mgr.size(), 0u);

  // After reset, IDs restart from 0.
  int id = mgr.getOrAssignId({0.0, 0.0, 0.0}, "cup", makeTime(2.0));
  EXPECT_EQ(id, 0);
}

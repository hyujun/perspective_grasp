#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "cross_camera_associator/global_id_manager.hpp"

using perspective_grasp::GlobalIdManager;

class GlobalIdManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Time makeTime(double seconds) {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
  }

  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(GlobalIdManagerTest, FirstObjectGetsIdZero) {
  GlobalIdManager mgr(0.05);
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  int id = mgr.getOrAssignId(pos, "cup", makeTime(1.0));
  EXPECT_EQ(id, 0);
}

TEST_F(GlobalIdManagerTest, SamePositionReusesId) {
  GlobalIdManager mgr(0.05);
  Eigen::Vector3d pos(1.0, 2.0, 3.0);

  int id1 = mgr.getOrAssignId(pos, "cup", makeTime(1.0));
  // Slightly shifted position within threshold
  Eigen::Vector3d pos2(1.01, 2.01, 3.01);
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

TEST_F(GlobalIdManagerTest, StalePruningRemovesOldObjects) {
  GlobalIdManager mgr(0.05);
  Eigen::Vector3d pos(1.0, 2.0, 3.0);

  mgr.getOrAssignId(pos, "cup", makeTime(1.0));
  EXPECT_EQ(mgr.size(), 1u);

  // Prune with timeout=2s at t=4.0 (object last seen at t=1.0, age=3s > 2s)
  mgr.pruneStaleObjects(2.0, makeTime(4.0));
  EXPECT_EQ(mgr.size(), 0u);
}

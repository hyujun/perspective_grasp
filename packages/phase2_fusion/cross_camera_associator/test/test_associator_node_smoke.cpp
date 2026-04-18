#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <perception_msgs/msg/associated_pose_array.hpp>

#include "cross_camera_associator/associator_node.hpp"

using namespace std::chrono_literals;

namespace {

class AssociatorSmokeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

rclcpp::NodeOptions makeOptions(
    const std::vector<std::string>& camera_namespaces) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"camera_namespaces", camera_namespaces},
      {"base_frame", std::string("ur5e_base_link")},
      {"association_distance_threshold", 0.05},
      {"lambda_rot", 0.1},
      {"temporal_threshold", 0.10},
      {"max_snapshot_age_ms", 50.0},
      {"association_rate_hz", 30.0},
      {"stale_object_timeout_sec", 3.0},
  });
  return opts;
}

}  // namespace

TEST_F(AssociatorSmokeTest, ConstructsWithDefaultOptions) {
  auto node =
      std::make_shared<perspective_grasp::AssociatorNode>(makeOptions({"/cam0"}));
  EXPECT_STREQ(node->get_name(), "cross_camera_associator");
}

TEST_F(AssociatorSmokeTest, ConstructsWithMultipleCameras) {
  auto node = std::make_shared<perspective_grasp::AssociatorNode>(
      makeOptions({"/cam0", "/cam1", "/cam2"}));
  EXPECT_STREQ(node->get_name(), "cross_camera_associator");
  EXPECT_EQ(
      node->get_parameter("camera_namespaces").as_string_array().size(), 3u);
}

TEST_F(AssociatorSmokeTest, PublishesAssociatedPosesTopic) {
  // Subscribe to /associated/poses and verify the publisher is advertised.
  auto node = std::make_shared<perspective_grasp::AssociatorNode>(
      makeOptions({"/cam0", "/cam1"}));

  auto sub_node = std::make_shared<rclcpp::Node>("associated_poses_listener");
  auto sub = sub_node->create_subscription<
      perception_msgs::msg::AssociatedPoseArray>(
      "/associated/poses", rclcpp::QoS(1).best_effort(),
      [](perception_msgs::msg::AssociatedPoseArray::ConstSharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(sub_node);

  // Spin briefly so the timer fires at least once and no crash occurs.
  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_EQ(sub->get_publisher_count(), 1u);
}

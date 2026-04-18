#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <perception_msgs/msg/pose_with_meta.hpp>
#include <perception_msgs/msg/pose_with_meta_array.hpp>

#include "pose_graph_smoother/smoother_node.hpp"

using namespace std::chrono_literals;
using perspective_grasp::SmootherNode;
using LifecycleState = lifecycle_msgs::msg::State;

namespace {

class SmootherSmokeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

rclcpp::NodeOptions makeOptions() {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"fallback_parent_frame",
       std::string("camera_color_optical_frame")},
  });
  return opts;
}

[[maybe_unused]] perception_msgs::msg::PoseWithMetaArray makeArray(
    int object_id, double x, double y, double z,
    const std::string& frame = "camera_color_optical_frame") {
  perception_msgs::msg::PoseWithMetaArray arr;
  arr.header.frame_id = frame;
  perception_msgs::msg::PoseWithMeta pwm;
  pwm.object_id = object_id;
  pwm.class_name = "cup";
  pwm.source = "iekf";
  pwm.confidence = 1.0f;
  pwm.pose.header = arr.header;
  pwm.pose.pose.position.x = x;
  pwm.pose.pose.position.y = y;
  pwm.pose.pose.position.z = z;
  pwm.pose.pose.orientation.w = 1.0;
  arr.poses.push_back(pwm);
  return arr;
}

}  // namespace

// --------------------------------------------------------------------------
// Construction & parameters
// --------------------------------------------------------------------------

TEST_F(SmootherSmokeTest, ConstructsAsUnconfigured) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  EXPECT_STREQ(node->get_name(), "pose_graph_smoother");
  EXPECT_EQ(node->get_current_state().id(),
            LifecycleState::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SmootherSmokeTest, FallbackParentFrameParameterIsRead) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  EXPECT_EQ(node->get_parameter("fallback_parent_frame").as_string(),
            "camera_color_optical_frame");
}

// --------------------------------------------------------------------------
// Lifecycle transitions
// --------------------------------------------------------------------------

TEST_F(SmootherSmokeTest, ConfigureTransitionSucceeds) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  auto ret = node->configure();
  EXPECT_EQ(ret.id(), LifecycleState::PRIMARY_STATE_INACTIVE);
}

TEST_F(SmootherSmokeTest, ActivateAfterConfigureReachesActive) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  ASSERT_EQ(node->configure().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  auto ret = node->activate();
  EXPECT_EQ(ret.id(), LifecycleState::PRIMARY_STATE_ACTIVE);
}

TEST_F(SmootherSmokeTest, DeactivateReturnsToInactive) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  ASSERT_EQ(node->configure().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), LifecycleState::PRIMARY_STATE_ACTIVE);
  auto ret = node->deactivate();
  EXPECT_EQ(ret.id(), LifecycleState::PRIMARY_STATE_INACTIVE);
}

TEST_F(SmootherSmokeTest, CleanupReturnsToUnconfigured) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  ASSERT_EQ(node->configure().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  auto ret = node->cleanup();
  EXPECT_EQ(ret.id(), LifecycleState::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SmootherSmokeTest, FullCycleConfigureActivateDeactivateCleanup) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  ASSERT_EQ(node->configure().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), LifecycleState::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node->deactivate().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), LifecycleState::PRIMARY_STATE_UNCONFIGURED);
}

// --------------------------------------------------------------------------
// Passthrough behaviour — active in both GTSAM and non-GTSAM builds, since
// the sliding-window optimizer is not implemented yet.
// --------------------------------------------------------------------------

TEST_F(SmootherSmokeTest, PassthroughRepublishesWhenActivated) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  ASSERT_EQ(node->configure().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), LifecycleState::PRIMARY_STATE_ACTIVE);

  auto talker = std::make_shared<rclcpp::Node>("filter_stub");
  auto pub = talker->create_publisher<perception_msgs::msg::PoseWithMetaArray>(
      "/pose_filter/filtered_poses", rclcpp::QoS(1).best_effort());

  auto listener = std::make_shared<rclcpp::Node>("smoothed_listener");
  int received = 0;
  double last_x = 0.0;
  auto sub = listener->create_subscription<
      perception_msgs::msg::PoseWithMetaArray>(
      "/smoother/smoothed_poses", rclcpp::QoS(1).best_effort(),
      [&](perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg) {
        if (!msg->poses.empty()) {
          ++received;
          last_x = msg->poses.front().pose.pose.position.x;
        }
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(talker);
  exec.add_node(listener);

  const auto warmup = std::chrono::steady_clock::now() + 150ms;
  while (std::chrono::steady_clock::now() < warmup) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(makeArray(7, 0.25, 0.0, 0.0));
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_GT(received, 0) << "Smoother did not republish any smoothed poses";
  EXPECT_NEAR(last_x, 0.25, 1e-9);
}

TEST_F(SmootherSmokeTest, PreservesUpstreamFrameId) {
  // Multi-camera convention publishes filtered poses in ur5e_base_link.
  // The smoother must NOT overwrite that with the fallback parent frame.
  auto node = std::make_shared<SmootherNode>(makeOptions());
  ASSERT_EQ(node->configure().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), LifecycleState::PRIMARY_STATE_ACTIVE);

  auto talker = std::make_shared<rclcpp::Node>("filter_stub");
  auto pub = talker->create_publisher<perception_msgs::msg::PoseWithMetaArray>(
      "/pose_filter/filtered_poses", rclcpp::QoS(1).best_effort());

  auto listener = std::make_shared<rclcpp::Node>("smoothed_listener");
  std::string last_frame;
  auto sub = listener->create_subscription<
      perception_msgs::msg::PoseWithMetaArray>(
      "/smoother/smoothed_poses", rclcpp::QoS(1).best_effort(),
      [&](perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg) {
        last_frame = msg->header.frame_id;
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(talker);
  exec.add_node(listener);

  const auto warmup = std::chrono::steady_clock::now() + 150ms;
  while (std::chrono::steady_clock::now() < warmup) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  const auto deadline = std::chrono::steady_clock::now() + 400ms;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(makeArray(3, 0.1, 0.0, 0.0, "ur5e_base_link"));
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_EQ(last_frame, "ur5e_base_link")
      << "Smoother overrode upstream frame with fallback";
}

TEST_F(SmootherSmokeTest, DeactivatedDoesNotPublish) {
  auto node = std::make_shared<SmootherNode>(makeOptions());
  ASSERT_EQ(node->configure().id(), LifecycleState::PRIMARY_STATE_INACTIVE);
  // Deliberately do NOT activate; publisher is inactive.

  auto talker = std::make_shared<rclcpp::Node>("filter_stub");
  auto pub = talker->create_publisher<perception_msgs::msg::PoseWithMetaArray>(
      "/pose_filter/filtered_poses", rclcpp::QoS(1).best_effort());

  auto listener = std::make_shared<rclcpp::Node>("smoothed_listener");
  int received = 0;
  auto sub = listener->create_subscription<
      perception_msgs::msg::PoseWithMetaArray>(
      "/smoother/smoothed_poses", rclcpp::QoS(1).best_effort(),
      [&](perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr) {
        ++received;
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(talker);
  exec.add_node(listener);

  const auto deadline = std::chrono::steady_clock::now() + 400ms;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(makeArray(1, 0.1, 0.0, 0.0));
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_EQ(received, 0)
      << "Smoother published while the lifecycle publisher was inactive";
}


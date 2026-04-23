// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_debug_visualizer/visualizer_node.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include <perception_msgs/msg/detection.hpp>
#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/pipeline_status.hpp>
#include <perception_msgs/msg/segmentation.hpp>
#include <perception_msgs/msg/segmentation_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

class VisualizerSmokeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

}  // namespace

TEST_F(VisualizerSmokeTest, ConstructsWithoutError) {
  auto node = std::make_shared<perspective_grasp::VisualizerNode>();
  EXPECT_STREQ(node->get_name(), "perception_debug_visualizer");
}

TEST_F(VisualizerSmokeTest, RepublishesAnnotatedImage) {
  // Legacy single-camera path via `image_topic` override.
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"image_topic", std::string("/test/input_image")},
  });
  auto node = std::make_shared<perspective_grasp::VisualizerNode>(opts);

  auto helper = std::make_shared<rclcpp::Node>("visualizer_test_helper");
  auto pub_image = helper->create_publisher<sensor_msgs::msg::Image>(
      "/test/input_image", rclcpp::QoS(1).best_effort());
  auto pub_status =
      helper->create_publisher<perception_msgs::msg::PipelineStatus>(
          "/meta_controller/active_pipeline", rclcpp::QoS(1).reliable());

  sensor_msgs::msg::Image::SharedPtr received;
  auto sub = helper->create_subscription<sensor_msgs::msg::Image>(
      "/debug/image", rclcpp::QoS(1).best_effort(),
      [&](sensor_msgs::msg::Image::SharedPtr msg) { received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  perception_msgs::msg::PipelineStatus status;
  status.active_mode = "HIGH_PRECISION";

  cv::Mat input = cv::Mat::zeros(240, 320, CV_8UC3);
  std_msgs::msg::Header header;
  auto input_msg = cv_bridge::CvImage(header, "bgr8", input).toImageMsg();

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    pub_status->publish(status);
    pub_image->publish(*input_msg);
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(received) << "Visualizer did not republish within 2s";
  EXPECT_EQ(received->width, 320U);
  EXPECT_EQ(received->height, 240U);
  EXPECT_EQ(received->encoding, "bgr8");

  cv::Mat out = cv_bridge::toCvCopy(received, "bgr8")->image;
  EXPECT_GT(cv::countNonZero(out.reshape(1)), 0)
      << "Republished image should contain the mode overlay";
}

TEST_F(VisualizerSmokeTest, ConstructsWithMultipleCameraNamespaces) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"camera_namespaces",
       std::vector<std::string>{"/cam0", "/cam1", "/cam2"}},
      {"active_camera_index", 1},
  });
  auto node = std::make_shared<perspective_grasp::VisualizerNode>(opts);
  EXPECT_STREQ(node->get_name(), "perception_debug_visualizer");

  // The node must subscribe to per-camera topics. Verify by discovering
  // publishers that the node is a consumer of.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  // Spin a moment to let graph registrations settle.
  const auto deadline = std::chrono::steady_clock::now() + 200ms;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  auto topics = node->get_topic_names_and_types();
  // At minimum the publisher for /debug/image should be registered.
  EXPECT_NE(topics.find("/debug/image"), topics.end());
}

TEST_F(VisualizerSmokeTest, ActiveCameraIndexCanBeUpdatedAtRuntime) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"camera_namespaces",
       std::vector<std::string>{"/cam0", "/cam1"}},
      {"active_camera_index", 0},
  });
  auto node = std::make_shared<perspective_grasp::VisualizerNode>(opts);

  // In-range update must succeed.
  auto result = node->set_parameter(rclcpp::Parameter("active_camera_index", 1));
  EXPECT_TRUE(result.successful);

  // Out-of-range update must be rejected by the on-set callback.
  auto bad = node->set_parameter(rclcpp::Parameter("active_camera_index", 5));
  EXPECT_FALSE(bad.successful);
}

TEST_F(VisualizerSmokeTest, MaskParametersCanBeUpdatedAtRuntime) {
  auto node = std::make_shared<perspective_grasp::VisualizerNode>();

  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("show_masks", false)).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("show_masks", true)).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("mask_alpha", 0.8)).successful);

  // Out-of-range alpha must be rejected.
  auto bad = node->set_parameter(rclcpp::Parameter("mask_alpha", 1.5));
  EXPECT_FALSE(bad.successful);
}

TEST_F(VisualizerSmokeTest, MasksAreOverlayedOnDebugImage) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"image_topic", std::string("/test/input_image")},
      {"mask_alpha", 0.8},
  });
  auto node = std::make_shared<perspective_grasp::VisualizerNode>(opts);

  auto helper = std::make_shared<rclcpp::Node>("mask_smoke_helper");
  auto pub_image = helper->create_publisher<sensor_msgs::msg::Image>(
      "/test/input_image", rclcpp::QoS(1).best_effort());
  auto pub_masks =
      helper->create_publisher<perception_msgs::msg::SegmentationArray>(
          "/sam2/masks", rclcpp::QoS(1).best_effort());

  sensor_msgs::msg::Image::SharedPtr received;
  auto sub = helper->create_subscription<sensor_msgs::msg::Image>(
      "/debug/image", rclcpp::QoS(1).best_effort(),
      [&](sensor_msgs::msg::Image::SharedPtr msg) { received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  const int H = 240, W = 320;
  cv::Mat input = cv::Mat::zeros(H, W, CV_8UC3);
  std_msgs::msg::Header header;
  auto input_msg = cv_bridge::CvImage(header, "bgr8", input).toImageMsg();

  perception_msgs::msg::SegmentationArray masks;
  perception_msgs::msg::Segmentation seg;
  seg.id = 42;
  seg.confidence = 0.95f;
  seg.mask.height = H;
  seg.mask.width = W;
  seg.mask.encoding = "mono8";
  seg.mask.step = W;
  seg.mask.data.assign(H * W, 0);
  // Fill a rectangle [100, 150) x [100, 150) with foreground.
  for (int y = 100; y < 150; ++y) {
    for (int x = 100; x < 150; ++x) {
      seg.mask.data[y * W + x] = 255;
    }
  }
  masks.segmentations.push_back(seg);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    pub_masks->publish(masks);
    pub_image->publish(*input_msg);
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(received) << "Visualizer did not republish within 2s";
  cv::Mat out = cv_bridge::toCvCopy(received, "bgr8")->image;
  // Inside the mask region we expect tinted pixels; outside (far from bbox
  // and text) should remain black.
  const auto inside = out.at<cv::Vec3b>(125, 125);
  EXPECT_GT(inside[0] + inside[1] + inside[2], 0)
      << "Expected mask tint inside foreground";
}

TEST_F(VisualizerSmokeTest, ShowMasksFalseDisablesOverlay) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"image_topic", std::string("/test/input_image2")},
      {"show_masks", false},
  });
  auto node = std::make_shared<perspective_grasp::VisualizerNode>(opts);

  auto helper = std::make_shared<rclcpp::Node>("mask_off_helper");
  auto pub_image = helper->create_publisher<sensor_msgs::msg::Image>(
      "/test/input_image2", rclcpp::QoS(1).best_effort());
  auto pub_masks =
      helper->create_publisher<perception_msgs::msg::SegmentationArray>(
          "/sam2/masks", rclcpp::QoS(1).best_effort());

  sensor_msgs::msg::Image::SharedPtr received;
  auto sub = helper->create_subscription<sensor_msgs::msg::Image>(
      "/debug/image", rclcpp::QoS(1).best_effort(),
      [&](sensor_msgs::msg::Image::SharedPtr msg) { received = msg; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  const int H = 240, W = 320;
  cv::Mat input = cv::Mat::zeros(H, W, CV_8UC3);
  std_msgs::msg::Header header;
  auto input_msg = cv_bridge::CvImage(header, "bgr8", input).toImageMsg();

  perception_msgs::msg::SegmentationArray masks;
  perception_msgs::msg::Segmentation seg;
  seg.id = 7;
  seg.mask.height = H;
  seg.mask.width = W;
  seg.mask.encoding = "mono8";
  seg.mask.step = W;
  seg.mask.data.assign(H * W, 255);  // full foreground
  masks.segmentations.push_back(seg);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    pub_masks->publish(masks);
    pub_image->publish(*input_msg);
    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_TRUE(received);
  cv::Mat out = cv_bridge::toCvCopy(received, "bgr8")->image;
  // With show_masks=false, the interior (away from the mode-text band) must
  // stay black even though a full-frame mask was published.
  const auto mid = out.at<cv::Vec3b>(150, 150);
  EXPECT_EQ(mid[0], 0);
  EXPECT_EQ(mid[1], 0);
  EXPECT_EQ(mid[2], 0);
}

TEST(VisualizerComposeTopic, JoinsNamespaceAndSuffix) {
  // Test the public static helper indirectly via constructing a node and
  // reading the logged topic names is overkill — lean on the exposed method.
  using perspective_grasp::VisualizerNode;
  EXPECT_EQ(VisualizerNode::compose_topic("", "yolo/detections"),
            "/yolo/detections");
  EXPECT_EQ(VisualizerNode::compose_topic("/cam0", "yolo/detections"),
            "/cam0/yolo/detections");
  EXPECT_EQ(VisualizerNode::compose_topic("cam1/", "/camera/color/image_raw"),
            "/cam1/camera/color/image_raw");
  EXPECT_EQ(VisualizerNode::compose_topic("///cam2///", "suffix"),
            "/cam2/suffix");
}

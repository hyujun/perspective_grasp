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
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <memory>
#include <thread>

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

  // First push a mode so the overlay includes it.
  perception_msgs::msg::PipelineStatus status;
  status.active_mode = "HIGH_PRECISION";

  // Publish image + status, then spin until we get a republished frame.
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

  // The republished image must carry the mode overlay → more non-zero pixels
  // than the all-zero input.
  cv::Mat out = cv_bridge::toCvCopy(received, "bgr8")->image;
  EXPECT_GT(cv::countNonZero(out.reshape(1)), 0)
      << "Republished image should contain the mode overlay";
}

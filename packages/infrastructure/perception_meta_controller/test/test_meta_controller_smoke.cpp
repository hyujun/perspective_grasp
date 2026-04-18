// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_meta_controller/meta_controller_node.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <perception_msgs/msg/pipeline_status.hpp>
#include <perception_msgs/srv/set_mode.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace {

class MetaControllerSmokeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    // Allow tests to share a single init; shutdown happens at process exit.
  }

  /// Run a SetMode service call end-to-end and return the response.
  perception_msgs::srv::SetMode::Response callSetMode(
      const std::shared_ptr<perspective_grasp::MetaControllerNode>& node,
      const std::string& mode) {
    auto client_node = std::make_shared<rclcpp::Node>("set_mode_test_client");
    auto client = client_node->create_client<perception_msgs::srv::SetMode>(
        "/meta_controller/set_mode");

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(client_node);

    // Wait for service
    const auto start = std::chrono::steady_clock::now();
    while (!client->wait_for_service(10ms)) {
      exec.spin_some();
      if (std::chrono::steady_clock::now() - start > 2s) {
        ADD_FAILURE() << "SetMode service never appeared";
        return perception_msgs::srv::SetMode::Response{};
      }
    }

    auto request = std::make_shared<perception_msgs::srv::SetMode::Request>();
    request->mode = mode;
    auto future = client->async_send_request(request);

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (future.wait_for(10ms) != std::future_status::ready) {
      exec.spin_some();
      if (std::chrono::steady_clock::now() > deadline) {
        ADD_FAILURE() << "SetMode request timed out";
        return perception_msgs::srv::SetMode::Response{};
      }
    }
    return *future.get();
  }
};

}  // namespace

TEST_F(MetaControllerSmokeTest, ConstructsWithoutError) {
  auto node = std::make_shared<perspective_grasp::MetaControllerNode>();
  EXPECT_STREQ(node->get_name(), "perception_meta_controller");
}

TEST_F(MetaControllerSmokeTest, AcceptsKnownMode) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"default_mode", std::string("NORMAL")},
      {"num_cameras", 1},
      {"status_publish_rate_hz", 10.0},
  });
  auto node = std::make_shared<perspective_grasp::MetaControllerNode>(opts);

  auto resp = callSetMode(node, "HIGH_PRECISION");
  EXPECT_TRUE(resp.success);
  EXPECT_EQ(resp.active_pipeline, "HIGH_PRECISION");
}

TEST_F(MetaControllerSmokeTest, RejectsUnknownMode) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"default_mode", std::string("NORMAL")},
      {"num_cameras", 1},
      {"status_publish_rate_hz", 10.0},
  });
  auto node = std::make_shared<perspective_grasp::MetaControllerNode>(opts);

  auto resp = callSetMode(node, "NOT_A_MODE");
  EXPECT_FALSE(resp.success);
  // Active pipeline must not change on rejection.
  EXPECT_EQ(resp.active_pipeline, "NORMAL");
}

TEST_F(MetaControllerSmokeTest, PublishesStatusOnTimer) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"default_mode", std::string("NORMAL")},
      {"num_cameras", 2},
      {"status_publish_rate_hz", 20.0},
  });
  auto node = std::make_shared<perspective_grasp::MetaControllerNode>(opts);

  auto sub_node = std::make_shared<rclcpp::Node>("status_subscriber");
  perception_msgs::msg::PipelineStatus last_status;
  int received = 0;
  auto sub = sub_node->create_subscription<perception_msgs::msg::PipelineStatus>(
      "/meta_controller/active_pipeline", rclcpp::QoS(1).reliable(),
      [&](const perception_msgs::msg::PipelineStatus::SharedPtr msg) {
        last_status = *msg;
        ++received;
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(sub_node);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (received < 2 && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_GE(received, 1) << "No PipelineStatus messages received within 2s";
  EXPECT_EQ(last_status.active_mode, "NORMAL");
  // num_cameras=2 → cross_camera_associator must appear.
  bool has_associator = false;
  for (const auto& n : last_status.active_nodes) {
    if (n == "cross_camera_associator") {
      has_associator = true;
      break;
    }
  }
  EXPECT_TRUE(has_associator);
}

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_merge_node/merge_node.hpp"

using namespace std::chrono_literals;

namespace {

class MergeNodeSmokeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

rclcpp::NodeOptions makeOptions(
    const std::vector<std::string>& source_topics) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"source_topics", source_topics},
      {"target_frame", std::string("ur5e_base_link")},
      {"voxel_leaf_size", 0.002},
      {"table_height", 0.0},
      {"max_object_height", 0.30},
      {"outlier_mean_k", 30},
      {"outlier_stddev", 1.5},
      {"overlap_radius", 0.005},
      {"sync_slop_ms", 5.0},
      {"sync_queue_size", 10},
  });
  return opts;
}

void spinBriefly(const std::shared_ptr<rclcpp::Node>& node,
                 std::chrono::milliseconds duration) {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
}

}  // namespace

TEST_F(MergeNodeSmokeTest, PassthroughModeWithSingleSource) {
  auto node = std::make_shared<perspective_grasp::MergeNode>(
      makeOptions({"/cam0/points"}));
  EXPECT_STREQ(node->get_name(), "pcl_merge_node");
  spinBriefly(node, 200ms);
}

TEST_F(MergeNodeSmokeTest, Merge2ModeWithTwoSources) {
  auto node = std::make_shared<perspective_grasp::MergeNode>(
      makeOptions({"/cam0/points", "/cam1/points"}));
  EXPECT_STREQ(node->get_name(), "pcl_merge_node");
  spinBriefly(node, 200ms);
}

TEST_F(MergeNodeSmokeTest, MergeNModeWithThreeSources) {
  auto node = std::make_shared<perspective_grasp::MergeNode>(
      makeOptions({"/cam0/points", "/cam1/points", "/cam2/points"}));
  EXPECT_STREQ(node->get_name(), "pcl_merge_node");
  spinBriefly(node, 200ms);
}

TEST_F(MergeNodeSmokeTest, AdvertisesMergedPointsTopic) {
  auto node = std::make_shared<perspective_grasp::MergeNode>(
      makeOptions({"/cam0/points", "/cam1/points"}));

  auto sub_node = std::make_shared<rclcpp::Node>("merged_points_listener");
  auto sub = sub_node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/merged/points", rclcpp::SensorDataQoS().keep_last(1),
      [](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(sub_node);
  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_EQ(sub->get_publisher_count(), 1u);
}

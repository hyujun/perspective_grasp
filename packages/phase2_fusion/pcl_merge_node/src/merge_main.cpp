#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pcl_merge_node/merge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perspective_grasp::MergeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

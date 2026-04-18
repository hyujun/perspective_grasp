#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pose_filter_cpp/pose_filter_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perspective_grasp::PoseFilterNode>());
  rclcpp::shutdown();
  return 0;
}

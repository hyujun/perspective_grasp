#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cross_camera_associator/associator_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perspective_grasp::AssociatorNode>());
  rclcpp::shutdown();
  return 0;
}

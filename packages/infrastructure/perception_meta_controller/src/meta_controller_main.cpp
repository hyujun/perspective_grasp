#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "perception_meta_controller/meta_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perspective_grasp::MetaControllerNode>());
  rclcpp::shutdown();
  return 0;
}

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "perception_debug_visualizer/visualizer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perspective_grasp::VisualizerNode>());
  rclcpp::shutdown();
  return 0;
}

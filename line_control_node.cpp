#include "rclcpp/rclcpp.hpp"
#include "line_control/line_control.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // Initializes ROS2
  auto node = std::make_shared<LineControl>(); // Creates an instance of LineControl class
  RCLCPP_INFO(node->get_logger(), "go to spin"); // Log info message
  rclcpp::spin(node); // Spins the node to keep it active and processing callbacks
  rclcpp::shutdown(); // Shuts down the ROS2 node after spinning
  return 0;
}

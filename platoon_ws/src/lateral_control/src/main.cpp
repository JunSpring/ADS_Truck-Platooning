#include "lateral_control/lateral_control_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lateral_control::LateralControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}

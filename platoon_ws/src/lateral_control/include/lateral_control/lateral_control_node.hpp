#ifndef LATERAL_CONTROL__LATERAL_CONTROL_NODE_HPP_
#define LATERAL_CONTROL__LATERAL_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>

#include <lateral_control/pure_pursuit.hpp>  // PurePursuit 선언
#include <vector>
#include <utility>
#include <optional>
#include <memory>

namespace lateral_control
{

class LateralControlNode : public rclcpp::Node
{
public:
  explicit LateralControlNode();

private:
  /* Callbacks and Methods */
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void publishSteering(double steer_deg);

  /* Members */
  int truck_id_{0};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_pub_;
  std::unique_ptr<PurePursuit> pp_;
  double prev_steering_{0.0};
  double look_ahead_distance_{20.0};
};

}  // namespace lateral_control

#endif  // LATERAL_CONTROL__LATERAL_CONTROL_NODE_HPP_

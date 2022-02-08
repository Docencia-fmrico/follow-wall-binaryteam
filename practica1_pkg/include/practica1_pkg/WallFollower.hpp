#ifndef PRACTICA1_PKG__WALLFOLLOWER_HPP_
#define PRACTICA1_PKG__WALLFOLLOWER_HPP_

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using namespace std::chrono_literals;

class WallFollower : public rclcpp_lifecycle::LifecycleNode {
  enum State {
    OBSTACLE = 0,
    FREE_WAY
  };

public:
  WallFollower();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void behaviour();

  void turn_left();

  void move_in_a_curve();

protected:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
  // Intelligent pointer to a velocity publisher.
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  State state_ = FREE_WAY;
};
#endif  // PRACTICA1_PKG__WALLFOLLOWER_HPP_

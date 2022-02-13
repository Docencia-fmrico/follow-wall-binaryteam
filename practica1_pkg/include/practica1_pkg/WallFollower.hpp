// Copyright 2020 Binary-Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PRACTICA1_PKG__WALLFOLLOWER_HPP_
#define PRACTICA1_PKG__WALLFOLLOWER_HPP_

#include <memory>
#include <vector>
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using rcl_interfaces::msg::ParameterType;

class WallFollower : public rclcpp_lifecycle::LifecycleNode
{
  enum State
  {
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

  geometry_msgs::msg::Twist getCurvature();

protected:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  float min_distance_in_the_cone(std::vector<float> ranges, int cone_start, int cone_end);

private:
  // Intelligent pointer to a velocity publisher.
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  State state_ = FREE_WAY;
  rclcpp::Time last_obstacle_ts_;
};
#endif  // PRACTICA1_PKG__WALLFOLLOWER_HPP_

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
#include  <vector>
#include "practica1_pkg/WallFollower.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

WallFollower::WallFollower()
: rclcpp_lifecycle::LifecycleNode("follower_node")
{
  velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("nav_vel", 100);
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_raw", 1, std::bind(&WallFollower::laser_callback, this, _1));
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT WallFollower::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT WallFollower::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

  velocity_pub_->on_activate();
  timer_ = create_wall_timer(50ms, std::bind(&WallFollower::behaviour, this));
  last_obstacle_ts_ = now();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT WallFollower::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

  velocity_pub_->on_deactivate();
  timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT WallFollower::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());

  velocity_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT WallFollower::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());

  velocity_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT WallFollower::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

void WallFollower::turn_left()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.0;
  msg.angular.z = 0.5;
  velocity_pub_->publish(msg);
}

geometry_msgs::msg::Twist WallFollower::getCurvature()
{
  geometry_msgs::msg::Twist msg;

  float default_linear = 0.25;
  float default_angular = -0.3;

  auto time_without_converge = now() - last_obstacle_ts_;

  // Change curve because of the time
  msg.linear.x = default_linear;

  if (time_without_converge.seconds() > 16) {
    msg.angular.z = default_angular + 0.2;
  } else {
    msg.angular.z = default_angular;
  }
  return msg;
}

void WallFollower::move_in_a_curve()
{
  velocity_pub_->publish(getCurvature());
}

void WallFollower::behaviour()
{
  if (state_ == OBSTACLE) {
    RCLCPP_INFO(get_logger(), "STATE: OBSTACLE");
    turn_left();
  } else if (state_ == FREE_WAY) {
    RCLCPP_INFO(get_logger(), "STATE: FREE_WAY");
    move_in_a_curve();
  }
}

float WallFollower::min_distance_in_the_cone(
  std::vector<float> ranges, int cone_start, int cone_end)
{
  float min_distance = 25;
  for (int i = cone_start; i < cone_end; i++) {
    if (ranges[i] < min_distance) {
      min_distance = ranges[i];
    }
  }
  return min_distance;
}

void WallFollower::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int cone_start = 120;
  int cone_end = 506;

  float min_distance = min_distance_in_the_cone(msg->ranges, cone_start, cone_end);

  float OBSTACLE_DISTANCE = 0.40;

  if (min_distance < OBSTACLE_DISTANCE) {
    state_ = OBSTACLE;
    last_obstacle_ts_ = now();
  } else {
    state_ = FREE_WAY;
  }
}

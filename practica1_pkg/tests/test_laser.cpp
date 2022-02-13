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

#include <memory>
#include <limits>
#include <vector>
#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "practica1_pkg/WallFollower.hpp"

class WallFollowerTest : public WallFollower
{
public:
  float min_distance_in_the_cone_test(std::vector<float> ranges, int cone_start, int cone_end)
  {
    return min_distance_in_the_cone(ranges, cone_start, cone_end);
  }
};

TEST(test_laser, test_laser_min_out_cone)
{
  auto node = std::make_shared<WallFollowerTest>();

  int cone_start = 120;
  int cone_end = 506;
  std::vector<float> ranges(666, 25);
  // Number out of cone, must not be the minimum
  ranges.at(100) = 0.07;
  ASSERT_EQ(node->min_distance_in_the_cone_test(ranges, cone_start, cone_end), 25);
}

TEST(test_laser, test_laser_min_in_cone)
{
  auto node = std::make_shared<WallFollowerTest>();

  int cone_start = 120;
  int cone_end = 506;
  std::vector<float> ranges(666, 25);

  ranges.at(120) = 0.05;
  ranges.at(500) = 0.09;
  ranges.at(150) = 0.05;
  ASSERT_FLOAT_EQ(node->min_distance_in_the_cone_test(ranges, cone_start, cone_end), 0.05);
}

TEST(test_laser, support_infinity)
{
  auto node = std::make_shared<WallFollowerTest>();

  int cone_start = 120;
  int cone_end = 506;
  std::vector<float> ranges(666, std::numeric_limits<float>::infinity());

  ranges.at(150) = 0.5;
  ASSERT_FLOAT_EQ(node->min_distance_in_the_cone_test(ranges, cone_start, cone_end), 0.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}

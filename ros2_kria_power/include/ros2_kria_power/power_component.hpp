/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
     \   \
     /   /
    /___/   /\
    \   \  /  \
     \___\/\___\

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef COMPOSITION__POWER_COMPONENT_HPP_
#define COMPOSITION__POWER_COMPONENT_HPP_

#include "ros2_kria_power/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ros2_kria_msgs/msg/power.hpp>
#include <chrono>


namespace composition
{

class Power : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit Power(const rclcpp::NodeOptions & options);

protected:
  void on_timer();
  void initialize();

private:
  size_t count_;
  rclcpp::Publisher<ros2_kria_msgs::msg::Power>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds dt_;
  char filename_[255];
  long power_reading_;
};

}  // namespace composition

#endif  // COMPOSITION__POWER_COMPONENT_HPP_

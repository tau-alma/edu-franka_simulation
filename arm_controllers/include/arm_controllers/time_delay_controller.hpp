// Copyright (c) 2025 Tampere University, Autonomous Mobile Machines
//
// Licensed under the MIT License.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define num_joints 7

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace arm_controllers {

class TimeDelayController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string robot_description_;
  Vector7d q_;
  Vector7d dq_;
  void updateJointStates();

  double last_time_;
  double count_;

  double init_pos_[num_joints];
  double desired_cmd_[num_joints];
  double desired_cmd_old_[num_joints];
  double desired_pos_[num_joints];
  double current_pos_[num_joints];
  double desired_vel_[num_joints];
  double current_vel_[num_joints];
  double current_vel_old_[num_joints];
  double desired_acc_[num_joints];
  double current_acc_[num_joints];
  double error_[num_joints];
  double error_vel_[num_joints];
  double ded_[num_joints];
  double tde_[num_joints];

  double mag_;
  double feq_;

  double Mbar_[num_joints];

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_desired_pos_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_pos_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_desired_vel_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_vel_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_desired_acc_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_acc_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_error_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_error_vel_;

  std_msgs::msg::Float64MultiArray msg_desired_pos_;
  std_msgs::msg::Float64MultiArray msg_current_pos_;
  std_msgs::msg::Float64MultiArray msg_desired_vel_;
  std_msgs::msg::Float64MultiArray msg_current_vel_;
  std_msgs::msg::Float64MultiArray msg_desired_acc_;
  std_msgs::msg::Float64MultiArray msg_current_acc_;
  std_msgs::msg::Float64MultiArray msg_error_;
  std_msgs::msg::Float64MultiArray msg_error_vel_;

};

}  // namespace arm_controllers

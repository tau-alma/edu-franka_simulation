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

#include <arm_controllers/time_delay_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <Eigen/Eigen>

namespace arm_controllers {

controller_interface::InterfaceConfiguration
TimeDelayController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Add the effort control interfaces for the joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
TimeDelayController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add the position and velocity state interfaces for the joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

controller_interface::return_type TimeDelayController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {

  // Update the joint states    
  updateJointStates();
  
  // time delay controller functionality:
  for(int i = 0; i < num_joints; ++i)
  {
    // Calculate desired position using sinusoidal function
    desired_pos_[i] = init_pos_[i] + mag_ * sin(2 * PI * feq_ * period.seconds() * count_);
    // Get current position from joint state
    current_pos_[i] = q_(i);

    // Calculate desired velocity using derivative of sinusoidal function
    desired_vel_[i] = mag_ * 2 * PI * feq_ * cos(2 * PI * feq_ * period.seconds() * count_);
    // Get current velocity from joint state
    current_vel_[i] = dq_(i);

    // Calculate desired acceleration using second derivative of sinusoidal function
    desired_acc_[i] = -mag_ * 2 * PI * feq_ * 2 * PI * feq_ * sin(2 * PI * feq_ * period.seconds() * count_);
    // Calculate current acceleration
    current_acc_[i] = (current_vel_[i] - current_vel_old_[i]) / period.seconds();
  
    // Calculate position error
    error_[i] = desired_pos_[i] - current_pos_[i];
    // Calculate velocity error
    error_vel_[i] = desired_vel_[i] - current_vel_[i];

    // Calculate desired acceleration with PD control
    ded_[i] = desired_acc_[i] + 20.0 * error_vel_[i] + 100.0 * error_[i];
    // Calculate time delay error
    tde_[i] = desired_cmd_old_[i] - Mbar_[i] * current_acc_[i];
  
    // Calculate desired command
    desired_cmd_[i] = Mbar_[i] * ded_[i] + tde_[i];

    // Set command to the joint
    command_interfaces_[i].set_value(desired_cmd_[i]);

    // Update old values for next iteration
    current_vel_old_[i] = current_vel_[i];
    desired_cmd_old_[i] = desired_cmd_[i];				
  }

  // Clear the data arrays
  msg_desired_pos_.data.clear();
  msg_current_pos_.data.clear();
  msg_desired_vel_.data.clear();
  msg_current_vel_.data.clear();
  msg_desired_acc_.data.clear();
  msg_current_acc_.data.clear();
  msg_error_.data.clear();
  msg_error_vel_.data.clear();

  // Update the last time and count
  last_time_ = time.seconds();
	count_++;

    // Fill the data arrays with the calculated values
    for(int i=0; i<num_joints; i++)
    {
      msg_desired_pos_.data.push_back(desired_pos_[i]*R2D);
      msg_current_pos_.data.push_back(current_pos_[i]*R2D);
      msg_desired_vel_.data.push_back(desired_vel_[i]*R2D);
      msg_current_vel_.data.push_back(current_vel_[i]*R2D);
      msg_desired_acc_.data.push_back(desired_acc_[i]*R2D);
      msg_current_acc_.data.push_back(current_acc_[i]*R2D);
      msg_error_.data.push_back(error_[i]*R2D);
      msg_error_vel_.data.push_back(error_vel_[i]*R2D);

    }	

  // Publish data to topics
  pub_desired_pos_->publish(msg_desired_pos_);
  pub_current_pos_->publish(msg_current_pos_);
  pub_desired_vel_->publish(msg_desired_vel_);
  pub_current_vel_->publish(msg_current_vel_);
  pub_desired_acc_->publish(msg_desired_acc_);
  pub_current_acc_->publish(msg_current_acc_);
  pub_error_->publish(msg_error_);
  pub_error_vel_->publish(msg_error_vel_);

  return controller_interface::return_type::OK;
}

CallbackReturn TimeDelayController::on_init() {
  auto node = get_node();  // Access the node from the base class
  
  // Create publishers for the desired and current positions, velocities, accelerations, errors
  pub_desired_pos_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("desired_pos", 10);
  pub_current_pos_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("current_pos", 10);
  pub_desired_vel_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("desired_vel", 10);
  pub_current_vel_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("current_vel", 10);
  pub_desired_acc_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("desired_acc", 10);
  pub_current_acc_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("current_acc", 10);
  pub_error_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("error", 10);
  pub_error_vel_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("error_vel", 10);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TimeDelayController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Get the robot description parameter
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();

  // Check if the parameter was retrieved successfully
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TimeDelayController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Initialize the joint states
  updateJointStates();

  // Initialize the time and count
  last_time_ = 0.0;
  count_ = 0.0;

  // Initialize the desired and current values
  for (int i = 0; i < num_joints; i++) {
      init_pos_[i] = q_(i);

      desired_cmd_[i] = 0.0;
      desired_cmd_old_[i] = 0.0;
      desired_pos_[i] = 0.0;
      current_pos_[i] = 0.0;
      desired_vel_[i] = 0.0;
      current_vel_[i] = 0.0;
      current_vel_old_[i] = 0.0;
      desired_acc_[i] = 0.0;
      current_acc_[i] = 0.0;

      error_[i] = 0.0;
      error_vel_[i] = 0.0;
      ded_[i] = 0.0;
      tde_[i] = 0.0;
  }

  // Initialize the magnitude and frequency of the sinusoidal function
  mag_ = 45.0 * D2R;
  feq_ = 0.5;

  // Initialize the Mbar values
  Mbar_[0] = 0.01;
  Mbar_[1] = 0.01;
  Mbar_[2] = 0.005;
  Mbar_[3] = 0.0015;
  Mbar_[4] = 0.001;
  Mbar_[5] = 0.00012;
  Mbar_[6] = 0.00012;

  // Activate the publishers
  pub_desired_pos_->on_activate();
  pub_current_pos_->on_activate();
  pub_desired_vel_->on_activate();
  pub_current_vel_->on_activate();
  pub_desired_acc_->on_activate();
  pub_current_acc_->on_activate();
  pub_error_->on_activate();
  pub_error_vel_->on_activate();


  return CallbackReturn::SUCCESS;
}

void TimeDelayController::updateJointStates() {
  // Pre-check array size to avoid bounds checking in loop
  if (state_interfaces_.size() != 2 * num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
    return;
  }

  // Get the current joint positions and velocities
  auto* interfaces = state_interfaces_.data();
  for (size_t i = 0; i < num_joints; ++i) {
    // Access interfaces directly with pointer arithmetic
    const auto& position_interface = interfaces[2 * i];
    const auto& velocity_interface = interfaces[2 * i + 1];
    
    // Interface name comparison
    const auto& pos_name = position_interface.get_interface_name();
    const auto& vel_name = velocity_interface.get_interface_name();
    
    if (pos_name != "position") {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s", 
                   pos_name.c_str());
      return;
    }
    if (vel_name != "velocity") {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s", 
                   vel_name.c_str());
      return;
    }

    // Store the joint states
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace arm_controllers
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_controllers::TimeDelayController,
                       controller_interface::ControllerInterface)

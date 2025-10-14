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

#include <arm_controllers/computed_torque_controller.hpp>

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
ComputedTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Add the effort control interfaces for the joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
ComputedTorqueController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add the position and velocity state interfaces for the joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

controller_interface::return_type ComputedTorqueController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  // Update the joint states
  updateJointStates();

  // double dt = period.seconds();
  t = t + 0.001;

  // save position and velocity values to q_ and qdot_ for each joint
  for (int i = 0; i < num_joints; i++)
  {
      q_(i) = position_interface_values_(i);
      qdot_(i) = velocity_interface_values_(i);
  }

  // Calculate the desired Trajecoty in Joint Space
  for (size_t i = 0; i < num_joints; i++)
  {
      //qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t); 
      //qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t);          
      //qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2 * t);
      qd_(i)      = 0.0;
      qd_dot_(i)  = 0.0;
      qd_ddot_(i) = 0.0;
  }

  // Motion Controller in Joint Space:
  // - Error Definition in Joint Space 
  e_.data = qd_.data - q_.data;
  e_dot_.data = qd_dot_.data - qdot_.data;
  e_int_.data = qd_.data - q_.data; 

  // Compute model(M,C,G) 
  id_solver_->JntToMass(q_, M_);
  id_solver_->JntToCoriolis(q_, qdot_, C_);
  id_solver_->JntToGravity(q_, G_); 


  // Convert KDL::JntArray and KDL::JntSpaceInertiaMatrix to Eigen matrices
  Eigen::MatrixXd M_eigen = M_.data;
  Eigen::VectorXd qd_ddot_eigen = qd_ddot_.data;
  Eigen::VectorXd Kp_eigen = Kp_.data;
  Eigen::VectorXd e_eigen = e_.data;
  Eigen::VectorXd Kd_eigen = Kd_.data;
  Eigen::VectorXd e_dot_eigen = e_dot_.data;


  // DEBUG: statements to print matrix dimensions

  // RCLCPP_INFO(get_node()->get_logger(), "M_ dimensions: %ld x %ld", M_eigen.rows(), M_eigen.cols());
  // RCLCPP_INFO(get_node()->get_logger(), "qd_ddot_ size: %ld", qd_ddot_eigen.size());
  // RCLCPP_INFO(get_node()->get_logger(), "Kp_ size: %ld", Kp_eigen.size());
  // RCLCPP_INFO(get_node()->get_logger(), "e_ size: %ld", e_eigen.size());
  // RCLCPP_INFO(get_node()->get_logger(), "Kd_ size: %ld", Kd_eigen.size());
  // RCLCPP_INFO(get_node()->get_logger(), "e_dot_ size: %ld", e_dot_eigen.size());

  // RCLCPP_INFO(get_node()->get_logger(), "q_ size: %ld", q_.data.size());
  // RCLCPP_INFO(get_node()->get_logger(), "qdot_ size: %ld", qdot_.data.size());
  // RCLCPP_INFO(get_node()->get_logger(), "qd_ size: %ld", qd_.data.size());

  //RCLCPP_INFO(get_node()->get_logger(), "q_ data: %f, %f, %f, %f, %f, %f, %f, %f", q_.data[0], q_.data[1], q_.data[2], q_.data[3], q_.data[4], q_.data[5], q_.data[6], q_.data[7]);

  // if (M_eigen.rows() != num_joints)
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "CHECK DIMENSIONS");
  //   return controller_interface::return_type::ERROR;
  // }

  // Apply Torque Command to Actuator
  //aux_d_.data = M_eigen * (qd_ddot_eigen + Kp_eigen.cwiseProduct(e_eigen) + Kd_eigen.cwiseProduct(e_dot_eigen));
  //comp_d_.data = C_.data + G_.data;
  //tau_d_.data = aux_d_.data + comp_d_.data;
  
  aux_d_.data  = -(Kp_.data.cwiseProduct(q_.data) + Kd_.data.cwiseProduct(qdot_.data)); // = -Kp*q - Kd*qdot
  tau_d_.data  = M_.data * aux_d_.data + C_.data + G_.data;


  for (int i = 0; i < num_joints; i++)
  {
      // For torque command:
      command_interfaces_[i].set_value(tau_d_(i));
      // For no control command (zero torque for the joints):
      // command_interfaces_[i].set_value(0);
  }

  // save_data
  // Simulation time (unit: sec)
  SaveData_[0] = t;

  // Desired position in joint space (unit: rad)
  SaveData_[1] = qd_(0);
  SaveData_[2] = qd_(1);
  SaveData_[3] = qd_(2);
  SaveData_[4] = qd_(3);
  SaveData_[5] = qd_(4);
  SaveData_[6] = qd_(5);
  SaveData_[7] = qd_(6);

  // Desired velocity in joint space (unit: rad/s)
  SaveData_[8] = qd_dot_(0);
  SaveData_[9] = qd_dot_(1);
  SaveData_[10] = qd_dot_(2);
  SaveData_[11] = qd_dot_(3);
  SaveData_[12] = qd_dot_(4);
  SaveData_[13] = qd_dot_(5);
  SaveData_[14] = qd_dot_(6);

  // Desired acceleration in joint space (unit: rad/s^2)
  SaveData_[15] = qd_ddot_(0);
  SaveData_[16] = qd_ddot_(1);
  SaveData_[17] = qd_ddot_(2);
  SaveData_[18] = qd_ddot_(3);
  SaveData_[19] = qd_ddot_(4);
  SaveData_[20] = qd_ddot_(5);
  SaveData_[21] = qd_ddot_(6);

  // Actual position in joint space (unit: rad)
  SaveData_[22] = q_(0);
  SaveData_[23] = q_(1);
  SaveData_[24] = q_(2);
  SaveData_[25] = q_(3);
  SaveData_[26] = q_(4);
  SaveData_[27] = q_(5);
  SaveData_[28] = q_(6);

  // Actual velocity in joint space (unit: rad/s)
  SaveData_[29] = qdot_(0);
  SaveData_[30] = qdot_(1);
  SaveData_[31] = qdot_(2);
  SaveData_[32] = qdot_(3);
  SaveData_[33] = qdot_(4);
  SaveData_[34] = qdot_(5);
  SaveData_[35] = qdot_(6);

  // Error position in joint space (unit: rad)
  SaveData_[36] = e_(0);
  SaveData_[37] = e_(1);
  SaveData_[38] = e_(2);
  SaveData_[39] = e_(3);
  SaveData_[40] = e_(4);
  SaveData_[41] = e_(5);
  SaveData_[42] = e_(6);

  // Error velocity in joint space (unit: rad/s)
  SaveData_[43] = e_dot_(0);
  SaveData_[44] = e_dot_(1);
  SaveData_[45] = e_dot_(2);
  SaveData_[46] = e_dot_(3);
  SaveData_[47] = e_dot_(4);
  SaveData_[48] = e_dot_(5);
  SaveData_[49] = e_dot_(6);

  // Error intergal value in joint space (unit: rad*sec)
  SaveData_[50] = e_int_(0);
  SaveData_[51] = e_int_(1);
  SaveData_[52] = e_int_(2);
  SaveData_[53] = e_int_(3);
  SaveData_[54] = e_int_(4);
  SaveData_[55] = e_int_(5);
  SaveData_[56] = e_int_(6);

 // Clear the data arrays
  msg_qd_.data.clear();
  msg_q_.data.clear();
  msg_e_.data.clear();
  msg_SaveData_.data.clear();

  // Fill the data arrays with the calculated values
  for (int i = 0; i < num_joints; i++)
  {
      msg_qd_.data.push_back(qd_(i));
      msg_q_.data.push_back(q_(i));
      msg_e_.data.push_back(e_(i));
  }
  // Fill the data arrays with the calculated values
  for (int i = 0; i < SaveDataMax; i++)
  {
      msg_SaveData_.data.push_back(SaveData_[i]);
  }

  // Publish data to topics
  pub_qd_->publish(msg_qd_);
  pub_q_->publish(msg_q_);
  pub_e_->publish(msg_e_);
  pub_SaveData_->publish(msg_SaveData_);


  // print_state every 100 iterations (0.1 sec) (uncomment to print)

  // static int count = 0;
  // if (count > 99)
  // {
  //   printf("*********************************************************\n\n");
  //   printf("*** Simulation Time (unit: sec)  ***\n");
  //   printf("t = %f\n", t);
  //   printf("\n");

  //   printf("*** Desired State in Joint Space (unit: deg) ***\n");
  //   printf("qd_(0): %f, ", qd_(0)*R2D);
  //   printf("qd_(1): %f, ", qd_(1)*R2D);
  //   printf("qd_(2): %f, ", qd_(2)*R2D);
  //   printf("qd_(3): %f, ", qd_(3)*R2D);
  //   printf("qd_(4): %f, ", qd_(4)*R2D);
  //   printf("qd_(5): %f, ", qd_(5)*R2D);
  //   printf("qd_(6): %f\n", qd_(6)*R2D);
  //   printf("\n");

  //   printf("*** Actual State in Joint Space (unit: deg) ***\n");
  //   printf("q_(0): %f, ", q_(0) * R2D);
  //   printf("q_(1): %f, ", q_(1) * R2D);
  //   printf("q_(2): %f, ", q_(2) * R2D);
  //   printf("q_(3): %f, ", q_(3) * R2D);
  //   printf("q_(4): %f, ", q_(4) * R2D);
  //   printf("q_(5): %f, ", q_(5) * R2D);
  //   printf("q_(6): %f\n", q_(6) * R2D);
  //   printf("\n");

  //   printf("*** Joint Space Error (unit: deg)  ***\n");
  //   printf("%f, ", R2D * e_(0));
  //   printf("%f, ", R2D * e_(1));
  //   printf("%f, ", R2D * e_(2));
  //   printf("%f, ", R2D * e_(3));
  //   printf("%f, ", R2D * e_(4));
  //   printf("%f, ", R2D * e_(5));
  //   printf("%f\n", R2D * e_(6));
  //   printf("\n");

  //   count = 0;
  // }
  // count++;

  return controller_interface::return_type::OK;
}

CallbackReturn ComputedTorqueController::on_init() {

  RCLCPP_INFO(get_node()->get_logger(), "Robot Initialization (on_init) started");

  // Initialize the joint names, gains, and the number of joints
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("p_gains", {});
    auto_declare<std::vector<double>>("i_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  M_.resize(num_joints);
  C_.resize(num_joints);
  G_.resize(num_joints);
  
  // Set the gravity vector
  gravity_ = KDL::Vector::Zero(); 
  gravity_(2) = -9.81;            

  // Initialize the KDL variables
  tau_d_.data = Eigen::VectorXd::Zero(num_joints);
  qd_.data = Eigen::VectorXd::Zero(num_joints);
  qd_dot_.data = Eigen::VectorXd::Zero(num_joints);
  qd_ddot_.data = Eigen::VectorXd::Zero(num_joints);
  q_.data = Eigen::VectorXd::Zero(num_joints);
  qdot_.data = Eigen::VectorXd::Zero(num_joints);
  e_.data = Eigen::VectorXd::Zero(num_joints);
  e_dot_.data = Eigen::VectorXd::Zero(num_joints);
  e_int_.data = Eigen::VectorXd::Zero(num_joints);

  // Create publishers for the desired and current joint positions, velocities, and accelerations
  pub_qd_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("qd", 1000);
  pub_q_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("q", 1000);
  pub_e_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("e", 1000);
  pub_SaveData_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("SaveData", 1000);

  RCLCPP_INFO(get_node()->get_logger(), "Robot Initialization (on_init) done");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ComputedTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  RCLCPP_INFO(get_node()->get_logger(), "Robot Configuration (on_configure) started");

  // Get the robot description parameter
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();

  // Check if the robot description was retrieved successfully
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    return CallbackReturn::FAILURE;
  }

  // Get the joint names from the parameter server
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  if (joint_names_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names_ not set");
    return CallbackReturn::FAILURE;
  }
  // Check if there are the correct number of joint names
  if (joint_names_.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names_ should be of size %d but is of size %ld",
                 num_joints, joint_names_.size());
    return CallbackReturn::FAILURE;
  }

  // Get the node's namespace
  Kp_.resize(num_joints);
  Kd_.resize(num_joints);
  Ki_.resize(num_joints);

  // Get the gains from the parameter server
  std::vector<double> Kp(num_joints), Ki(num_joints), Kd(num_joints);
  auto p_gains = get_node()->get_parameter("p_gains").as_double_array();
  auto i_gains = get_node()->get_parameter("i_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  // Check if the gains were retrieved successfully
  if (p_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "p_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (p_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "p_gains should be of size %d but is of size %ld",
                 num_joints, p_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (i_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "i_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (i_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "i_gains should be of size %d but is of size %ld",
                 num_joints, i_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }

  // Set the gains for the controller
  for (int i = 0; i < num_joints; ++i) {
    Kp_(i) = p_gains.at(i);
    Kp[i] = p_gains.at(i);
    Ki_(i) = i_gains.at(i);
    Ki[i] = i_gains.at(i);
    Kd_(i) = d_gains.at(i);
    Kd[i] = d_gains.at(i);
  }

  // Get the URDF model and the joint URDF objects
  urdf::Model urdf;
  if (!urdf.initString(robot_description_))
  {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
      return CallbackReturn::ERROR;
  }
  else
  {
      RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
  }

  // Get the joint URDF objects
  for (int i = 0; i < num_joints; i++)
  {
    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
    if (!joint_urdf)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
        return CallbackReturn::ERROR;
    }
    joint_urdfs_.push_back(joint_urdf);
  }

  // Get the KDL tree from the robot description
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct kdl tree");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Constructed kdl tree");
  }

  // Get the root and tip link names from the parameter server
  // If the parameter is not found, return an error
  std::string root_name, tip_name;
  if (get_node()->has_parameter("root_link"))
  {
    root_name = get_node()->get_parameter("root_link").as_string();
    RCLCPP_INFO(get_node()->get_logger(), "Found root link name form yaml: %s", root_name.c_str());
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find root link name");
    return CallbackReturn::ERROR;
  }
  if (get_node()->has_parameter("tip_link"))
  {
    tip_name = get_node()->get_parameter("tip_link").as_string();
    RCLCPP_INFO(get_node()->get_logger(), "Found tip link name form yaml: %s", tip_name.c_str());
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find tip link name");
    return CallbackReturn::ERROR;
  }

  // Get the KDL chain from the KDL tree
  // if kdl tree has no chain from root to tip, return error
  if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_name << " --> " << tip_name);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }

    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_name.c_str(), tip_name.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", kdl_tree_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", kdl_tree_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_tree_ segments are:");

    // Print the segments of the KDL tree
    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;
    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
      RCLCPP_INFO(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", kdl_chain_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", kdl_chain_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); i++) {
        const KDL::Segment& segment = kdl_chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
  }

  // Create the KDL chain dyn param solver
  id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

  M_.resize(kdl_chain_.getNrOfJoints());
  C_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());

  // print kdltree, kdlchain, jointnames, jointurdfs for learning purposes
  fprintf(stderr, "Number of segments in kdl_tree_: %d\n", kdl_tree_.getNrOfSegments());
  fprintf(stderr, "Number of joints in kdl_chain_: %d\n", kdl_chain_.getNrOfJoints());
  fprintf(stderr, "Joint names in joint_names_: ");
  for (int i = 0; i < num_joints; i++)
  {
    fprintf(stderr, "%s ", joint_names_[i].c_str());
  }
  fprintf(stderr, "\n");

  RCLCPP_INFO(get_node()->get_logger(), "Robot Configuration (on_configure) done");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ComputedTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Initialize the joint states
  updateJointStates();

  // Initialize the KDL variables
  M_.data.setZero();
  C_.data.setZero();
  G_.data.setZero();

  t = 0.0;  // Initialize the simulation time variable

  // Initialize the variables
  qd_.resize(num_joints);
  qd_dot_.resize(num_joints);
  qd_ddot_.resize(num_joints);
  q_.resize(num_joints);
  qdot_.resize(num_joints);
  e_.resize(num_joints);
  e_dot_.resize(num_joints);
  e_int_.resize(num_joints);

  aux_d_.resize(num_joints);
  comp_d_.resize(num_joints);
  tau_d_.resize(num_joints);

  Kp_.resize(num_joints);
  Ki_.resize(num_joints);
  Kd_.resize(num_joints);

  for (int i = 0; i < SaveDataMax; i++) {
    SaveData_[i] = 0.0;
  }

  // Activate the publishers
  pub_qd_->on_activate();
  pub_q_->on_activate();
  pub_e_->on_activate();
  pub_SaveData_->on_activate();

  return CallbackReturn::SUCCESS;
}

void ComputedTorqueController::updateJointStates() {
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

    // Direct value assignment
    position_interface_values_(i) = position_interface.get_value();
    velocity_interface_values_(i) = velocity_interface.get_value();
  }
}

}  // namespace arm_controllers
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_controllers::ComputedTorqueController,
                       controller_interface::ControllerInterface)

// Copyright (c) 2025 Tampere University, Autonomous Mobile Machines
// MIT License

#include <memory>
#include <string>
#include <vector>
#include <exception>
#include <cmath>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace arm_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr int kNumTask = 6;         // [x y z roll pitch yaw]
static constexpr int kNumJoints = 7;       // Panda has 7 joints
static constexpr double R2D = 180.0 / M_PI;

class ComputedTorqueCLIKController : public controller_interface::ControllerInterface {
public:
  ComputedTorqueCLIKController() = default;
  ~ComputedTorqueCLIKController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // effort command interfaces for Panda
    for (int i = 1; i <= kNumJoints; ++i) {
      cfg.names.push_back("panda_joint" + std::to_string(i) + "/effort");
    }
    return cfg;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= kNumJoints; ++i) {
      cfg.names.push_back("panda_joint" + std::to_string(i) + "/position");
      cfg.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
    }
    return cfg;
  }

  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration& period) override {
    // 0) read joint states
    if (state_interfaces_.size() != 2 * kNumJoints) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces: %zu", state_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }
    for (int i = 0; i < kNumJoints; ++i) {
      q_(i)     = state_interfaces_[2 * i + 0].get_value();
      qdot_(i)  = state_interfaces_[2 * i + 1].get_value();
    }

    const double dt = period.seconds();
    t_ += dt;

    // 1) FK & Jacobian at current q
    fk_pos_solver_->JntToCart(q_, x_);
    jnt_to_jac_solver_->JntToJac(q_, J_);
    xdot_ = J_.data * qdot_.data; // 6x1

    // 2) desired task-space trajectory
    if (ctr_obj_ == 1) { // regulation
      if (!command_active_) {
        xd_.p = KDL::Vector(0.30, 0.0, 0.40);
        xd_.M = KDL::Rotation::RPY(0, 0, 0);
        xd_dot_.setZero();
        xd_ddot_.setZero();
      } else {
        xd_.p = KDL::Vector(x_cmd_(0), x_cmd_(1), x_cmd_(2));
        xd_.M = KDL::Rotation::RPY(x_cmd_(3), x_cmd_(4), x_cmd_(5));
        xd_dot_.setZero();
        xd_ddot_.setZero();
      }
    } else {            // simple tracking in Y
      const double A = 0.1, w = M_PI; // amplitude & frequency
      xd_.p = KDL::Vector(0.30, 0.20 + A * std::sin(w * (t_ - t_set_)), 0.40);
      xd_.M = KDL::Rotation::RPY(0, 0, 0);
      xd_dot_ << 0.0, A * w * std::cos(w * (t_ - t_set_)), 0.0, 0.0, 0.0, 0.0;
      xd_ddot_ << 0.0, -A * w * w * std::sin(w * (t_ - t_set_)), 0.0, 0.0, 0.0, 0.0;
    }

    // 3) task-space errors (RPY error via KDL::diff)
    KDL::Twist ex_twist = KDL::diff(x_, xd_); // returns 6x1 twist from xd to x (note the order)
    ex_ << ex_twist.vel.x(), ex_twist.vel.y(), ex_twist.vel.z(),
            ex_twist.rot.x(), ex_twist.rot.y(), ex_twist.rot.z();
    ex_dot_ = xd_dot_ - xdot_;

    // 4) CLIK (open-loop or closed-loop) to get qd
    // Use pseudo-inverse J^T (damped) for robustness
    const Eigen::MatrixXd JT = J_.data.transpose();                 // 7x6
    const Eigen::MatrixXd JJt = J_.data * JT;                        // 6x6
    Eigen::MatrixXd JJt_damped = JJt + lambda2_ * Eigen::MatrixXd::Identity(kNumTask, kNumTask);
    // Prefer a solver over explicit inverse to avoid Eigen inverse symbol issues
    Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(kNumTask, kNumTask);
    Eigen::MatrixXd JJt_inv = JJt_damped.ldlt().solve(I6);
    Eigen::MatrixXd J_pinv = JT * JJt_inv;                           // 7x6

    if (ctr_obj_ == 1) { // regulation
      qd_.data = qd_old_.data + (JT * (K_regulation_ * ex_)) * dt;   // 7x1
    } else { // tracking
      if (ik_mode_ == 1) { // open-loop: qd += J^+ * xdot_d * dt
        qd_.data = qd_old_.data + (J_pinv * xd_dot_) * dt;
      } else { // closed-loop: qd += J^+ * (xdot_d + K*ex) * dt
        qd_.data = qd_old_.data + (J_pinv * (xd_dot_ + K_tracking_ * ex_)) * dt;
      }
    }
    qd_old_.data = qd_.data;
    qd_dot_.data.setZero();
    qd_ddot_.data.setZero();

    // 5) joint-space PD + computed torque
    e_.data      = qd_.data - q_.data;
    e_dot_.data  = qd_dot_.data - qdot_.data;
    e_int_.data += e_.data * dt;

    id_solver_->JntToMass(q_, M_);
    id_solver_->JntToCoriolis(q_, qdot_, C_);
    id_solver_->JntToGravity(q_, G_);

    // tau = M(q)[qd_ddot + Kp*e + Kd*e_dot] + C + G
    aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
    comp_d_.data = C_.data + G_.data;
    tau_d_.data = aux_d_.data + comp_d_.data;

    for (int i = 0; i < kNumJoints; ++i) {
      command_interfaces_[i].set_value(tau_d_(i));
    }

    // (optional) publish debug
    if (++print_counter_ >= 100) {
      print_counter_ = 0;
      RCLCPP_INFO(get_node()->get_logger(), "t=%.3f  |  ex[mm]=[%.1f %.1f %.1f]  er[deg]=[%.1f %.1f %.1f]",
                  t_, 1000*ex_(0), 1000*ex_(1), 1000*ex_(2), R2D*ex_(3), R2D*ex_(4), R2D*ex_(5));
    }

    // publish arrays
    publish_arrays();

    return controller_interface::return_type::OK;
  }

  CallbackReturn on_init() override {
    // declare params
    try {
      auto_declare<std::vector<std::string>>("joints", {});
      auto_declare<std::vector<double>>("p_gains", {});
      auto_declare<std::vector<double>>("i_gains", {});
      auto_declare<std::vector<double>>("d_gains", {});
      auto_declare<std::string>("root_link", "panda_link0");
      auto_declare<std::string>("tip_link", "panda_link8");
      auto_declare<int>("ctr_obj", 1);        // 1: regulation, 2: tracking
      auto_declare<int>("ik_mode", 2);        // 1: open-loop, 2: closed-loop
      auto_declare<double>("K_regulation", 2.0);
      auto_declare<double>("K_tracking",  2.0);
      auto_declare<double>("damping",     1e-6); // for damped pinv
    } catch (const std::exception &e) {
      fprintf(stderr, "Exception during on_init: %s\n", e.what());
      return CallbackReturn::ERROR;
    }

    // publishers
    pub_qd_  = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("qd", 10);
    pub_q_   = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("q", 10);
    pub_e_   = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("e", 10);
    pub_xd_  = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("xd", 10);
    pub_x_   = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("x", 10);
    pub_ex_  = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("ex", 10);

    // subscriber for task-space commands [x y z r p y]
    sub_cmd_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "command", 1,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        if (msg->data.size() != kNumTask) {
          RCLCPP_ERROR(get_node()->get_logger(), "command size %zu != %d", msg->data.size(), kNumTask);
          return;
        }
        for (int i = 0; i < kNumTask; ++i) x_cmd_(i) = msg->data[i];
        command_active_ = true;
      });

    // init arrays
    q_.data        = Eigen::VectorXd::Zero(kNumJoints);
    qdot_.data     = Eigen::VectorXd::Zero(kNumJoints);
    qd_.data       = Eigen::VectorXd::Zero(kNumJoints);
    qd_old_.data   = Eigen::VectorXd::Zero(kNumJoints);
    qd_dot_.data   = Eigen::VectorXd::Zero(kNumJoints);
    qd_ddot_.data  = Eigen::VectorXd::Zero(kNumJoints);

    e_.data        = Eigen::VectorXd::Zero(kNumJoints);
    e_dot_.data    = Eigen::VectorXd::Zero(kNumJoints);
    e_int_.data    = Eigen::VectorXd::Zero(kNumJoints);

    tau_d_.data    = Eigen::VectorXd::Zero(kNumJoints);
    aux_d_.data    = Eigen::VectorXd::Zero(kNumJoints);
    comp_d_.data   = Eigen::VectorXd::Zero(kNumJoints);

    xdot_.setZero(); xd_dot_.setZero(); xd_ddot_.setZero(); ex_.setZero(); ex_dot_.setZero();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    // fetch URDF from /robot_state_publisher
    auto params_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    params_client->wait_for_service();
    auto fut = params_client->get_parameters({"robot_description"});
    auto res = fut.get();
    if (res.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description not available");
      return CallbackReturn::FAILURE;
    }
    robot_description_ = res.front().value_to_string();

    // read controller params
    ctr_obj_     = get_node()->get_parameter("ctr_obj").as_int();
    ik_mode_     = get_node()->get_parameter("ik_mode").as_int();
    K_regulation_= get_node()->get_parameter("K_regulation").as_double();
    K_tracking_  = get_node()->get_parameter("K_tracking").as_double();
    lambda2_     = get_node()->get_parameter("damping").as_double();

    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.size() != static_cast<size_t>(kNumJoints)) {
      RCLCPP_ERROR(get_node()->get_logger(), "joints size must be %d", kNumJoints);
      return CallbackReturn::FAILURE;
    }

    // gains
    auto pg = get_node()->get_parameter("p_gains").as_double_array();
    auto ig = get_node()->get_parameter("i_gains").as_double_array();
    auto dg = get_node()->get_parameter("d_gains").as_double_array();
    if (pg.size()!=kNumJoints || ig.size()!=kNumJoints || dg.size()!=kNumJoints) {
      RCLCPP_ERROR(get_node()->get_logger(), "gain vectors must be length %d", kNumJoints);
      return CallbackReturn::FAILURE;
    }
    Kp_.resize(kNumJoints); Ki_.resize(kNumJoints); Kd_.resize(kNumJoints);
    for (int i=0;i<kNumJoints;++i){ Kp_(i)=pg[i]; Ki_(i)=ig[i]; Kd_(i)=dg[i]; }

    // URDF -> KDL
    urdf::Model urdf;
    if (!urdf.initString(robot_description_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF");
      return CallbackReturn::FAILURE;
    }
    for (const auto &jn : joint_names_) {
      auto ju = urdf.getJoint(jn);
      if (!ju) { RCLCPP_ERROR(get_node()->get_logger(), "Joint %s not in URDF", jn.c_str()); return CallbackReturn::FAILURE; }
      joint_urdfs_.push_back(ju);
    }
    if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree");
      return CallbackReturn::FAILURE;
    }

    const std::string root = get_node()->get_parameter("root_link").as_string();
    const std::string tip  = get_node()->get_parameter("tip_link").as_string();
    if (!kdl_tree_.getChain(root, tip, kdl_chain_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "No chain %s -> %s", root.c_str(), tip.c_str());
      return CallbackReturn::FAILURE;
    }

    gravity_ = KDL::Vector(0,0,-9.81);
    id_solver_        = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);
    fk_pos_solver_    = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    jnt_to_jac_solver_= std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);

    // resize KDL containers to chain size
    J_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());

    t_ = 0.0; command_active_ = false; print_counter_ = 0;

    RCLCPP_INFO(get_node()->get_logger(), "Configured CLIK controller: chain joints=%u segs=%u", kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    pub_qd_->on_activate();
    pub_q_->on_activate();
    pub_e_->on_activate();
    pub_xd_->on_activate();
    pub_x_->on_activate();
    pub_ex_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    pub_qd_->on_deactivate();
    pub_q_->on_deactivate();
    pub_e_->on_deactivate();
    pub_xd_->on_deactivate();
    pub_x_->on_deactivate();
    pub_ex_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

private:
  // helpers
  void publish_arrays() {
    // qd, q, e
    std_msgs::msg::Float64MultiArray a;

    a.data.clear(); for (int i=0;i<kNumJoints;++i) a.data.push_back(qd_(i)); pub_qd_->publish(a);
    a.data.clear(); for (int i=0;i<kNumJoints;++i) a.data.push_back(q_(i));  pub_q_->publish(a);
    a.data.clear(); for (int i=0;i<kNumJoints;++i) a.data.push_back(e_(i));  pub_e_->publish(a);

    // xd (pos rpy)
    a.data.clear();
    {
      double r,p,y; xd_.M.GetRPY(r,p,y);
      a.data.push_back(xd_.p.x());
      a.data.push_back(xd_.p.y());
      a.data.push_back(xd_.p.z());
      a.data.push_back(r);
      a.data.push_back(p);
      a.data.push_back(y);
    }
    pub_xd_->publish(a);

    // x (pos rpy)
    a.data.clear();
    {
      double r,p,y; x_.M.GetRPY(r,p,y);
      a.data.push_back(x_.p.x());
      a.data.push_back(x_.p.y());
      a.data.push_back(x_.p.z());
      a.data.push_back(r);
      a.data.push_back(p);
      a.data.push_back(y);
    }
    pub_x_->publish(a);

    // ex
    a.data.clear(); for (int i=0;i<kNumTask;++i) a.data.push_back(ex_(i)); pub_ex_->publish(a);
  }

  // parameters/state
  std::vector<std::string> joint_names_;
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
  std::string robot_description_;

  int ctr_obj_{1};
  int ik_mode_{2};
  double K_regulation_{2.0};
  double K_tracking_{2.0};
  double lambda2_{1e-6};
  bool command_active_{false};
  double t_{0.0};
  const double t_set_{1.0};
  int print_counter_{0};

  // ROS I/O
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  pub_qd_, pub_q_, pub_e_, pub_xd_, pub_x_, pub_ex_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cmd_;

  // KDL structures
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::Vector gravity_;

  std::unique_ptr<KDL::ChainDynParam> id_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  KDL::Jacobian J_{kNumJoints};
  KDL::JntSpaceInertiaMatrix M_;
  KDL::JntArray C_, G_;

  // joint-space states
  KDL::JntArray q_, qdot_, qd_, qd_old_, qd_dot_, qd_ddot_;
  KDL::JntArray e_, e_dot_, e_int_;
  KDL::JntArray tau_d_, aux_d_, comp_d_;
  KDL::JntArray Kp_, Ki_, Kd_;

  // task-space states
  KDL::Frame x_, xd_;
  Eigen::Matrix<double,kNumTask,1> xd_dot_, xd_ddot_, xdot_, ex_, ex_dot_;
  KDL::JntArray x_cmd_{kNumTask}; // used as simple container
};

} // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(arm_controllers::ComputedTorqueCLIKController, controller_interface::ControllerInterface)


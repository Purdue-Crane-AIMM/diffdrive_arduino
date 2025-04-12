#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <mutex>
#include <algorithm>

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino")),
      pid_left_(0.5, 0.1, 0.05),  // Guessed PID gains for left motor
      pid_right_(0.5, 0.1, 0.05) // Guessed PID gains for right motor
{}

void DiffDriveArduino::configureOdometryCallback() {
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  // Use the persistent node instead of a temporary node
  odom_sub_ = subscription_node_->create_subscription<nav_msgs::msg::Odometry>(
      "/lio_sam/mapping/odometry", qos_profile,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_data_ = *msg;

        // Calculate left and right wheel velocities from odometry
        odom_l_wheel_vel_ = odom_data_.twist.twist.linear.x - odom_data_.twist.twist.angular.z * cfg_.wheel_separation / 2.0;
        odom_r_wheel_vel_ = odom_data_.twist.twist.linear.x + odom_data_.twist.twist.angular.z * cfg_.wheel_separation / 2.0;
      });
}

void DiffDriveArduino::configureImuCallback() {
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // Use the persistent node for the subscription
  imu_sub_ = subscription_node_->create_subscription<sensor_msgs::msg::Imu>(
      "/zed/zed_node/imu/data", qos_profile,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_data_ = *msg;

        // Use IMU angular velocity for high-frequency updates
        angular_velocity_ = imu_data_.angular_velocity.z;

        // Adjust wheel velocities based on angular velocity
        imu_l_wheel_vel_ = -angular_velocity_ * cfg_.wheel_separation / 2.0;
        imu_r_wheel_vel_ = angular_velocity_ * cfg_.wheel_separation / 2.0;
      });
}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info) {
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

//   cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
//   cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
//   cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
//   cfg_.device = info_.hardware_parameters["device"];
//   cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
//   cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
//   cfg_.wheel_separation = std::stod(info_.hardware_parameters["wheel_separation"]);

  // Read unified PID parameters
  double kp = std::stod(info_.hardware_parameters["pid_kp"]);
  double ki = std::stod(info_.hardware_parameters["pid_ki"]);
  double kd = std::stod(info_.hardware_parameters["pid_kd"]);

  // Initialize PID controllers with unified values
  pid_left_ = PID(kp, ki, kd);
  pid_right_ = PID(kp, ki, kd);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name);
  r_wheel_.setup(cfg_.right_wheel_name);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

  // Create a persistent node for callbacks
  subscription_node_ = rclcpp::Node::make_shared("diffdrive_arduino_interface");

  // Configure callbacks using the persistent node
  configureOdometryCallback();
  configureImuCallback();

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces() {
  // We need to set up a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface("gripper_joint", hardware_interface::HW_IF_POSITION, &gripper_position_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces() {
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("gripper_joint", hardware_interface::HW_IF_POSITION, &gripper_position_));
  // command_interfaces.emplace_back(hardware_interface::CommandInterface("actuator", "actuator", &actuator_position_));
  return command_interfaces;
}


return_type DiffDriveArduino::start() {
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffDriveArduino::stop() {
  RCLCPP_INFO(logger_, "Stopping Controller...");
  arduino_.disconnect();
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read() {
  // std::lock_guard<std::mutex> odom_lock(odom_mutex_);
  // std::lock_guard<std::mutex> imu_lock(imu_mutex_);

  // // Merge odometry and IMU velocities
  // l_wheel_.vel = odom_l_wheel_vel_ + imu_l_wheel_vel_;
  // r_wheel_.vel = odom_r_wheel_vel_ + imu_r_wheel_vel_;
  // RCLCPP_INFO(logger_, "I read");

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write() {
  if (!arduino_.connected()) {
    return return_type::ERROR;
  }

  // Spin the persistent node to process incoming callbacks
  rclcpp::spin_some(subscription_node_);

  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    std::lock_guard<std::mutex> imu_lock(imu_mutex_);

    // Merge odometry and IMU velocities by weighted average
    l_wheel_.vel = (2.0*odom_l_wheel_vel_ + imu_l_wheel_vel_) / 3.0;
    r_wheel_.vel = (2.0*odom_r_wheel_vel_ + imu_r_wheel_vel_) / 3.0;
    RCLCPP_INFO(logger_, "I read %f,  %f", l_wheel_.vel, r_wheel_.vel);
  }

  // Apply PID control to calculate motor commands
  double left_command = pid_left_.compute(l_wheel_.cmd, l_wheel_.vel);
  double right_command = pid_right_.compute(r_wheel_.cmd, r_wheel_.vel);

  // Send motor commands
  arduino_.setMotorValues(left_command, right_command);
  RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Wanting motor command: %f %f", l_wheel_.cmd, r_wheel_.cmd);

  // Send gripper command only if gripper_position_ is valid
  if (gripper_position_ >= 0.0 && gripper_position_ <= 1.0) {
    arduino_.setGripperState(gripper_position_);
  } else {
    RCLCPP_WARN(logger_, "Invalid gripper position: %f", gripper_position_);
  }

  return return_type::OK;
}

// PID Controller Implementation
double PID::compute(double target, double current) {
  const int PWM_NEUTRAL = 1500;
  const int PWM_MIN = 1000;
  const int PWM_MAX = 2000;

  // Compute PID terms
  double error = target - current;
  integral_ += error;
  double derivative = error - previous_error_;
  previous_error_ = error;

  // Compute raw PID output
  double raw_output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Map raw PID output to PWM range
  double scaled_output = PWM_NEUTRAL + raw_output;
  return std::max(PWM_MIN, std::min(PWM_MAX, int(scaled_output))); // Clamp to valid range
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)
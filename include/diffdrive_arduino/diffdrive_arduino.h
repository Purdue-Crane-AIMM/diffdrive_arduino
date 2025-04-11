#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include <cstring>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"

// PID Controller Declaration
class PID {
public:
  PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0) {}
  double compute(double target, double current);
private:
  double kp_;
  double ki_;
  double kd_;
  double integral_;
  double previous_error_;
};

using hardware_interface::return_type;

class DiffDriveArduino : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  DiffDriveArduino();

  return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type start() override;

  return_type stop() override;

  return_type read() override;

  return_type write() override;

private:
  void configureOdometryCallback(); // Declare the method
  void configureImuCallback();      // Declare the method

  Config cfg_;
  ArduinoComms arduino_;

  Wheel l_wheel_;
  Wheel r_wheel_;
  double gripper_position_;
  double actuator_position_;

  rclcpp::Logger logger_;

  std::chrono::time_point<std::chrono::system_clock> time_;

  // New PID controllers for left and right motors
  PID pid_left_;
  PID pid_right_;

  // New subscription members and sensor data holders
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  nav_msgs::msg::Odometry odom_data_;
  sensor_msgs::msg::Imu imu_data_;

  // Mutexes for thread-safe sensor data access
  std::mutex odom_mutex_;
  std::mutex imu_mutex_;

  // High-frequency data update
  double angular_velocity_;

  // Wheel velocity variables
  double odom_l_wheel_vel_; // Add missing member
  double imu_l_wheel_vel_;  // Add missing member
  double odom_r_wheel_vel_; // Add missing member
  double imu_r_wheel_vel_;  // Add missing member

  // Add persistent node member for subscriptions used in callbacks
  rclcpp::Node::SharedPtr subscription_node_;
};

#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H
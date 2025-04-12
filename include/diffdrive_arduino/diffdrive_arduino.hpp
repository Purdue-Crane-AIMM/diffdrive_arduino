#ifndef DIFFDRIVE_ARDUINO_NODE_HPP_
#define DIFFDRIVE_ARDUINO_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"
#include "diffdrive_arduino/wheel.hpp"
#include "diffdrive_arduino/arduino_comms.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace diffdrive_arduino
{

class DiffDriveArduinoNode : public rclcpp::Node
{
public:
    DiffDriveArduinoNode();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void gripperCallback(const std_msgs::msg::Int8::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    int velocityToMotorCommand(double velocity);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


    rclcpp::TimerBase::SharedPtr control_timer_;
    ArduinoComms comms_;
    Wheel left_wheel_;
    Wheel right_wheel_;
    bool use_pid_;
    double target_linear_;
    double target_angular_;
    nav_msgs::msg::Odometry current_odom_;
    bool have_odom_;
    // Differential drive parameter.
    double wheel_base_;
};

} // namespace diffdrive_arduino

#endif  // DIFFDRIVE_ARDUINO_NODE_HPP_
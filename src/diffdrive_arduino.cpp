#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diffdrive_arduino/wheel.hpp"
#include "diffdrive_arduino/arduino_comms.hpp"
#include <experimental/filesystem>  // Use experimental filesystem for C++14
#include <regex>

namespace fs = std::experimental::filesystem;

std::string find_arduino_device()
{
    std::string device_path = "/dev/ttyUSB0";  // fallback
    std::string dir = "/dev/serial/by-id/";
    std::regex arduino_regex("arduino", std::regex::icase);

    for (const auto &entry : fs::directory_iterator(dir))
    {
        std::string path = entry.path();
        if (std::regex_search(path, arduino_regex))
        {
            device_path = path;
            break;
        }
    }
    return device_path;
}

namespace diffdrive_arduino
{

class DiffDriveArduinoNode : public rclcpp::Node
{
public:
    DiffDriveArduinoNode() : Node("diffdrive_arduino_node"),
                             left_wheel_("left"),
                             right_wheel_("right"),
                             comms_(find_arduino_device(), 57600, 90),
                             target_linear_(0.0),
                             target_angular_(0.0),
                             have_odom_(false),
                             wheel_base_(1.5)  // default wheel separation in meters
    {
        declare_parameter("turn_gain", 0.5);
        declare_parameter("use_pid", true);
        declare_parameter("left_wheel.kp", 1.0);
        declare_parameter("left_wheel.ki", 0.0);
        declare_parameter("left_wheel.kd", 0.0);
        declare_parameter("right_wheel.kp", 1.0);
        declare_parameter("right_wheel.ki", 0.0);
        declare_parameter("right_wheel.kd", 0.0);
        declare_parameter("wheel_base", wheel_base_);

        get_parameter("use_pid", use_pid_);
        get_parameter("wheel_base", wheel_base_);

        double l_kp, l_ki, l_kd, r_kp, r_ki, r_kd;
        get_parameter("left_wheel.kp", l_kp);
        get_parameter("left_wheel.ki", l_ki);
        get_parameter("left_wheel.kd", l_kd);
        get_parameter("right_wheel.kp", r_kp);
        get_parameter("right_wheel.ki", r_ki);
        get_parameter("right_wheel.kd", r_kd);

        left_wheel_.set_pid_gains(l_kp, l_ki, l_kd);
        right_wheel_.set_pid_gains(r_kp, r_ki, r_kd);

        // Subscribe to velocity command topic.
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd/vel", 10, std::bind(&DiffDriveArduinoNode::cmdVelCallback, this, std::placeholders::_1));

        // Subscribe to gripper commands.
        gripper_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/gripper/command", 10, std::bind(&DiffDriveArduinoNode::gripperCallback, this, std::placeholders::_1));

        // If using PID, subscribe to an odometry topic for real state feedback.
        if (use_pid_)
        {
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&DiffDriveArduinoNode::odomCallback, this, std::placeholders::_1));
        }

        control_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&DiffDriveArduinoNode::controlLoop, this));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Update target velocities from the command topic.
        target_linear_ = msg->linear.x;
        target_angular_ = msg->angular.z;
    }

    void gripperCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        comms_.setGripperState(msg->data);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Save the latest odometry information.
        current_odom_ = *msg;
        have_odom_ = true;
    }

    int velocityToMotorCommand(double velocity)
    {
        const double max_velocity = 1.0;
        const double min_velocity = 0.01;

        if (velocity > max_velocity)
        {
            velocity = max_velocity;
        }
        else if (velocity < -max_velocity)
        {
            velocity = -max_velocity;
        }

        int motor_cmd = 1500; // neutral command.
        if (velocity > min_velocity)
        {
            double scale = (2000 - 1650) / max_velocity;
            motor_cmd = static_cast<int>(1650 + velocity * scale);
        }
        else if (velocity < -min_velocity)
        {
            double scale = (1350 - 1000) / max_velocity;
            motor_cmd = static_cast<int>(1350 + velocity * scale);
        }
        return motor_cmd;
    }

    void controlLoop()
    {
        // Compute target wheel velocities using differential drive kinematics.
        const double v_left = target_linear_ - wheel_base_ * target_angular_;
        const double v_right = target_linear_ + wheel_base_ * target_angular_;
        int left_cmd, right_cmd;

        if (use_pid_)
        {
            double left_measured = 0.0;
            double right_measured = 0.0;
            if (have_odom_)
            {
                // Estimate individual wheel speeds from the overall odometry.
                // For a differential drive robot:
                //   left_speed  = v - (wheel_base/2)*omega
                //   right_speed = v + (wheel_base/2)*omega
                left_measured = current_odom_.twist.twist.linear.x - (wheel_base_ / 2.0) * current_odom_.twist.twist.angular.z;
                right_measured = current_odom_.twist.twist.linear.x + (wheel_base_ / 2.0) * current_odom_.twist.twist.angular.z;
            }
            // Update PID controllers with target and measured speeds.
            double left_output = left_wheel_.update(v_left, left_measured);
            double right_output = right_wheel_.update(v_right, right_measured);
            left_cmd = velocityToMotorCommand(left_output);
            right_cmd = velocityToMotorCommand(right_output);
        }
        else
        {
            // Open-loop control without PID.
            left_cmd = velocityToMotorCommand(v_left);
            right_cmd = velocityToMotorCommand(v_right);
        }

        comms_.setMotorValues(left_cmd, right_cmd);
    }

    // Subscribers.
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Timer for control loop.
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Communication and wheel objects.
    ArduinoComms comms_;
    Wheel left_wheel_;
    Wheel right_wheel_;

    // Parameters and control state.
    double wheel_base_;
    bool use_pid_;
    double target_linear_;
    double target_angular_;

    // Odometry feedback.
    nav_msgs::msg::Odometry current_odom_;
    bool have_odom_;
};

}  // namespace diffdrive_arduino

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<diffdrive_arduino::DiffDriveArduinoNode>());
    rclcpp::shutdown();
    return 0;
}

#include "diffdrive_arduino/fake_robot.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"


FakeRobot::FakeRobot()
  : logger_(rclcpp::get_logger("FakeRobot"))
{}



return_type FakeRobot::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

  l_wheel_.setup(cfg_.left_wheel_name);
  r_wheel_.setup(cfg_.right_wheel_name);

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
  // We need to set up a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


return_type FakeRobot::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");
  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type FakeRobot::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type FakeRobot::read()
{
  return return_type::OK;

  
}

hardware_interface::return_type FakeRobot::write()
{

  // Set the wheel velocities to directly match what is commanded

  l_wheel_.vel = l_wheel_.cmd;
  r_wheel_.vel = r_wheel_.cmd;


  return return_type::OK;  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  FakeRobot,
  hardware_interface::SystemInterface
)
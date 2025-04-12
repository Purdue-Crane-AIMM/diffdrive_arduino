#include "diffdrive_arduino/wheel.hpp"
#include <cmath>

namespace diffdrive_arduino
{

Wheel::Wheel(const std::string &wheel_name)
{
  setup(wheel_name);
  // Initialize PID parameters and state variables
  kp = 0.0;
  ki = 0.0;
  kd = 0.0;
  last_error = 0.0;
  integral = 0.0;
  cmd = 0.0;
  // Set default motor command range; adjust these values as needed.
  min_range = 1000;
  max_range = 2000;
}

void Wheel::setup(const std::string &wheel_name)
{
  name = wheel_name;
}

void Wheel::set_pid_gains(double kp_, double ki_, double kd_)
{
  kp = kp_;
  ki = ki_;
  kd = kd_;
  // Reset PID state variables when gains are updated.
  last_error = 0.0;
  integral = 0.0;
}

// Updated PID update: now takes both target and actual state.
double Wheel::update(double target, double actual)
{
  constexpr double dt = 0.1; // Control loop period in seconds (100 ms)
  
  // Compute error using the actual measured state.
  double error = target - actual;
  
  // Update the integral term.
  integral += error * dt;
  
  // Compute the derivative term.
  double derivative = (error - last_error) / dt;
  
  // Compute PID output increment; update stored command value.
  cmd += kp * error + ki * integral + kd * derivative;
  
  // Update last error for the next derivative calculation.
  last_error = error;
  
  // Return the updated control signal (still in the PID internal range).
  return cmd;
}

double Wheel::getMotorCommand()
{
  // Maps the PID command output (assumed to be in range [-3, 3])
  // to the required motor command range [min_range, max_range].
  const double fromMin = -3.0;
  const double fromMax = 3.0;
  const double scale = (max_range - min_range) / (fromMax - fromMin);
  return (cmd - fromMin) * scale + min_range;
}

} // namespace diffdrive_arduino

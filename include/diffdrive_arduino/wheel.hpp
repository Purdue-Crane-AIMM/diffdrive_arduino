#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

#include <string>

namespace diffdrive_arduino {

class Wheel {
public:
    // Public member variables
    std::string name = "";
    double cmd = 0;
    double vel = 0;
    double pos = 0;
    int min_range = 1050;
    int max_range = 1950;

    // Constructors
    Wheel() = default;
    Wheel(const std::string &wheel_name);
    
    // Setup function
    void setup(const std::string &wheel_name);

    // PID control functions
    // Sets the PID gains (proportional, integral, derivative)
    void set_pid_gains(double kp, double ki, double kd);
    // Updates the PID controller using the target value and actual (measured) state
    // Returns the updated internal command output.
    double update(double target, double actual);
    
    // Maps the PID output (internal command) to the motor command range
    double getMotorCommand();

    // PID internal parameters and state
    double kp;
    double ki;
    double kd;
    double last_error;
    double integral;
};

} // namespace diffdrive_arduino

#endif // DIFFDRIVE_ARDUINO_WHEEL_H

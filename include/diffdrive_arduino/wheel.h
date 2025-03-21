#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

#include <string>


class Wheel {
    public:

    std::string name = "";
    double cmd = 0;
    double vel = 0;
    double pos = 0;
    int min_range = 1000;
    int max_range = 2000;

    Wheel() = default;

    Wheel(const std::string &wheel_name);
    
    void setup(const std::string &wheel_name);

    double getMotorCommand();
};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H
#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

#include <string>



class Wheel
{
    public:

    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name);
    
    void setup(const std::string &wheel_name);
};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H
#include "diffdrive_arduino/wheel.h"

#include <cmath>


Wheel::Wheel(const std::string &wheel_name)
{
  setup(wheel_name);
}


void Wheel::setup(const std::string &wheel_name)
{
  name = wheel_name;

}

double Wheel::getMotorCommand() {
  double fromMin = -3;
  double fromMax = 3;
  double scale = (max_range - min_range) / (fromMax - fromMin);
  return (cmd - fromMin) * scale + min_range;
}
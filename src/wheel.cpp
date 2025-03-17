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